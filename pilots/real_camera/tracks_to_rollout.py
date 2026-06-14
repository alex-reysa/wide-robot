#!/usr/bin/env python3
"""Convert a ``real_camera.tracks.v0`` episode into a leakage-clean ``csg.rollout.v0``.

This is the **first and only** place in the real-camera pilot where rollout evidence is
minted. Everything upstream (``marker_tracker`` → marker observations, ``video_to_tracks``
→ tracks) is perception bookkeeping that may carry source identity (tag ids, role names
like "cube"/"tray", colours); this module is the door where that identity is **dropped**
and neutral ``body_NNN`` evidence is handed to the shared :func:`assemble_rollout` (which
re-checks the leakage contract on the way out). It does NOT decide PASS/FAIL/UNCERTAIN —
that is ``verify_episode``'s job; here we either produce a valid neutral rollout or raise
:class:`TracksError` for a structurally-broken episode.

``real_camera.tracks.v0`` (the contract this module owns and validates fail-closed):

    {
      "schemaVersion": "real_camera.tracks.v0",
      "episodeId": "<str>",
      "videoSha256": "<hex|null>",            # provenance only (null for synthetic fixtures)
      "calibrationHash": "<hex|null>",
      "fps": <float > 0>,
      "frameSize": [<w>, <h>],                # optional
      "objects": [                            # ORDERED: index i -> neutral body_{i:03d}
        {"sourceRole": "cube", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": false, "sizeM": [0.04, 0.04, 0.04], "markerIds": [7]},
        {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": true,  "sizeM": [0.24, 0.18, 0.03], "markerIds": [10, 11, 12, 13]}
      ],
      "frames": [                             # >= MIN_TRACK_FRAMES, monotonic timeS
        {"frameIndex": 0, "timeS": 0.0,
         "poses": {"cube": {"positionM": {"x":..,"y":..,"z":..},
                            "orientationWxyz": {"w":..,"x":..,"y":..,"z":..},  # optional
                            "confidence": 0.97},
                   "tray": {...}}},
        ...
      ],
      "occlusionIntervals": [...]             # optional, documentary
    }

``sourceRole`` and ``markerIds`` are **quarantined**: they never enter the rollout's
bodies, ids, objectIdMap, or per-frame pose keys — only neutral ``body_NNN`` does. The
ordered ``objects`` list defines the neutral mapping deterministically (object 0 →
``body_000``, …); for ``object_inside_container`` the convention is cube=body_000 (mover),
tray=body_001 (static container).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import Json, load_json, write_json
from pilots.external_rollout import assemble_rollout, xyzw_to_wxyz, _IDENTITY_WXYZ

# Mirror of the frozen extractor's MOTION_EPS_M (csg/rollout_extract.py): a body that
# moves less than this between first and last frame is "static". We re-declare it here
# (not import from csg) to keep the pilot's clamp explicit and decoupled; a test pins
# the two equal so a future csg retune is caught rather than silently diverging.
MOTION_EPS_M = 0.005

TRACKS_SCHEMA_VERSION = "real_camera.tracks.v0"
CAMERA_BACKEND = "real_camera_external"

# A real-camera episode must have at least this many frames for the extractor to see a
# persisted terminal relation (csg MIN_PERSIST_FRAMES is 2; we require a small margin).
MIN_TRACK_FRAMES = 3

_NEUTRAL_ID_FMT = "body_{:03d}"

# No honest effector is tracked in marker-only 3A. We emit a constant, clearly
# off-workspace effector sentinel + gripperClosed=False on every frame so the frozen
# extractor infers NO contact/grasp/co-motion (it cannot, honestly, here) while still
# satisfying the rollout's required frame keys.
_EFFECTOR_SENTINEL = {
    "frameId": "world",
    "positionM": {"x": 0.0, "y": 0.0, "z": 1.0},
    "orientationWxyz": dict(_IDENTITY_WXYZ),
    "confidence": 0.0,
}

_BODY_SCALAR_FIELDS = ("physicalKind", "mobility", "isContainer", "sizeApproximate")
_REQUIRED_OBJECT_FIELDS = ("sourceRole", "physicalKind", "mobility", "sizeM")


class TracksError(ValueError):
    """A ``real_camera.tracks.v0`` episode is structurally invalid or incomplete.

    Raised fail-closed (missing required fields, too few frames, non-monotonic
    timestamps, a frame missing a declared object's pose). ``verify_episode`` catches
    this and reports it as a ``perception_failure`` UNCERTAIN verdict — never a PASS.
    """


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise TracksError(msg)


def _pos_xyz(pose: Mapping[str, Any], where: str) -> List[float]:
    _require(isinstance(pose, Mapping), f"{where}: pose is not an object: {pose!r}")
    p = pose.get("positionM")
    _require(isinstance(p, Mapping), f"{where}: pose.positionM missing/not an object")
    try:
        return [float(p["x"]), float(p["y"]), float(p["z"])]
    except (KeyError, TypeError, ValueError) as e:
        raise TracksError(f"{where}: pose.positionM must have numeric x,y,z ({e})")


def _roles(tracks: Mapping[str, Any]) -> List[str]:
    return [str(o["sourceRole"]) for o in tracks["objects"]]


def validate_tracks_envelope(tracks: Mapping[str, Any]) -> None:
    """Validate the structural envelope of a ``real_camera.tracks.v0`` episode WITHOUT
    requiring every object to be present in every frame.

    Raw tracks straight out of ``video_to_tracks`` may legitimately OMIT an object in
    some frames (occlusion); that is judged by ``verify_episode``'s quality gate, not
    rejected here. This checks only the top-level shape: schema, ids, fps, object specs,
    frame count, monotonic timestamps, and that any present pose is well-formed.
    """
    _require(isinstance(tracks, Mapping), "tracks must be an object")
    _require(tracks.get("schemaVersion") == TRACKS_SCHEMA_VERSION,
             f"tracks.schemaVersion must be {TRACKS_SCHEMA_VERSION!r}, got {tracks.get('schemaVersion')!r}")
    _require(bool(tracks.get("episodeId")), "tracks.episodeId is required")
    fps = tracks.get("fps")
    _require(isinstance(fps, (int, float)) and fps > 0, f"tracks.fps must be a positive number, got {fps!r}")

    objects = tracks.get("objects")
    _require(isinstance(objects, list) and len(objects) >= 1, "tracks.objects must be a non-empty list")
    roles: List[str] = []
    for i, obj in enumerate(objects):
        _require(isinstance(obj, Mapping), f"objects[{i}] is not an object")
        missing = [k for k in _REQUIRED_OBJECT_FIELDS if k not in obj]
        _require(not missing, f"objects[{i}] missing required fields {missing}")
        size = obj.get("sizeM")
        _require(isinstance(size, Sequence) and not isinstance(size, str) and len(size) >= 3,
                 f"objects[{i}].sizeM must be a 3-element [x,y,z], got {size!r}")
        roles.append(str(obj["sourceRole"]))
    _require(len(set(roles)) == len(roles), f"objects[].sourceRole must be unique, got {roles}")

    frames = tracks.get("frames")
    _require(isinstance(frames, list) and len(frames) >= MIN_TRACK_FRAMES,
             f"tracks.frames must be a list of >= {MIN_TRACK_FRAMES} frames, got {len(frames) if isinstance(frames, list) else frames!r}")
    last_t: Optional[float] = None
    for fi, frame in enumerate(frames):
        _require(isinstance(frame, Mapping), f"frames[{fi}] is not an object")
        t = frame.get("timeS")
        _require(isinstance(t, (int, float)), f"frames[{fi}].timeS must be numeric, got {t!r}")
        if last_t is not None:
            _require(float(t) >= last_t, f"frames[{fi}].timeS={t} is not monotonic (prev {last_t})")
        last_t = float(t)
        poses = frame.get("poses")
        _require(isinstance(poses, Mapping), f"frames[{fi}].poses must be an object")
        for role, pose in poses.items():
            if pose is not None:
                _pos_xyz(pose, f"frames[{fi}].poses[{role}]")
                if isinstance(pose, Mapping) and "confidence" in pose:
                    c = pose["confidence"]
                    _require(isinstance(c, (int, float)) and not isinstance(c, bool),
                             f"frames[{fi}].poses[{role}].confidence must be numeric, got {c!r}")


def validate_tracks_v0(tracks: Mapping[str, Any]) -> None:
    """Fail-closed validation that an episode is structurally CONVERTIBLE to a rollout:
    the envelope plus the requirement that EVERY declared object has a valid pose in
    EVERY frame (no occlusion). ``tracks_to_rollout`` calls this before minting evidence;
    an occluded episode raises here and ``verify_episode`` reports it as UNCERTAIN.

    Does NOT judge tracking *quality* (confidence thresholds) — that is the quality gate.
    """
    validate_tracks_envelope(tracks)
    roles = _roles(tracks)
    for fi, frame in enumerate(tracks["frames"]):
        poses = frame["poses"]
        for role in roles:
            _require(role in poses and poses[role] is not None,
                     f"frames[{fi}].poses is missing declared object {role!r} (occlusion/dropout)")
            _pos_xyz(poses[role], f"frames[{fi}].poses[{role}]")


def _neutral_body(index: int, obj: Mapping[str, Any]) -> Json:
    """Build a sceneBody using ONLY whitelisted fields. sourceRole/markerIds/colour are
    dropped here — they never reach the rollout."""
    bid = _NEUTRAL_ID_FMT.format(index)
    size = list(obj["sizeM"])
    body: Json = {
        "objectId": bid,
        "bodyId": bid,
        "physicalKind": str(obj["physicalKind"]),
        "mobility": str(obj["mobility"]),
        "sizeM": [float(size[0]), float(size[1]), float(size[2])],
        "sizeApproximate": bool(obj.get("sizeApproximate", False)),
        "isContainer": bool(obj.get("isContainer", False)),
    }
    return body


def _median(values: Sequence[float]) -> float:
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def tracks_to_rollout(tracks: Mapping[str, Any]) -> Json:
    """Neutralise a validated ``real_camera.tracks.v0`` episode into a ``csg.rollout.v0``.

    Static bodies (``mobility == "STATIC"``, e.g. the tray) are snapped to their median
    world position across frames so marker jitter cannot push them past ``MOTION_EPS_M``
    and get promoted to a moving "figure" by the extractor's figure-ground selection
    (which would break the (cube, tray) pair). The raw max static-body motion is recorded
    in diagnostics so ``verify_episode`` can flag an over-jittery container as UNCERTAIN.
    """
    validate_tracks_v0(tracks)
    objects = list(tracks["objects"])
    frames_in = list(tracks["frames"])
    roles = [str(o["sourceRole"]) for o in objects]
    role_to_bid = {role: _NEUTRAL_ID_FMT.format(i) for i, role in enumerate(roles)}

    bodies = [_neutral_body(i, obj) for i, obj in enumerate(objects)]

    # Median world pose per static body (clamp), + jitter bookkeeping. For a static
    # body we record the MAX excursion of any frame from the clamped median (not
    # first-vs-last, which a jitter-then-return marker would hide) so verify_episode can
    # flag an over-jittery container as UNCERTAIN even though the clamp neutralises it here.
    static_excursion: Dict[str, float] = {}
    static_clamp: Dict[str, List[float]] = {}
    for i, obj in enumerate(objects):
        role = roles[i]
        xs = [_pos_xyz(f["poses"][role], "") for f in frames_in]
        if str(obj["mobility"]) == "STATIC":
            med = [_median([x[k] for x in xs]) for k in range(3)]
            static_clamp[role] = med
            static_excursion[role] = max(
                (sum((x[k] - med[k]) ** 2 for k in range(3)) ** 0.5 for x in xs), default=0.0)

    min_conf = 1.0
    out_frames: List[Json] = []
    for fi, frame in enumerate(frames_in):
        object_poses: Json = {}
        for role in roles:
            pose = frame["poses"][role]
            xyz = static_clamp.get(role) or _pos_xyz(pose, f"frames[{fi}].poses[{role}]")
            conf = float(pose.get("confidence", 1.0))
            min_conf = min(min_conf, conf)
            orient = pose.get("orientationWxyz")
            if orient is None and "orientationXyzw" in pose:
                try:
                    orient = xyzw_to_wxyz(pose["orientationXyzw"])
                except (IndexError, TypeError, ValueError) as e:
                    raise TracksError(
                        f"frames[{fi}].poses[{role}].orientationXyzw malformed (need 4 numbers): {e}")
            object_poses[role_to_bid[role]] = {
                "frameId": "world",
                "positionM": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
                "orientationWxyz": dict(orient) if isinstance(orient, Mapping) else dict(_IDENTITY_WXYZ),
                "confidence": conf,
            }
        out_frames.append({
            "timeS": float(frame["timeS"]),
            "phase": "external",
            "effectorPose": dict(_EFFECTOR_SENTINEL),
            "gripperClosed": False,
            "objectPoses": object_poses,
            "articulation": {},
        })

    max_static_motion = max(static_excursion.values(), default=0.0)
    diagnostics = {
        "episodeId": str(tracks["episodeId"]),
        "videoSha256": tracks.get("videoSha256"),
        "calibrationHash": tracks.get("calibrationHash"),
        "fps": float(tracks["fps"]),
        "numTrackFrames": len(out_frames),
        "minPoseConfidence": min_conf,
        "maxStaticBodyMotionM": max_static_motion,
        "staticBodyClampApplied": sorted(role_to_bid[r] for r in static_clamp),
        "neutralBodyCount": len(bodies),
        "source": "real_camera",
    }

    return assemble_rollout(
        bodies=bodies,
        frames=out_frames,
        object_id_map={},  # external trace: no target identities to map
        backend=CAMERA_BACKEND,
        skill_source="real_camera",
        physical_validity_reason=(
            "external real-camera trace: csg cannot re-check physics from marker tracks; "
            "physics-unverified by contract (csg/validity.md)"),
        extra_diagnostics=diagnostics,
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert a real_camera.tracks.v0 episode into a leakage-clean csg.rollout.v0.")
    parser.add_argument("--tracks", required=True, help="input real_camera.tracks.v0 JSON")
    parser.add_argument("--out", required=True, help="output csg.rollout.v0 JSON path")
    args = parser.parse_args(argv)

    tracks = load_json(Path(args.tracks))
    rollout = tracks_to_rollout(tracks)
    write_json(Path(args.out), rollout)
    print(f"tracks_to_rollout: wrote {args.out} backend={rollout['backend']} "
          f"bodies={len(rollout['sceneBodies'])} frames={len(rollout['frames'])} "
          f"physicalValidity={rollout['diagnostics']['physicalValidity']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
