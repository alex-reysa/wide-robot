#!/usr/bin/env python3
"""Convert an ``rh20t.tracks.v0`` episode into a leakage-clean ``csg.rollout.v0``.

This is the **rollout door** for the RH20T pilot: the first and only place RH20T
evidence is minted into the neutral artifact the FROZEN verifier reads. Everything
upstream (the reviewed annotation sidecar) may carry source identity — RH20T task ids
(``task_0017``), task descriptions ("pen"/"holder"), scene paths (``RH20T_cfg3/...``),
and human source roles (``mover``/``container``). This module is the door where that
identity is **dropped** and neutral ``body_NNN`` evidence is handed to the shared
:func:`assemble_rollout` (which re-checks the leakage contract on the way out).

``rh20t.tracks.v0`` (the contract this module owns and validates fail-closed):

    {
      "schemaVersion": "rh20t.tracks.v0",
      "episodeId": "task_0017_user_0001_scene_0001_cfg_0003",   # source-identifying; quarantined below
      "source": {"dataset": "RH20T", "taskId": "task_0017",
                 "taskDescription": "...", "scenePath": "RH20T_cfg3/...",
                 "archiveSha256": "<hex>"},                      # provenance, NOT copied raw into the rollout
      "fps": <float > 0>,
      "objects": [                            # ORDERED: index i -> neutral body_{i:03d}
        {"sourceRole": "mover",     "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": false, "sizeM": [0.04, 0.04, 0.04]},
        {"sourceRole": "container", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": true,  "sizeM": [0.24, 0.18, 0.03]}
      ],
      "frames": [                             # >= MIN_TRACK_FRAMES, monotonic timeS
        {"frameIndex": 0, "timeS": 0.0,
         "poses": {"mover":     {"positionM": {"x":..,"y":..,"z":..}, "confidence": 0.95},
                   "container": {"positionM": {"x":..,"y":..,"z":..}, "confidence": 0.99}}},
        ...
      ]
    }

QUARANTINE (stronger than the real-camera pilot, by necessity): an RH20T ``episodeId``
*is* the source identity (``task_0017_user_..._scene_...``), so — unlike real_camera,
which keeps a neutral ``episodeId`` like ``"ep_test"`` in diagnostics — this module does
NOT copy the raw episode id, task id, task description, or scene path into the rollout,
not even into diagnostics. The rollout is fully source-blind: ``tests/test_rh20t_rollout``
asserts that ``task_0017`` / ``RH20T_cfg3`` / the source role names appear NOWHERE in the
rollout blob. Human-readable provenance (task id, scene path, archive sha) lives in the
committed ``rh20t.tracks.v0`` / annotation sidecar / report — artifacts the verifier never
reads — not in the rollout. Diagnostics carry only a one-way ``episodeRef`` hash (a stable
pointer that does not reveal the task) and the content-derived ``archiveSha256`` — which
``validate_tracks_v0`` rejects fail-closed unless it is a 64-char hex digest or null, so a
human paste-error (a scene path / task id pasted where the sha belongs) cannot smuggle
source identity into the supposedly source-blind rollout.
"""
from __future__ import annotations

import argparse
import hashlib
import re
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import Json, load_json, write_json
from pilots.external_rollout import _IDENTITY_WXYZ, assemble_rollout

TRACKS_SCHEMA_VERSION = "rh20t.tracks.v0"
RH20T_BACKEND = "rh20t_external"

# Mirror of the frozen extractor's MOTION_EPS_M (csg/rollout_extract.py): a body that
# moves less than this between first and last frame is "static". Re-declared here (not
# imported from csg) to keep the pilot's static clamp explicit and decoupled.
MOTION_EPS_M = 0.005

# A real-camera/RH20T episode must have at least this many frames for the extractor to
# see a persisted terminal relation (csg MIN_PERSIST_FRAMES is 2; require a small margin).
MIN_TRACK_FRAMES = 3

_NEUTRAL_ID_FMT = "body_{:03d}"

# A content-derived SHA-256 digest: exactly 64 lowercase hex chars. The ONLY free-form,
# source-derived string allowed into the rollout's diagnostics is ``archiveSha256``, so it
# is validated against this fail-closed (a human paste-error or sloppy/malicious annotator
# pasting a scene path / task id there would otherwise leak source identity into the
# committed, supposedly source-blind rollout — diagnostics is not covered by the frozen
# leakage gate). See validate_tracks_v0.
_SHA256_HEX = re.compile(r"[0-9a-f]{64}")

# No honest effector is tracked in the RH20T annotation seam. Emit a constant, clearly
# off-workspace effector sentinel + gripperClosed=False on every frame so the frozen
# extractor infers NO contact/grasp/co-motion (it cannot, honestly, here) while still
# satisfying the rollout's required frame keys.
_EFFECTOR_SENTINEL = {
    "frameId": "world",
    "positionM": {"x": 0.0, "y": 0.0, "z": 1.0},
    "orientationWxyz": dict(_IDENTITY_WXYZ),
    "confidence": 0.0,
}

_REQUIRED_OBJECT_FIELDS = ("sourceRole", "physicalKind", "mobility", "sizeM")


class RH20TTracksError(ValueError):
    """An ``rh20t.tracks.v0`` episode is structurally invalid or incomplete.

    Raised fail-closed (missing required fields, too few frames, non-monotonic
    timestamps, a frame missing a declared object's pose, non-numeric confidence).
    ``verify_episode`` catches this and reports a ``source_evidence_invalid`` UNCERTAIN
    verdict — never a PASS.
    """


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise RH20TTracksError(msg)


def _pos_xyz(pose: Mapping[str, Any], where: str) -> List[float]:
    _require(isinstance(pose, Mapping), f"{where}: pose must be an object")
    p = pose.get("positionM")
    _require(isinstance(p, Mapping), f"{where}: positionM missing")
    try:
        return [float(p["x"]), float(p["y"]), float(p["z"])]
    except (KeyError, TypeError, ValueError) as exc:
        raise RH20TTracksError(f"{where}: positionM must contain numeric x/y/z: {exc}")


def _roles(tracks: Mapping[str, Any]) -> List[str]:
    return [str(obj["sourceRole"]) for obj in tracks["objects"]]


def validate_tracks_v0(tracks: Mapping[str, Any]) -> None:
    """Fail-closed validation that an episode is structurally CONVERTIBLE to a rollout:
    schema, episode id, fps, >= 2 unique-role objects (mover + container) with sizes,
    >= MIN_TRACK_FRAMES frames with monotonic timestamps, and EVERY declared object
    present with a numeric-confidence well-formed pose in EVERY frame (no occlusion)."""
    _require(isinstance(tracks, Mapping), "tracks must be an object")
    _require(tracks.get("schemaVersion") == TRACKS_SCHEMA_VERSION,
             f"schemaVersion must be {TRACKS_SCHEMA_VERSION!r}")
    _require(bool(tracks.get("episodeId")), "episodeId is required")
    # The only free-form, source-derived string that enters the rollout (via diagnostics)
    # is source.archiveSha256. The rollout is supposed to be fully source-blind, and the
    # frozen leakage gate does NOT inspect diagnostics, so the door must reject a poisoned
    # sha here, fail-closed: a non-hash value (e.g. a pasted scene path / task id) would
    # otherwise smuggle source identity into the committed rollout JSON.
    source = tracks.get("source")
    if source is not None:
        _require(isinstance(source, Mapping), "source must be an object when present")
        sha = source.get("archiveSha256")
        _require(sha is None or (isinstance(sha, str) and _SHA256_HEX.fullmatch(sha) is not None),
                 "source.archiveSha256 must be a 64-char lowercase hex SHA-256 or null "
                 "(a non-hash value would leak RH20T source identity into the rollout)")
    fps = tracks.get("fps")
    _require(isinstance(fps, (int, float)) and not isinstance(fps, bool) and fps > 0,
             "fps must be a positive number")
    objects = tracks.get("objects")
    _require(isinstance(objects, list) and len(objects) >= 2,
             "objects must contain mover and container")
    roles: List[str] = []
    for i, obj in enumerate(objects):
        _require(isinstance(obj, Mapping), f"objects[{i}] must be an object")
        for key in _REQUIRED_OBJECT_FIELDS:
            _require(key in obj, f"objects[{i}] missing {key}")
        size = obj["sizeM"]
        _require(isinstance(size, Sequence) and not isinstance(size, str) and len(size) >= 3,
                 f"objects[{i}].sizeM must be [x,y,z]")
        roles.append(str(obj["sourceRole"]))
    _require(len(set(roles)) == len(roles), "sourceRole values must be unique")
    frames = tracks.get("frames")
    _require(isinstance(frames, list) and len(frames) >= MIN_TRACK_FRAMES,
             f"frames must contain at least {MIN_TRACK_FRAMES} entries")
    last_t: Optional[float] = None
    for fi, frame in enumerate(frames):
        _require(isinstance(frame, Mapping), f"frames[{fi}] must be an object")
        t = frame.get("timeS")
        _require(isinstance(t, (int, float)) and not isinstance(t, bool),
                 f"frames[{fi}].timeS must be numeric")
        if last_t is not None:
            _require(float(t) >= last_t, f"frames[{fi}].timeS is not monotonic")
        last_t = float(t)
        poses = frame.get("poses")
        _require(isinstance(poses, Mapping), f"frames[{fi}].poses must be an object")
        for role in roles:
            _require(role in poses and poses[role] is not None,
                     f"frames[{fi}] missing pose for {role!r}")
            _pos_xyz(poses[role], f"frames[{fi}].poses[{role}]")
            conf = poses[role].get("confidence", 1.0)
            _require(isinstance(conf, (int, float)) and not isinstance(conf, bool),
                     f"frames[{fi}].poses[{role}].confidence must be numeric")


def _neutral_body(index: int, obj: Mapping[str, Any]) -> Json:
    """Build a sceneBody using ONLY whitelisted fields. ``sourceRole`` and any source
    metadata are dropped here — they never reach the rollout."""
    size = list(obj["sizeM"])
    bid = _NEUTRAL_ID_FMT.format(index)
    return {
        "objectId": bid,
        "bodyId": bid,
        "physicalKind": str(obj["physicalKind"]),
        "mobility": str(obj["mobility"]),
        "sizeM": [float(size[0]), float(size[1]), float(size[2])],
        "sizeApproximate": bool(obj.get("sizeApproximate", True)),
        "isContainer": bool(obj.get("isContainer", False)),
    }


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    return ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def tracks_to_rollout(tracks: Mapping[str, Any]) -> Json:
    """Neutralise a validated ``rh20t.tracks.v0`` episode into a ``csg.rollout.v0``.

    Static bodies (``mobility == "STATIC"``, e.g. the container) are snapped to their
    median world position across frames so annotation jitter cannot push them past
    ``MOTION_EPS_M`` and get promoted to a moving "figure" by the extractor's figure-
    ground selection (which would break the (mover, container) pair). The raw max
    static-body excursion is recorded in diagnostics so callers can audit jitter.
    """
    validate_tracks_v0(tracks)
    objects = list(tracks["objects"])
    roles = _roles(tracks)
    frames_in = list(tracks["frames"])
    bodies = [_neutral_body(i, obj) for i, obj in enumerate(objects)]
    role_to_bid = {role: _NEUTRAL_ID_FMT.format(i) for i, role in enumerate(roles)}

    # Median world pose per static body (clamp) + jitter bookkeeping (MAX excursion of
    # any frame from the clamped median — not first-vs-last, which a jitter-then-return
    # would hide).
    static_clamp: Dict[str, List[float]] = {}
    static_excursion: Dict[str, float] = {}
    for obj, role in zip(objects, roles):
        if str(obj["mobility"]) != "STATIC":
            continue
        xyzs = [_pos_xyz(frame["poses"][role], f"poses[{role}]") for frame in frames_in]
        med = [_median([xyz[k] for xyz in xyzs]) for k in range(3)]
        static_clamp[role] = med
        static_excursion[role] = max(
            (sum((xyz[k] - med[k]) ** 2 for k in range(3)) ** 0.5 for xyz in xyzs),
            default=0.0,
        )

    min_conf = 1.0
    out_frames: List[Json] = []
    for frame in frames_in:
        object_poses: Json = {}
        for role in roles:
            pose = frame["poses"][role]
            xyz = static_clamp.get(role) or _pos_xyz(pose, f"poses[{role}]")
            min_conf = min(min_conf, float(pose.get("confidence", 1.0)))
            object_poses[role_to_bid[role]] = {
                "frameId": "world",
                "positionM": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
                "orientationWxyz": dict(_IDENTITY_WXYZ),
                "confidence": float(pose.get("confidence", 1.0)),
            }
        out_frames.append({
            "timeS": float(frame["timeS"]),
            "phase": "external",
            "effectorPose": dict(_EFFECTOR_SENTINEL),
            "gripperClosed": False,
            "objectPoses": object_poses,
            "articulation": {},
        })

    source = tracks.get("source") or {}
    # episodeRef: a one-way hash of the source-identifying episode id. A stable pointer
    # for cross-referencing the committed tracks/report WITHOUT revealing the RH20T task
    # identity in the rollout (the rollout stays fully source-blind; see module docstring).
    episode_ref = hashlib.sha256(str(tracks["episodeId"]).encode("utf-8")).hexdigest()[:16]
    diagnostics = {
        "episodeRef": episode_ref,
        "sourceDataset": "RH20T",
        "archiveSha256": source.get("archiveSha256"),  # validated 64-hex-or-null by validate_tracks_v0
        "fps": float(tracks["fps"]),
        "numTrackFrames": len(out_frames),
        "minPoseConfidence": min_conf,
        "maxStaticBodyMotionM": max(static_excursion.values(), default=0.0),
        "staticBodyClampApplied": sorted(role_to_bid[r] for r in static_clamp),
        "neutralBodyCount": len(bodies),
        "source": "rh20t",
    }
    return assemble_rollout(
        bodies=bodies,
        frames=out_frames,
        object_id_map={},  # external trace: no target identities to map
        backend=RH20T_BACKEND,
        skill_source="rh20t",
        physical_validity_reason=(
            "external RH20T trace: csg cannot re-check physics from dataset/annotation "
            "tracks; physics-unverified by contract (csg/validity.md)"
        ),
        extra_diagnostics=diagnostics,
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Convert rh20t.tracks.v0 into csg.rollout.v0")
    parser.add_argument("--tracks", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    rollout = tracks_to_rollout(load_json(Path(args.tracks)))
    write_json(Path(args.out), rollout)
    print(f"rh20t tracks_to_rollout: wrote {args.out} backend={rollout['backend']} "
          f"bodies={len(rollout['sceneBodies'])} frames={len(rollout['frames'])} "
          f"physicalValidity={rollout['diagnostics']['physicalValidity']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
