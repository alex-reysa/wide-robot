#!/usr/bin/env python3
"""Turn detected markers into a ``real_camera.tracks.v0`` episode.

Orchestration only: iterate frames → ask a :class:`~pilots.real_camera.marker_tracker.MarkerDetector`
for marker observations → map each marker to its object role (via the calibration's
``markerMap``) → estimate each object's world pose (via an injectable
:class:`MarkerPoseEstimator`) → assemble tracks. A frame where an object's markers are not
detected legitimately OMITS that object (occlusion); ``verify_episode``'s quality gate, not
this module, judges whether that uncertainty is tolerable.

Both OpenCV-touching pieces are injectable and lazy:
  * detection — :class:`~pilots.real_camera.marker_tracker.ArucoDetector` (real) /
    ``FakeDetector`` (tests);
  * pose estimation — :class:`PnPPoseEstimator` (real cv2.solvePnP) / :class:`FakePoseEstimator`
    (tests, no cv2).
So the whole video→tracks orchestration is unit-testable with NO OpenCV and NO real video.
``video_to_tracks`` itself NEVER mints rollout evidence — that is ``tracks_to_rollout``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Protocol, Tuple

from csg.common import Json, load_json, write_json

from pilots.real_camera.calibrate_table import calibration_hash, validate_calibration_v0
from pilots.real_camera.marker_tracker import ArucoDetector, MarkerDetector, MarkerObservation
from pilots.real_camera.tracks_to_rollout import TRACKS_SCHEMA_VERSION, validate_tracks_envelope

WorldPose = Tuple[Dict[str, float], Dict[str, float], float]  # (positionM, orientationWxyz, confidence)


class MarkerPoseEstimator(Protocol):
    """Estimate one marker's object-center WORLD pose from its observation + calibration."""

    def estimate(self, observation: MarkerObservation, marker_entry: Mapping[str, Any],
                 calibration: Mapping[str, Any]) -> WorldPose:
        ...


def _marker_index(calibration: Mapping[str, Any]) -> Dict[int, Mapping[str, Any]]:
    return {int(m["markerId"]): m for m in calibration["markerMap"]}


def _tracks_objects(calibration: Mapping[str, Any]) -> List[Json]:
    """Build tracks.objects from calibration.objects, attaching each role's marker ids."""
    role_markers: Dict[str, List[int]] = {}
    for m in calibration["markerMap"]:
        role_markers.setdefault(str(m["sourceRole"]), []).append(int(m["markerId"]))
    out: List[Json] = []
    for obj in calibration["objects"]:
        role = str(obj["sourceRole"])
        out.append({
            "sourceRole": role,
            "physicalKind": obj["physicalKind"],
            "mobility": obj["mobility"],
            "isContainer": bool(obj.get("isContainer", False)),
            "sizeM": [float(v) for v in obj["sizeM"]],
            "markerIds": sorted(role_markers.get(role, [])),
        })
    return out


def build_tracks(
    frames: Iterable[Any],
    *,
    detector: MarkerDetector,
    estimator: MarkerPoseEstimator,
    calibration: Mapping[str, Any],
    fps: float,
    episode_id: str,
    video_sha256: Optional[str] = None,
) -> Json:
    """Assemble a ``real_camera.tracks.v0`` episode (envelope-valid; occlusion allowed)."""
    validate_calibration_v0(calibration)
    midx = _marker_index(calibration)

    out_frames: List[Json] = []
    for i, frame in enumerate(frames):
        observations = detector.detect(frame)
        # role -> list of (pose, confidence) from each detected marker for that object
        per_role: Dict[str, List[WorldPose]] = {}
        for obs in observations:
            entry = midx.get(int(obs.marker_id))
            if entry is None:
                continue  # an unmapped tag (e.g. a board marker) — ignore, not an object
            role = str(entry["sourceRole"])
            per_role.setdefault(role, []).append(estimator.estimate(obs, entry, calibration))
        poses: Json = {}
        for role, candidates in per_role.items():
            # Fuse multiple markers for one object: take the highest-confidence estimate.
            pos, orient, conf = max(candidates, key=lambda c: c[2])
            poses[role] = {
                "positionM": {"x": float(pos["x"]), "y": float(pos["y"]), "z": float(pos["z"])},
                "orientationWxyz": dict(orient),
                "confidence": float(conf),
            }
        out_frames.append({"frameIndex": i, "timeS": i / float(fps), "poses": poses})

    tracks: Json = {
        "schemaVersion": TRACKS_SCHEMA_VERSION,
        "episodeId": str(episode_id),
        "videoSha256": video_sha256,
        "calibrationHash": calibration.get("markerMapHash") or calibration_hash(calibration),
        "fps": float(fps),
        "frameSize": list(calibration.get("imageSize", [])),
        "objects": _tracks_objects(calibration),
        "frames": out_frames,
    }
    validate_tracks_envelope(tracks)
    return tracks


_IDENTITY4 = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _mat3_of(m4: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[float(m4[r][c]) for c in range(3)] for r in range(3)]


def _mat3_mul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat3_vec(m: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _apply_affine4(m4: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    """Apply a 4x4 rigid transform (R|t) to a 3-vector."""
    return [sum(m4[i][k] * v[k] for k in range(3)) + m4[i][3] for i in range(3)]


def rotmat_to_wxyz(R: Sequence[Sequence[float]]) -> Dict[str, float]:
    """3x3 rotation matrix → WXYZ quaternion (pure Python; cv2-free)."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2.0
        w, x, y, z = 0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s
    elif R[0][0] >= R[1][1] and R[0][0] >= R[2][2]:
        s = (1.0 + R[0][0] - R[1][1] - R[2][2]) ** 0.5 * 2.0
        w, x, y, z = (R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s
    elif R[1][1] >= R[2][2]:
        s = (1.0 + R[1][1] - R[0][0] - R[2][2]) ** 0.5 * 2.0
        w, x, y, z = (R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s
    else:
        s = (1.0 + R[2][2] - R[0][0] - R[1][1]) ** 0.5 * 2.0
        w, x, y, z = (R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s
    return {"w": float(w), "x": float(x), "y": float(y), "z": float(z)}


def compose_marker_world_pose(
    tvec_cam: Sequence[float],
    R_cam: Sequence[Sequence[float]],
    camera_to_world: Sequence[Sequence[float]],
    offset_marker: Sequence[float],
    confidence: float,
) -> WorldPose:
    """Compose a marker's WORLD pose from its camera-frame pose + the camera→world extrinsic.

    Pure Python (cv2-free, unit-tested): the marker ORIGIN in camera coords (``tvec_cam``)
    is projected to world by ``camera_to_world``; the marker→object-center ``offset_marker``
    (expressed in the marker's own frame) is rotated by the marker's WORLD rotation
    (R_world = R_cameraToWorld · R_cam) before being added. This is what makes INSIDE/
    ON_TOP_OF correct when the camera is NOT table-aligned.
    """
    R_cw = _mat3_of(camera_to_world)
    world_origin = _apply_affine4(camera_to_world, tvec_cam)
    R_world = _mat3_mul(R_cw, [list(map(float, row)) for row in R_cam])
    off_world = _mat3_vec(R_world, [float(o) for o in offset_marker])
    pos = {"x": world_origin[0] + off_world[0],
           "y": world_origin[1] + off_world[1],
           "z": world_origin[2] + off_world[2]}
    return (pos, rotmat_to_wxyz(R_world), float(confidence))


def marker_square_length_m(marker_entry: Mapping[str, Any], calibration: Mapping[str, Any]) -> float:
    """Physical side length of this marker square, allowing mixed-size print sheets."""
    value = marker_entry.get("markerLengthM", calibration["markerLengthM"])
    return float(value)


class PnPPoseEstimator:  # pragma: no cover - real cv2 path, smoke-tested only
    """Real marker→world pose: cv2.solvePnP against a square marker (camera frame) then
    the validated ``cameraToWorld`` extrinsic via :func:`compose_marker_world_pose`.

    Lazy-imports cv2/numpy. Smoke-tested only (needs OpenCV); the world-projection math is
    covered by a cv2-free unit test, and the orchestration by FakePoseEstimator in CI.
    """

    def estimate(self, observation: MarkerObservation, marker_entry: Mapping[str, Any],
                 calibration: Mapping[str, Any]) -> WorldPose:
        import cv2
        import numpy as np

        half = marker_square_length_m(marker_entry, calibration) / 2.0
        # Marker corner model in the marker's own frame (matches ArUco corner order).
        obj_pts = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                           dtype=np.float64)
        img_pts = np.array(observation.corners, dtype=np.float64)
        cam_mtx = np.array(calibration["cameraMatrix"], dtype=np.float64)
        dist = np.array(calibration.get("distCoeffs", [0, 0, 0, 0, 0]), dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam_mtx, dist)
        if not ok:
            raise ValueError(f"solvePnP failed for marker {observation.marker_id}")
        R, _ = cv2.Rodrigues(rvec)  # marker->camera rotation
        return compose_marker_world_pose(
            tvec.flatten().tolist(), R.tolist(),
            calibration.get("cameraToWorld") or _IDENTITY4,
            marker_entry.get("offsetM", [0.0, 0.0, 0.0]),
            float(observation.confidence))


def iter_video_frames(video_path: str | Path):  # pragma: no cover - real cv2 path
    """Yield BGR frames from a video file (lazy cv2). Smoke-tested only."""
    import cv2
    cap = cv2.VideoCapture(str(video_path))
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def sha256_file(path: str | Path, _chunk: int = 1 << 20) -> str:
    """Streaming SHA256 of a file (used to tool-bind raw-video provenance into tracks)."""
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(_chunk), b""):
            h.update(block)
    return h.hexdigest()


def main(argv: Optional[list[str]] = None) -> int:  # pragma: no cover - real cv2 path
    parser = argparse.ArgumentParser(
        description="Extract real_camera.tracks.v0 from a video using ArUco markers (needs OpenCV).")
    parser.add_argument("--video", required=True, help="input video file")
    parser.add_argument("--calibration", required=True, help="real_camera.calibration.v0 JSON")
    parser.add_argument("--out", required=True, help="output tracks JSON path")
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--video-sha256", default=None,
                        help="raw-video SHA256 for provenance; computed from --video if omitted")
    args = parser.parse_args(argv)

    calibration = load_json(Path(args.calibration))
    video_sha = args.video_sha256 or sha256_file(args.video)
    tracks = build_tracks(
        iter_video_frames(args.video),
        detector=ArucoDetector(),
        estimator=PnPPoseEstimator(),
        calibration=calibration,
        fps=args.fps,
        episode_id=args.episode_id or Path(args.video).stem,
        video_sha256=video_sha,
    )
    write_json(Path(args.out), tracks)
    print(f"video_to_tracks: wrote {args.out} frames={len(tracks['frames'])} objects={len(tracks['objects'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
