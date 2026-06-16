#!/usr/bin/env python3
"""Author real ``real_camera.calibration.v0`` documents for the Sony + iPhone capture.

The committed ``sony_table_v0.calibration.json`` is synthetic/obsolete (marker id 7, a 0.04 m
cube, identity extrinsic). This module derives the **real** calibration from the as-built
capture (``recordings/manifest.json`` is authoritative):

  * cube  : markers 2 (top) / 3 (front) @ 35 mm on a 50 mm cube, MOVABLE        -> body_000
  * tray  : markers 6 (front wall) / 7 (inside floor) @ 50 mm, ~180x180x40 mm    -> body_001
  * table : markers 0 / 1 @ 75 mm, used ONLY to recover the world frame (Z = up)

Design choices (kept deliberately APPROXIMATE — the goal is reliable INSIDE / NOT-INSIDE /
RIM / UNCERTAIN, not metric reconstruction; see the plan):

  * Intrinsics are derived from each camera's lens/sensor (zero distortion), at the clip's
    ACTUAL resolution (Sony is 4K, iPhone 1080p). ``fx`` may take ONE coarse scale correction
    from an AprilTag-based check; it is NOT iterated to make verdicts pass.
  * Metric scale comes primarily from the printed AprilTag sizes (each marker's known
    ``markerLengthM`` metricises its own solvePnP pose). Lateral camera X/Y is fx-independent.
  * The extrinsic (world Z = table-up) is recovered per clip from the FLAT markers (0,1,7):
    we average their solvePnP surface normals (mutually consistent to a few degrees in the
    real footage) for world-up, and anchor the origin at a flat marker. The table markers are
    always visible, so this is robust to a nudged tripod or a repositioned tray.

OpenCV is required for the detect/solvePnP paths (lazy-imported); assembling/validating a
calibration dict from already-detected corners is pure-numpy. Nothing here is imported by
``csg/``.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from csg.common import Json, write_json
from pilots.real_camera.calibrate_table import make_calibration, validate_calibration_v0

# ---------------------------------------------------------------------------------------
# As-built physical constants (recordings/manifest.json is authoritative).
# ---------------------------------------------------------------------------------------
CUBE_MARKER_IDS = (2, 3)            # 2 = top face, 3 = front face
TRAY_MARKER_IDS = (6, 7)           # 6 = front (outer) wall, 7 = inside floor
TABLE_MARKER_IDS = (0, 1)          # extrinsic only; never tracked as an object
FLAT_MARKER_IDS = (0, 1, 7)        # markers that lie flat (normal == table-up)

MARKER_SIZE_M: Dict[int, float] = {0: 0.075, 1: 0.075, 2: 0.035, 3: 0.035, 6: 0.050, 7: 0.050}

CUBE_SIZE_M = [0.05, 0.05, 0.05]   # known task cube (NOT inferred from footage)
CUBE_HALF_M = 0.025                # marker face-center -> cube center, along the marker's -Z

# Tray vertical model. The physical rim is ~4 cm above the floor (marker 7 sits ON the floor).
# We also extend the box ~3 cm BELOW the floor as a tolerance: in the near-top-down view the
# cube's vertical estimate (camera depth ≈ world up — the weak axis) is noisy by ~1-2 cm, which
# would otherwise drop the cube's bottom below the tray floor and break the INSIDE
# 'not below floor' test for a cube genuinely resting inside. The TOP (rim) is unchanged, so the
# INSIDE-vs-ON_RIM discrimination is preserved.
# CALIBRATED / EFFECTIVE tray footprint == the physical ~18x18 cm. A single global +1 cm/side
# expansion to 20 cm was tested under the acceptance protocol (one global value, no per-clip tuning,
# rerun all 78) and REJECTED: it false-PASSes a genuine near-not-inside clip (the cube sits clearly
# outside, beside the tray, but the enlarged box reaches it). The ~1-2 cm approximate-calibration
# error is comparable to the gap between "inside against the wall" and "outside against the wall", so
# no global expansion separates them. Kept at 18 cm -> conservative but calibration-limited (see
# INGESTION_RESULTS.md acceptance log).
TRAY_FOOTPRINT_M = [0.18, 0.18]
TRAY_RIM_M = 0.04                  # rim height above the floor
TRAY_FLOOR_TOL_M = 0.03            # box extends this far below the floor (top-view z-noise tolerance)
TRAY_SIZE_M = [TRAY_FOOTPRINT_M[0], TRAY_FOOTPRINT_M[1], TRAY_RIM_M + TRAY_FLOOR_TOL_M]
TRAY_CENTER_Z_M = (TRAY_RIM_M - TRAY_FLOOR_TOL_M) / 2.0  # marker-7 floor -> tray box center (z)

# Geometric cube top<->front marker-centre separation = s/sqrt(2) for a centred pair on a
# cube of side s. Used as the AprilTag-based scale sanity check (NOT a hard tuning target).
CUBE_23_NOMINAL_M = CUBE_SIZE_M[0] / math.sqrt(2.0)

# ---------------------------------------------------------------------------------------
# Camera intrinsics (derived, zero distortion).
# ---------------------------------------------------------------------------------------
# Full-frame-equivalent horizontal FOV from a 35.9 mm-wide reference sensor. Sony A7IV in
# APS-C/Super35 with a 24 mm lens ~= 36 mm full-frame-equivalent; iPhone wide ~= 26 mm.
_REF_SENSOR_W_MM = 35.9
CAMERA_PROFILES: Dict[str, Dict[str, Any]] = {
    "sony_front": {"equivFocalMm": 36.0, "cameraModel": "Sony ILCE-7M4 (APS-C/Super35, 24mm)"},
    "iphone_top": {"equivFocalMm": 26.0, "cameraModel": "iPhone (top view)"},
}


def derive_intrinsics(equiv_focal_mm: float, width: int, height: int,
                      fx_scale: float = 1.0) -> List[List[float]]:
    """Pinhole camera matrix from a full-frame-equivalent focal length at (width, height).

    ``fx_scale`` applies a single optional coarse correction (default 1.0 == none).
    Principal point at image center; square pixels (fx == fy)."""
    hfov = 2.0 * math.atan(_REF_SENSOR_W_MM / (2.0 * equiv_focal_mm))
    fx = (width / 2.0) / math.tan(hfov / 2.0) * float(fx_scale)
    return [[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]]


# ---------------------------------------------------------------------------------------
# Marker map + objects (ORDER MATTERS: object 0 -> body_000 = cube/mover, object 1 = tray).
# ---------------------------------------------------------------------------------------
_NOMINAL_OFFSET7 = [0.0, 0.0, TRAY_CENTER_Z_M]


def build_marker_map(marker6_offset_m: Optional[Sequence[float]],
                     marker7_offset_m: Sequence[float] = tuple(_NOMINAL_OFFSET7)) -> List[Json]:
    """The runtime marker map. Table markers 0/1 are intentionally absent (extrinsic only;
    unmapped tags are ignored by ``video_to_tracks``). Cube markers carry the same
    ``[0,0,-CUBE_HALF]`` face-center->center offset (works for both the flat top face and the
    vertical front face because each offset is in its own marker frame). BOTH tray markers map
    to the SAME fitted tray center (``fit_tray_offsets``): marker 7 (flat floor, sits toward
    the back of the tray) and marker 6 (front wall) — essential because the cube occludes
    marker 7 when placed inside, so marker 6 must carry the static tray when 7 disappears."""
    off6 = list(marker6_offset_m) if marker6_offset_m is not None else [0.0, -TRAY_SIZE_M[1] / 2.0, TRAY_CENTER_Z_M]
    return [
        {"markerId": 2, "sourceRole": "cube", "markerLengthM": MARKER_SIZE_M[2],
         "offsetM": [0.0, 0.0, -CUBE_HALF_M]},
        {"markerId": 3, "sourceRole": "cube", "markerLengthM": MARKER_SIZE_M[3],
         "offsetM": [0.0, 0.0, -CUBE_HALF_M]},
        {"markerId": 7, "sourceRole": "tray", "markerLengthM": MARKER_SIZE_M[7],
         "offsetM": [float(v) for v in marker7_offset_m]},
        {"markerId": 6, "sourceRole": "tray", "markerLengthM": MARKER_SIZE_M[6],
         "offsetM": [float(v) for v in off6]},
    ]


def build_objects() -> List[Json]:
    """Ordered objects: cube (MOVABLE, body_000) then tray (STATIC container, body_001)."""
    return [
        {"sourceRole": "cube", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": False, "sizeM": list(CUBE_SIZE_M)},
        {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": True, "sizeM": list(TRAY_SIZE_M)},
    ]


# ---------------------------------------------------------------------------------------
# solvePnP helpers (lazy cv2/numpy).
# ---------------------------------------------------------------------------------------
def _marker_obj_points(side_m: float):
    import numpy as np
    h = side_m / 2.0
    # ArUco corner order: TL, TR, BR, BL — matches video_to_tracks.PnPPoseEstimator.
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def solve_marker_pose(corners: Sequence[Sequence[float]], side_m: float, camera_matrix):
    """(tvec, R) of a single square marker in the camera frame (IPPE_SQUARE — stable for a
    planar tag). ``camera_matrix`` is a 3x3 array-like; zero distortion assumed."""
    import cv2
    import numpy as np
    obj = _marker_obj_points(side_m)
    img = np.array(corners, dtype=np.float64)
    K = np.array(camera_matrix, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("solvePnP failed")
    R, _ = cv2.Rodrigues(rvec)
    return tvec.flatten(), R


# ---------------------------------------------------------------------------------------
# Detection aggregation over early frames (robust to per-frame jitter).
# ---------------------------------------------------------------------------------------
def aggregate_marker_corners(video_path: str | Path, *, max_frames: int = 30,
                             min_seen: int = 3) -> Tuple[Dict[int, List[List[float]]], Tuple[int, int]]:
    """Median pixel corners per marker id over the first ``max_frames`` frames of a clip.

    Returns ``({markerId: [[x,y]*4]}, (width, height))``. Only markers seen in >= ``min_seen``
    frames are kept (drops one-frame false positives). Lazy cv2 (needs OpenCV)."""
    import numpy as np
    from pilots.real_camera.marker_tracker import ArucoDetector
    from pilots.real_camera.video_to_tracks import iter_video_frames

    det = ArucoDetector()
    per_id: Dict[int, List[Any]] = {}
    size: Tuple[int, int] = (0, 0)
    for i, frame in enumerate(iter_video_frames(video_path)):
        if i >= max_frames:
            break
        if size == (0, 0):
            h, w = frame.shape[:2]
            size = (int(w), int(h))
        for obs in det.detect(frame):
            per_id.setdefault(int(obs.marker_id), []).append(np.array(obs.corners, dtype=np.float64))
    out: Dict[int, List[List[float]]] = {}
    for mid, stack in per_id.items():
        if len(stack) < min_seen:
            continue
        med = np.median(np.stack(stack, axis=0), axis=0)
        out[mid] = med.tolist()
    return out, size


# ---------------------------------------------------------------------------------------
# Extrinsic: world Z = table-up from the flat markers (0, 1, 7).
# ---------------------------------------------------------------------------------------
def estimate_camera_to_world(corners_by_id: Mapping[int, Sequence[Sequence[float]]],
                             camera_matrix) -> Tuple[List[List[float]], Json]:
    """Recover the camera->world (world-from-camera) 4x4 extrinsic with world Z = table-up.

    World-up = the averaged surface normal of the detected FLAT markers (0,1,7). Origin is
    anchored at a flat marker center; the in-plane yaw is taken from markers 0->1 when both
    are present (yaw is irrelevant to INSIDE/ON_TOP, which use axis-aligned object boxes).
    Returns ``(cameraToWorld, diagnostics)``. Raises if no flat marker is visible."""
    import numpy as np

    flats = [m for m in FLAT_MARKER_IDS if m in corners_by_id]
    if not flats:
        raise ValueError("no flat marker (0/1/7) visible — cannot recover world frame")

    normals = []
    centers: Dict[int, Any] = {}
    for mid in flats:
        t, R = solve_marker_pose(corners_by_id[mid], MARKER_SIZE_M[mid], camera_matrix)
        centers[mid] = t
        normals.append(R[:, 2])  # marker +Z (out of the printed face, toward the camera)
    up = np.mean(np.stack(normals, axis=0), axis=0)
    up = up / np.linalg.norm(up)

    # In-plane X axis ALIGNED TO THE TRAY, so the frozen extractor's axis-aligned tray box
    # matches the (rotated) tray footprint — otherwise a cube placed near a tray wall maps
    # outside the world-axis box. Preference: tray depth (marker6 front wall -> marker7 floor),
    # else marker 7's own in-plane axis (top view), else table markers 0->1, else arbitrary.
    yaw_source = "arbitrary"
    if 6 in corners_by_id and 7 in corners_by_id:
        t6, _ = solve_marker_pose(corners_by_id[6], MARKER_SIZE_M[6], camera_matrix)
        x_raw = centers[7] - t6 if 7 in centers else \
            solve_marker_pose(corners_by_id[7], MARKER_SIZE_M[7], camera_matrix)[0] - t6
        yaw_source = "tray_depth_6to7"
    elif 7 in corners_by_id:
        _, R7 = solve_marker_pose(corners_by_id[7], MARKER_SIZE_M[7], camera_matrix)
        x_raw = R7[:, 0]  # marker 7 is glued aligned to the tray edges
        yaw_source = "marker7_axis"
    elif 0 in centers and 1 in centers:
        x_raw = centers[1] - centers[0]
        yaw_source = "table_0to1"
    else:
        x_raw = np.array([1.0, 0.0, 0.0]) if abs(up[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = x_raw - np.dot(x_raw, up) * up
    nx = np.linalg.norm(x_axis)
    x_axis = x_axis / nx if nx > 1e-6 else np.array([1.0, 0.0, 0.0])
    y_axis = np.cross(up, x_axis)

    # Anchor the origin at the flat marker NEAREST the objects (marker 7, on the tray floor) so
    # the small residual up-vector tilt acts over a ~10 cm lever to the cube/tray instead of the
    # ~0.5 m lever from a far table marker — that lever is what biased the cube's z below the
    # tray floor and broke the INSIDE 'not below floor' test.
    origin_id = 7 if 7 in centers else flats[0]
    origin = centers[origin_id]
    R_wc = np.stack([x_axis, y_axis, up], axis=0)          # rows = world axes in camera frame
    t_wc = -R_wc @ origin
    cam_to_world = np.eye(4)
    cam_to_world[:3, :3] = R_wc
    cam_to_world[:3, 3] = t_wc

    # Pairwise normal agreement (a coarse "is the table really flat" signal).
    spread = 0.0
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            a = float(np.degrees(np.arccos(np.clip(abs(np.dot(normals[i], normals[j])), -1, 1))))
            spread = max(spread, a)
    diag = {"flatMarkers": flats, "maxNormalSpreadDeg": round(spread, 2),
            "yawSource": yaw_source, "upVectorCam": [round(float(v), 4) for v in up]}
    return cam_to_world.tolist(), diag


def marker_world_position(corners_by_id: Mapping[int, Sequence[Sequence[float]]], camera_matrix,
                          cam_to_world: Sequence[Sequence[float]], marker_id: int) -> Optional[List[float]]:
    """World position of a marker's origin (None if absent). Both cameras anchor world to the
    same table markers 0/1, so a world-frame offset (e.g. marker7 -> tray center, fitted where
    BOTH tray markers are visible) transfers across cameras without depending on the fragile
    in-plane yaw of a single planar tag."""
    import numpy as np
    if marker_id not in corners_by_id:
        return None
    M = np.array(cam_to_world, dtype=np.float64)
    t, _ = solve_marker_pose(corners_by_id[marker_id], MARKER_SIZE_M[marker_id], camera_matrix)
    return [round(float(v), 5) for v in (M[:3, :3] @ np.array(t) + M[:3, 3])]


def tray_center_from_marker_offset(corners_by_id: Mapping[int, Sequence[Sequence[float]]], camera_matrix,
                                   cam_to_world: Sequence[Sequence[float]], marker_id: int,
                                   offset_marker: Sequence[float]) -> Optional[List[float]]:
    """World tray center implied by applying a marker-frame offset to a marker's world pose:
    ``P_world + R_world @ offset``. This is how a FROZEN marker->tray-center offset (e.g. from
    manual_calibration's clicked corners) transfers to a clip without per-clip tuning — marker 7
    is glued to the tray, so the offset in its own frame is a fixed physical property and is
    yaw-convention-independent (unlike a world-frame offset). Returns None if the marker is absent."""
    import numpy as np
    if marker_id not in corners_by_id:
        return None
    M = np.array(cam_to_world, dtype=np.float64)
    R_cw, t_cw = M[:3, :3], M[:3, 3]
    t, R = solve_marker_pose(corners_by_id[marker_id], MARKER_SIZE_M[marker_id], camera_matrix)
    pos = R_cw @ np.array(t) + t_cw
    R_world = R_cw @ R
    off = np.array([float(o) for o in offset_marker])
    return [round(float(v), 5) for v in (pos + R_world @ off)]


def measure_cube_marker_spacing(corners_by_id: Mapping[int, Sequence[Sequence[float]]],
                                camera_matrix) -> Optional[float]:
    """3D distance (m) between cube markers 2 and 3 centers — the AprilTag-based scale check
    (nominal == CUBE_23_NOMINAL_M). Returns None if either marker is absent."""
    import numpy as np
    if 2 not in corners_by_id or 3 not in corners_by_id:
        return None
    t2, _ = solve_marker_pose(corners_by_id[2], MARKER_SIZE_M[2], camera_matrix)
    t3, _ = solve_marker_pose(corners_by_id[3], MARKER_SIZE_M[3], camera_matrix)
    return float(np.linalg.norm(np.array(t2) - np.array(t3)))


def fit_tray_offsets(corners_by_id: Mapping[int, Sequence[Sequence[float]]], camera_matrix,
                     cam_to_world: Sequence[Sequence[float]],
                     tray_depth_m: float = TRAY_SIZE_M[1]
                     ) -> Tuple[Optional[List[float]], Optional[List[float]], Optional[List[float]]]:
    """Fit the marker6 & marker7 -> tray-center offsets (each in its OWN marker frame) so both
    map to the same true tray center. The tray center is anchored on the FRONT-WALL marker 6
    (a known tray feature at the front-face center) pushed half the tray depth inward (opposite
    its outward normal), at floor (marker 7) height + half rim. The two offsets are fixed
    physical properties of the tray, so fitting once and reusing is valid (and the offset in a
    marker's own frame is invariant to the camera's world-yaw choice).

    Returns ``(offset6, offset7, tray_center_world)``. If marker 6 is absent (top view),
    ``offset6`` is None and the center falls back to marker 7's column (floor + half height)."""
    import numpy as np
    if 7 not in corners_by_id:
        return None, None, None
    M = np.array(cam_to_world, dtype=np.float64)
    R_cw, t_cw = M[:3, :3], M[:3, 3]

    def world(mid: int):
        t, R = solve_marker_pose(corners_by_id[mid], MARKER_SIZE_M[mid], camera_matrix)
        return R_cw @ np.array(t) + t_cw, R_cw @ R

    P7, R7w = world(7)
    if 6 in corners_by_id:
        # Marker 6 (front wall) and marker 7 (inside floor, ~back-center) span the tray's depth
        # (~0.17 m apart horizontally ≈ the tray depth), so their XY midpoint is a robust tray
        # center — far less fragile than pushing a half-depth along a single marker's normal.
        P6, R6w = world(6)
        center = np.array([(P6[0] + P7[0]) / 2.0, (P6[1] + P7[1]) / 2.0, P7[2] + TRAY_CENTER_Z_M])
        off6 = [round(float(v), 5) for v in (R6w.T @ (center - P6))]
    else:
        center = np.array([P7[0], P7[1], P7[2] + TRAY_CENTER_Z_M])
        off6 = None
    off7 = [round(float(v), 5) for v in (R7w.T @ (center - P7))]
    return off6, off7, [round(float(v), 4) for v in center]


# ---------------------------------------------------------------------------------------
# Assembly.
# ---------------------------------------------------------------------------------------
def assemble_calibration(*, camera: str, width: int, height: int,
                         cam_to_world: Sequence[Sequence[float]],
                         marker6_offset_m: Optional[Sequence[float]],
                         marker7_offset_m: Optional[Sequence[float]] = None,
                         fx_scale: float = 1.0,
                         tray_size_m: Optional[Sequence[float]] = None) -> Json:
    """Build a validated ``real_camera.calibration.v0`` from a recovered extrinsic + geometry."""
    profile = CAMERA_PROFILES[camera]
    camera_matrix = derive_intrinsics(profile["equivFocalMm"], width, height, fx_scale)
    objects = build_objects()
    if tray_size_m is not None:
        objects[1]["sizeM"] = [float(v) for v in tray_size_m]
    off7 = list(marker7_offset_m) if marker7_offset_m is not None else _NOMINAL_OFFSET7
    calib = make_calibration(
        camera_matrix=camera_matrix,
        image_size=[width, height],
        marker_length_m=MARKER_SIZE_M[7],          # default; per-marker overrides in the map
        marker_map=build_marker_map(marker6_offset_m, off7),
        objects=objects,
        dist_coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
        camera_to_world=cam_to_world,
        camera_model=profile["cameraModel"],
        lens={"equivFocalMm": profile["equivFocalMm"], "fxScale": fx_scale,
              "calibrationQuality": "approximate"},
    )
    validate_calibration_v0(calib)
    return calib


def calibration_for_clip(video_path: str | Path, camera: str, *, fx_scale: float = 1.0,
                         marker6_offset_m: Optional[Sequence[float]] = None,
                         marker7_offset_m: Optional[Sequence[float]] = None,
                         tray_size_m: Optional[Sequence[float]] = None,
                         max_frames: int = 30) -> Tuple[Json, Json]:
    """Per-clip calibration: aggregate early-frame detections, derive intrinsics at the clip's
    real resolution, recover the world-up extrinsic, and assemble. The tray marker offsets
    (constant physical properties) are fitted here when not supplied and the markers are
    visible. Returns ``(calibration, diagnostics)``."""
    corners, (w, h) = aggregate_marker_corners(video_path, max_frames=max_frames)
    profile = CAMERA_PROFILES[camera]
    camera_matrix = derive_intrinsics(profile["equivFocalMm"], w, h, fx_scale)
    cam_to_world, ediag = estimate_camera_to_world(corners, camera_matrix)

    fit6, fit7, tray_center = fit_tray_offsets(corners, camera_matrix, cam_to_world)
    off6 = list(marker6_offset_m) if marker6_offset_m is not None else fit6
    off7 = list(marker7_offset_m) if marker7_offset_m is not None else fit7
    calib = assemble_calibration(camera=camera, width=w, height=h, cam_to_world=cam_to_world,
                                 marker6_offset_m=off6, marker7_offset_m=off7,
                                 fx_scale=fx_scale, tray_size_m=tray_size_m)
    # Tray center implied by applying off7 (fitted OR a frozen manual offset) to THIS clip's
    # marker 7 — the per-clip static center the ingest driver injects when a manual sidecar is in
    # play (marker 7 moves with the homemade tray, so this tracks a repositioned tray correctly).
    marker7_tray_center = (tray_center_from_marker_offset(corners, camera_matrix, cam_to_world, 7, off7)
                           if off7 is not None else None)
    diag = {"camera": camera, "resolution": [w, h], "detectedMarkers": sorted(corners.keys()),
            "extrinsic": ediag, "marker6Offset": off6, "marker7Offset": off7,
            "trayCenterWorld": tray_center, "marker7TrayCenter": marker7_tray_center,
            "marker7World": marker_world_position(corners, camera_matrix, cam_to_world, 7),
            "cubeMarkerSpacingM": measure_cube_marker_spacing(corners, camera_matrix),
            "cubeSpacingNominalM": round(CUBE_23_NOMINAL_M, 5), "fxScale": fx_scale}
    return calib, diag


def main(argv: Optional[list[str]] = None) -> int:  # pragma: no cover - real cv2 path
    parser = argparse.ArgumentParser(
        description="Author a real_camera.calibration.v0 from a reference clip (needs OpenCV).")
    parser.add_argument("--video", required=True, help="reference clip (cube OUTSIDE the tray)")
    parser.add_argument("--camera", required=True, choices=sorted(CAMERA_PROFILES))
    parser.add_argument("--out", help="output calibration JSON path (optional)")
    parser.add_argument("--fx-scale", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=30)
    args = parser.parse_args(argv)

    calib, diag = calibration_for_clip(args.video, args.camera, fx_scale=args.fx_scale,
                                       max_frames=args.max_frames)
    print(json.dumps(diag, indent=2))
    if args.out:
        write_json(Path(args.out), calib)
        print(f"author_calibration: wrote {args.out} hash={calib['markerMapHash']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
