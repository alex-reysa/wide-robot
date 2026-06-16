#!/usr/bin/env python3
"""Manual tray-boundary calibration — turn four clicked inner-tray corners into a FROZEN,
source-bound tray geometry that overrides the fragile marker-fit.

WHY this exists (see the overlays from ``visualize_episode``): the marker-derived tray center
(``author_calibration.fit_tray_offsets`` — a marker6↔7-midpoint heuristic, or marker-7's column
on the top view) lands ~1-2 cm off the physical cardboard tray, so genuinely-inside cubes read
NEAR_NOT_INSIDE. A global footprint fudge was tested and rejected (it false-PASSed a real
near-miss). The honest fix is to MEASURE the tray boundary once: click the four inner-floor
corners on a single reference frame; back-project them onto the table plane through that clip's
(good) extrinsic; recover the true tray center; and express it as the marker-7 → tray-center
offset *in marker 7's own frame*. That offset is a fixed physical property of the tray (marker 7
is glued to it), so freezing it once and re-applying it via each clip's marker-7 pose corrects
EVERY clip without per-clip tuning — and it is yaw-convention-independent (unlike the world-frame
``world_off7`` the batch driver uses today).

Split like the rest of the pilot: a pure cv2-free core (back-projection, tray geometry, the
marker-frame offset, the sidecar schema) that is fully unit-tested, and a thin lazy-cv2 path that
renders a corner-reading "helper" frame and assembles the sidecar from the detected markers + the
clicked pixels. ``csg/`` is only read indirectly (none here); nothing in ``csg/`` changes.

Workflow (your OpenCV is the headless build — no click window, so read pixels off the image)::

    PY=/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3
    # 1) render a frame with a pixel grid + detected tags, open it in Preview, read 4 corners:
    $PY -m pilots.real_camera.manual_calibration --episode oic_success_001 --camera sony_front \\
        --frame start --helper --out-helper output/manual/
    # 2) feed the clicked inner-tray-FLOOR corners back to freeze the sidecar:
    $PY -m pilots.real_camera.manual_calibration --episode oic_success_001 --camera sony_front \\
        --frame start --corners "frontLeft=1820,1180 frontRight=2160,1170 backRight=2090,980 backLeft=1760,990"
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from csg.common import write_json
from pilots.real_camera import author_calibration as ac
from pilots.real_camera.visualize_episode import (
    invert_rigid_transform,
    load_clip_artifacts,
    select_frame_index,
)

MANUAL_SCHEMA_VERSION = "real_camera.manual_tray_corners.v0"
CORNER_KEYS = ("frontLeft", "frontRight", "backRight", "backLeft")
CAMERAS = ("sony_front", "iphone_top")
_DATASET_ROOT = Path(__file__).resolve().parents[2] / "datasets" / "sony_object_inside_container_v0"
_DEFAULT_HELPER_DIR = Path(__file__).resolve().parents[2] / "output" / "manual"


# ===========================================================================
# Pure geometry core (NO cv2, NO numpy) — unit-tested.
# ===========================================================================


def pixel_to_world_on_plane(pixel: Sequence[float], camera_matrix: Sequence[Sequence[float]],
                            camera_to_world: Sequence[Sequence[float]],
                            plane_z: float = 0.0) -> Optional[List[float]]:
    """Back-project a pixel onto the horizontal world plane ``z == plane_z`` (ray-plane hit).

    The pixel's camera ray ``K^-1 [u,v,1]`` is rotated into world by ``cameraToWorld``'s
    rotation; the camera center is its translation. Returns the world point, or ``None`` if the
    ray is parallel to the plane or would meet it behind the camera (``s <= 0``)."""
    u, v = float(pixel[0]), float(pixel[1])
    fx, fy = float(camera_matrix[0][0]), float(camera_matrix[1][1])
    cx, cy = float(camera_matrix[0][2]), float(camera_matrix[1][2])
    d_cam = [(u - cx) / fx, (v - cy) / fy, 1.0]
    R = [[float(camera_to_world[i][j]) for j in range(3)] for i in range(3)]
    C = [float(camera_to_world[i][3]) for i in range(3)]          # camera center in world
    d_world = [sum(R[i][k] * d_cam[k] for k in range(3)) for i in range(3)]
    if abs(d_world[2]) < 1e-12:
        return None
    s = (plane_z - C[2]) / d_world[2]
    if s <= 0:
        return None
    return [C[i] + s * d_world[i] for i in range(3)]


def _xy_dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def tray_geometry_from_world_corners(corners: Mapping[str, Sequence[float]]) -> Dict[str, Any]:
    """Tray center / footprint / yaw from the four world-frame inner-floor corners.

    ``footprintM = [width, depth]`` averages the two front/back (width) and two side (depth)
    edges; ``centerM`` is the corner centroid; ``yawRad`` is the front-edge heading (frontLeft ->
    frontRight) in the world XY plane. Robust to a slightly non-rectangular homemade tray."""
    fl, fr = corners["frontLeft"], corners["frontRight"]
    br, bl = corners["backRight"], corners["backLeft"]
    pts = [fl, fr, br, bl]
    center = [sum(p[i] for p in pts) / 4.0 for i in range(3)]
    width = 0.5 * (_xy_dist(fl, fr) + _xy_dist(bl, br))
    depth = 0.5 * (_xy_dist(fl, bl) + _xy_dist(fr, br))
    yaw = math.atan2(fr[1] - fl[1], fr[0] - fl[0])
    return {"centerM": [round(c, 6) for c in center],
            "footprintM": [round(width, 6), round(depth, 6)],
            "yawRad": round(yaw, 6),
            "edgeLengthsM": {"front": round(_xy_dist(fl, fr), 6), "back": round(_xy_dist(bl, br), 6),
                             "left": round(_xy_dist(fl, bl), 6), "right": round(_xy_dist(fr, br), 6)}}


def marker_frame_offset(world_center: Sequence[float], marker_world_pos: Sequence[float],
                        marker_world_R: Sequence[Sequence[float]]) -> List[float]:
    """The tray-center expressed in the marker's OWN frame: ``R_world^T @ (center - pos)``.

    This is the frozen physical constant: re-applying it to any clip via that clip's marker pose
    (``pos + R @ offset``) recovers the tray center there, independent of the world yaw choice."""
    d = [float(world_center[i]) - float(marker_world_pos[i]) for i in range(3)]
    Rt = [[float(marker_world_R[j][i]) for j in range(3)] for i in range(3)]  # transpose
    return [round(sum(Rt[i][k] * d[k] for k in range(3)), 6) for i in range(3)]


def validate_manual_corners_v0(doc: Mapping[str, Any]) -> None:
    """Schema-check a ``real_camera.manual_tray_corners.v0`` sidecar (raises ValueError)."""
    if doc.get("schemaVersion") != MANUAL_SCHEMA_VERSION:
        raise ValueError(f"schemaVersion must be {MANUAL_SCHEMA_VERSION!r}, got {doc.get('schemaVersion')!r}")
    if doc.get("camera") not in CAMERAS:
        raise ValueError(f"camera must be one of {CAMERAS}, got {doc.get('camera')!r}")
    px = doc.get("innerTrayFloorCornersPx")
    if not isinstance(px, Mapping):
        raise ValueError("innerTrayFloorCornersPx must be an object with the 4 named corners")
    for key in CORNER_KEYS:
        pt = px.get(key)
        if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
            raise ValueError(f"innerTrayFloorCornersPx.{key} must be [x, y] pixels")


# ===========================================================================
# Marker world poses on a single reference frame (lazy cv2 + numpy).
# ===========================================================================


def _marker_world_poses(frame, camera: str, *, fx_scale: float = 1.0
                        ) -> Tuple[Dict[int, Tuple[List[float], List[List[float]]]],
                                   List[List[float]], List[List[float]], Dict[str, Any]]:  # pragma: no cover - cv2 path
    """Detect markers in ONE BGR frame and return ``{id: (worldPos, worldR)}`` plus the camera
    matrix and ``cameraToWorld`` (world-up from the flat markers). Same intrinsics/extrinsic math
    as ``author_calibration`` so the manual fix lives in the SAME world frame as ingestion."""
    import numpy as np
    from pilots.real_camera.marker_tracker import ArucoDetector

    h, w = frame.shape[:2]
    corners_by_id: Dict[int, Any] = {}
    for obs in ArucoDetector().detect(frame):
        corners_by_id[int(obs.marker_id)] = np.array(obs.corners, dtype=np.float64)

    profile = ac.CAMERA_PROFILES[camera]
    K = ac.derive_intrinsics(profile["equivFocalMm"], w, h, fx_scale)
    cam_to_world, ediag = ac.estimate_camera_to_world(corners_by_id, K)
    M = np.array(cam_to_world, dtype=np.float64)
    R_cw, t_cw = M[:3, :3], M[:3, 3]

    poses: Dict[int, Tuple[List[float], List[List[float]]]] = {}
    for mid, cpix in corners_by_id.items():
        if mid not in ac.MARKER_SIZE_M:
            continue
        t, R = ac.solve_marker_pose(cpix, ac.MARKER_SIZE_M[mid], K)
        world_pos = (R_cw @ np.array(t) + t_cw).tolist()
        world_R = (R_cw @ R).tolist()
        poses[mid] = (world_pos, world_R)
    diag = {"detectedMarkers": sorted(corners_by_id.keys()), "extrinsic": ediag, "imageSize": [w, h]}
    return poses, K, cam_to_world, diag


# ===========================================================================
# Helper-frame render (lazy cv2) — a pixel grid + detected tags to read corners off.
# ===========================================================================


def render_corner_helper(episode_id: str, camera: str, which: Any, *,
                         out_dir: Path = _DEFAULT_HELPER_DIR,
                         dataset_root: Path = _DATASET_ROOT) -> Path:  # pragma: no cover - cv2 path
    """Write a JPEG of the chosen reference frame with a labelled pixel grid + detected tag ids,
    so the four inner-tray-floor corners can be read off in an image viewer."""
    import cv2
    from pilots.real_camera.marker_tracker import ArucoDetector
    from pilots.real_camera.visualize_episode import _read_frame_at, resolve_video_path
    from csg.common import load_json

    art = load_clip_artifacts(episode_id, camera, dataset_root=dataset_root)
    frame_index = select_frame_index(art["tracks"]["frames"], which)
    image = _read_frame_at(art["videoPath"], frame_index)
    h, w = image.shape[:2]
    scale = max(1.0, w / 1920.0)
    step = 100 if w <= 1920 else 200  # grid spacing in pixels

    grid = (60, 60, 60)
    for x in range(0, w, step):
        cv2.line(image, (x, 0), (x, h), grid, 1, cv2.LINE_AA)
        cv2.putText(image, str(x), (x + 3, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, str(x), (x + 3, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (255, 255, 255), 1, cv2.LINE_AA)
    for y in range(0, h, step):
        cv2.line(image, (0, y), (w, y), grid, 1, cv2.LINE_AA)
        cv2.putText(image, str(y), (3, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, str(y), (3, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (255, 255, 255), 1, cv2.LINE_AA)

    for obs in ArucoDetector().detect(image):
        poly = [(int(round(x)), int(round(y))) for x, y in obs.corners]
        for i in range(4):
            cv2.line(image, poly[i], poly[(i + 1) % 4], (255, 255, 0), max(1, int(scale)), cv2.LINE_AA)
        cx = sum(p[0] for p in poly) // 4
        cy = sum(p[1] for p in poly) // 4
        cv2.putText(image, f"id{obs.marker_id}", (cx + 5, cy), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7 * scale, (255, 255, 0), max(1, int(scale)), cv2.LINE_AA)

    lines = [f"{episode_id} [{camera}] frame {frame_index}  size {w}x{h}",
             "Click the 4 INNER-FLOOR tray corners (where wall meets floor):",
             "frontLeft  frontRight  backRight  backLeft  -- read x,y off the grid"]
    for i, line in enumerate(lines):
        y = int(60 * scale) + i * int(34 * scale)
        cv2.putText(image, line, (int(20 * scale), y), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(image, line, (int(20 * scale), y), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 255, 0), 2, cv2.LINE_AA)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{episode_id}__{camera}__corner_helper.jpg"
    cv2.imwrite(str(out_path), image)
    return out_path


# ===========================================================================
# Build + freeze the sidecar from clicked corners (lazy cv2).
# ===========================================================================


def build_manual_calibration(episode_id: str, camera: str, which: Any,
                             corners_px: Mapping[str, Sequence[float]], *,
                             plane_z: float = 0.0, fx_scale: float = 1.0,
                             dataset_root: Path = _DATASET_ROOT) -> Dict[str, Any]:  # pragma: no cover - cv2 path
    """From clicked corner pixels on the reference frame, compute the frozen tray geometry and
    the marker-6/7 → tray-center offsets, and return the full sidecar dict (not yet written)."""
    from pilots.real_camera.visualize_episode import _read_frame_at

    art = load_clip_artifacts(episode_id, camera, dataset_root=dataset_root)
    frame_index = select_frame_index(art["tracks"]["frames"], which)
    frame = _read_frame_at(art["videoPath"], frame_index)
    poses, K, cam_to_world, diag = _marker_world_poses(frame, camera, fx_scale=fx_scale)
    if 7 not in poses:
        raise ValueError(f"marker 7 (tray floor) not detected on {episode_id} {camera} frame "
                         f"{frame_index}; detected={diag['detectedMarkers']}. Pick a frame where "
                         f"the empty tray floor is visible.")

    world_corners = {}
    for key in CORNER_KEYS:
        if key not in corners_px:
            raise ValueError(f"missing clicked corner {key!r}")
        w = pixel_to_world_on_plane(corners_px[key], K, cam_to_world, plane_z=plane_z)
        if w is None:
            raise ValueError(f"corner {key!r} ray does not meet plane z={plane_z}")
        world_corners[key] = [round(c, 6) for c in w]

    geom = tray_geometry_from_world_corners(world_corners)
    center = geom["centerM"]
    p7, r7 = poses[7]
    off7 = marker_frame_offset(center, p7, r7)
    off6 = None
    if 6 in poses:
        p6, r6 = poses[6]
        off6 = marker_frame_offset(center, p6, r6)

    return {
        "schemaVersion": MANUAL_SCHEMA_VERSION,
        "camera": camera,
        "referenceEpisodeId": episode_id,
        "referenceFrameIndex": frame_index,
        "imageSize": diag["imageSize"],
        "planeZ": float(plane_z),
        "innerTrayFloorCornersPx": {k: [float(corners_px[k][0]), float(corners_px[k][1])] for k in CORNER_KEYS},
        "derived": {
            "worldCornersM": world_corners,
            "trayCenterWorldM": center,
            "measuredFootprintM": geom["footprintM"],
            "trayYawRad": geom["yawRad"],
            "edgeLengthsM": geom["edgeLengthsM"],
            "marker7OffsetM": off7,
            "marker6OffsetM": off6,
            "detectedMarkers": diag["detectedMarkers"],
            "extrinsic": diag["extrinsic"],
        },
        "notes": ("Frozen source-bound tray boundary. marker7OffsetM is the marker-7->tray-center "
                  "offset in marker 7's own frame; re-apply per clip as P7_world + R7_world @ offset "
                  "to recover the tray center without per-clip tuning."),
    }


def _parse_corners(spec: str) -> Dict[str, List[float]]:
    """Parse ``"frontLeft=1820,1180 frontRight=2160,1170 ..."`` into a corner dict."""
    out: Dict[str, List[float]] = {}
    for tok in spec.split():
        if "=" not in tok:
            raise ValueError(f"bad corner token {tok!r} (expected name=x,y)")
        name, xy = tok.split("=", 1)
        x, y = xy.split(",")
        out[name] = [float(x), float(y)]
    return out


def sidecar_path(camera: str, dataset_root: Path = _DATASET_ROOT) -> Path:
    return dataset_root / "calibration" / f"manual_tray_corners_{camera}_v0.json"


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover - cv2 path
    parser = argparse.ArgumentParser(
        description="Manual tray-corner calibration: render a corner-reading helper frame, or "
                    "freeze a tray-boundary sidecar from clicked inner-floor corners (needs OpenCV).")
    parser.add_argument("--episode", required=True, help="reference episodeId (tray empty / cube outside)")
    parser.add_argument("--camera", required=True, choices=CAMERAS)
    parser.add_argument("--frame", default="start", help="terminal | start | middle | <int> (default: start)")
    parser.add_argument("--helper", action="store_true", help="render the pixel-grid helper frame and exit")
    parser.add_argument("--out-helper", default=str(_DEFAULT_HELPER_DIR), help="helper-image output dir")
    parser.add_argument("--corners", help='clicked corners: "frontLeft=x,y frontRight=x,y backRight=x,y backLeft=x,y"')
    parser.add_argument("--plane-z", type=float, default=0.0, help="tray-floor world z (default 0.0)")
    parser.add_argument("--fx-scale", type=float, default=1.0)
    parser.add_argument("--out", help="sidecar output path (default: calibration/manual_tray_corners_<cam>_v0.json)")
    args = parser.parse_args(argv)

    frame_arg: Any = args.frame
    if isinstance(frame_arg, str) and frame_arg.strip().lstrip("-").isdigit():
        frame_arg = int(frame_arg)

    if args.helper or not args.corners:
        out = render_corner_helper(args.episode, args.camera, frame_arg, out_dir=Path(args.out_helper))
        print(f"manual_calibration: wrote helper {out}")
        if not args.corners:
            return 0

    corners = _parse_corners(args.corners)
    doc = build_manual_calibration(args.episode, args.camera, frame_arg, corners, plane_z=args.plane_z,
                                   fx_scale=args.fx_scale)
    validate_manual_corners_v0(doc)
    out_path = Path(args.out) if args.out else sidecar_path(args.camera)
    write_json(out_path, doc)
    d = doc["derived"]
    print(f"manual_calibration: wrote {out_path}")
    print(f"  trayCenterWorldM = {d['trayCenterWorldM']}  measuredFootprintM = {d['measuredFootprintM']}")
    print(f"  marker7OffsetM   = {d['marker7OffsetM']}   marker6OffsetM = {d['marker6OffsetM']}")
    print(f"  yawRad = {d['trayYawRad']}  edges = {d['edgeLengthsM']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
