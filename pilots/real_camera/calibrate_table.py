#!/usr/bin/env python3
"""Build / load / validate ``real_camera.calibration.v0`` — the camera + table contract.

The Sony A7IV (or any camera) enters the pilot ONLY through this calibration metadata:
camera intrinsics + distortion + lens/zoom + resolution, the table→world transform (from
an ArUco board on the table), the marker map (which tag id is on which object, and the
fixed marker→object-center offset), and the object geometry (cube size, tray
footprint/rim/floor). Everything downstream is device-agnostic.

OpenCV is optional and only the real ``calibrate_camera_from_images`` path needs it
(lazy-imported). Authoring/validating/hashing a calibration dict is pure-stdlib, so the
schema and the synthetic-fixture pipeline run with no cv2 installed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import Json, load_json, write_json

CALIBRATION_SCHEMA_VERSION = "real_camera.calibration.v0"

_REQUIRED_KEYS = ("schemaVersion", "cameraMatrix", "imageSize", "markerLengthM", "markerMap", "objects")
_REQUIRED_OBJECT_FIELDS = ("sourceRole", "physicalKind", "mobility", "sizeM")
_REQUIRED_MARKER_FIELDS = ("markerId", "sourceRole")


class CalibrationError(ValueError):
    """A ``real_camera.calibration.v0`` document is invalid (fail-closed)."""


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise CalibrationError(msg)


def calibration_hash(calibration: Mapping[str, Any]) -> str:
    """Stable sha256 over the identity-defining parts of a calibration (camera profile +
    marker map + object geometry). Stamped into tracks so a track can be bound to the exact
    calibration that produced it; recalibration (new zoom/focus/resolution/marker layout)
    changes the hash."""
    ident = {
        "cameraMatrix": calibration.get("cameraMatrix"),
        "distCoeffs": calibration.get("distCoeffs"),
        "imageSize": calibration.get("imageSize"),
        "markerLengthM": calibration.get("markerLengthM"),
        "markerMap": calibration.get("markerMap"),
        "objects": calibration.get("objects"),
        "cameraToWorld": calibration.get("cameraToWorld"),
        "tableToWorld": calibration.get("tableToWorld"),
    }
    blob = json.dumps(ident, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def validate_calibration_v0(calibration: Mapping[str, Any]) -> None:
    """Fail-closed structural validation of a calibration document."""
    _require(isinstance(calibration, Mapping), "calibration must be an object")
    _require(calibration.get("schemaVersion") == CALIBRATION_SCHEMA_VERSION,
             f"calibration.schemaVersion must be {CALIBRATION_SCHEMA_VERSION!r}, "
             f"got {calibration.get('schemaVersion')!r}")
    missing = [k for k in _REQUIRED_KEYS if k not in calibration]
    _require(not missing, f"calibration missing required keys {missing}")

    cm = calibration["cameraMatrix"]
    _require(isinstance(cm, list) and len(cm) == 3 and all(isinstance(r, list) and len(r) == 3 for r in cm),
             "calibration.cameraMatrix must be a 3x3 matrix")
    img = calibration["imageSize"]
    _require(isinstance(img, Sequence) and not isinstance(img, str) and len(img) == 2,
             "calibration.imageSize must be [width, height]")
    _require(isinstance(calibration["markerLengthM"], (int, float)) and calibration["markerLengthM"] > 0,
             "calibration.markerLengthM must be a positive number")

    # The camera→world (= world-from-camera) extrinsic projects camera-frame marker poses
    # into the table/world frame the predicates reason in. Identity means "camera assumed
    # world/table-aligned" — only valid for a top-down aligned rig; real footage needs the
    # real extrinsic (see estimate_camera_to_world_from_board). Required and shape-checked
    # so a real-video pipeline cannot silently skip the table transform.
    for key in ("cameraToWorld", "tableToWorld"):
        m = calibration.get(key)
        _require(m is None or (isinstance(m, list) and len(m) == 4
                               and all(isinstance(r, list) and len(r) == 4 for r in m)),
                 f"calibration.{key}, when present, must be a 4x4 matrix")

    objects = calibration["objects"]
    _require(isinstance(objects, list) and objects, "calibration.objects must be a non-empty list")
    roles = set()
    for i, obj in enumerate(objects):
        _require(isinstance(obj, Mapping), f"objects[{i}] is not an object")
        miss = [k for k in _REQUIRED_OBJECT_FIELDS if k not in obj]
        _require(not miss, f"objects[{i}] missing required fields {miss}")
        size = obj.get("sizeM")
        _require(isinstance(size, Sequence) and not isinstance(size, str) and len(size) >= 3,
                 f"objects[{i}].sizeM must be a 3-element [x,y,z]")
        roles.add(str(obj["sourceRole"]))

    marker_map = calibration["markerMap"]
    _require(isinstance(marker_map, list) and marker_map, "calibration.markerMap must be a non-empty list")
    seen_ids = set()
    for i, m in enumerate(marker_map):
        _require(isinstance(m, Mapping), f"markerMap[{i}] is not an object")
        miss = [k for k in _REQUIRED_MARKER_FIELDS if k not in m]
        _require(not miss, f"markerMap[{i}] missing required fields {miss}")
        _require(str(m["sourceRole"]) in roles,
                 f"markerMap[{i}].sourceRole {m['sourceRole']!r} is not a declared object role {sorted(roles)}")
        mid = int(m["markerId"])
        _require(mid not in seen_ids, f"markerMap has duplicate markerId {mid}")
        seen_ids.add(mid)


def make_calibration(
    *,
    camera_matrix: Sequence[Sequence[float]],
    image_size: Sequence[int],
    marker_length_m: float,
    marker_map: Sequence[Mapping[str, Any]],
    objects: Sequence[Mapping[str, Any]],
    dist_coeffs: Optional[Sequence[float]] = None,
    camera_to_world: Optional[Sequence[Sequence[float]]] = None,
    table_to_world: Optional[Sequence[Sequence[float]]] = None,
    camera_model: Optional[str] = None,
    lens: Optional[Mapping[str, Any]] = None,
) -> Json:
    """Author a validated ``real_camera.calibration.v0`` document from known parameters.

    ``camera_to_world`` (world-from-camera 4x4) projects camera-frame marker poses into
    the table/world frame; it defaults to identity (camera assumed world-aligned — valid
    only for a top-down aligned rig; real footage must pass the board-derived extrinsic,
    see :func:`estimate_camera_to_world_from_board`). ``table_to_world`` defaults to
    identity (the table frame IS the world frame). The returned dict carries its own
    ``markerMapHash`` (== :func:`calibration_hash`)."""
    calib: Json = {
        "schemaVersion": CALIBRATION_SCHEMA_VERSION,
        "cameraModel": camera_model,
        "lens": dict(lens) if lens else None,
        "imageSize": [int(image_size[0]), int(image_size[1])],
        "cameraMatrix": [[float(v) for v in row] for row in camera_matrix],
        "distCoeffs": [float(v) for v in (dist_coeffs or [0.0, 0.0, 0.0, 0.0, 0.0])],
        "markerLengthM": float(marker_length_m),
        "cameraToWorld": [[float(v) for v in row] for row in (camera_to_world or _identity4())],
        "tableToWorld": [[float(v) for v in row] for row in (table_to_world or _identity4())],
        "markerMap": [dict(m) for m in marker_map],
        "objects": [dict(o) for o in objects],
    }
    calib["markerMapHash"] = calibration_hash(calib)
    validate_calibration_v0(calib)
    return calib


def _identity4() -> List[List[float]]:
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def load_calibration(path: str | Path) -> Json:
    calib = load_json(Path(path))
    validate_calibration_v0(calib)
    return calib


def calibrate_camera_from_images(
    image_paths: Sequence[str | Path],
    *,
    chessboard_size: Sequence[int],
    square_size_m: float,
):  # pragma: no cover - real cv2 path, smoke-tested only (needs cv2 + images)
    """Estimate camera intrinsics from chessboard images (lazy cv2). Returns
    ``(camera_matrix, dist_coeffs, image_size)``. Smoke-tested only — needs OpenCV and
    real calibration images, neither present in CI."""
    import cv2
    import numpy as np

    cols, rows = int(chessboard_size[0]), int(chessboard_size[1])
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_size_m)
    obj_points, img_points, size = [], [], None
    for p in image_paths:
        img = cv2.imread(str(p))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if found:
            obj_points.append(objp)
            img_points.append(corners)
    if not obj_points:
        raise CalibrationError("no chessboard corners found in any calibration image")
    _, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    return mtx.tolist(), dist.flatten().tolist(), list(size)


def estimate_camera_to_world_from_board(
    image_world_points: Sequence[Sequence[float]],
    image_pixel_points: Sequence[Sequence[float]],
    *,
    camera_matrix: Sequence[Sequence[float]],
    dist_coeffs: Optional[Sequence[float]] = None,
):  # pragma: no cover - real cv2 path, smoke-tested only (needs cv2 + a detected board)
    """Solve the camera→world (world-from-camera) 4x4 extrinsic from a table ArUco board.

    ``image_world_points`` are known table/world coordinates of board marker corners and
    ``image_pixel_points`` their detected pixels. solvePnP gives world→camera (board pose
    in the camera); we invert it to get the camera→world transform stored in the
    calibration and applied by ``video_to_tracks.PnPPoseEstimator``. Smoke-tested only."""
    import cv2
    import numpy as np

    obj = np.array(image_world_points, dtype=np.float64)
    img = np.array(image_pixel_points, dtype=np.float64)
    cam = np.array(camera_matrix, dtype=np.float64)
    dist = np.array(dist_coeffs or [0, 0, 0, 0, 0], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, cam, dist)
    if not ok:
        raise CalibrationError("solvePnP failed to locate the table board")
    R, _ = cv2.Rodrigues(rvec)                 # world->camera rotation
    world_from_cam = np.eye(4)
    world_from_cam[:3, :3] = R.T               # invert rotation
    world_from_cam[:3, 3] = (-R.T @ tvec).flatten()  # invert translation
    return world_from_cam.tolist()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate (or hash) a real_camera.calibration.v0 document.")
    parser.add_argument("--calibration", required=True, help="calibration JSON to validate")
    parser.add_argument("--rehash", action="store_true", help="recompute markerMapHash and rewrite the file")
    args = parser.parse_args(argv)

    calib = load_json(Path(args.calibration))
    validate_calibration_v0(calib)
    h = calibration_hash(calib)
    if args.rehash:
        calib["markerMapHash"] = h
        write_json(Path(args.calibration), calib)
    print(f"calibration OK: schemaVersion={calib['schemaVersion']} objects={len(calib['objects'])} "
          f"markers={len(calib['markerMap'])} markerMapHash={h}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
