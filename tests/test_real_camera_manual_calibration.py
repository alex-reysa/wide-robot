"""Manual tray-corner calibration (Phase 3A next step) — cv2-free unit tests.

``manual_calibration`` turns four clicked inner-tray-floor corner pixels into the frozen,
source-bound tray geometry that overrides the fragile marker-fit (the offset the overlays
showed capping success recall). The DRAWING (helper frame) and the marker detection need
OpenCV, but the geometry — back-projecting a pixel onto the table plane, recovering the tray
center/footprint/yaw, and the marker-frame offset — is pure and tested here with NO cv2.
``csg/`` is untouched.
"""
import math

import pytest

from pilots.real_camera import manual_calibration as mc
from pilots.real_camera.visualize_episode import project_world_point


# ---------------------------------------------------------------------------
# Back-projection: pixel -> world point on a horizontal plane
# ---------------------------------------------------------------------------

_K = [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]]
# A camera 2 m above the origin, looking straight DOWN (camera +z = world -z):
#   camera +x -> world +x, camera +y -> world -y, camera +z -> world -z.
_CAM_TO_WORLD_TOPDOWN = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 2.0],
    [0.0, 0.0, 0.0, 1.0],
]


def test_pixel_to_world_principal_ray_hits_plane_below_camera():
    p = mc.pixel_to_world_on_plane((320.0, 240.0), _K, _CAM_TO_WORLD_TOPDOWN, plane_z=0.0)
    assert p is not None
    assert p[0] == pytest.approx(0.0)
    assert p[1] == pytest.approx(0.0)
    assert p[2] == pytest.approx(0.0)


def test_pixel_to_world_offaxis_scales_by_depth():
    # 2 m depth, pixel offset 0.1*fx -> world offset 0.1 * 2 m = 0.2 m along +x.
    p = mc.pixel_to_world_on_plane((320.0 + 0.1 * 800.0, 240.0), _K, _CAM_TO_WORLD_TOPDOWN, plane_z=0.0)
    assert p[0] == pytest.approx(0.2)
    assert p[1] == pytest.approx(0.0)


def test_pixel_to_world_none_when_ray_parallel_or_behind():
    # A camera looking along +x (no vertical component) never meets a z-plane.
    sideways = [[0.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0, 0, 0, 1.0]]
    assert mc.pixel_to_world_on_plane((320.0, 240.0), _K, sideways, plane_z=0.0) is None


def test_project_then_backproject_round_trips_floor_corners():
    # The integration guarantee: 4 known tray-floor corners -> pixels -> back to world.
    corners = {
        "frontLeft": [-0.09, -0.085, 0.0], "frontRight": [0.09, -0.085, 0.0],
        "backRight": [0.09, 0.095, 0.0], "backLeft": [-0.09, 0.095, 0.0],
    }
    world_to_cam = mc.invert_rigid_transform(_CAM_TO_WORLD_TOPDOWN)
    for name, w in corners.items():
        uv = project_world_point(w, world_to_cam, _K)
        back = mc.pixel_to_world_on_plane(uv, _K, _CAM_TO_WORLD_TOPDOWN, plane_z=0.0)
        assert back == pytest.approx(w, abs=1e-9), name


# ---------------------------------------------------------------------------
# Tray geometry from world corners
# ---------------------------------------------------------------------------


def test_tray_geometry_axis_aligned_square():
    corners = {
        "frontLeft": [-0.09, -0.09, 0.0], "frontRight": [0.09, -0.09, 0.0],
        "backRight": [0.09, 0.09, 0.0], "backLeft": [-0.09, 0.09, 0.0],
    }
    geom = mc.tray_geometry_from_world_corners(corners)
    assert geom["centerM"] == pytest.approx([0.0, 0.0, 0.0])
    assert geom["footprintM"][0] == pytest.approx(0.18)
    assert geom["footprintM"][1] == pytest.approx(0.18)
    assert geom["yawRad"] == pytest.approx(0.0, abs=1e-9)


def test_tray_geometry_translated_rectangle():
    corners = {
        "frontLeft": [0.20, -0.05, 0.01], "frontRight": [0.40, -0.05, 0.01],
        "backRight": [0.40, 0.13, 0.01], "backLeft": [0.20, 0.13, 0.01],
    }
    geom = mc.tray_geometry_from_world_corners(corners)
    assert geom["centerM"][0] == pytest.approx(0.30)
    assert geom["centerM"][1] == pytest.approx(0.04)
    assert geom["footprintM"][0] == pytest.approx(0.20)  # width (front edge)
    assert geom["footprintM"][1] == pytest.approx(0.18)  # depth (side edge)


def test_tray_geometry_recovers_yaw():
    # Rotate a 0.18 square by +30deg about z, centered at origin.
    a = math.radians(30.0)
    base = {"frontLeft": (-0.09, -0.09), "frontRight": (0.09, -0.09),
            "backRight": (0.09, 0.09), "backLeft": (-0.09, 0.09)}
    corners = {}
    for k, (x, y) in base.items():
        corners[k] = [x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a), 0.0]
    geom = mc.tray_geometry_from_world_corners(corners)
    assert geom["yawRad"] == pytest.approx(a, abs=1e-6)
    assert geom["footprintM"][0] == pytest.approx(0.18)
    assert geom["footprintM"][1] == pytest.approx(0.18)


# ---------------------------------------------------------------------------
# Marker-frame offset (the frozen physical constant transferred across clips)
# ---------------------------------------------------------------------------


def test_marker_frame_offset_identity_rotation():
    off = mc.marker_frame_offset([0.0, 0.0, 0.0], [0.05, 0.0, 0.02], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    assert off == pytest.approx([-0.05, 0.0, -0.02])


def test_marker_frame_offset_is_rotation_invariant_recovers_center():
    # offset expressed in the marker frame, re-applied via the marker's world rotation, must
    # land back on the true center: center == pos + R @ offset.
    Rz90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    pos = [0.10, -0.20, 0.0]
    center = [0.0, 0.0, 0.02]
    off = mc.marker_frame_offset(center, pos, Rz90)
    recovered = [pos[i] + sum(Rz90[i][k] * off[k] for k in range(3)) for i in range(3)]
    assert recovered == pytest.approx(center)


# ---------------------------------------------------------------------------
# Sidecar schema
# ---------------------------------------------------------------------------


def _valid_doc():
    return {
        "schemaVersion": mc.MANUAL_SCHEMA_VERSION,
        "camera": "sony_front",
        "referenceEpisodeId": "oic_success_001",
        "referenceFrameIndex": 5,
        "imageSize": [3840, 2160],
        "planeZ": 0.0,
        "innerTrayFloorCornersPx": {
            "frontLeft": [100, 200], "frontRight": [300, 200],
            "backRight": [300, 50], "backLeft": [100, 50],
        },
    }


def test_validate_manual_corners_accepts_valid_doc():
    mc.validate_manual_corners_v0(_valid_doc())  # no raise


def test_validate_manual_corners_rejects_missing_corner():
    doc = _valid_doc()
    del doc["innerTrayFloorCornersPx"]["backLeft"]
    with pytest.raises(ValueError, match="backLeft"):
        mc.validate_manual_corners_v0(doc)


def test_validate_manual_corners_rejects_bad_camera():
    doc = _valid_doc()
    doc["camera"] = "webcam"
    with pytest.raises(ValueError, match="camera"):
        mc.validate_manual_corners_v0(doc)


def test_validate_manual_corners_rejects_bad_schema_version():
    doc = _valid_doc()
    doc["schemaVersion"] = "something.else"
    with pytest.raises(ValueError, match="schemaVersion"):
        mc.validate_manual_corners_v0(doc)
