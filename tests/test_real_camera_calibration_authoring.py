"""cv2-free unit tests for the real-camera calibration authoring + tracks post-processing.

The solvePnP/extrinsic paths need OpenCV (smoke-tested elsewhere); everything here — intrinsics
derivation, marker-map/object authoring, the tray vertical model, and the tracks transforms that
make an occluded real clip judgeable by the frozen verifier — is pure Python and runs in CI.
"""
import pytest

from pilots.real_camera import author_calibration as ac
from pilots.real_camera.calibrate_table import validate_calibration_v0
from pilots.real_camera.track_postprocess import (
    interpolate_mover_gaps,
    stabilize_static_objects,
    trim_to_mover_span,
)


# --------------------------------------------------------------------------- calibration authoring
def test_objects_are_ordered_cube_then_tray():
    objs = ac.build_objects()
    assert [o["sourceRole"] for o in objs] == ["cube", "tray"]  # object 0 -> body_000 = mover
    assert objs[0]["mobility"] == "MOVABLE" and objs[0]["isContainer"] is False
    assert objs[1]["mobility"] == "STATIC" and objs[1]["isContainer"] is True
    assert objs[0]["sizeM"] == [0.05, 0.05, 0.05]  # known 50 mm task cube, not inferred


def test_marker_map_ids_sizes_and_offsets():
    mm = ac.build_marker_map([0.02, 0.0, 0.0], [0.0, 0.0, ac.TRAY_CENTER_Z_M])
    by_id = {m["markerId"]: m for m in mm}
    assert set(by_id) == {2, 3, 6, 7}  # table markers 0/1 are extrinsic-only, never mapped
    assert {by_id[2]["sourceRole"], by_id[3]["sourceRole"]} == {"cube"}
    assert {by_id[6]["sourceRole"], by_id[7]["sourceRole"]} == {"tray"}
    assert by_id[2]["markerLengthM"] == 0.035 and by_id[6]["markerLengthM"] == 0.050
    # both cube faces carry the same own-frame face-center -> center offset
    assert by_id[2]["offsetM"] == [0.0, 0.0, -ac.CUBE_HALF_M] == by_id[3]["offsetM"]


def test_tray_vertical_model_extends_below_floor_but_keeps_rim():
    # box spans floor-tol BELOW the floor (top-view z-noise tolerance) up to the rim
    assert ac.TRAY_SIZE_M[2] == pytest.approx(ac.TRAY_RIM_M + ac.TRAY_FLOOR_TOL_M)
    assert ac.TRAY_CENTER_Z_M == pytest.approx((ac.TRAY_RIM_M - ac.TRAY_FLOOR_TOL_M) / 2.0)
    top = ac.TRAY_CENTER_Z_M + ac.TRAY_SIZE_M[2] / 2.0
    bottom = ac.TRAY_CENTER_Z_M - ac.TRAY_SIZE_M[2] / 2.0
    assert top == pytest.approx(ac.TRAY_RIM_M)          # rim unchanged -> INSIDE-vs-ON_RIM preserved
    assert bottom == pytest.approx(-ac.TRAY_FLOOR_TOL_M)


def test_derive_intrinsics_scales_with_resolution_and_fx_scale():
    K = ac.derive_intrinsics(36.0, 3840, 2160)
    assert K[0][2] == 1920.0 and K[1][2] == 1080.0           # principal point at image center
    assert K[0][0] == pytest.approx(K[1][1])                  # square pixels
    K2 = ac.derive_intrinsics(36.0, 3840, 2160, fx_scale=2.0)
    assert K2[0][0] == pytest.approx(2.0 * K[0][0])           # one coarse scale knob


def test_assemble_calibration_is_schema_valid_and_marks_approximate():
    eye = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    calib = ac.assemble_calibration(camera="iphone_top", width=1920, height=1080,
                                    cam_to_world=eye, marker6_offset_m=None,
                                    marker7_offset_m=[0.0, 0.0, ac.TRAY_CENTER_Z_M])
    validate_calibration_v0(calib)  # raises if invalid
    assert calib["lens"]["calibrationQuality"] == "approximate"
    assert calib["objects"][1]["sizeM"][2] == pytest.approx(ac.TRAY_SIZE_M[2])


# --------------------------------------------------------------------------- tracks post-processing
def _tracks(cube_frames, tray_frames=None):
    """Minimal real_camera.tracks.v0-shaped dict; cube_frames[i] is a (x,y,z) or None."""
    objs = [{"sourceRole": "cube", "mobility": "MOVABLE"}, {"sourceRole": "tray", "mobility": "STATIC"}]
    frames = []
    for i, c in enumerate(cube_frames):
        poses = {}
        if c is not None:
            poses["cube"] = {"positionM": {"x": c[0], "y": c[1], "z": c[2]}, "confidence": 1.0}
        if tray_frames is not None and tray_frames[i] is not None:
            t = tray_frames[i]
            poses["tray"] = {"positionM": {"x": t[0], "y": t[1], "z": t[2]}, "confidence": 1.0}
        frames.append({"frameIndex": i, "timeS": i * 0.1, "poses": poses})
    return {"objects": objs, "frames": frames}


def test_trim_to_mover_span_drops_leading_and_trailing_cubeless_frames():
    tr = _tracks([None, None, (0, 0, 0), (1, 0, 0), (2, 0, 0), None])
    trim_to_mover_span(tr)
    assert len(tr["frames"]) == 3
    assert tr["frames"][0]["poses"]["cube"]["positionM"]["x"] == 0
    assert tr["frames"][-1]["poses"]["cube"]["positionM"]["x"] == 2


def test_interpolate_mover_gaps_fills_short_gaps_only():
    # one 1-frame gap (<=max) is interpolated; a 3-frame gap (>max=2) is left unfilled
    tr = _tracks([(0, 0, 0), None, (2, 0, 0), None, None, None, (6, 0, 6)])
    interpolate_mover_gaps(tr, max_gap=2)
    filled = tr["frames"][1]["poses"]["cube"]
    assert filled["positionM"]["x"] == pytest.approx(1.0)  # midpoint of (0)->(2)
    assert filled["interpolated"] is True
    assert "cube" not in tr["frames"][3]["poses"]          # long gap stays empty -> gate flags it


def test_stabilize_static_override_holds_tray_and_leaves_cube_untouched():
    tr = _tracks([(0, 0, 0), (1, 0, 0), (2, 0, 0)],
                 tray_frames=[(9, 9, 9), None, (9, 9, 9)])  # tray occluded mid-clip
    stabilize_static_objects(tr, overrides={"tray": [5.0, 5.0, 0.5]})
    for f in tr["frames"]:                                 # tray present + fixed in EVERY frame
        assert f["poses"]["tray"]["positionM"] == {"x": 5.0, "y": 5.0, "z": 0.5}
    # mover is never fabricated by stabilize
    assert [f["poses"]["cube"]["positionM"]["x"] for f in tr["frames"]] == [0, 1, 2]


def test_stabilize_static_median_when_no_override():
    tr = _tracks([(0, 0, 0), (1, 0, 0), (2, 0, 0)],
                 tray_frames=[(8, 0, 0), (10, 0, 0), (12, 0, 0)])
    stabilize_static_objects(tr)  # no override -> per-axis median (x=10)
    assert all(f["poses"]["tray"]["positionM"]["x"] == 10 for f in tr["frames"])
