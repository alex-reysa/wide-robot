"""Real-camera overlay visualisation (Phase 3A) — cv2-free unit tests.

``visualize_episode`` renders how the ingestion pipeline maps a real clip: detected
AprilTags, the virtual tray footprint, the shrunk INSIDE footprint, the cube box, and the
terminal relation / verdict. The DRAWING needs OpenCV, but every decision the overlay makes
(which frame, where the footprints land in world coords, how a world point projects to a
pixel, which relation holds) is pure geometry — so it is unit-tested here with NO cv2 and NO
real video. ``csg/`` is never touched (we only READ ``csg.predicates``).
"""
import math

import pytest

from csg.predicates import DEFAULT
from pilots.real_camera import visualize_episode as vz


# ---------------------------------------------------------------------------
# Frame selection
# ---------------------------------------------------------------------------


def _frame(idx, *, cube=True, tray=True):
    poses = {}
    if cube:
        poses["cube"] = {"positionM": {"x": 0.0, "y": 0.0, "z": 0.0}, "confidence": 1.0}
    if tray:
        poses["tray"] = {"positionM": {"x": -0.09, "y": 0.0, "z": 0.005}, "confidence": 1.0}
    return {"frameIndex": idx, "timeS": idx / 30.0, "poses": poses}


def _frames_with_gaps():
    # cube present on 2..7 only (0,1 and 8,9 are occluded -> no cube pose)
    out = []
    for i in range(10):
        out.append(_frame(i, cube=(2 <= i <= 7)))
    return out


def test_select_frame_index_terminal_is_last_cube_frame():
    assert vz.select_frame_index(_frames_with_gaps(), "terminal") == 7


def test_select_frame_index_start_is_first_cube_frame():
    assert vz.select_frame_index(_frames_with_gaps(), "start") == 2


def test_select_frame_index_integer_selects_exact_frame():
    assert vz.select_frame_index(_frames_with_gaps(), 5) == 5
    assert vz.select_frame_index(_frames_with_gaps(), "5") == 5


def test_select_frame_index_middle_is_a_cube_frame():
    idx = vz.select_frame_index(_frames_with_gaps(), "middle")
    assert idx in {2, 3, 4, 5, 6, 7}


def test_select_frame_index_unknown_marker_raises():
    with pytest.raises((ValueError, KeyError)):
        vz.select_frame_index(_frames_with_gaps(), 99)  # no such frameIndex


# ---------------------------------------------------------------------------
# Footprints (world-frame corner geometry)
# ---------------------------------------------------------------------------


def test_tray_footprint_corners_are_half_extent_in_xy():
    corners = vz.tray_footprint_corners((0.0, 0.0, 0.0), (0.18, 0.18, 0.07))
    assert len(corners) == 4
    xs = sorted({round(c[0], 6) for c in corners})
    ys = sorted({round(c[1], 6) for c in corners})
    assert xs == [-0.09, 0.09]
    assert ys == [-0.09, 0.09]


def test_inside_footprint_corners_shrink_by_margin():
    corners = vz.inside_footprint_corners((0.0, 0.0, 0.0), (0.18, 0.18, 0.07), 0.005)
    xs = sorted({round(c[0], 6) for c in corners})
    ys = sorted({round(c[1], 6) for c in corners})
    assert xs == [-0.085, 0.085]
    assert ys == [-0.085, 0.085]


def test_inside_footprint_matches_predicate_default_margin():
    # The overlay's yellow box must be the SAME shrunk footprint the verifier judges with.
    size = (0.18, 0.18, 0.07)
    corners = vz.inside_footprint_corners((0.0, 0.0, 0.0), size, DEFAULT.inside_footprint_margin_m)
    half = round(0.09 - DEFAULT.inside_footprint_margin_m, 6)
    assert sorted({round(c[0], 6) for c in corners}) == [-half, half]


def test_footprints_translate_with_tray_center():
    corners = vz.tray_footprint_corners((0.30, -0.10, 0.02), (0.18, 0.18, 0.07))
    xs = sorted({round(c[0], 6) for c in corners})
    ys = sorted({round(c[1], 6) for c in corners})
    assert xs == [0.21, 0.39]
    assert ys == [-0.19, -0.01]


# ---------------------------------------------------------------------------
# World -> pixel projection
# ---------------------------------------------------------------------------


_IDENT4 = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
_K = [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]]


def test_project_world_point_on_axis_hits_principal_point():
    uv = vz.project_world_point((0.0, 0.0, 1.0), _IDENT4, _K)
    assert uv is not None
    assert uv[0] == pytest.approx(320.0)
    assert uv[1] == pytest.approx(240.0)


def test_project_world_point_offset_scales_with_focal_over_depth():
    uv = vz.project_world_point((0.1, 0.0, 1.0), _IDENT4, _K)
    assert uv[0] == pytest.approx(800.0 * 0.1 + 320.0)  # 400
    assert uv[1] == pytest.approx(240.0)


def test_project_world_point_behind_camera_is_none():
    assert vz.project_world_point((0.0, 0.0, -1.0), _IDENT4, _K) is None
    assert vz.project_world_point((0.0, 0.0, 0.0), _IDENT4, _K) is None


# ---------------------------------------------------------------------------
# Rigid-transform inverse (cameraToWorld -> worldToCamera)
# ---------------------------------------------------------------------------


def test_invert_rigid_transform_rotation_and_translation():
    Rz90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    M = [Rz90[0] + [1.0], Rz90[1] + [2.0], Rz90[2] + [3.0], [0.0, 0.0, 0.0, 1.0]]
    inv = vz.invert_rigid_transform(M)
    expected = [[0.0, 1.0, 0.0, -2.0], [-1.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, -3.0], [0.0, 0.0, 0.0, 1.0]]
    for r in range(4):
        for c in range(4):
            assert inv[r][c] == pytest.approx(expected[r][c])


def test_invert_rigid_transform_round_trips_a_world_point():
    Rz90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    M = [Rz90[0] + [0.5], Rz90[1] + [-0.2], Rz90[2] + [0.7], [0.0, 0.0, 0.0, 1.0]]
    inv = vz.invert_rigid_transform(M)
    p = (0.13, -0.04, 0.02)
    # cam = inv @ p_world ; world_back = M @ cam should recover p
    cam = [sum(inv[i][k] * p[k] for k in range(3)) + inv[i][3] for i in range(3)]
    back = [sum(M[i][k] * cam[k] for k in range(3)) + M[i][3] for i in range(3)]
    assert back == pytest.approx(list(p))


# ---------------------------------------------------------------------------
# Pose reading + sizes
# ---------------------------------------------------------------------------


def test_pose_xyz_reads_dict_positions():
    f = _frame(3)
    assert vz.pose_xyz(f, "cube") == pytest.approx((0.0, 0.0, 0.0))
    assert vz.pose_xyz(f, "tray") == pytest.approx((-0.09, 0.0, 0.005))


def test_pose_xyz_reads_list_positions():
    f = {"frameIndex": 0, "poses": {"cube": {"positionM": [0.1, 0.2, 0.3]}}}
    assert vz.pose_xyz(f, "cube") == pytest.approx((0.1, 0.2, 0.3))


def test_pose_xyz_missing_role_is_none():
    assert vz.pose_xyz(_frame(0, cube=False), "cube") is None


def test_object_size_finds_role():
    objects = [
        {"sourceRole": "cube", "sizeM": [0.05, 0.05, 0.05]},
        {"sourceRole": "tray", "sizeM": [0.18, 0.18, 0.07]},
    ]
    assert vz.object_size(objects, "tray") == [0.18, 0.18, 0.07]
    assert vz.object_size(objects, "cube") == [0.05, 0.05, 0.05]
    assert vz.object_size(objects, "ghost") is None


# ---------------------------------------------------------------------------
# Primary relation label (reuses csg.predicates — same semantics as the verifier)
# ---------------------------------------------------------------------------


def test_primary_relation_label_inside():
    label = vz.primary_relation_label((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                      (0.05, 0.05, 0.05), (0.18, 0.18, 0.07))
    assert label == "INSIDE"


def test_primary_relation_label_near_but_outside():
    label = vz.primary_relation_label((0.16, 0.0, 0.0), (0.0, 0.0, 0.0),
                                      (0.05, 0.05, 0.05), (0.18, 0.18, 0.07))
    assert label == "NEAR"


def test_primary_relation_label_far():
    label = vz.primary_relation_label((0.6, 0.0, 0.0), (0.0, 0.0, 0.0),
                                      (0.05, 0.05, 0.05), (0.18, 0.18, 0.07))
    assert label == "FAR_FROM"


# ---------------------------------------------------------------------------
# Data lookups
# ---------------------------------------------------------------------------


def test_resolve_video_path_joins_recordings_root(tmp_path):
    manifest = {"videos": [
        {"episodeId": "oic_success_001", "camera": "sony_front",
         "relativePath": "raw_videos/success/oic_success_001__sony_front.mp4"},
        {"episodeId": "oic_success_001", "camera": "iphone_top",
         "relativePath": "raw_videos/success/oic_success_001__iphone_top.mp4"},
    ]}
    p = vz.resolve_video_path(manifest, "oic_success_001", "iphone_top", tmp_path)
    assert p == tmp_path / "raw_videos/success/oic_success_001__iphone_top.mp4"


def test_resolve_video_path_unknown_raises(tmp_path):
    with pytest.raises(KeyError):
        vz.resolve_video_path({"videos": []}, "nope", "sony_front", tmp_path)


def test_verdict_for_finds_episode_camera_row():
    verdicts = {"rows": [
        {"episodeId": "oic_success_001", "camera": "sony_front", "actualTerminal": "FAIL",
         "terminalClass": "NEAR_NOT_INSIDE"},
        {"episodeId": "oic_success_001", "camera": "iphone_top", "actualTerminal": "PASS",
         "terminalClass": None},
    ]}
    row = vz.verdict_for(verdicts, "oic_success_001", "iphone_top")
    assert row["actualTerminal"] == "PASS"
    assert vz.verdict_for(verdicts, "oic_success_001", "missing") is None


# ---------------------------------------------------------------------------
# 3D box corners (cube wireframe)
# ---------------------------------------------------------------------------


def test_box_corners_3d_has_eight_unique_corners():
    corners = vz.box_corners_3d((0.0, 0.0, 0.0), (0.05, 0.05, 0.05))
    assert len(corners) == 8
    assert all(abs(abs(c[i]) - 0.025) < 1e-9 for c in corners for i in range(3))
    assert len({tuple(round(v, 6) for v in c) for c in corners}) == 8
