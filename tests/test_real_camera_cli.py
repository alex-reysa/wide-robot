"""Real-camera CLI + video→tracks orchestration (Phase 3A).

In-process smoke of the pipeline (calibration → tracks → rollout → verify) driven by a
FakeDetector + FakePoseEstimator so it runs with NO OpenCV and NO real video. Also: the
pyproject camera-extra lock, a csg-frozen guard, and a real cv2/ArUco round-trip that is
SKIPPED when OpenCV is unavailable. ``csg/`` is never touched.
"""
import json
import subprocess
from pathlib import Path

import pytest

from pilots.real_camera import tracks_to_rollout as t2r
from pilots.real_camera import verify_episode as ve
from pilots.real_camera.calibrate_table import make_calibration, validate_calibration_v0
from pilots.real_camera.marker_tracker import FakeDetector, MarkerObservation
from pilots.real_camera.video_to_tracks import MarkerPoseEstimator, build_tracks

_REPO = Path(__file__).resolve().parents[1]
TX, TY, TZ = 0.30, 0.0, 0.015
_INSIDE = (TX, TY, 0.03)
_START_NEAR = (TX + 0.16, TY, 0.05)


def _approach_then(end_xyz):
    sx, sy, sz = _START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _calibration():
    return make_calibration(
        camera_matrix=[[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]],
        image_size=[1920, 1080],
        marker_length_m=0.03,
        marker_map=[
            {"markerId": 7, "sourceRole": "cube", "offsetM": [0.0, 0.0, 0.0]},
            {"markerId": 10, "sourceRole": "tray", "offsetM": [0.0, 0.0, 0.0]},
        ],
        objects=[
            {"sourceRole": "cube", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
             "isContainer": False, "sizeM": [0.04, 0.04, 0.04]},
            {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
             "isContainer": True, "sizeM": [0.24, 0.18, 0.03]},
        ],
        camera_model="Sony ILCE-7M4",
    )


def _fake_marker(mid, x, y, z, conf=0.95):
    # World pose is stashed in the corner slots so the FakePoseEstimator can decode it
    # (the detector→observation→estimator seam stays real, just without OpenCV).
    return MarkerObservation(mid, [[x, y], [z, conf], [0.0, 0.0], [0.0, 0.0]], conf)


class _FakePoseEstimator(MarkerPoseEstimator):
    def estimate(self, observation, marker_entry, calibration):
        x, y = observation.corners[0]
        z, conf = observation.corners[1]
        ox, oy, oz = marker_entry.get("offsetM", [0.0, 0.0, 0.0])
        return ({"x": x + ox, "y": y + oy, "z": z + oz},
                {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, conf)


def _script(cube_seq, *, drop_cube_frames=(), cube_conf=0.95):
    """Per-frame marker observation lists: cube marker (7) + static tray marker (10)."""
    frames = []
    for i, (cx, cy, cz) in enumerate(cube_seq):
        markers = [_fake_marker(10, TX, TY, TZ, 0.99)]  # tray always visible
        if i not in drop_cube_frames:
            markers.insert(0, _fake_marker(7, cx, cy, cz, cube_conf))
        frames.append(markers)
    return frames


def _build(cube_seq, **kw):
    calib = _calibration()
    return build_tracks(
        list(range(len(cube_seq))),  # opaque "frames" (indices) — FakeDetector ignores content
        detector=FakeDetector(_script(cube_seq, **kw)),
        estimator=_FakePoseEstimator(),
        calibration=calib,
        fps=30.0,
        episode_id="ep_cli",
    )


# ---------------------------------------------------------------------------
# video_to_tracks orchestration (no cv2)
# ---------------------------------------------------------------------------


def test_build_tracks_orchestration_success():
    tracks = _build(_approach_then(_INSIDE))
    assert tracks["schemaVersion"] == "real_camera.tracks.v0"
    assert {o["sourceRole"] for o in tracks["objects"]} == {"cube", "tray"}
    assert all("cube" in f["poses"] and "tray" in f["poses"] for f in tracks["frames"])
    # full pipeline verdict: a clean detected put-in PASSes both targets
    both = ve.verify_episode_both(tracks=tracks)
    assert both["object_inside_container_terminal_only"]["status"] == "PASS"
    assert both["object_inside_container_relation_event"]["status"] == "PASS"


def test_build_tracks_occlusion_surfaces_as_uncertain():
    # Cube marker undetected in 3 of 6 frames -> those frames omit the cube -> dropout.
    tracks = _build(_approach_then(_INSIDE), drop_cube_frames=(1, 2, 4))
    missing = [f for f in tracks["frames"] if "cube" not in f["poses"]]
    assert len(missing) == 3
    both = ve.verify_episode_both(tracks=tracks)
    assert both["object_inside_container_relation_event"]["status"] == "UNCERTAIN"
    assert both["object_inside_container_relation_event"]["passed"] is False


def test_unmapped_markers_are_ignored():
    # A stray board marker id not in the calibration markerMap must not become an object.
    calib = _calibration()
    script = []
    for i, (cx, cy, cz) in enumerate(_approach_then(_INSIDE)):
        script.append([_fake_marker(7, cx, cy, cz), _fake_marker(10, TX, TY, TZ, 0.99),
                       _fake_marker(99, 0.0, 0.0, 0.0)])  # 99 is unmapped
    tracks = build_tracks(list(range(6)), detector=FakeDetector(script),
                          estimator=_FakePoseEstimator(), calibration=calib, fps=30.0, episode_id="ep")
    for f in tracks["frames"]:
        assert set(f["poses"]) <= {"cube", "tray"}


# ---------------------------------------------------------------------------
# CLI smoke (in-process, tmp files) — tracks -> rollout -> verify
# ---------------------------------------------------------------------------


def test_cli_tracks_to_rollout_to_verify(tmp_path):
    tracks = _build(_approach_then(_INSIDE))
    tracks_path = tmp_path / "ep.tracks.json"
    rollout_path = tmp_path / "ep.rollout.json"
    tracks_path.write_text(json.dumps(tracks))

    assert t2r.main(["--tracks", str(tracks_path), "--out", str(rollout_path)]) == 0
    rollout = json.loads(rollout_path.read_text())
    assert rollout["schemaVersion"] == "csg.rollout.v0"
    assert rollout["backend"] == "real_camera_external"
    assert rollout["diagnostics"]["physicalValidity"] is None

    # verify a pre-built rollout against both bundled targets -> exit 0 (both PASS)
    assert ve.main(["--rollout", str(rollout_path)]) == 0
    # verify straight from tracks (runs the UNCERTAIN gate too) -> exit 0
    assert ve.main(["--tracks", str(tracks_path)]) == 0
    # a single explicit target also works
    target = _REPO / "pilots" / "real_camera" / "targets" / "object_inside_container_relation_event.json"
    assert ve.main(["--tracks", str(tracks_path), "--target", str(target)]) == 0


def test_cli_uncertain_episode_exits_nonzero(tmp_path):
    tracks = _build(_approach_then(_INSIDE), drop_cube_frames=(1, 2, 4))
    tracks_path = tmp_path / "occ.tracks.json"
    tracks_path.write_text(json.dumps(tracks))
    assert ve.main(["--tracks", str(tracks_path)]) == 1  # UNCERTAIN -> nonzero


# ---------------------------------------------------------------------------
# pyproject camera extra + csg-frozen guard
# ---------------------------------------------------------------------------


def test_pyproject_declares_camera_extra():
    import tomllib
    data = tomllib.loads((_REPO / "pyproject.toml").read_text())
    extras = data["project"]["optional-dependencies"]
    assert "camera" in extras, "pyproject must declare a 'camera' optional-dependency"
    blob = " ".join(extras["camera"])
    assert "opencv-contrib-python-headless" in blob
    assert "numpy" in blob


def test_calibration_helper_is_valid():
    validate_calibration_v0(_calibration())  # make_calibration produces a schema-valid doc


# ---------------------------------------------------------------------------
# Committed dataset regression — manifest verdicts must hold against the frozen verifier
# ---------------------------------------------------------------------------


def test_committed_dataset_episodes_match_manifest_verdicts():
    from csg.common import load_json
    root = _REPO / "datasets" / "sony_object_inside_container_v0"
    manifest = load_json(root / "manifest.json")
    assert manifest["episodes"], "dataset manifest has no episodes"
    for ep in manifest["episodes"]:
        tracks = load_json(root / ep["tracks"])
        both = ve.verify_episode_both(tracks=tracks)
        actual = {k: v["status"] for k, v in both.items()}
        assert actual == ep["expectedVerdicts"], (ep["episodeId"], actual)
        # the committed rollout must be leakage-clean and physics-unverified
        rollout = load_json(root / ep["rollout"])
        assert rollout["backend"] == "real_camera_external"
        assert rollout["diagnostics"]["physicalValidity"] is None
        assert rollout["objectIdMap"] == {}


# ---------------------------------------------------------------------------
# PnP world-projection math (cv2-free) — camera extrinsic + rotated offset
# ---------------------------------------------------------------------------


def test_compose_marker_world_pose_applies_nonidentity_extrinsic_and_offset():
    from pilots.real_camera.video_to_tracks import compose_marker_world_pose
    # camera->world = +90deg about Z, then translate by (1,2,3)
    Rz90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    cam_to_world = [Rz90[0] + [1.0], Rz90[1] + [2.0], Rz90[2] + [3.0], [0.0, 0.0, 0.0, 1.0]]
    # marker origin 1 m along camera +x, marker axis-aligned with camera, offset along marker +x
    pos, quat, conf = compose_marker_world_pose(
        tvec_cam=[1.0, 0.0, 0.0], R_cam=[[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
        camera_to_world=cam_to_world, offset_marker=[0.1, 0.0, 0.0], confidence=0.9)
    # world origin = Rz90 @ (1,0,0) + (1,2,3) = (0,1,0)+(1,2,3) = (1,3,3); offset (0.1,0,0)
    # rotates by Rz90 to (0,0.1,0) -> final (1, 3.1, 3)
    assert pos["x"] == pytest.approx(1.0)
    assert pos["y"] == pytest.approx(3.1)
    assert pos["z"] == pytest.approx(3.0)
    # rotation is +90deg about z: quat ~ (cos45, 0, 0, sin45)
    assert quat["w"] == pytest.approx(0.70710678, abs=1e-6)
    assert quat["z"] == pytest.approx(0.70710678, abs=1e-6)
    assert conf == pytest.approx(0.9)


def test_compose_marker_world_pose_identity_is_camera_frame():
    from pilots.real_camera.video_to_tracks import compose_marker_world_pose
    ident = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    pos, _, _ = compose_marker_world_pose([0.3, -0.1, 0.5], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
                                          ident, [0.0, 0.0, 0.0], 1.0)
    assert (pos["x"], pos["y"], pos["z"]) == pytest.approx((0.3, -0.1, 0.5))


def test_calibration_validates_camera_to_world_shape():
    from pilots.real_camera.calibrate_table import CalibrationError, validate_calibration_v0
    calib = _calibration()
    assert len(calib["cameraToWorld"]) == 4  # make_calibration includes the extrinsic (identity default)
    calib["cameraToWorld"] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # 3x3 -> invalid
    with pytest.raises(CalibrationError, match="cameraToWorld"):
        validate_calibration_v0(calib)


def test_sha256_file_binds_video_provenance(tmp_path):
    import hashlib
    from pilots.real_camera.video_to_tracks import sha256_file
    p = tmp_path / "clip.bytes"
    p.write_bytes(b"sony-tripod-frames")
    assert sha256_file(p) == hashlib.sha256(b"sony-tripod-frames").hexdigest()


def test_csg_is_byte_frozen():
    try:
        out = subprocess.run(["git", "diff", "--name-only", "--", "csg"],
                             cwd=_REPO, capture_output=True, text=True, timeout=30)
    except (FileNotFoundError, subprocess.SubprocessError):
        pytest.skip("git not available")
    assert out.returncode == 0
    assert out.stdout.strip() == "", f"csg/ must stay byte-frozen, changed: {out.stdout}"


# ---------------------------------------------------------------------------
# Optional: real OpenCV ArUco round-trip (skipped when cv2 absent)
# ---------------------------------------------------------------------------


def test_aruco_detector_roundtrip_smoke():
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    from pilots.real_camera.marker_tracker import ArucoDetector, camera_available
    assert camera_available() is True
    aruco = cv2.aruco
    if hasattr(aruco, "getPredefinedDictionary"):
        ar_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    else:  # pragma: no cover
        ar_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    gen = getattr(aruco, "generateImageMarker", None) or getattr(aruco, "drawMarker")
    marker = gen(ar_dict, 7, 240)
    canvas = np.full((400, 400), 255, dtype=np.uint8)
    canvas[80:320, 80:320] = marker
    bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    obs = ArucoDetector().detect(bgr)
    assert any(o.marker_id == 7 for o in obs), [o.marker_id for o in obs]
