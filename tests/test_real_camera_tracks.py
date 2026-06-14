"""Real-camera tracks parsing + the fail-closed UNCERTAIN gate (Phase 3A).

Covers ``real_camera.tracks.v0`` structural validation (``tracks_to_rollout``) and the
evidence-quality gate (``verify_episode``) that surfaces uncertain tracking as UNCERTAIN
instead of letting it reach the verifier as a fake PASS (roadmap §3A: "uncertain tracking
is surfaced as uncertainty, not hidden"). Runs with NO opencv/numpy; ``csg/`` untouched.
"""
import copy

import pytest

from pilots.real_camera.tracks_to_rollout import TracksError, tracks_to_rollout, validate_tracks_v0
from pilots.real_camera.verify_episode import assess_evidence_quality, verify_episode, verify_episode_both

TX, TY, TZ = 0.30, 0.0, 0.015
_INSIDE = (TX, TY, 0.03)
_ON_RIM = (TX, TY, 0.05)
_NEAR_NOT_INSIDE = (TX + 0.13, TY, 0.05)
_FAR = (TX + 0.35, TY, 0.02)
_START_NEAR = (TX + 0.16, TY, 0.05)


def _approach_then(end_xyz):
    sx, sy, sz = _START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _tracks(cube_seq, *, cube_conf=0.95, tray_conf=0.99, tray_xyz=(TX, TY, TZ)):
    objs = [
        {"sourceRole": "cube", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": False, "sizeM": [0.04, 0.04, 0.04], "markerIds": [7]},
        {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": True, "sizeM": [0.24, 0.18, 0.03], "markerIds": [10, 11, 12, 13]},
    ]
    confs = cube_conf if isinstance(cube_conf, list) else [cube_conf] * len(cube_seq)
    frames = []
    for i, (cx, cy, cz) in enumerate(cube_seq):
        frames.append({"frameIndex": i, "timeS": i * 0.1, "poses": {
            "cube": {"positionM": {"x": cx, "y": cy, "z": cz}, "confidence": confs[i]},
            "tray": {"positionM": {"x": tray_xyz[0], "y": tray_xyz[1], "z": tray_xyz[2]}, "confidence": tray_conf},
        }})
    return {"schemaVersion": "real_camera.tracks.v0", "episodeId": "ep",
            "videoSha256": None, "calibrationHash": None, "fps": 30.0,
            "objects": objs, "frames": frames}


# ---------------------------------------------------------------------------
# Structural validation (fail-closed)
# ---------------------------------------------------------------------------


def test_valid_tracks_pass_validation_and_convert():
    tr = _tracks(_approach_then(_INSIDE))
    validate_tracks_v0(tr)  # no raise
    rollout = tracks_to_rollout(tr)
    assert rollout["schemaVersion"] == "csg.rollout.v0"
    assert rollout["diagnostics"]["physicalValidity"] is None
    assert len(rollout["sceneBodies"]) == 2


@pytest.mark.parametrize("mutate,match_re", [
    (lambda t: t.__setitem__("schemaVersion", "real_camera.tracks.v1"), "schemaVersion"),
    (lambda t: t.__setitem__("episodeId", ""), "episodeId"),
    (lambda t: t.__setitem__("fps", 0), "fps"),
    (lambda t: t.__setitem__("objects", []), "non-empty"),
    (lambda t: t["objects"][0].pop("sizeM"), "missing required"),
    (lambda t: t.__setitem__("frames", t["frames"][:1]), ">= 3"),
    (lambda t: t["frames"][2]["poses"].pop("cube"), "missing declared object"),
    (lambda t: t["frames"].__setitem__(3, {**t["frames"][3], "timeS": -1.0}), "monotonic"),
])
def test_validation_is_fail_closed(mutate, match_re):
    tr = _tracks(_approach_then(_INSIDE))
    mutate(tr)
    with pytest.raises(TracksError, match=match_re):
        validate_tracks_v0(tr)
    with pytest.raises(TracksError):
        tracks_to_rollout(tr)


# ---------------------------------------------------------------------------
# Evidence-quality gate -> UNCERTAIN, never a fake PASS
# ---------------------------------------------------------------------------


def _rele_target():
    from csg.common import load_json
    from pathlib import Path
    return load_json(Path(__file__).resolve().parents[1] / "pilots" / "real_camera" / "targets" /
                     "object_inside_container_relation_event.json")


def test_clean_success_evidence_is_ok():
    q = assess_evidence_quality(_tracks(_approach_then(_INSIDE)))
    assert q["ok"] is True and q["status"] == "OK" and q["failureClass"] is None


def test_terminal_marker_missing_is_perception_failure():
    tr = _tracks(_approach_then(_INSIDE))
    tr["frames"][-1]["poses"].pop("cube")            # cube marker lost on the deciding frame
    q = assess_evidence_quality(tr)
    assert q["ok"] is False and q["failureClass"] == "perception_failure"
    # and verify_episode returns UNCERTAIN (NOT a PASS) even though the geometry would pass
    case = verify_episode(_rele_target(), tracks=tr)
    assert case["status"] == "UNCERTAIN"
    assert case["passed"] is False
    assert case["physicalValidity"] is None


def test_low_terminal_confidence_is_uncertain_not_pass():
    # A geometry that WOULD pass relation-event, but the terminal cube marker is weak.
    confs = [0.95] * 6
    confs[-1] = 0.2
    tr = _tracks(_approach_then(_INSIDE), cube_conf=confs)
    case = verify_episode(_rele_target(), tracks=tr)
    assert case["status"] == "UNCERTAIN"
    assert case["failureClass"] == "extractor_uncertainty"
    assert case["passed"] is False


def test_low_initial_confidence_is_uncertain_not_pass():
    # The relation-event verdict hinges on the INITIAL "started NEAR" evidence as much as the
    # terminal frame. A weak frame-0 marker must be UNCERTAIN, never a PASS (the frozen
    # extractor rewrites pose confidence to 1.0, so this must be caught before rollout minting).
    confs = [0.95] * 6
    confs[0] = 0.2
    tr = _tracks(_approach_then(_INSIDE), cube_conf=confs)
    q = assess_evidence_quality(tr)
    assert q["ok"] is False and q["failureClass"] == "extractor_uncertainty"
    assert any("initial" in r for r in q["reasons"])
    case = verify_episode(_rele_target(), tracks=tr)
    assert case["status"] == "UNCERTAIN" and case["passed"] is False


def test_non_terminal_bad_confidence_fails_closed_not_crash():
    # A single NON-endpose frame with non-numeric confidence passes the dropout gate (it is
    # treated as weak), but must not crash tracks_to_rollout: the tracks contract rejects
    # non-numeric confidence as a TracksError -> verify_episode reports UNCERTAIN.
    tr = _tracks(_approach_then(_INSIDE))
    tr["frames"][2]["poses"]["cube"]["confidence"] = "high"
    with pytest.raises(TracksError, match="confidence must be numeric"):
        tracks_to_rollout(tr)
    case = verify_episode(_rele_target(), tracks=tr)  # must not raise
    assert case["status"] == "UNCERTAIN"
    assert case["passed"] is False


def test_high_dropout_is_extractor_uncertainty():
    # Cube occluded (missing) in 3 of 6 frames -> dropout 0.5 > 0.2, terminal still present.
    tr = _tracks(_approach_then(_INSIDE))
    for i in (1, 2, 4):
        tr["frames"][i]["poses"].pop("cube")
    q = assess_evidence_quality(tr)
    assert q["ok"] is False and q["failureClass"] == "extractor_uncertainty"
    assert verify_episode(_rele_target(), tracks=tr)["status"] == "UNCERTAIN"


def test_overjittery_static_container_is_uncertain():
    tr = _tracks(_approach_then(_INSIDE))
    for i, frame in enumerate(tr["frames"]):
        frame["poses"]["tray"]["positionM"]["x"] = TX + (0.08 if i % 2 else -0.08)  # 0.16 m swing
    q = assess_evidence_quality(tr)
    assert q["ok"] is False and q["failureClass"] == "extractor_uncertainty"
    assert any("jitter" in r for r in q["reasons"])


# ---------------------------------------------------------------------------
# End-to-end via verify_episode_both — the four user scenarios + failure classes
# ---------------------------------------------------------------------------


def test_success_both_targets_pass():
    both = verify_episode_both(tracks=_tracks(_approach_then(_INSIDE)))
    assert both["object_inside_container_terminal_only"]["status"] == "PASS"
    assert both["object_inside_container_relation_event"]["status"] == "PASS"


def test_born_inside_terminal_pass_relation_event_fail_class():
    born = [(TX - 0.03, TY, 0.03), (TX - 0.01, TY, 0.03), (TX + 0.01, TY, 0.03),
            (TX + 0.03, TY, 0.03), (TX + 0.01, TY, 0.03), (TX - 0.01, TY, 0.03)]
    both = verify_episode_both(tracks=_tracks(born))
    assert both["object_inside_container_terminal_only"]["status"] == "PASS"
    rele = both["object_inside_container_relation_event"]
    assert rele["status"] == "FAIL"
    assert rele["cameraFailureClass"] == "BORN_INSIDE_NO_TRANSITION"


@pytest.mark.parametrize("end_xyz,expected_class", [
    (_NEAR_NOT_INSIDE, "NEAR_NOT_INSIDE"),
    (_ON_RIM, "LEFT_ON_RIM"),
    (_FAR, "DROPPED_OUTSIDE"),
])
def test_failure_classes_from_terminal_relation(end_xyz, expected_class):
    both = verify_episode_both(tracks=_tracks(_approach_then(end_xyz)))
    term = both["object_inside_container_terminal_only"]
    rele = both["object_inside_container_relation_event"]
    assert term["status"] == "FAIL" and rele["status"] == "FAIL"
    assert rele["cameraFailureClass"] == expected_class


# ---------------------------------------------------------------------------
# Robustness — malformed input fails closed (UNCERTAIN), never an uncaught crash
# ---------------------------------------------------------------------------


def test_malformed_orientation_is_tracks_error_then_uncertain():
    tr = _tracks(_approach_then(_INSIDE))
    tr["frames"][2]["poses"]["cube"]["orientationXyzw"] = [0.0, 0.0]  # not 4 numbers
    with pytest.raises(TracksError, match="orientationXyzw"):
        tracks_to_rollout(tr)
    case = verify_episode(_rele_target(), tracks=tr)  # must not crash
    assert case["status"] == "UNCERTAIN"
    assert case["failureClass"] == "perception_failure"


def test_non_numeric_confidence_does_not_crash_the_gate():
    tr = _tracks(_approach_then(_INSIDE))
    tr["frames"][-1]["poses"]["cube"]["confidence"] = "high"  # non-numeric on the deciding frame
    q = assess_evidence_quality(tr)  # docstring promises: never raises
    assert q["ok"] is False and q["failureClass"] == "extractor_uncertainty"
    assert verify_episode(_rele_target(), tracks=tr)["status"] == "UNCERTAIN"


def test_leaky_prebuilt_rollout_fails_closed_in_verify_episode():
    rollout = tracks_to_rollout(_tracks(_approach_then(_INSIDE)))
    rollout["targetCsg"] = {"leaked": True}  # smuggled target authoring
    case = verify_episode(_rele_target(), rollout=rollout)  # must report, not raise
    assert case["status"] == "UNCERTAIN"
    assert case["failureClass"] == "leakage_violation"
    assert case["passed"] is False


def test_verify_episode_accepts_a_prebuilt_rollout():
    # When handed an already-minted rollout, verify_episode skips the UNCERTAIN gate.
    rollout = tracks_to_rollout(_tracks(_approach_then(_INSIDE)))
    case = verify_episode(_rele_target(), rollout=rollout)
    assert case["status"] == "PASS"
    assert "trackingMetrics" not in case  # no tracks -> no quality metrics
    with pytest.raises(ValueError, match="exactly one"):
        verify_episode(_rele_target(), tracks=_tracks(_approach_then(_INSIDE)), rollout=rollout)
