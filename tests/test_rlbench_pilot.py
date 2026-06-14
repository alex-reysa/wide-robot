"""RLBench external-trace pilot — seam, leakage discipline, and open_drawer ingest.

These run with NO RLBench installed: they exercise the rollout-assembly contract, the
hardened leakage gate, the real ``open_drawer`` converter (driven by *fake* observations
and neutral measurements), the cross-task confusion report, and the external-entry path
through the FROZEN csg verifier, using the committed synthetic ``csg.rollout.v0`` fixture
as the canonical stand-in. The point is to lock the leakage discipline for external
traces, and to prove the converter produces a leakage-clean trace that PASSes its own
task and FAILs the others, before any live RLBench ingest.

Live RLBench recording is covered by the skipped tests at the bottom (gated on RLBench
being importable or ``RLBENCH_PILOT_LIVE=1``).
"""
import copy
import importlib.util
import os
from pathlib import Path

import pytest

from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg
from csg.benchmark import leakage_report

from pilots.rlbench import adapter
from pilots.rlbench.adapter import (
    ExternalTraceLeakage,
    assemble_rollout,
    assert_rollout_leakage_clean,
    rlbench_demo_to_rollout,
    _xyzw_to_wxyz,
)
from pilots.rlbench.run_external import (
    external_confusion_report,
    load_gold_targets,
    verify_external_rollout,
)

_REPO = Path(__file__).resolve().parents[1]
_FIXTURE = _REPO / "pilots" / "rlbench" / "fixtures" / "synthetic_open_drawer.rollout.json"
_TARGET = _REPO / "gold_tests" / "open_drawer" / "target.json"
_GOLD_DIR = _REPO / "gold_tests"

# Committed live evidence (Runpod, 2026-06-14) + the value-only diagnostic target.
_VALUE_ONLY_TARGET = _REPO / "pilots" / "rlbench" / "targets" / "open_drawer_rlbench_value_only.json"
_LIVE_FIXTURE_DIR = _REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260614"
_LIVE_VARIATIONS = ("bottom", "middle", "top")


def _live_rollout(variation):
    return load_json(_LIVE_FIXTURE_DIR / f"open_drawer_{variation}_demo00.rollout.json")


# ---------------------------------------------------------------------------
# The seam: an external-shaped rollout flows through the frozen verifier
# ---------------------------------------------------------------------------


def test_synthetic_external_rollout_passes_frozen_verifier():
    # The committed open_drawer external-shaped trace must PASS the frozen verifier,
    # leakage-clean, with physicalValidity reported null (physics-unverified, honest).
    target = load_json(_TARGET)
    rollout = load_json(_FIXTURE)
    assert rollout["schemaVersion"] == "csg.rollout.v0"
    assert rollout["backend"] == "rlbench_external"
    case = verify_external_rollout(target, rollout, case_name="open_drawer")
    assert case["passed"] is True, case["hardMismatches"]
    assert case["matcherPassed"] is True
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None  # external sim: not re-checkable → physics-unverified


def test_committed_fixture_is_leakage_clean():
    # The committed fixture itself must satisfy the hardened gate (empty objectIdMap,
    # neutral nested articulation id, neutral frame keys) — it is the canonical example
    # of a clean external trace.
    rollout = load_json(_FIXTURE)
    assert_rollout_leakage_clean(rollout)
    assert rollout["objectIdMap"] == {}


def test_external_path_uses_the_same_frozen_functions():
    # verify_external_rollout must give the same matcher/leakage verdict as calling
    # the frozen csg functions directly — it adds no logic the verifier doesn't have.
    target = load_json(_TARGET)
    rollout = load_json(_FIXTURE)
    robot = extract_robot_csg(rollout)
    direct = match(target, robot, MatcherConfig())
    direct_leak = leakage_report(robot)
    case = verify_external_rollout(target, rollout)
    assert case["matcherPassed"] == direct.passed
    assert case["leakageClean"] == direct_leak["clean"]


# ---------------------------------------------------------------------------
# Leakage discipline is ENFORCED on external traces (the whole pilot question)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("forbidden_key", ["targetCsg", "plannerView", "solverMetadata"])
def test_injected_target_authoring_trips_the_leakage_gate(forbidden_key):
    # A cheating/buggy adapter that smuggles target-authored info into the rollout
    # must be caught at the door, BEFORE the matcher runs — extract_robot_csg never
    # sees it as a pass.
    rollout = load_json(_FIXTURE)
    rollout[forbidden_key] = {"leaked": "the target should never be readable here"}
    with pytest.raises(ExternalTraceLeakage, match="forbidden"):
        assert_rollout_leakage_clean(rollout)
    target = load_json(_TARGET)
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(target, rollout)


def test_non_whitelisted_body_field_is_rejected():
    # A body field that could encode target identity (RLBench category label) is not
    # in ROLLOUT_BODY_FIELDS and must be rejected.
    rollout = load_json(_FIXTURE)
    rollout["sceneBodies"][0]["categoryLabel"] = "drawer"  # RLBench authoring → leakage
    with pytest.raises(ExternalTraceLeakage, match="non-whitelisted"):
        assert_rollout_leakage_clean(rollout)


def test_non_neutral_body_id_is_rejected():
    rollout = load_json(_FIXTURE)
    rollout["sceneBodies"][0]["objectId"] = "drawer_frame"  # target identity, not body_NNN
    with pytest.raises(ExternalTraceLeakage, match="neutral"):
        assert_rollout_leakage_clean(rollout)


# --- the hardened (nested) leakage checks the pilot adds ---


def test_objectid_map_with_target_id_is_rejected():
    # objectIdMap target id → robot id is solver bookkeeping; an external trace carrying
    # a target/RLBench identity key (or value) must be rejected (emit it empty).
    rollout = load_json(_FIXTURE)
    rollout["objectIdMap"] = {"h_drawer": "body_000"}  # non-neutral KEY
    with pytest.raises(ExternalTraceLeakage, match="objectIdMap"):
        assert_rollout_leakage_clean(rollout)
    rollout["objectIdMap"] = {"body_000": "drawer_frame"}  # non-neutral VALUE
    with pytest.raises(ExternalTraceLeakage, match="objectIdMap"):
        assert_rollout_leakage_clean(rollout)


def test_nested_articulated_object_id_is_rejected():
    # The body whitelist passes `articulation` through as a block; its nested
    # articulatedObjectId must still be neutral (it can smuggle a joint/target name).
    rollout = load_json(_FIXTURE)
    rollout["sceneBodies"][0]["articulation"]["articulatedObjectId"] = "drawer_joint_top"
    with pytest.raises(ExternalTraceLeakage, match="articulatedObjectId"):
        assert_rollout_leakage_clean(rollout)


def test_non_neutral_object_pose_key_is_rejected():
    rollout = load_json(_FIXTURE)
    frame = rollout["frames"][0]
    frame["objectPoses"] = {"h_drawer": next(iter(frame["objectPoses"].values()))}
    with pytest.raises(ExternalTraceLeakage, match="objectPoses"):
        assert_rollout_leakage_clean(rollout)


def test_non_neutral_articulation_key_is_rejected():
    rollout = load_json(_FIXTURE)
    rollout["frames"][0]["articulation"] = {"drawer_joint_top": 0.02}
    with pytest.raises(ExternalTraceLeakage, match="articulation"):
        assert_rollout_leakage_clean(rollout)


# --- alias-drift: the gate must read fields the SAME way the frozen extractor does,
#     so a snake_case spelling the extractor accepts cannot bypass the gate ---


def test_snake_case_scene_bodies_does_not_bypass_the_gate():
    # csg.rollout_extract reads get_any(rollout, "sceneBodies", "scene_bodies"); a body
    # under the snake spelling, carrying an authored id + label, must still be rejected
    # — not silently skipped because the gate looked only at the camelCase key.
    rollout = load_json(_FIXTURE)
    body = rollout.pop("sceneBodies")[0]
    body["objectId"] = "h_drawer"          # authored target id
    body["categoryLabel"] = "drawer"        # authored label (non-whitelisted)
    rollout["scene_bodies"] = [body]
    with pytest.raises(ExternalTraceLeakage):
        assert_rollout_leakage_clean(rollout)


def test_snake_case_object_poses_does_not_bypass_the_gate():
    # Extractor reads get_any(frame, "objectPoses", "object_poses"); a non-neutral key
    # under the snake spelling must be rejected too.
    rollout = load_json(_FIXTURE)
    frame = rollout["frames"][0]
    pose = next(iter(frame.pop("objectPoses").values()))
    frame["object_poses"] = {"h_drawer": pose}
    with pytest.raises(ExternalTraceLeakage, match="objectPoses"):
        assert_rollout_leakage_clean(rollout)


@pytest.mark.parametrize("mutate", [
    lambda r: r.__setitem__("objectIdMap", ["h_drawer"]),                       # list, not mapping
    lambda r: r["frames"][0].__setitem__("objectPoses", ["h_drawer"]),          # list, not mapping
    lambda r: r["sceneBodies"][0].__setitem__("articulation", "drawer_joint_top"),  # string, not mapping
])
def test_malformed_non_mapping_identity_carriers_are_rejected(mutate):
    # The gate is fail-closed: a present-but-malformed (non-mapping) identity carrier is
    # rejected, not silently skipped — even though today's frozen extractor would ignore
    # it, the contract refuses to let a malformed/adversarial trace through the door.
    rollout = load_json(_FIXTURE)
    mutate(rollout)
    with pytest.raises(ExternalTraceLeakage, match="must be an object"):
        assert_rollout_leakage_clean(rollout)


# ---------------------------------------------------------------------------
# assemble_rollout: the real, tested half of the adapter
# ---------------------------------------------------------------------------


def test_assemble_rollout_sanitizes_bodies_and_reports_unverified_physics():
    # Bodies carrying authoring fields are stripped by the whitelist; the assembled
    # rollout reports physicalValidity null and is leakage-clean.
    bodies = [{
        "objectId": "body_000", "physicalKind": "ARTICULATED", "sizeM": [0.4, 0.3, 0.2],
        "mobility": "ARTICULATED",
        "categoryLabel": "drawer", "sourceObjectId": "drawer_top", "parts": ["handle"],  # must be stripped
    }]
    frames = [{
        "timeS": 0.0, "phase": "external", "gripperClosed": False,
        "effectorPose": {"frameId": "world", "positionM": {"x": 0, "y": 0, "z": 0.3},
                          "orientationWxyz": {"w": 1, "x": 0, "y": 0, "z": 0}},
        "objectPoses": {"body_000": {"positionM": {"x": 0.5, "y": 0, "z": 0.1}}},
    }]
    rollout = assemble_rollout(bodies=bodies, frames=frames)
    assert rollout["schemaVersion"] == "csg.rollout.v0"
    assert rollout["diagnostics"]["physicalValidity"] is None
    body = rollout["sceneBodies"][0]
    assert "categoryLabel" not in body and "sourceObjectId" not in body and "parts" not in body
    assert_rollout_leakage_clean(rollout)  # must not raise


def test_assemble_rollout_refuses_physical_validity_true():
    bodies = [{"objectId": "body_000", "physicalKind": "RIGID", "sizeM": [0.1, 0.1, 0.1]}]
    frames = [{"timeS": 0.0, "gripperClosed": False,
               "effectorPose": {"positionM": {"x": 0, "y": 0, "z": 0}},
               "objectPoses": {}}]
    with pytest.raises(ExternalTraceLeakage, match="physicalValidity"):
        assemble_rollout(bodies=bodies, frames=frames, extra_diagnostics={"physicalValidity": True})


def test_assemble_rollout_requires_frame_keys():
    bodies = [{"objectId": "body_000", "physicalKind": "RIGID", "sizeM": [0.1, 0.1, 0.1]}]
    with pytest.raises(ValueError, match="missing required keys"):
        assemble_rollout(bodies=bodies, frames=[{"timeS": 0.0}])


def test_assemble_rollout_allows_physical_validity_false_which_self_fails():
    # The contract is asymmetric (csg/validity.md): an external trace may NOT claim
    # physicalValidity:true, but MAY honestly report false — which the verifier then
    # FAILs (validity is not False). Documenting the asymmetry so it can't drift.
    bodies = [{"objectId": "body_000", "physicalKind": "RIGID", "sizeM": [0.1, 0.1, 0.1]}]
    frames = [{"timeS": 0.0, "gripperClosed": False,
               "effectorPose": {"positionM": {"x": 0, "y": 0, "z": 0}}, "objectPoses": {}}]
    rollout = assemble_rollout(bodies=bodies, frames=frames, extra_diagnostics={"physicalValidity": False})
    assert rollout["diagnostics"]["physicalValidity"] is False


# ---------------------------------------------------------------------------
# rlbench_demo_to_rollout: the real open_drawer ingest (driven by fakes, no RLBench)
# ---------------------------------------------------------------------------


def _fixture_demo_and_measurements(size=None, ramp_to=None):
    """Back-convert the committed clean fixture into the (fake obs, neutral measurement)
    pair the converter consumes. Reproducing the fixture's trajectory guarantees the
    converted rollout is behaviourally equivalent, so it must PASS just like the
    fixture. ``size`` overrides the drawer's bounding box (per-variation geometry);
    ``ramp_to`` remaps the articulation so each variation opens to a genuinely distinct
    final extension (the fixture opens to 0.18) — kept within the target's tolerance so
    the trajectory differs while the verdict stays PASS."""
    fixture = load_json(_FIXTURE)
    body_size = list(size or fixture["sceneBodies"][0]["sizeM"])
    arts = [f["articulation"]["body_000"] for f in fixture["frames"]]
    lo, hi = min(arts), max(arts)

    def _remap(v):
        if ramp_to is None or hi == lo:
            return v
        return lo + (v - lo) / (hi - lo) * (ramp_to - lo)

    demo, measurements = [], []
    for i, frame in enumerate(fixture["frames"]):
        pos = frame["effectorPose"]["positionM"]
        wxyz = frame["effectorPose"]["orientationWxyz"]
        # CSG WXYZ → RLBench XYZW gripper_pose [x, y, z, qx, qy, qz, qw].
        demo.append({
            "gripper_pose": [pos["x"], pos["y"], pos["z"], wxyz["x"], wxyz["y"], wxyz["z"], wxyz["w"]],
            "gripper_open": 0.0 if frame["gripperClosed"] else 1.0,
        })
        measurements.append({
            "frameIndex": i,
            "timeS": frame["timeS"],
            "bodyPose": frame["objectPoses"]["body_000"],
            "articulationValue": _remap(frame["articulation"]["body_000"]),
            "bodySizeM": list(body_size),
            "sizeApproximate": False,
        })
    return demo, measurements


def test_xyzw_to_wxyz_reorders_quaternion():
    assert _xyzw_to_wxyz([0.1, 0.2, 0.3, 0.9]) == {"w": 0.9, "x": 0.1, "y": 0.2, "z": 0.3}


def test_gripper_open_below_half_is_closed():
    # The contract is strict `< 0.5`: exactly 0.5 is OPEN (locks the tie-point so a later
    # `<`→`<=` refactor is caught). RLBench Discrete gripper can sit at 0.5 on transitions.
    demo = [
        {"gripper_pose": [0, 0, 0.3, 0, 0, 0, 1], "gripper_open": 0.9},   # open
        {"gripper_pose": [0, 0, 0.3, 0, 0, 0, 1], "gripper_open": 0.5},   # OPEN (boundary)
        {"gripper_pose": [0, 0, 0.3, 0, 0, 0, 1], "gripper_open": 0.49},  # closed
        {"gripper_pose": [0, 0, 0.3, 0, 0, 0, 1], "gripper_open": 0.0},   # closed
    ]
    meas = [{"bodyPose": {"positionM": {"x": 0, "y": 0, "z": 0}}, "articulationValue": 0.02,
             "bodySizeM": [0.1, 0.1, 0.1], "sizeApproximate": False} for _ in demo]
    rollout = rlbench_demo_to_rollout(demo, task="open_drawer", measurements=meas)
    assert [f["gripperClosed"] for f in rollout["frames"]] == [False, False, True, True]


def test_converter_effector_quaternion_is_wxyz():
    demo = [{"gripper_pose": [0.4, -0.1, 0.3, 0.1, 0.2, 0.3, 0.9], "gripper_open": 1.0}]
    meas = [{"bodyPose": {"positionM": {"x": 0.2, "y": -0.1, "z": 0.03}}, "articulationValue": 0.02,
             "bodySizeM": [0.4, 0.3, 0.15], "sizeApproximate": False}]
    rollout = rlbench_demo_to_rollout(demo, task="open_drawer", measurements=meas)
    orient = rollout["frames"][0]["effectorPose"]["orientationWxyz"]
    assert orient == {"w": 0.9, "x": 0.1, "y": 0.2, "z": 0.3}


@pytest.mark.parametrize("variation,size,ramp_to", [
    # Each variation differs both in declared geometry AND in the kinematic open
    # extension (0.16 / 0.18 / 0.20), all within the open_drawer target's tolerance.
    ("bottom", [0.40, 0.30, 0.10], 0.16),
    ("middle", [0.40, 0.30, 0.15], 0.18),
    ("top", [0.40, 0.30, 0.20], 0.20),
])
def test_converter_open_drawer_passes_for_all_variations(variation, size, ramp_to):
    # All three official variations: a converted rollout must PASS the frozen verifier,
    # be leakage-clean, report physics-unverified, and PASS the confusion (own task only).
    demo, measurements = _fixture_demo_and_measurements(size=size, ramp_to=ramp_to)
    rollout = rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements)
    assert rollout["backend"] == "rlbench_external"
    assert rollout["objectIdMap"] == {}
    # The declared per-variation geometry must actually survive into the rollout body
    # (so the parametrization is observed, not cosmetic), and the open extension must
    # match the requested kinematics.
    assert rollout["sceneBodies"][0]["sizeM"] == size
    assert rollout["frames"][-1]["articulation"]["body_000"] == pytest.approx(ramp_to)
    assert_rollout_leakage_clean(rollout)

    target = load_json(_TARGET)
    case = verify_external_rollout(target, rollout, case_name="open_drawer")
    assert case["passed"] is True, (variation, case["hardMismatches"])
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None

    confusion = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    assert confusion["confusionClean"] is True, (variation, confusion)
    assert confusion["passes"] == ["open_drawer"]


def test_converter_requires_measurements():
    demo, _ = _fixture_demo_and_measurements()
    with pytest.raises(ValueError, match="measurements"):
        rlbench_demo_to_rollout(demo, task="open_drawer")


def test_converter_rejects_length_mismatch():
    demo, measurements = _fixture_demo_and_measurements()
    with pytest.raises(ValueError, match="observations"):
        rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements[:-1])


def test_converter_rejects_unsupported_task():
    demo, measurements = _fixture_demo_and_measurements()
    with pytest.raises(NotImplementedError, match="open_drawer"):
        rlbench_demo_to_rollout(demo, task="push_object", measurements=measurements)


def test_converter_rejects_non_neutral_measurement_field():
    # A measurement that carries an RLBench label/handle/id (anything outside the
    # neutral shape) must trip the leakage guard inside the converter.
    demo, measurements = _fixture_demo_and_measurements()
    measurements[0] = {**measurements[0], "categoryLabel": "drawer"}
    with pytest.raises(ExternalTraceLeakage, match="non-neutral"):
        rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements)


def test_converter_rejects_short_body_size():
    demo, measurements = _fixture_demo_and_measurements()
    measurements[0] = {**measurements[0], "bodySizeM": [0.4, 0.3]}  # missing z
    with pytest.raises(ValueError, match="bodySizeM"):
        rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements)


def test_converter_rejects_non_positive_control_rate():
    demo, measurements = _fixture_demo_and_measurements()
    with pytest.raises(ValueError, match="control_rate_hz"):
        rlbench_demo_to_rollout(demo, task="open_drawer", control_rate_hz=0.0, measurements=measurements)


def test_converter_rejects_empty_demo():
    with pytest.raises(ValueError, match="empty demo"):
        rlbench_demo_to_rollout([], task="open_drawer", measurements=[])


def test_converter_rejects_observation_missing_gripper_pose():
    demo, measurements = _fixture_demo_and_measurements()
    demo[0] = {"gripper_open": 1.0}  # no gripper_pose
    with pytest.raises(ValueError, match="gripper_pose"):
        rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements)


def test_converter_rejects_observation_missing_gripper_open():
    demo, measurements = _fixture_demo_and_measurements()
    demo[0] = {"gripper_pose": [0, 0, 0.3, 0, 0, 0, 1]}  # no gripper_open
    with pytest.raises(ValueError, match="gripper_open"):
        rlbench_demo_to_rollout(demo, task="open_drawer", measurements=measurements)


# ---------------------------------------------------------------------------
# Cross-task confusion: an open_drawer trace must PASS open_drawer, FAIL the rest
# ---------------------------------------------------------------------------


def test_confusion_open_drawer_passes_only_its_own_target():
    rollout = load_json(_FIXTURE)
    confusion = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    assert confusion["results"]["open_drawer"] is True
    assert confusion["passes"] == ["open_drawer"]
    assert confusion["unexpectedOffTaskPasses"] == []
    assert confusion["missedExpected"] is None
    assert confusion["confusionClean"] is True
    # Every other gold task must FAIL the open_drawer trace.
    for name, passed in confusion["results"].items():
        if name != "open_drawer":
            assert passed is False, name


def test_confusion_flags_a_missed_diagonal():
    # Matching a non-equivalent target as the expected case fails the confusion: the
    # open_drawer trace does not PASS push_object, so the expected diagonal is missed.
    rollout = load_json(_FIXTURE)
    confusion = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="push_object")
    assert confusion["confusionClean"] is False
    assert confusion["missedExpected"] == "push_object"


def test_confusion_flags_an_unexpected_off_task_pass():
    # The detector's load-bearing mode: a too-easy PASS on a task the trace should not
    # match (the pilot's single biggest risk) must be FLAGGED, not silently clean. With
    # expected_case=push_object, the open_drawer trace PASSing open_drawer is off-task.
    rollout = load_json(_FIXTURE)
    confusion = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="push_object")
    assert confusion["unexpectedOffTaskPasses"] == ["open_drawer"]
    assert confusion["confusionClean"] is False


def test_confusion_runs_only_on_a_leakage_clean_rollout():
    rollout = load_json(_FIXTURE)
    rollout["targetCsg"] = {"leaked": True}
    with pytest.raises(ExternalTraceLeakage):
        external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")


# ---------------------------------------------------------------------------
# The optional dependency stays optional: imports work with no RLBench/PyRep
# ---------------------------------------------------------------------------


def test_adapter_imports_without_rlbench_installed():
    # The module must import on a machine with no RLBench/PyRep — the dependency is
    # optional and only touched inside the (lazy) recorder.
    import sys
    assert "rlbench" not in {m.split(".")[0] for m in list(sys.modules) if m == "rlbench"}
    assert adapter.RLBENCH_FIELD_MAPPING  # mapping contract is documented in-code


def test_recorder_imports_without_rlbench_installed():
    import pilots.rlbench.record_open_drawer as rec
    assert rec.rlbench_available() == (importlib.util.find_spec("rlbench") is not None
                                       and importlib.util.find_spec("pyrep") is not None)
    assert set(rec.DEFAULT_VARIATIONS) == {"bottom", "middle", "top"}
    # The quarantined handle names must NEVER be referenced from the rollout-producing
    # converter; they live only in the recorder.
    assert "drawer_joint_top" not in adapter.RLBENCH_FIELD_MAPPING.values()


# --- recorder neutralisation + joint-slot resolution, driven by fakes (no RLBench) ---


class _FakeShape:
    """Stands in for a quarantined PyRep Shape handle (drawer_frame)."""
    def __init__(self, pose, bbox):
        self._pose, self._bbox = pose, bbox

    def get_pose(self):
        return list(self._pose)  # [x, y, z, qx, qy, qz, qw]

    def get_bounding_box(self):
        return list(self._bbox)  # [x0, x1, y0, y1, z0, z1]


class _FakeJoint:
    def __init__(self, position):
        self._position = position

    def get_joint_position(self):
        return self._position


class _FakeObs:
    def __init__(self, low_dim):
        self.task_low_dim_state = low_dim


def _fake_open_drawer_demo(open_ext=0.18, n=6, extra_open_slot=False):
    # low-dim state per frame: [px, py, pz, quat_z(0), quat_w-ish(0), <active joint>],
    # the active drawer joint ramps 0 → open_ext; the leading slots are static (a
    # closed-drawer tie at frame 0, exactly the ambiguity the resolver must avoid).
    demo = []
    for i in range(n):
        j = open_ext * i / (n - 1)
        state = [0.25, -0.10, 0.80, 0.0, 0.0, j]
        if extra_open_slot:
            state.append(open_ext)  # a second slot equal to the open value → ambiguous
        demo.append(_FakeObs(state))
    return demo


def test_recorder_measurements_are_neutral_and_resolve_the_ramping_joint():
    import pilots.rlbench.record_open_drawer as rec

    demo = _fake_open_drawer_demo(open_ext=0.18, n=6)
    frame = _FakeShape(pose=[0.22, -0.12, 0.03, 0, 0, 0, 1],
                       bbox=[-0.20, 0.20, -0.15, 0.15, -0.075, 0.075])
    joint = _FakeJoint(0.18)
    measurements = rec._demo_to_measurements(demo, frame_obj=frame, joint_obj=joint)

    # No handle name / label / id leaks: every measurement key is in the neutral shape.
    for m in measurements:
        assert set(m) <= set(adapter.NEUTRAL_MEASUREMENT_FIELDS)
    # The active (ramping) slot was resolved — not a static frame-0 zero.
    assert measurements[0]["articulationValue"] == pytest.approx(0.0)
    assert measurements[-1]["articulationValue"] == pytest.approx(0.18)
    # Body size is the bbox full extents; pose is the (neutral) frame pose.
    assert measurements[0]["bodySizeM"] == pytest.approx([0.40, 0.30, 0.15])
    assert measurements[0]["bodyPose"]["positionM"] == {"x": 0.22, "y": -0.12, "z": 0.03}


def test_recorder_accepts_numpy_low_dim_state_from_live_rlbench():
    import numpy as np
    import pilots.rlbench.record_open_drawer as rec

    demo = _fake_open_drawer_demo(open_ext=0.18, n=6)
    for obs in demo:
        obs.task_low_dim_state = np.asarray(obs.task_low_dim_state, dtype=float)
    frame = _FakeShape(pose=[0.22, -0.12, 0.03, 0, 0, 0, 1],
                       bbox=[-0.20, 0.20, -0.15, 0.15, -0.075, 0.075])
    joint = _FakeJoint(0.18)

    measurements = rec._demo_to_measurements(demo, frame_obj=frame, joint_obj=joint)

    assert measurements[0]["articulationValue"] == pytest.approx(0.0)
    assert measurements[-1]["articulationValue"] == pytest.approx(0.18)


def test_recorder_raises_on_ambiguous_joint_slot():
    import pilots.rlbench.record_open_drawer as rec

    demo = _fake_open_drawer_demo(open_ext=0.18, n=6, extra_open_slot=True)
    frame = _FakeShape(pose=[0.22, -0.12, 0.03, 0, 0, 0, 1],
                       bbox=[-0.20, 0.20, -0.15, 0.15, -0.075, 0.075])
    joint = _FakeJoint(0.18)
    with pytest.raises(RuntimeError, match="uniquely resolve"):
        rec._demo_to_measurements(demo, frame_obj=frame, joint_obj=joint)


# ---------------------------------------------------------------------------
# Committed live RLBench evidence (Runpod 2026-06-14): the negative gold result
# (A) and the value-only diagnostic positive (B), reproducible with NO RLBench.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_live_rlbench_fixture_is_committed_and_leakage_clean(variation):
    # The promoted live rollouts are the real evidence both results rest on. They must
    # be present (a clean clone can reproduce A/B without Runpod), be leakage-clean, and
    # honestly report physicalValidity null (external kinematic trace: physics-unverified).
    rollout = _live_rollout(variation)  # raises if the fixture was not promoted
    assert rollout["schemaVersion"] == "csg.rollout.v0"
    assert rollout["backend"] == "rlbench_external"
    assert rollout["objectIdMap"] == {}
    assert_rollout_leakage_clean(rollout)
    assert rollout["diagnostics"]["physicalValidity"] is None


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_gold_open_drawer_target_fails_live_rlbench_negative_result(variation):
    # Result A — the committed NEGATIVE result. Real RLBench OpenDrawer demos are
    # leakage-clean, but the current gold target does NOT accept them: the drawer opens
    # past the gold goal+tolerance (goal_satisfaction) and the gold's human-style
    # CONTACT_BEGIN→ARTICULATION_CHANGE order is absent (event_order). This staying FAIL
    # is load-bearing — an accidental PASS would mean the seam silently drifted.
    target = load_json(_TARGET)
    rollout = _live_rollout(variation)
    case = verify_external_rollout(target, rollout, case_name="open_drawer")
    assert case["passed"] is False
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    assert set(case["hardMismatches"]) == {"event_order", "goal_satisfaction"}


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_value_only_target_passes_live_rlbench(variation):
    # Result B — the value-only diagnostic PASSES the same real traces: a leakage-clean
    # RLBench OpenDrawer demo is accepted once the target asks only "did the drawer reach
    # the RLBench-calibrated extension?". Same frozen verifier, same rollout; only the
    # target's asserted semantics shrank to terminal articulation value.
    target = load_json(_VALUE_ONLY_TARGET)
    rollout = _live_rollout(variation)
    case = verify_external_rollout(target, rollout, case_name="open_drawer_rlbench_value_only")
    assert case["passed"] is True, (variation, case["hardMismatches"])
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    assert case["hardMismatches"] == []


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_value_only_pass_rests_on_terminal_value_not_vacuity(variation):
    # The value-only PASS must be a real terminal-value match, not a vacuous accept-all.
    # goal_satisfaction carries genuine target support and agrees; the deliberately
    # deferred probes (contact/event presence + order) carry ZERO support, so the PASS
    # asserts nothing about — and cannot be propped up by — contact or ordering.
    target = load_json(_VALUE_ONLY_TARGET)
    robot = extract_robot_csg(_live_rollout(variation))
    res = match(target, robot, MatcherConfig())
    assert res.passed is True
    assert res.vacuous is False
    assert res.probe_agreement["goal_satisfaction"] is True
    assert res.probe_support["goal_satisfaction"] == 1          # the single HARD articulation goal
    assert res.object_mapping == {"h_drawer": "body_000"}       # drawer stayed relevant and mapped
    for deferred in ("event_presence", "event_order", "articulation_transitions"):
        assert res.probe_support[deferred] == 0, deferred


def test_value_only_target_is_rlbench_calibrated_not_a_tautology():
    # The value-only goal (0.234 m) is RLBench-calibrated, not "any extension": it must
    # REJECT the 0.18 m synthetic fixture (which the gold 0.18 target accepts). This
    # guards against the target being loosened into something every drawer trace passes.
    vo = load_json(_VALUE_ONLY_TARGET)
    gold = load_json(_TARGET)
    synthetic = load_json(_FIXTURE)  # opens to 0.18, not 0.234
    assert synthetic["frames"][-1]["articulation"]["body_000"] == 0.18  # the value the margin rests on
    robot = extract_robot_csg(synthetic)
    assert match(vo, robot, MatcherConfig()).passed is False
    assert match(gold, robot, MatcherConfig()).passed is True
    # The rejection margin (|0.234 - 0.18| = 0.054 m) only just clears the enforced
    # window; pin the tolerance so a future widening to >= 0.055 m fails THIS guard at its
    # own site (where it would otherwise silently turn vacuous: accept-all), not only the
    # separate metadata test.
    assert MatcherConfig().articulation_tol == 0.05


def test_value_only_target_omits_event_and_contact_sections():
    # Structural guarantee of the diagnostic: the deferred sections are absent from the
    # file, and the only thing it asserts is one HARD articulation goal at the RLBench
    # value. If a later edit reintroduces events/contacts, this is the tripwire.
    vo = load_json(_VALUE_ONLY_TARGET)
    for deferred in ("events", "contacts", "temporalEdges", "objectStates"):
        assert deferred not in vo, deferred
    goals = vo["plannerView"]["stages"][0]["goalConstraints"]
    assert [g["kind"] for g in goals] == ["ARTICULATION_GOAL"]
    assert goals[0]["hard"] is True
    assert goals[0]["articulation"]["targetJointValue"] == 0.234


def test_value_only_tolerance_metadata_is_documentary_not_enforced():
    # pilotMetadata.articulationToleranceM documents INTENT only; the frozen matcher
    # ignores per-target metadata and uses the global MatcherConfig.articulation_tol.
    # Prove it executably: tightening the metadata to an absurd 0.0001 m does NOT change
    # the verdict (still PASS), because the field is never read. If a future change
    # started honoring per-target tolerance, this test flips and forces a doc update.
    vo = load_json(_VALUE_ONLY_TARGET)
    assert vo["pilotMetadata"]["articulationToleranceM"] == 0.03
    tightened = copy.deepcopy(vo)
    tightened["pilotMetadata"]["articulationToleranceM"] = 0.0001
    robot = extract_robot_csg(_live_rollout("bottom"))
    assert match(tightened, robot, MatcherConfig()).passed is True
    assert MatcherConfig().articulation_tol == 0.05  # the tolerance that is actually enforced


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_committed_runpod_summary_reproduces_locally(variation):
    # The committed Runpod summary sidecar is provenance; re-running the frozen verifier
    # on the committed rollout must reproduce its recorded verdict — otherwise the
    # promoted evidence and its sidecar have drifted apart.
    rollout = _live_rollout(variation)
    summary = load_json(_LIVE_FIXTURE_DIR / f"open_drawer_{variation}_demo00.summary.json")
    recorded = summary["verification"]
    case = verify_external_rollout(load_json(_TARGET), rollout, case_name="open_drawer")
    assert case["passed"] == recorded["passed"]
    assert case["matcherPassed"] == recorded["matcherPassed"]
    assert case["leakageClean"] == recorded["leakageClean"]
    assert case["physicalValidity"] == recorded["physicalValidity"]
    assert set(case["hardMismatches"]) == set(recorded["hardMismatches"])


@pytest.mark.parametrize("variation", _LIVE_VARIATIONS)
def test_confusion_on_live_rlbench_is_off_task_clean_and_reproduces_sidecar(variation):
    # The pilot's single biggest risk is a too-easy OFF-TASK pass, so the committed LIVE
    # evidence must be checked for it directly — not only via the synthetic stand-in. The
    # real RLBench drawer trace must PASS no gold target at all, so it cannot accidentally
    # match a different task: pinning unexpectedOffTaskPasses == [] makes a future gold
    # change that lets the live trace match another task fail the suite. We also reproduce
    # the committed sidecar's confusion block exactly.
    rollout = _live_rollout(variation)
    recorded = load_json(
        _LIVE_FIXTURE_DIR / f"open_drawer_{variation}_demo00.summary.json"
    )["verification"]["confusion"]
    conf = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    # Load-bearing invariant — must hold regardless of any future gold-target drift.
    assert conf["unexpectedOffTaskPasses"] == []
    # Reproduce the recorded provenance block. NB: confusionClean is False here BY DESIGN
    # — the negative Result A misses its own gold diagonal (missedExpected open_drawer);
    # we assert the RECORDED value, never a blanket True.
    assert conf["results"] == recorded["results"]
    assert conf["passes"] == recorded["passes"]
    assert conf["confusionClean"] == recorded["confusionClean"]
    assert conf["missedExpected"] == recorded["missedExpected"]
    assert conf["unexpectedOffTaskPasses"] == recorded["unexpectedOffTaskPasses"]


# ---------------------------------------------------------------------------
# Live RLBench tests — skipped unless RLBench is importable or explicitly enabled.
# ---------------------------------------------------------------------------

_RLBENCH_AVAILABLE = (
    importlib.util.find_spec("rlbench") is not None
    and importlib.util.find_spec("pyrep") is not None
)
_LIVE_ENABLED = _RLBENCH_AVAILABLE or os.environ.get("RLBENCH_PILOT_LIVE") == "1"


@pytest.mark.skipif(not _LIVE_ENABLED, reason="RLBench/PyRep + live CoppeliaSim required")
@pytest.mark.parametrize("variation", ["bottom", "middle", "top"])
def test_live_record_open_drawer_passes_and_confuses(variation, tmp_path):  # pragma: no cover - live only
    # Live path: record one demo for the variation, convert, and assert it PASSes the
    # frozen verifier leakage-clean AND the confusion holds (own task only).
    from pilots.rlbench import record_open_drawer as rec

    records = rec.record_variation(variation, amount=1, headless=True)
    assert records, "expected at least one recorded demo"
    rollout = rec.build_rollout(records[0])
    assert_rollout_leakage_clean(rollout)

    target = load_json(_TARGET)
    case = verify_external_rollout(target, rollout, case_name="open_drawer")
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    confusion = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    assert confusion["unexpectedOffTaskPasses"] == []
