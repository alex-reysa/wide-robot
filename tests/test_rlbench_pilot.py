"""RLBench external-trace pilot — seam + leakage discipline tests.

These run with NO RLBench installed: they exercise the rollout-assembly contract and
the external-entry path through the FROZEN csg verifier, using the committed synthetic
``csg.rollout.v0`` fixture (a stand-in for what the RLBench adapter must emit). The
point is to lock the leakage discipline for external traces before any real ingest.
"""
import copy
import json
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
)
from pilots.rlbench.run_external import verify_external_rollout

_REPO = Path(__file__).resolve().parents[1]
_FIXTURE = _REPO / "pilots" / "rlbench" / "fixtures" / "synthetic_open_drawer.rollout.json"
_TARGET = _REPO / "gold_tests" / "open_drawer" / "target.json"


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


# ---------------------------------------------------------------------------
# The RLBench-ingest half is an explicit, un-mistakable stub
# ---------------------------------------------------------------------------


def test_rlbench_demo_to_rollout_is_an_explicit_stub():
    with pytest.raises(NotImplementedError, match="pilot stub"):
        rlbench_demo_to_rollout(object(), task="open_drawer")


def test_adapter_imports_without_rlbench_installed():
    # The module must import on a machine with no RLBench/PyRep — the dependency is
    # optional and only touched inside the (lazy) ingest stub.
    assert "rlbench" not in {m.split(".")[0] for m in list(__import__("sys").modules) if m == "rlbench"}
    assert adapter.RLBENCH_FIELD_MAPPING  # mapping contract is documented in-code
