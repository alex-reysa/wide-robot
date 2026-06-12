import pytest

mujoco = pytest.importorskip("mujoco")

from csg.benchmark import run_invalid_fixtures
from conftest import GOLD


def test_invalid_fixtures_fail_for_expected_reasons(tmp_path):
    report = run_invalid_fixtures(GOLD.parent / "gold_invalid", tmp_path)

    assert report["schemaVersion"] == "csg.invalid_fixture_report.v1"
    assert report["sourceProvenance"]["schemaVersion"] == "csg.source_provenance.v1"
    assert report["sourceProvenance"]["snapshot"]["algorithm"] == "sha256"
    assert report["summary"] == {"total": 9, "matched": 9, "mismatched": 0}

    expected_checks = {
        "put_cube_in_tray__early_release": "quasi_static_support_at_release",
        "put_cube_in_tray__wide_grasp": "gripper_feasibility",
        "put_cube_in_tray__impossible_reach": "workspace_reachability",
        "put_cube_in_tray__teleport_after_release": "pose_continuity",
        "place_on_top__penetrate_goal": "non_penetration",
        "open_drawer__overlimit_articulation": "articulation_limits",
    }
    expected_semantic = {
        "push_object__missing_contact": ("contact_missing", "contact_word"),
        "place_on_top__wrong_relation": ("relation_not_achieved", "goal_satisfaction"),
        "place_on_top__wrong_event_order": ("event_order_wrong", "event_order"),
    }
    by_id = {fixture["fixtureId"]: fixture for fixture in report["fixtures"]}
    assert set(by_id) == set(expected_checks) | set(expected_semantic)

    for fixture_id, check_name in expected_checks.items():
        fixture = by_id[fixture_id]
        assert fixture["expectedFailureMatched"] is True
        assert fixture["mismatches"] == []
        assert check_name in fixture["failedValidityChecks"]

        result = fixture["result"]
        assert result["status"] == "FAIL"
        assert result["physicalValidity"] is False
        assert result["failureClassification"]["category"] == "physical_invalidity"
        assert result["physicalValidityReport"]["checks"][check_name]["passed"] is False

    for fixture_id, (category, hard_probe) in expected_semantic.items():
        fixture = by_id[fixture_id]
        assert fixture["expectedFailureMatched"] is True
        assert fixture["mismatches"] == []
        assert fixture["failedValidityChecks"] == []

        result = fixture["result"]
        assert result["status"] == "FAIL"
        assert result["physicalValidity"] is True
        assert result["failureClassification"]["category"] == category
        assert hard_probe in result["hardMismatches"]

    assert (tmp_path / "invalid_fixtures_report.json").is_file()
    for fixture_id in set(expected_checks) | set(expected_semantic):
        assert (tmp_path / fixture_id / "rollout.json").is_file()
    for fixture_id in expected_checks:
        assert (tmp_path / fixture_id / "validity_report.json").is_file()
