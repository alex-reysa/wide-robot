"""End-to-end loop tests: the acceptance gate that was impossible before.

    target -> scene -> solver -> frames -> extract (frames only) -> match

A correct honest rollout must PASS all hard probes with zero leakage, achieved
WITHOUT the extractor ever seeing the target.
"""
import pytest

from csg.solver import solve
from csg.rollout_extract import extract_robot_csg
from csg.matcher import match
from csg.benchmark import leakage_report, run_benchmark
from csg.common import load_json
from conftest import GOLD


@pytest.mark.parametrize("task", ["put_cube_in_tray", "open_drawer", "place_on_top", "push_object", "insert_object"])
def test_honest_loop_passes(task):
    target = load_json(GOLD / task / "target.json")
    run = solve(target)
    robot = extract_robot_csg(run.rollout)
    result = match(target, robot)
    assert result.passed, [p for p in result.hard_probes if not result.probe_agreement[p]]
    assert leakage_report(robot)["clean"]


def test_push_loop_contact_and_initial_state():
    """Watch items for the push skill: the rollout's contact is a non-grasp
    TOUCHING contact (no RELEASE event), and the puck starts FAR_FROM the goal."""
    target = load_json(GOLD / "push_object" / "target.json")
    run = solve(target)
    assert run.rollout["diagnostics"]["skill"] == "push"
    robot = extract_robot_csg(run.rollout)
    modes = [c["mode"] for c in robot["contacts"]]
    assert modes == ["TOUCHING_LIKELY"], modes
    kinds = {e["eventKind"] for e in robot["events"]}
    assert "RELEASE_INFERRED" not in kinds
    assert "CONTACT_BEGIN" in kinds
    first_rels = {r["relation"] for r in robot["relations"] if r["relationId"].endswith("_first")}
    assert first_rels == {"FAR_FROM"}, first_rels


@pytest.mark.parametrize("task", ["put_cube_in_tray", "place_on_top", "insert_object"])
def test_pick_place_loop_has_no_spurious_push_contacts(task):
    """Watch item: open-gripper approach/release phases pass through touching
    poses and must not emit TOUCHING contacts alongside the grasp."""
    target = load_json(GOLD / task / "target.json")
    run = solve(target)
    robot = extract_robot_csg(run.rollout)
    modes = [c["mode"] for c in robot["contacts"]]
    assert modes == ["GRASP_LIKELY"], modes


def test_benchmark_all_pass(tmp_path):
    targets = [GOLD / "put_cube_in_tray" / "target.json", GOLD / "open_drawer" / "target.json"]
    report = run_benchmark(targets, tmp_path)
    assert report["summary"]["failed"] == 0, report["cases"]


def test_solver_no_target_in_rollout():
    target = load_json(GOLD / "put_cube_in_tray" / "target.json")
    run = solve(target)
    assert "targetCsg" not in run.rollout
    # The rollout carries frames and scene bodies, not the observation graph.
    assert "frames" in run.rollout and "sceneBodies" in run.rollout
    assert "events" not in run.rollout and "relations" not in run.rollout
