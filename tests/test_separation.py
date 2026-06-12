"""Separation tests: robot CSGs that MUST fail (distance > 0 / not passed).

These are the cheats and errors the old matcher let through.
"""
import copy

from csg.matcher import match, MatcherConfig
from csg.canon import CanonConfig
from conftest import to_robot


def test_wrong_relation_fails(cube_target):
    robot = to_robot(cube_target)
    robot["events"][2]["observedDeltas"][0]["relationTransition"]["toRelation"] = "ON_TOP_OF"
    r = match(cube_target, robot)
    assert not r.passed
    assert not r.probe_agreement["terminal_state"]


def test_removed_at_end_fails(cube_target):
    """Cube placed in then taken out — terminal trap (was 0.194, below a wrong
    phrasing's 0.317, in the old matcher)."""
    robot = to_robot(cube_target)
    robot["events"].append({
        "eventId": "e5", "eventKind": "CONTAINMENT_CHANGE",
        "timeSpan": {"startTimeNs": "6000000000", "endTimeNs": "6500000000"},
        "involvedObjectIds": ["r_cube", "r_tray"],
        "observedDeltas": [{"objectId": "r_cube", "confidence": 0.95,
                            "relationTransition": {"subjectObjectId": "r_cube", "objectObjectId": "r_tray",
                                                   "fromRelation": "INSIDE", "toRelation": "NEAR"}}],
        "confidence": 0.95})
    r = match(cube_target, robot)
    assert not r.passed
    assert not r.probe_agreement["terminal_state"]
    assert not r.probe_agreement["goal_satisfaction"]


def test_wrong_event_order_fails(cube_target):
    robot = to_robot(cube_target)
    # Swap containment and release spans so release precedes containment.
    robot["events"][2]["timeSpan"] = {"startTimeNs": "4600000000", "endTimeNs": "4700000000"}
    robot["events"][3]["timeSpan"] = {"startTimeNs": "4000000000", "endTimeNs": "4050000000"}
    r = match(cube_target, robot)
    assert not r.passed
    assert not r.probe_agreement["event_order"]


def test_role_swap_fails(cube_target):
    """tray-inside-cube instead of cube-inside-tray."""
    robot = to_robot(cube_target)
    rt = robot["events"][2]["observedDeltas"][0]["relationTransition"]
    rt["subjectObjectId"], rt["objectObjectId"] = "r_tray", "r_cube"
    rt["toRelation"] = "INSIDE"
    assert not match(cube_target, robot).passed


def test_empty_robot_fails(cube_target):
    assert not match(cube_target, {}).passed


def test_objects_only_robot_fails(cube_target):
    robot = {"objects": to_robot(cube_target)["objects"]}
    # Must fail regardless of unknown-masking posture (was 0.00225 before).
    assert not match(cube_target, robot).passed


def test_padding_does_not_flip_fail_to_pass(cube_target):
    """Dilution: padding a mismatching graph with matching facts must not make
    a hard probe agree (PASS is exact set agreement)."""
    robot = to_robot(cube_target)
    robot["events"][2]["observedDeltas"][0]["relationTransition"]["toRelation"] = "ON_TOP_OF"
    base = match(cube_target, robot)
    padded_t = copy.deepcopy(cube_target)
    padded_r = copy.deepcopy(robot)
    for g, sub, obj in ((padded_t, "h_cube", "h_tray"), (padded_r, "r_cube", "r_tray")):
        g.setdefault("relations", [])
        for i in range(40):
            g["relations"].append({"relationId": f"pad{i}", "timeNs": str(int(7e9 + i * 1e7)),
                                    "subjectObjectId": sub, "objectObjectId": obj,
                                    "relation": "NEAR", "confidence": 0.9})
    assert not base.passed
    padded = match(padded_t, padded_r)
    # The dilution attack (turn a FAIL into a PASS by adding matching filler)
    # is structurally dead: PASS is exact set agreement, so a real mismatch
    # survives padding. (Padding may shift *which* hard probe catches it.)
    assert not padded.passed
    assert any(not padded.probe_agreement[p] for p in padded.hard_probes)
