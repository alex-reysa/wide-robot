"""Object-mapping rework tests (audit A6): 1-WL role fingerprints + guided DFS.

The old greedy fallback (>8 relevant objects) mapped identical-signature
objects positionally, so a demonstration moving cube #3 could be aligned to a
rollout bystander and false-FAIL. Fingerprints align by role, valid bijections
within a symmetry orbit are interchangeable, and real mismatches still fail.
"""
import time

from csg.matcher import MatcherConfig, match


def _cube(oid):
    return {
        "objectId": oid, "categoryLabel": "red cube", "categoryConfidence": 0.95,
        "physicalKind": "RIGID_OBJECT",
        "geometry": {"source": "FROM_6D_POSE_AND_CAD", "orientedBox": {"sizeM": {"x": 0.04, "y": 0.04, "z": 0.04}}},
    }


def _tray(oid):
    return {
        "objectId": oid, "categoryLabel": "black tray", "categoryConfidence": 0.93,
        "physicalKind": "RIGID_OBJECT",
        "geometry": {"source": "FROM_6D_POSE_AND_CAD", "orientedBox": {"sizeM": {"x": 0.24, "y": 0.18, "z": 0.06}}},
    }


def _many_cubes_graph(prefix, moved_idx, n_cubes=10, agent=("right_hand", "RIGHT_HAND")):
    """n identical cubes near a tray; cube #moved_idx is placed INSIDE it."""
    cube = f"{prefix}_cube_{moved_idx}"
    tray = f"{prefix}_tray"
    aid, akind = agent
    g = {
        "schemaVersion": "csg.v0",
        "graphId": f"{prefix}_many_cubes",
        "objects": [_cube(f"{prefix}_cube_{i}") for i in range(n_cubes)] + [_tray(tray)],
        "agentParts": [{"agentPartId": aid, "partKind": akind}],
        "relations": [
            {"relationId": f"r_{i}_{tag}", "timeNs": t, "subjectObjectId": f"{prefix}_cube_{i}",
             "objectObjectId": tray, "relation": "NEAR", "confidence": 0.9}
            for i in range(n_cubes) if i != moved_idx
            for tag, t in (("first", "0"), ("last", "6000000000"))
        ],
        "contacts": [
            {"contactId": "c1", "a": {"kind": "HUMAN_PART_ENTITY", "id": aid},
             "b": {"kind": "OBJECT_ENTITY", "id": cube},
             "timeSpan": {"startTimeNs": "1000000000", "endTimeNs": "4600000000"},
             "mode": "GRASP_LIKELY", "relativeMotion": "STICKING_LIKELY", "confidence": 0.9,
             "contactEvidence": {"motionCorrelation": 0.95, "stateChangeNearContactBoundary": True}},
        ],
        "events": [
            {"eventId": "e1", "eventKind": "CONTACT_BEGIN",
             "timeSpan": {"startTimeNs": "1000000000", "endTimeNs": "1100000000"},
             "involvedObjectIds": [cube], "involvedAgentPartIds": [aid], "confidence": 0.9},
            {"eventId": "e3", "eventKind": "CONTAINMENT_CHANGE",
             "timeSpan": {"startTimeNs": "4000000000", "endTimeNs": "4500000000"},
             "involvedObjectIds": [cube, tray],
             "observedDeltas": [{"objectId": cube, "confidence": 0.95,
                                 "relationTransition": {"subjectObjectId": cube, "objectObjectId": tray,
                                                        "fromRelation": "NEAR", "toRelation": "INSIDE"}}],
             "confidence": 0.95},
            {"eventId": "e4", "eventKind": "RELEASE_INFERRED",
             "timeSpan": {"startTimeNs": "4600000000", "endTimeNs": "4700000000"},
             "involvedObjectIds": [cube], "involvedAgentPartIds": [aid], "confidence": 0.85},
        ],
    }
    return g


def _many_cubes_target(moved_idx=3, n_cubes=10):
    g = _many_cubes_graph("h", moved_idx, n_cubes)
    cube, tray = f"h_cube_{moved_idx}", "h_tray"
    g["plannerView"] = {
        "bodies": [{"objectId": f"h_cube_{i}", "mobility": "MOVABLE"} for i in range(n_cubes)]
        + [{"objectId": tray, "mobility": "STATIC"}],
        "stages": [{"stageId": "s1", "confidence": 0.95, "goalConstraints": [
            {"constraintId": "g1", "kind": "OBJECT_RELATION_GOAL", "hard": True, "confidence": 0.95,
             "relation": {"subjectObjectId": cube, "objectObjectId": tray, "desiredRelation": "INSIDE"}}]}],
    }
    return g


def test_ten_identical_cubes_role_alignment():
    """Demonstration moves cube #3; rollout moves cube #7. Identical cubes are
    a symmetry orbit, so role fingerprints must align mover to mover — the old
    positional greedy mapped h_cube_3 -> r_cube_3 (a bystander) and false-FAILed."""
    target = _many_cubes_target(moved_idx=3)
    robot = _many_cubes_graph("r", moved_idx=7, agent=("robot_gripper", "ROBOT_GRIPPER"))
    r = match(target, robot)
    assert r.passed, [p for p in r.hard_probes if not r.probe_agreement[p]]
    assert r.object_mapping["h_cube_3"] == "r_cube_7"
    assert r.object_orbit_ambiguous  # 9 interchangeable bystanders
    assert r.diagnostics["target_symmetry_orbits"], "bystander orbit should be reported"


def test_ten_identical_cubes_is_fast():
    target = _many_cubes_target(moved_idx=3)
    robot = _many_cubes_graph("r", moved_idx=7, agent=("robot_gripper", "ROBOT_GRIPPER"))
    t0 = time.monotonic()
    match(target, robot)
    assert time.monotonic() - t0 < 2.0


def test_mapping_rework_does_not_mask_wrong_relation():
    """Same orbit scenario but the rollout achieves ON_TOP_OF: no bijection may
    rescue it."""
    target = _many_cubes_target(moved_idx=3)
    robot = _many_cubes_graph("r", moved_idx=7, agent=("robot_gripper", "ROBOT_GRIPPER"))
    robot["events"][1]["observedDeltas"][0]["relationTransition"]["toRelation"] = "ON_TOP_OF"
    robot["events"][1]["eventKind"] = "SUPPORT_CHANGE"
    r = match(target, robot)
    assert not r.passed


def test_mapping_rework_does_not_mask_missing_action():
    """Rollout where no cube is moved at all must FAIL."""
    target = _many_cubes_target(moved_idx=3)
    robot = _many_cubes_graph("r", moved_idx=7, agent=("robot_gripper", "ROBOT_GRIPPER"))
    robot["events"] = []
    robot["contacts"] = []
    r = match(target, robot)
    assert not r.passed


def test_unique_roles_still_unambiguous(cube_target, cube_robot_success):
    """The 2-object gold case has distinct roles: no orbit, no ambiguity."""
    r = match(cube_target, cube_robot_success)
    assert r.passed
    assert not r.object_orbit_ambiguous


def test_candidate_budget_respected():
    target = _many_cubes_target(moved_idx=3)
    robot = _many_cubes_graph("r", moved_idx=7, agent=("robot_gripper", "ROBOT_GRIPPER"))
    cfg = MatcherConfig(max_candidate_mappings=4)
    r = match(target, robot, cfg)
    assert r.diagnostics["n_candidate_mappings"] <= 4
    assert r.passed  # the role-ranked first candidates already contain the right one
