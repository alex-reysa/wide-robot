"""Frozen regressions for the second adversarial audit (V0.2).

Each test encodes an executed attack from the 2026-06 audit. A1/A2 = vacuous
targets, A3 = ritual-replay event ordering, A5 = converse-transition
inversion, A7 = symmetric probes failing richer-but-correct rollouts.
"""
import copy
import json

import pytest

from csg.canon import _normalize_transition
from csg.matcher import match
from conftest import to_robot


# -----------------------------------------------------------------------------
# A1/A2 — vacuity gate
# -----------------------------------------------------------------------------


def test_vacuous_target_rejected(cube_target):
    """A target with objects but no facts/goals must not accept everything."""
    empty_t = {"schemaVersion": "csg.v0", "graphId": "t",
               "objects": copy.deepcopy(cube_target["objects"])}
    empty_r = {"schemaVersion": "csg.v0", "graphId": "r",
               "objects": copy.deepcopy(to_robot(cube_target)["objects"])}
    r = match(empty_t, empty_r)
    assert r.vacuous
    assert not r.passed


def test_low_confidence_target_rejected(cube_target):
    """Perception cannot weaken the verifier by deflating its confidences:
    a fully-masked target is vacuous, not universally satisfiable."""
    low_t = copy.deepcopy(cube_target)

    def deflate(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == "confidence" and isinstance(v, (int, float)):
                    o[k] = 0.2
                else:
                    deflate(v)
        elif isinstance(o, list):
            for x in o:
                deflate(x)
    deflate(low_t)
    empty_r = {"schemaVersion": "csg.v0", "graphId": "r",
               "objects": copy.deepcopy(to_robot(cube_target)["objects"])}
    r = match(low_t, empty_r)
    assert r.vacuous
    assert not r.passed


def test_rich_target_not_vacuous(cube_target):
    r = match(cube_target, to_robot(cube_target))
    assert not r.vacuous
    assert r.probe_support["goal_satisfaction"] >= 1
    assert r.probe_support["relation_transitions"] >= 1


# -----------------------------------------------------------------------------
# A3 — event order is an injective order-preserving embedding
# -----------------------------------------------------------------------------


def _containment_event(eid, t0_ns, t1_ns, from_rel, to_rel):
    return {
        "eventId": eid, "eventKind": "CONTAINMENT_CHANGE",
        "timeSpan": {"startTimeNs": str(t0_ns), "endTimeNs": str(t1_ns)},
        "involvedObjectIds": ["r_cube", "r_tray"],
        "observedDeltas": [{"objectId": "r_cube", "confidence": 0.95,
                            "relationTransition": {"subjectObjectId": "r_cube", "objectObjectId": "r_tray",
                                                   "fromRelation": from_rel, "toRelation": to_rel}}],
        "confidence": 0.95}


def test_messy_prefix_then_genuine_redo_passes(cube_target):
    """ACCEPTED SEMANTICS (user decision, V0.2): a rollout where the cube was
    accidentally knocked in and out *before* a genuine, complete, in-order
    occurrence of the demonstrated sequence still PASSes. The embedding
    requires one consistent replay to exist; whether the demonstrated contact
    *caused* the achieving containment remains an Unobservable Critical
    Variable (physical_quotient.md §10.14)."""
    robot = to_robot(cube_target)
    robot["events"] = [
        _containment_event("e_early_in", 500000000, 550000000, "NEAR", "INSIDE"),
        _containment_event("e_early_out", 800000000, 850000000, "INSIDE", "NEAR"),
    ] + robot["events"]
    assert match(cube_target, robot).passed


def test_duplicated_target_event_requires_two_occurrences(cube_target):
    """A target that demonstrates an event twice (here: regrasp, two
    CONTACT_BEGINs) is not satisfied by a rollout containing it once. The old
    pair-containment probe collapsed duplicate signatures and accepted this."""
    target = copy.deepcopy(cube_target)
    e1b = copy.deepcopy(target["events"][0])
    e1b["eventId"] = "e1b"
    e1b["timeSpan"] = {"startTimeNs": "3000000000", "endTimeNs": "3100000000"}
    target["events"].insert(2, e1b)
    robot = to_robot(cube_target)  # single CONTACT_BEGIN
    r = match(target, robot)
    assert not r.passed
    assert not r.probe_agreement["event_order"]
    # presence is sig-set based and still agrees; only the embedding catches it
    assert r.probe_agreement["event_presence"]


def test_no_consistent_replay_fails(cube_target):
    """Containment occurrences exist only BEFORE the demonstrated precursors:
    no single in-order occurrence of the demonstrated sequence exists."""
    robot = to_robot(cube_target)
    # Replace the genuine containment (e3) with two early ones.
    robot["events"] = [
        _containment_event("e_early_in", 500000000, 550000000, "NEAR", "INSIDE"),
        _containment_event("e_early_in2", 700000000, 750000000, "NEAR", "INSIDE"),
    ] + [e for e in robot["events"] if e["eventId"] != "e3"]
    r = match(cube_target, robot)
    assert not r.passed
    assert not r.probe_agreement["event_order"]


# -----------------------------------------------------------------------------
# A5 — converse-transition normalization
# -----------------------------------------------------------------------------


def test_normalize_transition_endpoints_independent():
    """'tray no longer CONTAINS cube' must yield INSIDE(cube, tray) as the
    from-fact, not the inverted INSIDE(tray, cube)."""
    f_fact, t_fact = _normalize_transition("h_tray", "h_cube", "CONTAINS", "NEAR")
    assert f_fact == ("h_cube", "h_tray", "INSIDE")
    assert t_fact[2] == "NEAR"


def test_converse_removal_target_matches_direct_robot():
    """A removal demo authored with converse phrasing (CONTAINS -> NEAR) must
    match an honest robot graph phrased directly (INSIDE -> NEAR). Under the
    V0.1 normalization the target demanded the unsatisfiable INSIDE(tray, cube)."""
    def graph(prefix, converse):
        if converse:
            rt = {"subjectObjectId": f"{prefix}_tray", "objectObjectId": f"{prefix}_cube",
                  "fromRelation": "CONTAINS", "toRelation": "NEAR"}
        else:
            rt = {"subjectObjectId": f"{prefix}_cube", "objectObjectId": f"{prefix}_tray",
                  "fromRelation": "INSIDE", "toRelation": "NEAR"}
        g = {
            "schemaVersion": "csg.v0", "graphId": f"{prefix}_remove_cube",
            "objects": [
                {"objectId": f"{prefix}_cube", "categoryLabel": "cube", "physicalKind": "RIGID_OBJECT",
                 "geometry": {"source": "FROM_6D_POSE_AND_CAD", "orientedBox": {"sizeM": {"x": .04, "y": .04, "z": .04}}}},
                {"objectId": f"{prefix}_tray", "categoryLabel": "tray", "physicalKind": "STATIC_SCENE_SURFACE",
                 "geometry": {"source": "FROM_2D_MASK_ONLY", "maskOnly": {"note": "t"}}},
            ],
            "events": [{"eventId": "e_out", "eventKind": "CONTAINMENT_CHANGE",
                        "timeSpan": {"startTimeNs": "2000000000", "endTimeNs": "2500000000"},
                        "involvedObjectIds": [f"{prefix}_cube", f"{prefix}_tray"],
                        "observedDeltas": [{"objectId": f"{prefix}_cube", "confidence": 0.95,
                                            "relationTransition": rt}],
                        "confidence": 0.95}],
        }
        if prefix == "h":
            g["plannerView"] = {"stages": [{"stageId": "s1", "goalConstraints": [
                {"constraintId": "g1", "kind": "OBJECT_RELATION_GOAL", "hard": True, "confidence": 0.95,
                 "relation": {"subjectObjectId": "h_cube", "objectObjectId": "h_tray",
                              "desiredRelation": "NEAR"}}], "confidence": 0.95}]}
        return g
    r = match(graph("h", converse=True), graph("r", converse=False))
    assert r.passed, [p for p in r.hard_probes if not r.probe_agreement[p]]


# -----------------------------------------------------------------------------
# A7 — directional promoted contact_word / articulation
# -----------------------------------------------------------------------------


def _promoted_target(cube_target):
    t = copy.deepcopy(cube_target)
    t["plannerView"]["stages"][0]["contactPermissions"][0]["permission"] = "CONTACT_REQUIRED"
    return t


def test_promoted_contact_extra_benign_touch_passes(cube_target):
    """One extra gripper-tray graze must not fail a correct rollout: the
    promoted contact word is compared only on the constrained pairs."""
    t = _promoted_target(cube_target)
    robot = to_robot(cube_target)
    robot["contacts"].append({
        "contactId": "c_extra",
        "a": {"kind": "ROBOT_PART_ENTITY", "id": "robot_gripper"},
        "b": {"kind": "OBJECT_ENTITY", "id": "r_tray"},
        "timeSpan": {"startTimeNs": "3000000000", "endTimeNs": "3100000000"},
        "mode": "TOUCHING_LIKELY", "relativeMotion": "UNKNOWN_RELATIVE_MOTION",
        "confidence": 0.9, "contactEvidence": {}})
    r = match(t, robot)
    assert "contact_word" in r.hard_probes
    assert r.passed, [p for p in r.hard_probes if not r.probe_agreement[p]]


def test_promoted_contact_missing_required_word_fails(cube_target):
    t = _promoted_target(cube_target)
    robot = to_robot(cube_target)
    robot["contacts"] = []
    r = match(t, robot)
    assert not r.passed
    assert not r.probe_agreement["contact_word"]


def _two_drawer_graph(prefix, cabinet_values):
    def art_state(sid, oid, t_ns, val):
        return {"stateId": sid, "objectId": oid, "timeNs": str(t_ns),
                "articulation": {"articulatedObjectId": oid, "jointKind": "PRISMATIC",
                                 "jointValue": val, "valueKind": "EXTENSION_M", "confidence": 0.9},
                "confidence": 0.9}
    g = {
        "schemaVersion": "csg.v0", "graphId": f"{prefix}_two_drawers",
        "objects": [
            {"objectId": f"{prefix}_drawer", "categoryLabel": "drawer", "physicalKind": "ARTICULATED_OBJECT",
             "geometry": {"source": "FROM_MULTIVIEW_RECONSTRUCTION", "orientedBox": {"sizeM": {"x": .4, "y": .3, "z": .15}}}},
            {"objectId": f"{prefix}_cabinet", "categoryLabel": "cabinet", "physicalKind": "ARTICULATED_OBJECT",
             "geometry": {"source": "FROM_MULTIVIEW_RECONSTRUCTION", "orientedBox": {"sizeM": {"x": .4, "y": .3, "z": .15}}}},
        ],
        "relations": [{"relationId": "r0", "timeNs": "0", "subjectObjectId": f"{prefix}_drawer",
                       "objectObjectId": f"{prefix}_cabinet", "relation": "NEAR", "confidence": 0.9}],
        "objectStates": [
            art_state("d0", f"{prefix}_drawer", 0, 0.02),
            art_state("d1", f"{prefix}_drawer", 3000000000, 0.18),
            art_state("c0", f"{prefix}_cabinet", 0, cabinet_values[0]),
            art_state("c1", f"{prefix}_cabinet", 3000000000, cabinet_values[1]),
        ],
    }
    if prefix == "h":
        g["plannerView"] = {"stages": [{"stageId": "s1", "goalConstraints": [
            {"constraintId": "g1", "kind": "ARTICULATION_GOAL", "hard": True, "confidence": 0.9,
             "articulation": {"articulatedObjectId": "h_drawer", "jointKind": "PRISMATIC",
                              "targetJointValue": 0.18, "valueKind": "EXTENSION_M"}}], "confidence": 0.9}]}
    return g


def test_extra_robot_articulation_passes():
    """Robot also (incidentally) opened the cabinet: extra robot articulation
    facts are allowed (directional probe), like extra events/relations."""
    target = _two_drawer_graph("h", cabinet_values=(0.0, 0.0))
    # Target asserts cabinet FLAT; matching robot keeps it FLAT in the base
    # case. Here the *target* has no cabinet motion observed at all:
    target["objectStates"] = [s for s in target["objectStates"] if s["objectId"] != "h_cabinet"]
    robot = _two_drawer_graph("r", cabinet_values=(0.0, 0.3))
    robot.pop("plannerView", None)
    r = match(target, robot)
    assert r.passed, [p for p in r.hard_probes if not r.probe_agreement[p]]


def test_articulation_conflict_still_fails():
    """Target observed the cabinet staying FLAT; a robot that moved it is a
    conflict (the tuple carries direction), not an 'extra fact'."""
    target = _two_drawer_graph("h", cabinet_values=(0.0, 0.0))
    robot = _two_drawer_graph("r", cabinet_values=(0.0, 0.3))
    robot.pop("plannerView", None)
    r = match(target, robot)
    assert not r.passed
    assert not r.probe_agreement["articulation_transitions"]
