"""Anti-leakage (no-target-leakage) tests — roadmap Phase 2A (legacy "6D").

Guarantee the rollout extractor derives the robot CSG from rollout frames only,
never from the target answer key.
"""
import copy
import json
import random
from pathlib import Path

import pytest

from csg.solver import solve
from csg.rollout_extract import extract_robot_csg
from csg.benchmark import leakage_report
from conftest import GOLD


def _load(name):
    return json.loads((GOLD / name).read_text())


def test_rollout_carries_no_targetcsg():
    target = _load("put_cube_in_tray/target.json")
    run = solve(target)
    assert "targetCsg" not in run.rollout and "target_csg" not in run.rollout


def test_robot_csg_has_no_taskspec():
    target = _load("put_cube_in_tray/target.json")
    robot = extract_robot_csg(solve(target).rollout)
    rep = leakage_report(robot)
    assert rep["clean"], rep
    for k in ("plannerView", "targetCsg", "solverMetadata"):
        assert k not in robot


def test_provenance_is_sim_only():
    target = _load("open_drawer/target.json")
    robot = extract_robot_csg(solve(target).rollout)
    estimators = {e.get("estimator", "").upper() for e in robot.get("evidence", [])}
    assert estimators <= {"SIM_STATE_EXTRACTION", ""}, estimators


def test_canary_watermark_absent_from_robot():
    """Inject a unique watermark into the target; it must not surface in the
    robot CSG (which would prove copying)."""
    target = _load("put_cube_in_tray/target.json")
    token = "CANARY_%d" % random.Random(0).randint(10**6, 10**7)
    target = copy.deepcopy(target)
    target["objects"][0].setdefault("visualAttributes", []).append({"name": "canary", "value": token, "confidence": 0.99})
    robot = extract_robot_csg(solve(target).rollout)
    assert token not in json.dumps(robot)


def test_extractor_source_never_reads_targetcsg():
    """Static guard: the extractor must not *access* a targetCsg key. Prose
    mentions in the docstring are fine; quoted key access is not."""
    src = (Path(__file__).resolve().parent.parent / "csg" / "rollout_extract.py").read_text()
    forbidden = ["'targetCsg'", '"targetCsg"', "'target_csg'", '"target_csg"']
    offending = [ln for ln in src.splitlines()
                 if any(tok in ln for tok in forbidden) and "readTargetCsg" not in ln]
    assert not offending, offending


def test_isomorphism_alarm_distinguishes_copy_from_extract():
    """A verbatim id-renamed copy of the target and the honestly-extracted CSG
    should differ structurally (the copy still carries human-side provenance)."""
    target = _load("put_cube_in_tray/target.json")
    robot = extract_robot_csg(solve(target).rollout)
    assert robot.get("evidence", [{}])[0].get("estimator") == "SIM_STATE_EXTRACTION"
    assert "plannerView" not in robot


# -----------------------------------------------------------------------------
# V0.2 structural defenses (audit A4)
# -----------------------------------------------------------------------------

FREE_TEXT_FIELDS = {"categoryLabel", "taskCaption", "name", "value", "note",
                    "label", "modelName", "modelVersion", "artifactUri", "uri", "graphId"}


def _fuzz_all_strings(target, token):
    """Canary every free-text field AND consistently rename every id."""
    t = copy.deepcopy(target)

    def walk(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k in FREE_TEXT_FIELDS and isinstance(v, str):
                    o[k] = f"{v}_{token}"
                else:
                    walk(v)
        elif isinstance(o, list):
            for x in o:
                walk(x)
    walk(t)
    s = json.dumps(t)
    for ident in ("h_cube", "h_tray", "h_drawer", "right_hand", "e1", "e2", "e3", "e4", "c1", "te1", "te2", "g1", "s1"):
        s = s.replace(f'"{ident}"', f'"{ident}_{token}"')
    return json.loads(s)


@pytest.mark.parametrize("task", ["put_cube_in_tray", "open_drawer"])
def test_all_string_field_canary_fuzz(task):
    """No target-authored text — labels, ids, notes, captions, evidence
    strings — may surface in the extracted robot CSG. The old canary test
    covered only visualAttributes, the one field that did not leak (audit A4
    showed categoryLabel and ids flowed straight through sceneBodies)."""
    target = _load(f"{task}/target.json")
    token = "ZCANARYZ"
    fuzzed = _fuzz_all_strings(target, token)
    robot = extract_robot_csg(solve(fuzzed).rollout)
    assert token not in json.dumps(robot)


def test_counterfactual_invariance_of_extraction():
    """The robot CSG is a function of (frames, sceneBodies) only: mutating
    target fields the scene compiler does not consume (contacts, temporal
    edges, captions, visual attributes, evidence) must leave the extracted
    robot CSG byte-identical. Stronger than any self-reported flag."""
    target = _load("put_cube_in_tray/target.json")
    base = extract_robot_csg(solve(target).rollout)

    mutated = copy.deepcopy(target)
    mutated["taskCaption"] = "a totally different caption"
    mutated["contacts"] = []
    mutated["temporalEdges"] = []
    mutated["evidence"] = [{"evidenceId": "evX", "estimator": "MANUAL_ANNOTATION"}]
    for o in mutated["objects"]:
        o["visualAttributes"] = [{"name": "color", "value": "chartreuse", "confidence": 0.9}]
    other = extract_robot_csg(solve(mutated).rollout)

    assert json.dumps(base, sort_keys=True) == json.dumps(other, sort_keys=True)


def test_solver_responds_to_spec_not_memorized_trajectory():
    """Cross-sensitivity: flip the goal INSIDE -> ON_TOP_OF; the new rollout's
    extracted CSG must FAIL the *original* target. A solver replaying a
    memorized trajectory would pass both."""
    from csg.matcher import match
    target = _load("put_cube_in_tray/target.json")
    flipped = copy.deepcopy(target)
    flipped["plannerView"]["stages"][0]["goalConstraints"][0]["relation"]["desiredRelation"] = "ON_TOP_OF"
    robot = extract_robot_csg(solve(flipped).rollout)
    r = match(target, robot)
    assert not r.passed
    assert not r.probe_agreement["goal_satisfaction"]


def test_rollout_scene_bodies_are_sanitized():
    """The rollout may carry only whitelisted body fields with neutral ids."""
    target = _load("put_cube_in_tray/target.json")
    rollout = solve(target).rollout
    allowed = {"objectId", "bodyId", "physicalKind", "sizeM", "sizeApproximate",
               "mobility", "articulation", "isContainer", "containerCavity"}
    assert rollout["sceneBodies"]
    for b in rollout["sceneBodies"]:
        assert set(b) <= allowed, set(b) - allowed
        assert b["objectId"].startswith("body_")
        assert "categoryLabel" not in b and "sourceObjectId" not in b


def test_evidence_free_robot_csg_is_not_clean():
    """An evidence-free or empty-estimator CSG proves nothing about its
    provenance and must not pass the leakage gate."""
    target = _load("put_cube_in_tray/target.json")
    robot = extract_robot_csg(solve(target).rollout)
    assert leakage_report(robot)["clean"]
    stripped = copy.deepcopy(robot)
    stripped["evidence"] = []
    assert not leakage_report(stripped)["clean"]
    blank = copy.deepcopy(robot)
    blank["evidence"][0]["estimator"] = ""
    assert not leakage_report(blank)["clean"]
