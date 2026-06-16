"""RLBench external-sim object_inside_container — the matcher seam (Phase 2F-4).

The external-simulation leg of the ``object_inside_container`` flagship task (already
proven on MuJoCo internal sim, Sony/iPhone real camera, and RH20T real-robot video).
Synthetic RLBench ``PutItemInDrawer`` demos — mock ``Observation`` objects (effector pose +
gripper state) plus neutral per-frame ``measurements`` (item + container-volume poses/sizes)
— are converted by the RLBench rollout door
(:func:`pilots.rlbench.adapter_object_inside_container.put_item_in_drawer_demo_to_rollout`)
into ``csg.rollout.v0`` and judged by the FROZEN verifier against the two RLBench targets.
NO RLBench install, numpy, or cv2 is needed; ``csg/`` is never touched (only READ).

The two targets form a strictly-stronger pair (the RLBench analogue of the RH20T/real-camera
terminal_only → relation_event progression):
  * terminal_only  — only the item's TERMINAL relation to the container is INSIDE
    (hard OBJECT_RELATION_GOAL → goal_satisfaction);
  * relation_event — additionally the item STARTED NEAR (initial_state), ENDED INSIDE
    (terminal_state + relation_transitions), and a CONTAINMENT_CHANGE event is present
    (event_presence). event_order stays support 0 (one event, no pair).

Load-bearing subtlety pinned here (verified against the frozen extractor): a "born-inside"
item (inside the whole time, but moving) STILL emits a NEAR→INSIDE CONTAINMENT_CHANGE delta
because ``csg/rollout_extract.py`` seeds ``prev_rel="NEAR"`` UNCONDITIONALLY — so
event_presence/relation_transitions do NOT reject born-inside; only ``initial_state`` does.

QUARANTINE: RLBench/CoppeliaSim identity (task/variation names, the item/drawer/success
handles) lives only in the recorder. The rollout is source-blind — none of those tokens
appear in the rollout blob, not even in diagnostics prose.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path

import pytest

import csg.predicates as P
from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg

from pilots.external_rollout import ExternalTraceLeakage, assert_rollout_leakage_clean
from pilots.external_verify import external_confusion_report, load_gold_targets, verify_external_rollout
from pilots.rlbench.adapter_object_inside_container import (
    SUPPORTED_TASKS,
    put_item_in_drawer_demo_to_rollout,
)

_REPO = Path(__file__).resolve().parents[1]
_GOLD_DIR = _REPO / "gold_tests"
_TARGETS = _REPO / "pilots" / "rlbench" / "targets"
_TERMINAL = _TARGETS / "object_inside_container_terminal_only.json"
_REL_EVENT = _TARGETS / "object_inside_container_relation_event.json"
_PLACED = _TARGETS / "object_inside_container_placed_from_outside.json"
_FIXTURE = _REPO / "pilots" / "rlbench" / "fixtures" / "synthetic_put_item_in_drawer.rollout.json"

# Same proven tabletop geometry as the RH20T/real-camera fixtures: container center
# (TX,TY,TZ), size (0.24,0.18,0.03) -> footprint x in [0.18,0.42]; item is a 0.04^3 cube.
# (Representative offline geometry; the live container volume is the success-sensor bbox.)
TX, TY, TZ = 0.30, 0.0, 0.015
_CONTAINER = [0.24, 0.18, 0.03]
_ITEM = [0.04, 0.04, 0.04]
_INSIDE = (TX, TY, 0.03)
_NEAR_NOT_INSIDE = (TX + 0.13, TY, 0.05)
_ON_RIM = (TX, TY, 0.05)
_FAR = (TX + 0.35, TY, 0.02)
_START_NEAR = (TX + 0.16, TY, 0.05)

# Effector parked off-workspace + gripper OPEN on every frame, so the frozen extractor
# infers NO grasp/contact/co-motion (an honest synthetic put-in with no contact evidence —
# mirrors the RH20T seam). The live capture's real gripper is a Part-B concern.
_GRIPPER_OFF = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # xyz + XYZW identity


def _demo(n):
    """Mock RLBench demo: ``n`` Observations exposing only gripper_pose/gripper_open."""
    return [{"gripper_pose": list(_GRIPPER_OFF), "gripper_open": 1.0} for _ in range(n)]


def _measurements(item_seq, *, container_xyz=(TX, TY, TZ), fps=10.0,
                  item_conf=0.95, container_conf=0.99):
    out = []
    for i, (x, y, z) in enumerate(item_seq):
        out.append({
            "frameIndex": i,
            "timeS": i / fps,
            "itemPose": {"positionM": {"x": x, "y": y, "z": z}, "confidence": item_conf},
            "itemSizeM": list(_ITEM),
            "containerPose": {"positionM": {"x": container_xyz[0], "y": container_xyz[1],
                                            "z": container_xyz[2]}, "confidence": container_conf},
            "containerSizeM": list(_CONTAINER),
            "sizeApproximate": True,
        })
    return out


def _rollout(item_seq, **kw):
    r = put_item_in_drawer_demo_to_rollout(_demo(len(item_seq)), measurements=_measurements(item_seq, **kw))
    assert_rollout_leakage_clean(r)
    return r


def _approach_then(end_xyz):
    """Item starts NEAR (outside the container) and moves to ``end_xyz`` over 6 frames so the
    terminal relation persists and item displacement >> MOTION_EPS_M."""
    sx, sy, sz = _START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _approach_from_far(end_xyz):
    """Item starts FAR_FROM the container (carried in from across the table, like RLBench's
    bottom/middle drawer episodes) and is placed at ``end_xyz``."""
    sx, sy, sz = _FAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _verify(target_path, rollout):
    return verify_external_rollout(load_json(target_path), rollout, case_name="rlbench_oic")


def _box(center, size):
    return P.box_from(center, tuple(size))


# ---------------------------------------------------------------------------
# Geometry tripwire — pin the fixture coordinates against the frozen predicates
# ---------------------------------------------------------------------------


def test_fixture_geometry_classifies_as_intended():
    container = _box((TX, TY, TZ), _CONTAINER)
    assert P.is_inside(_box(_INSIDE, _ITEM), container) is True
    assert P.primary_topo_relation(_box(_INSIDE, _ITEM), container) == "INSIDE"
    assert P.is_inside(_box(_START_NEAR, _ITEM), container) is False
    assert P.is_near(_box(_START_NEAR, _ITEM), container) is True
    assert P.is_inside(_box(_ON_RIM, _ITEM), container) is False
    assert P.is_on_top_of(_box(_ON_RIM, _ITEM), container) is True
    assert P.is_inside(_box(_NEAR_NOT_INSIDE, _ITEM), container) is False
    assert P.is_near(_box(_NEAR_NOT_INSIDE, _ITEM), container) is True
    assert P.is_near(_box(_FAR, _ITEM), container) is False


# ---------------------------------------------------------------------------
# Targets structure / not-a-gold-task
# ---------------------------------------------------------------------------


def test_targets_structure_and_deferrals():
    term = load_json(_TERMINAL)
    rele = load_json(_REL_EVENT)
    for t in (term, rele):
        goals = t["plannerView"]["stages"][0]["goalConstraints"]
        assert [g["kind"] for g in goals] == ["OBJECT_RELATION_GOAL"]
        assert goals[0]["hard"] is True
        assert goals[0]["relation"]["desiredRelation"] == "INSIDE"
        assert t["agentParts"] == []
        assert "contacts" not in t and "temporalEdges" not in t
    assert "relations" not in term and "events" not in term
    assert [r["relation"] for r in rele["relations"]] == ["NEAR", "INSIDE"]
    assert [e["eventKind"] for e in rele["events"]] == ["CONTAINMENT_CHANGE"]
    trans = rele["events"][0]["observedDeltas"][0]["relationTransition"]
    assert (trans["fromRelation"], trans["toRelation"]) == ("NEAR", "INSIDE")


def test_targets_are_not_gold_tasks():
    assert not (_GOLD_DIR / "object_inside_container_terminal_only").exists()
    assert not (_GOLD_DIR / "object_inside_container_relation_event").exists()
    assert load_json(_TERMINAL)["pilotMetadata"]["diagnostic"] == "rlbench-object-inside-container-terminal-only"
    assert load_json(_REL_EVENT)["pilotMetadata"]["diagnostic"] == "rlbench-object-inside-container-relation-event"


# ---------------------------------------------------------------------------
# Positive — a real put-in PASSes both targets, non-vacuously, leakage-clean
# ---------------------------------------------------------------------------


def test_success_passes_both_targets_leakage_clean():
    rollout = _rollout(_approach_then(_INSIDE))
    assert rollout["backend"] == "rlbench_external"
    assert rollout["skillProgram"]["source"] == "rlbench"
    assert rollout["diagnostics"]["physicalValidity"] is None
    assert rollout["objectIdMap"] == {}
    assert [b["objectId"] for b in rollout["sceneBodies"]] == ["body_000", "body_001"]
    assert rollout["sceneBodies"][1]["isContainer"] is True

    for path in (_TERMINAL, _REL_EVENT):
        case = _verify(path, rollout)
        assert case["passed"] is True, (path.name, case["hardMismatches"])
        assert case["leakageClean"] is True
        assert case["physicalValidity"] is None
        assert case["hardMismatches"] == []

    res = match(load_json(_REL_EVENT), extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    for probe in ("goal_satisfaction", "initial_state", "terminal_state",
                  "relation_transitions", "event_presence"):
        assert res.probe_support[probe] == 1, probe
        assert res.probe_agreement[probe] is True, probe
    assert res.probe_support["event_order"] == 0


# ---------------------------------------------------------------------------
# Strictly stronger — born-inside PASSes terminal-only but FAILs relation-event
# on initial_state (NOT on the event/transition — the load-bearing subtlety)
# ---------------------------------------------------------------------------


def test_born_inside_passes_terminal_only_fails_relation_event_on_initial_state():
    born = [(TX - 0.03, TY, 0.03), (TX - 0.01, TY, 0.03), (TX + 0.01, TY, 0.03),
            (TX + 0.03, TY, 0.03), (TX + 0.01, TY, 0.03), (TX - 0.01, TY, 0.03)]
    rollout = _rollout(born)
    term = _verify(_TERMINAL, rollout)
    rele = _verify(_REL_EVENT, rollout)
    assert term["passed"] is True, term["hardMismatches"]
    assert rele["passed"] is False
    assert rele["hardMismatches"] == ["initial_state"], rele["hardMismatches"]
    assert "event_presence" not in rele["hardMismatches"]
    assert "relation_transitions" not in rele["hardMismatches"]
    assert "goal_satisfaction" not in rele["hardMismatches"]


# ---------------------------------------------------------------------------
# placed_from_outside (FAR start) vs relation_event (NEAR start) — the two strong
# targets partition episodes by their observed initial relation (RLBench's
# bottom/middle drawers start FAR, the top drawer starts NEAR). Each rejects
# born-inside via initial_state.
# ---------------------------------------------------------------------------


def test_placed_from_outside_structure():
    t = load_json(_PLACED)
    assert t["plannerView"]["stages"][0]["goalConstraints"][0]["relation"]["desiredRelation"] == "INSIDE"
    assert [r["relation"] for r in t["relations"]] == ["FAR_FROM", "INSIDE"]
    assert [e["eventKind"] for e in t["events"]] == ["CONTAINMENT_CHANGE"]
    assert "contacts" not in t and "temporalEdges" not in t
    assert not (_GOLD_DIR / "object_inside_container_placed_from_outside").exists()
    assert load_json(_PLACED)["pilotMetadata"]["diagnostic"] == "rlbench-object-inside-container-placed-from-outside"


def test_far_start_success_passes_placed_from_outside_not_relation_event():
    rollout = _rollout(_approach_from_far(_INSIDE))
    placed = _verify(_PLACED, rollout)
    assert placed["passed"] is True, placed["hardMismatches"]
    res = match(load_json(_PLACED), extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    for probe in ("goal_satisfaction", "initial_state", "terminal_state", "relation_transitions", "event_presence"):
        assert res.probe_support[probe] == 1 and res.probe_agreement[probe] is True, probe
    # a FAR-start episode does NOT match the NEAR-start target (initial_state distinguishes them)
    rele = _verify(_REL_EVENT, rollout)
    assert rele["passed"] is False
    assert rele["hardMismatches"] == ["initial_state"], rele["hardMismatches"]
    # terminal_only is agnostic to the start, so it PASSes either way
    assert _verify(_TERMINAL, rollout)["passed"] is True


def test_near_start_success_passes_relation_event_not_placed_from_outside():
    rollout = _rollout(_approach_then(_INSIDE))
    assert _verify(_REL_EVENT, rollout)["passed"] is True
    placed = _verify(_PLACED, rollout)
    assert placed["passed"] is False
    assert placed["hardMismatches"] == ["initial_state"], placed["hardMismatches"]


def test_born_inside_fails_placed_from_outside_on_initial_state():
    born = [(TX - 0.03, TY, 0.03), (TX - 0.01, TY, 0.03), (TX + 0.01, TY, 0.03),
            (TX + 0.03, TY, 0.03), (TX + 0.01, TY, 0.03), (TX - 0.01, TY, 0.03)]
    rollout = _rollout(born)
    placed = _verify(_PLACED, rollout)
    assert placed["passed"] is False
    assert placed["hardMismatches"] == ["initial_state"], placed["hardMismatches"]
    assert "event_presence" not in placed["hardMismatches"]


# ---------------------------------------------------------------------------
# Failure modes — each FAILs leakage-clean, naming the probe(s) it should trip
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,end_xyz", [
    ("near_not_inside", _NEAR_NOT_INSIDE),
    ("rim_placement", _ON_RIM),
    ("dropped_outside", _FAR),
])
def test_failure_modes(name, end_xyz):
    rollout = _rollout(_approach_then(end_xyz))
    term = _verify(_TERMINAL, rollout)
    assert term["passed"] is False, (name, term["hardMismatches"])
    assert term["leakageClean"] is True
    assert "goal_satisfaction" in term["hardMismatches"], (name, term["hardMismatches"])

    rele = _verify(_REL_EVENT, rollout)
    assert rele["passed"] is False, name
    assert rele["leakageClean"] is True
    assert {"goal_satisfaction", "terminal_state", "relation_transitions",
            "event_presence"} <= set(rele["hardMismatches"]), (name, rele["hardMismatches"])


def test_flat_never_moves_fails_both():
    # Item parked NEAR the whole time (never enters): no INSIDE, no transition.
    rollout = _rollout([_START_NEAR] * 6)
    for path in (_TERMINAL, _REL_EVENT):
        case = _verify(path, rollout)
        assert case["passed"] is False
        assert case["leakageClean"] is True
        assert "goal_satisfaction" in case["hardMismatches"]


# ---------------------------------------------------------------------------
# Leakage — a leaky trace is rejected at the door, before the matcher can PASS
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,mutate,match_re", [
    ("targetCsg", lambda r: r.__setitem__("targetCsg", {"leaked": True}), "forbidden"),
    ("plannerView", lambda r: r.__setitem__("plannerView", {"leaked": True}), "forbidden"),
    ("solverMetadata", lambda r: r.__setitem__("solverMetadata", {"leaked": True}), "forbidden"),
    ("objectIdMap", lambda r: r.__setitem__("objectIdMap", {"h_item": "body_000"}), "objectIdMap"),
    ("body_field", lambda r: r["sceneBodies"][0].__setitem__("categoryLabel", "item"), "non-whitelisted"),
    ("non_neutral_body_id", lambda r: r["sceneBodies"][0].__setitem__("objectId", "the_item"), "neutral"),
])
def test_leaky_trace_is_rejected_before_matcher_success(name, mutate, match_re):
    base = _rollout(_approach_then(_INSIDE))
    assert _verify(_REL_EVENT, base)["passed"] is True
    bad = copy.deepcopy(base)
    mutate(bad)
    with pytest.raises(ExternalTraceLeakage, match=match_re):
        assert_rollout_leakage_clean(bad)
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(load_json(_REL_EVENT), bad)


# ---------------------------------------------------------------------------
# Converter contract — measurement leakage caught at the door; task gating
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("leak_key", ["task", "variation", "success_bottom", "drawer_joint", "h_item"])
def test_converter_rejects_non_neutral_measurement_keys(leak_key):
    meas = _measurements(_approach_then(_INSIDE))
    meas[0][leak_key] = "leaked"
    with pytest.raises(ExternalTraceLeakage, match="non-neutral"):
        put_item_in_drawer_demo_to_rollout(_demo(len(meas)), measurements=meas)


def test_converter_rejects_unsupported_task_and_requires_measurements():
    assert SUPPORTED_TASKS == ("put_item_in_drawer",)
    with pytest.raises(NotImplementedError):
        put_item_in_drawer_demo_to_rollout(_demo(3), measurements=_measurements(_approach_then(_INSIDE))[:3],
                                           task="open_drawer")
    with pytest.raises(ValueError, match="measurements"):
        put_item_in_drawer_demo_to_rollout(_demo(3), measurements=None)


# ---------------------------------------------------------------------------
# Source identity quarantine — the rollout is fully source-blind
# ---------------------------------------------------------------------------


def test_rlbench_identity_is_quarantined():
    rollout = _rollout(_approach_then(_INSIDE))
    blob = json.dumps(rollout)
    # RLBench/CoppeliaSim source identity — task, variation, and handle names — must not
    # appear ANYWHERE in the rollout (not even in diagnostics prose, which is task-name-free
    # by design). "rlbench"/"external"/"isContainer"/"success"(top-level bool key) are
    # legitimate structure and are NOT identity, so they are not forbidden.
    for forbidden in ("put_item_in_drawer", "PutItemInDrawer", "drawer", "waypoint",
                      "success_", "item", "bottom", "middle", "top"):
        assert forbidden not in blob, f"{forbidden!r} leaked into the rollout"
    # structural: neutral body ids only, everywhere the extractor reads them
    for body in rollout["sceneBodies"]:
        assert str(body["objectId"]).startswith("body_")
    for frame in rollout["frames"]:
        assert all(k.startswith("body_") for k in frame["objectPoses"])
    assert rollout["diagnostics"]["source"] == "rlbench"
    assert rollout["diagnostics"]["staticBodyClampApplied"] == ["body_001"]


# ---------------------------------------------------------------------------
# Cross-task confusion — an RLBench success matches NO off-task gold target
# ---------------------------------------------------------------------------


def test_success_matches_no_off_task_gold_target():
    conf = external_confusion_report(
        _rollout(_approach_then(_INSIDE)), load_gold_targets(_GOLD_DIR),
        expected_case="put_cube_in_tray")
    # The relation/event subset has containment but NO contact/co-motion/release evidence
    # (effector parked off-workspace), so it matches no FULL gold task (every gold
    # containment task additionally demands those events). Honest result: matches nothing.
    assert conf["passes"] == [], conf["passes"]
    assert conf["unexpectedOffTaskPasses"] == []


# ---------------------------------------------------------------------------
# Committed synthetic fixture — the static repro positive PASSes both targets
# ---------------------------------------------------------------------------


def test_committed_synthetic_fixture_passes_both_targets():
    rollout = load_json(_FIXTURE)
    assert_rollout_leakage_clean(rollout)
    assert rollout["backend"] == "rlbench_external"
    assert rollout["diagnostics"]["physicalValidity"] is None
    for path in (_TERMINAL, _REL_EVENT):
        case = _verify(path, rollout)
        assert case["passed"] is True, (path.name, case["hardMismatches"])


# ---------------------------------------------------------------------------
# Committed LIVE evidence — 9 real RLBench PutItemInDrawer demos (Runpod/CoppeliaSim,
# 2026-06-16, 3 per variation). Reproducible from a clean clone with NO RLBench: the
# frozen verifier re-judges the committed rollouts. This is the Phase 2F-4 headline result.
# ---------------------------------------------------------------------------

_LIVE_DIR = _REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260616_put_item"


def test_committed_live_capture_9_of_9_terminal_inside():
    paths = sorted(_LIVE_DIR.glob("*.rollout.json"))
    assert len(paths) == 9, [p.name for p in paths]
    far = near = 0
    for p in paths:
        r = load_json(p)
        assert_rollout_leakage_clean(r)
        assert r["backend"] == "rlbench_external", p.name
        assert r["diagnostics"]["physicalValidity"] is None, p.name      # external = physics-unverified
        robot = extract_robot_csg(r)
        assert any(e["eventKind"] == "CONTAINMENT_CHANGE" for e in robot.get("events", [])), p.name
        # every real demo ends INSIDE -> terminal_only PASSes 9/9, leakage-clean
        term = _verify(_TERMINAL, r)
        assert term["passed"] is True and term["leakageClean"] is True, (p.name, term["hardMismatches"])

        # the strong target is chosen by the demo's OBSERVED initial relation: RLBench carries
        # the item in from across the table for the bottom/middle drawers (FAR_FROM) and from
        # nearby for the top drawer (NEAR). Each demo PASSes its matched target and FAILs the
        # other on initial_state alone — the verifier reads the initial condition faithfully.
        firsts = [rel["relation"] for rel in robot.get("relations", []) if rel["relationId"].endswith("_first")]
        is_far = firsts == ["FAR_FROM"]
        matched, other = (_PLACED, _REL_EVENT) if is_far else (_REL_EVENT, _PLACED)
        assert _verify(matched, r)["passed"] is True, (p.name, firsts, "matched strong target should PASS")
        oc = _verify(other, r)
        assert oc["passed"] is False and oc["hardMismatches"] == ["initial_state"], (p.name, oc["hardMismatches"])
        far += int(is_far)
        near += int(not is_far)
    # observed split across the committed capture: 6 placed-from-outside (bottom+middle) + 3 near (top)
    assert (far, near) == (6, 3), (far, near)



# ---------------------------------------------------------------------------
# Live record (skip-gated) — needs RLBench/PyRep + a live CoppeliaSim
# ---------------------------------------------------------------------------

_RLBENCH_AVAILABLE = (
    importlib.util.find_spec("rlbench") is not None
    and importlib.util.find_spec("pyrep") is not None
)
_LIVE_ENABLED = _RLBENCH_AVAILABLE or os.environ.get("RLBENCH_PILOT_LIVE") == "1"


@pytest.mark.skipif(not _LIVE_ENABLED, reason="RLBench/PyRep + live CoppeliaSim required")
@pytest.mark.parametrize("variation", ["bottom", "middle", "top"])
def test_live_record_put_item_in_drawer_passes_and_confuses(variation):  # pragma: no cover - live only
    from pilots.rlbench import record_put_item_in_drawer as rec

    records = rec.record_variation(variation, amount=1, headless=True)
    assert records, "expected at least one recorded demo"
    rollout = rec.build_rollout(records[0])
    assert_rollout_leakage_clean(rollout)

    rele = verify_external_rollout(load_json(_REL_EVENT), rollout, case_name="rlbench_oic")
    assert rele["leakageClean"] is True
    assert rele["physicalValidity"] is None
    conf = external_confusion_report(rollout, load_gold_targets(_GOLD_DIR), expected_case="put_cube_in_tray")
    assert conf["unexpectedOffTaskPasses"] == []
