"""RLBench articulation-event target — Result D, one honest step beyond value-only.

The value-only target (`Result B/C`) asserts only the *terminal* drawer extension. This
target adds the next increment that the FROZEN matcher actually enforces. Concretely it
enforces three things:
  * terminal extension ≈ `0.234 m` (hard ARTICULATION_GOAL → `goal_satisfaction`, numeric);
  * the drawer's articulation **increased** rather than stayed flat
    (`articulation_transitions` — DIRECTION only, see below);
  * an `ARTICULATION_CHANGE` event is present (`event_presence`).

It deliberately does **not** add contacts, handle contact, `CONTACT_BEGIN`, temporal edges,
or any contact-before-motion order: the adapter has no honest RLBench evidence for who/what
caused the motion (deferred to a later target).

What the matcher does NOT enforce — and this suite proves it — is the *numeric initial
value*. `csg.matcher._probe_articulation_transitions` reduces every transition to
`(object, jointKind, valueKind, INCREASE/DECREASE/FLAT)` (direction only, numeric endpoints
discarded), and the only numeric articulation check (`goal_satisfaction`) reads the robot's
*terminal* value, never the initial. So the target's `0.0` initial objectState and the
`0.0 → 0.234` fromState/toState are honest **authoring**, not verifier-enforced: a trace
that starts at `0.10` and increases to `0.234` PASSes too (see
`test_articulation_event_does_not_pin_initial_value`). The honest claim is "increased to the
calibrated value + event present", not "started at 0.0".

What this buys, made executable:
  * all 9 fresh rerun demos still PASS leakage-clean, `physicalValidity null`, non-vacuous:
    `goal_satisfaction`, `articulation_transitions`, and `event_presence` at support 1,
    while `event_order` stays support 0 (a single event has no pair to order against).
  * it is **strictly stronger** than value-only: a "born-open" drawer (every frame already
    at `0.234`, FLAT — no change) PASSes value-only but FAILs this target on
    `articulation_transitions` + `event_presence` — a terminal value alone is no longer
    enough; the articulation must have *increased*.
  * kinematic mutations FAIL: below/above the window FAIL `goal_satisfaction`; a flat or
    opened-then-closed trajectory additionally FAILs the added event/transition semantics.
  * a mis-calibrated target FAILs the real demos; a leaky trace is rejected before the
    matcher can PASS.

Runs with NO RLBench installed; `csg/` is never touched.
"""
import copy
from pathlib import Path

import pytest

from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg

from pilots.rlbench.adapter import ExternalTraceLeakage, assert_rollout_leakage_clean
from pilots.rlbench.run_external import (
    external_confusion_report,
    load_gold_targets,
    verify_external_rollout,
)

_REPO = Path(__file__).resolve().parents[1]
_GOLD_DIR = _REPO / "gold_tests"
_TARGETS_DIR = _REPO / "pilots" / "rlbench" / "targets"
_VALUE_ONLY_TARGET = _TARGETS_DIR / "open_drawer_rlbench_value_only.json"
_ARTICULATION_EVENT_TARGET = _TARGETS_DIR / "open_drawer_rlbench_articulation_event.json"
_RERUN_FIXTURE_DIR = _REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260614_rerun"

_RLBENCH_VALUE = 0.234
_ENFORCED_TOL = 0.05


def _rerun_paths():
    paths = sorted(_RERUN_FIXTURE_DIR.glob("*.rollout.json"))
    assert len(paths) == 9, [p.name for p in paths]
    return paths


_RERUN_PATHS = _rerun_paths()
_RERUN_IDS = [p.name[: -len(".rollout.json")] for p in _RERUN_PATHS]


def _goal_articulation(target):
    return target["plannerView"]["stages"][0]["goalConstraints"][0]["articulation"]


# Mutation helpers — touch ONLY float articulation values under body_000 (leakage-clean).


def _ramp(rollout, final):
    r = copy.deepcopy(rollout)
    n = len(r["frames"])
    for i, f in enumerate(r["frames"]):
        f["articulation"]["body_000"] = final * (i / (n - 1)) if n > 1 else final
    return r


def _ramp_from(rollout, start, final):
    """Deep copy whose body_000 articulation ramps linearly from a NONZERO ``start`` to
    ``final`` — used to show the frozen matcher does not pin the initial value."""
    r = copy.deepcopy(rollout)
    n = len(r["frames"])
    for i, f in enumerate(r["frames"]):
        f["articulation"]["body_000"] = start + (final - start) * (i / (n - 1)) if n > 1 else final
    return r


def _flat(rollout, value):
    r = copy.deepcopy(rollout)
    for f in r["frames"]:
        f["articulation"]["body_000"] = value
    return r


def _reverse(rollout, peak):
    r = copy.deepcopy(rollout)
    n = len(r["frames"])
    mid = n // 2
    for i, f in enumerate(r["frames"]):
        if i <= mid:
            f["articulation"]["body_000"] = peak * (i / mid) if mid else peak
        else:
            f["articulation"]["body_000"] = peak * ((n - 1 - i) / (n - 1 - mid))
    return r


# ---------------------------------------------------------------------------
# Target structure — calibrated value, one articulation event, two states, no contact/order
# ---------------------------------------------------------------------------


def test_articulation_event_target_structure():
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    # the same calibrated terminal value as value-only, expressed in goal + terminal state
    # + the event's to-state — internally consistent at 0.234.
    assert _goal_articulation(ae)["targetJointValue"] == _RLBENCH_VALUE
    assert _goal_articulation(ae)["valueKind"] == "EXTENSION_M"
    goals = ae["plannerView"]["stages"][0]["goalConstraints"]
    assert [g["kind"] for g in goals] == ["ARTICULATION_GOAL"]
    assert goals[0]["hard"] is True

    # exactly two articulation states, authored near-closed 0.0 -> calibrated 0.234.
    # NOTE: these numeric values are the target's AUTHORING. The frozen matcher reduces
    # transitions to a direction (INCREASE/DECREASE/FLAT) and only checks the terminal goal
    # numerically, so the 0.0 initial value is NOT verifier-enforced — see
    # test_articulation_event_does_not_pin_initial_value.
    states = ae["objectStates"]
    assert [s["articulation"]["jointValue"] for s in states] == [0.0, _RLBENCH_VALUE]
    assert all(s["objectId"] == "h_drawer" for s in states)

    # exactly one ARTICULATION_CHANGE event; its authored transition is 0.0 -> 0.234 (again,
    # only the increase DIRECTION + the event's presence reach the verifier, not the endpoints).
    events = ae["events"]
    assert [e["eventKind"] for e in events] == ["ARTICULATION_CHANGE"]
    trans = events[0]["observedDeltas"][0]["articulationTransition"]
    assert trans["fromState"]["jointValue"] == 0.0
    assert trans["toState"]["jointValue"] == _RLBENCH_VALUE

    # honest deferral: NO contact / contact-order machinery anywhere in the file.
    assert "contacts" not in ae
    assert "temporalEdges" not in ae
    assert all(e["eventKind"] != "CONTACT_BEGIN" for e in events)


def test_articulation_event_target_is_not_a_gold_task():
    # Like value-only, this is a pilot diagnostic and must NOT have leaked into gold_tests/.
    assert not (_GOLD_DIR / "open_drawer_rlbench_articulation_event").exists()
    assert load_json(_ARTICULATION_EVENT_TARGET)["pilotMetadata"]["diagnostic"] == "articulation-event"


# ---------------------------------------------------------------------------
# Positive — all 9 real demos PASS, leakage-clean, with the intended probe supports
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _RERUN_PATHS, ids=_RERUN_IDS)
def test_articulation_event_passes_all_real_demos_non_vacuously(path):
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    rollout = load_json(path)
    assert_rollout_leakage_clean(rollout)
    case = verify_external_rollout(ae, rollout, case_name="open_drawer_rlbench_articulation_event")
    assert case["passed"] is True, case["hardMismatches"]
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    assert case["hardMismatches"] == []

    res = match(ae, extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    # the three asserted semantics carry real support and agree: terminal value
    # (goal_satisfaction, numeric), an articulation INCREASE (articulation_transitions —
    # direction only), and an ARTICULATION_CHANGE event present (event_presence) ...
    assert res.probe_support["goal_satisfaction"] == 1
    assert res.probe_support["articulation_transitions"] == 1
    assert res.probe_support["event_presence"] == 1
    for probe in ("goal_satisfaction", "articulation_transitions", "event_presence"):
        assert res.probe_agreement[probe] is True, probe
    # ... while event ORDER is deliberately unsupported (one event, no pair to order).
    assert res.probe_support["event_order"] == 0


# ---------------------------------------------------------------------------
# Strictly stronger than value-only — terminal value alone is no longer enough
# ---------------------------------------------------------------------------


def test_articulation_event_is_strictly_stronger_than_value_only():
    # A "born-open" drawer: every frame already at 0.234, so the trajectory is FLAT — no
    # INCREASE. Its terminal value satisfies value-only, but the articulation-event target
    # rejects it because no articulation increase/change occurred (NOT because of any initial
    # value — see test_articulation_event_does_not_pin_initial_value). The added
    # event/transition semantics are load-bearing.
    born_open = _flat(load_json(_RERUN_PATHS[0]), _RLBENCH_VALUE)
    assert_rollout_leakage_clean(born_open)  # only float values changed -> still clean

    vo = verify_external_rollout(load_json(_VALUE_ONLY_TARGET), born_open, case_name="vo")
    ae = verify_external_rollout(load_json(_ARTICULATION_EVENT_TARGET), born_open, case_name="ae")
    assert vo["passed"] is True, vo["hardMismatches"]            # value-only: terminal value ok
    assert ae["passed"] is False                                # articulation-event: no increase
    assert "articulation_transitions" in ae["hardMismatches"]
    assert "event_presence" in ae["hardMismatches"]
    assert "goal_satisfaction" not in ae["hardMismatches"]      # terminal value still matched


@pytest.mark.parametrize("start", [0.10, 0.15])
def test_articulation_event_does_not_pin_initial_value(start):
    # Honest-limitation tripwire (locks in what Result D does NOT claim). The frozen matcher
    # reduces articulation transitions to a DIRECTION
    # (csg.matcher._probe_articulation_transitions: INCREASE/DECREASE/FLAT) and only checks the
    # TERMINAL value numerically (goal_satisfaction). So a trace that starts already part-open
    # (0.10 / 0.15) and INCREASES to the calibrated 0.234 still PASSes — the target's authored
    # 0.0 initial state is documentary, not verifier-enforced. This stops the "started at 0.0"
    # overclaim from creeping back.
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    for path in _RERUN_PATHS:
        nonzero_start = _ramp_from(load_json(path), start, _RLBENCH_VALUE)
        assert_rollout_leakage_clean(nonzero_start)  # only float values changed -> still clean
        case = verify_external_rollout(ae, nonzero_start, case_name="ae")
        assert case["passed"] is True, (start, path.name, case["hardMismatches"])
        res = match(ae, extract_robot_csg(nonzero_start), MatcherConfig())
        for probe in ("goal_satisfaction", "articulation_transitions", "event_presence"):
            assert res.probe_agreement[probe] is True, (start, path.name, probe)


# ---------------------------------------------------------------------------
# Kinematic mutations — wrong kinematics FAIL (leakage-clean)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,final", [("below_window", 0.18), ("above_window", 0.30)])
def test_terminal_out_of_window_fails_goal_satisfaction(name, final):
    # Below/above the enforced 0.234 +/- 0.05 window: the drawer still CHANGES (so the event
    # and transition register), but the calibrated goal is missed.
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    for path in _RERUN_PATHS:
        bad = _ramp(load_json(path), final)
        assert_rollout_leakage_clean(bad)
        case = verify_external_rollout(ae, bad, case_name="ae")
        assert case["passed"] is False, (name, path.name)
        assert "goal_satisfaction" in case["hardMismatches"], (name, path.name, case["hardMismatches"])
        assert case["leakageClean"] is True


@pytest.mark.parametrize("name,mutate", [
    ("never_opens", lambda r: _flat(r, 0.0)),
    ("opens_then_closes", lambda r: _reverse(r, 0.235)),
])
def test_no_or_reversed_change_fails_goal_and_added_event_semantics(name, mutate):
    # A flat (never opens) or opened-then-closed trajectory fails the calibrated goal AND
    # the semantics this target ADDS over value-only: no honest 0.0 -> 0.234 change occurred.
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    for path in _RERUN_PATHS:
        bad = mutate(load_json(path))
        assert_rollout_leakage_clean(bad)
        case = verify_external_rollout(ae, bad, case_name="ae")
        assert case["passed"] is False, (name, path.name)
        for probe in ("goal_satisfaction", "articulation_transitions", "event_presence"):
            assert probe in case["hardMismatches"], (name, path.name, probe, case["hardMismatches"])


# ---------------------------------------------------------------------------
# Target calibration — the 0.234 m value is load-bearing
# ---------------------------------------------------------------------------


def _retargeted(value):
    """A coherent miscalibrated copy: goal + terminal state + event to-state all at `value`."""
    ae = copy.deepcopy(load_json(_ARTICULATION_EVENT_TARGET))
    _goal_articulation(ae)["targetJointValue"] = value
    ae["objectStates"][1]["articulation"]["jointValue"] = value
    ae["events"][0]["observedDeltas"][0]["articulationTransition"]["toState"]["jointValue"] = value
    return ae


def test_retargeted_off_rlbench_value_fails_all_real_demos():
    off = _retargeted(0.18)
    for path in _RERUN_PATHS:
        case = verify_external_rollout(off, load_json(path), case_name="ae_off")
        assert case["passed"] is False, path.name
        assert "goal_satisfaction" in case["hardMismatches"], path.name


def test_calibrated_value_passes_all_real_demos():
    ae = load_json(_ARTICULATION_EVENT_TARGET)
    assert _goal_articulation(ae)["targetJointValue"] == _RLBENCH_VALUE
    for path in _RERUN_PATHS:
        assert verify_external_rollout(ae, load_json(path), case_name="ae")["passed"] is True, path.name


def test_enforced_tolerance_is_the_window_this_target_relies_on():
    assert MatcherConfig().articulation_tol == _ENFORCED_TOL
    # The enforced mechanics: a transition registers as INCREASE when last - first > tol, and
    # goal_satisfaction passes when the robot's terminal value is within tol of 0.234. The real
    # demos ramp ~0 -> 0.234 (delta >> tol), so the increase registers and the goal is met; pin
    # the tolerance so a future widening that blurred increase/flat or the goal window fails here.
    assert abs(_RLBENCH_VALUE - 0.0) > _ENFORCED_TOL


# ---------------------------------------------------------------------------
# Leakage — a leaky trace is rejected before the matcher can PASS
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,mutate,match_re", [
    ("targetCsg", lambda r: r.__setitem__("targetCsg", {"leaked": True}), "forbidden"),
    ("objectIdMap", lambda r: r.__setitem__("objectIdMap", {"h_drawer": "body_000"}), "objectIdMap"),
    ("body_field", lambda r: r["sceneBodies"][0].__setitem__("categoryLabel", "drawer"), "non-whitelisted"),
])
def test_leaky_trace_is_rejected_before_matcher_success(name, mutate, match_re):
    base = load_json(_RERUN_PATHS[0])
    assert verify_external_rollout(load_json(_ARTICULATION_EVENT_TARGET), base, case_name="ae")["passed"] is True
    bad = copy.deepcopy(base)
    mutate(bad)
    with pytest.raises(ExternalTraceLeakage, match=match_re):
        assert_rollout_leakage_clean(bad)
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(load_json(_ARTICULATION_EVENT_TARGET), bad, case_name="ae")


def test_articulation_event_demo_matches_no_off_task_gold_target():
    # The articulation-event target is a pilot diagnostic, not a gold task, so cross-task
    # confusion is unchanged: a real demo still matches NO gold target.
    conf = external_confusion_report(
        load_json(_RERUN_PATHS[0]), load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    assert conf["unexpectedOffTaskPasses"] == []
    assert conf["passes"] == []
