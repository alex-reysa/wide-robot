"""RLBench value-only NEGATIVE / mutation suite — proof the calibration is not too permissive.

Companion to ``tests/test_rlbench_pilot.py``. That file establishes the positive seam
(the value-only target ACCEPTS real RLBench ``OpenDrawer`` traces; the gold target
REJECTS them; both leakage-clean). This file is the adversary's question — *"is the
calibrated value-only target too permissive?"* — made executable against the committed
9-demo reproducibility rerun (``fixtures/live_runpod_20260614_rerun/``), so a clean
clone runs it with **no** RLBench installed:

  * **POSITIVE (9/9)** — each real trace PASSes value-only, leakage-clean,
    ``physicalValidity null``, with ``goal_satisfaction`` carrying real support and the
    deferred event/order/transition probes carrying ZERO support (the PASS is the
    terminal value, never vacuity).
  * **PRESERVED NEGATIVE (9/9)** — the gold ``open_drawer`` target still FAILs every
    real trace leakage-clean, hard mismatches exactly ``{event_order, goal_satisfaction}``.
  * **OFF-TASK (9/9)** — no real trace matches any off-task gold target (no unexpected
    off-task PASS; every non-``open_drawer`` target is a clean FAIL).
  * **KINEMATIC MUTATIONS** — a *leakage-clean* trace whose terminal drawer articulation
    is moved out of the calibrated window (below it, above it, flat-never-opens, or
    opened-then-closed) FAILs ``goal_satisfaction`` — the FAIL is a genuine matcher
    verdict, not a gate rejection.
  * **LEAKAGE MUTATIONS** — a trace that smuggles target authoring (``targetCsg`` /
    ``plannerView`` / non-neutral ``objectIdMap`` / non-neutral per-frame ``articulation``
    key / non-whitelisted body field) is rejected at the door, BEFORE the matcher can
    return PASS, by both the rollout-level gate and the full external-entry path.
  * **TARGET CALIBRATION** — a value-only target retargeted off the RLBench value (to the
    gold ``0.18 m``) FAILs the real traces; only the calibrated ``0.234 m`` accepts them.

``csg/`` is never touched — every mutation lives in ``pilots/`` inputs + test memory.
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
_GOLD_TARGET = _GOLD_DIR / "open_drawer" / "target.json"
_VALUE_ONLY_TARGET = _REPO / "pilots" / "rlbench" / "targets" / "open_drawer_rlbench_value_only.json"
_RERUN_FIXTURE_DIR = _REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260614_rerun"

# The RLBench-calibrated articulation goal and the GLOBALLY-enforced tolerance window
# (MatcherConfig.articulation_tol). Every "below/above window" mutation is calibrated to
# this window; the window itself is pinned in a dedicated test below.
_RLBENCH_VALUE = 0.234
_ENFORCED_TOL = 0.05


def _rerun_paths():
    """The committed 9-demo evidence (3 fresh demos x bottom/middle/top). Asserting the
    count at import makes a clean checkout that forgot to promote the fixtures fail loudly
    at collection, not silently run an empty parametrization."""
    paths = sorted(_RERUN_FIXTURE_DIR.glob("*.rollout.json"))
    assert len(paths) == 9, [p.name for p in paths]
    return paths


_RERUN_PATHS = _rerun_paths()
_RERUN_IDS = [p.name[: -len(".rollout.json")] for p in _RERUN_PATHS]


# ---------------------------------------------------------------------------
# Mutation helpers — rewrite ONLY float articulation values under body_000, so a
# kinematically-wrong trace stays leakage-clean (the FAIL must be the matcher's).
# ---------------------------------------------------------------------------


def _ramp_articulation(rollout, final):
    """Deep copy whose body_000 articulation ramps linearly ``0 -> final``."""
    r = copy.deepcopy(rollout)
    frames = r["frames"]
    n = len(frames)
    for i, f in enumerate(frames):
        f["articulation"]["body_000"] = final * (i / (n - 1)) if n > 1 else final
    return r


def _flatten_articulation(rollout, value=0.0):
    """Deep copy whose drawer never moves (every frame at ``value``)."""
    r = copy.deepcopy(rollout)
    for f in r["frames"]:
        f["articulation"]["body_000"] = value
    return r


def _reverse_articulation(rollout, peak):
    """Open to ``peak`` then close back to 0. ``goal_satisfaction`` reads the *terminal*
    value (matcher ``_articulation_endpoints`` takes the last frame), which returns to 0 —
    the drawer transiently opened but did not END open."""
    r = copy.deepcopy(rollout)
    frames = r["frames"]
    n = len(frames)
    mid = n // 2
    for i, f in enumerate(frames):
        if i <= mid:
            f["articulation"]["body_000"] = peak * (i / mid) if mid else peak
        else:
            f["articulation"]["body_000"] = peak * ((n - 1 - i) / (n - 1 - mid))
    return r


# ---------------------------------------------------------------------------
# POSITIVE — all 9 real demos PASS value-only, leakage-clean, non-vacuously
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _RERUN_PATHS, ids=_RERUN_IDS)
def test_rerun_demo_passes_value_only_leakage_clean_and_non_vacuous(path):
    rollout = load_json(path)
    assert_rollout_leakage_clean(rollout)  # the evidence is clean before any verdict
    vo = load_json(_VALUE_ONLY_TARGET)
    case = verify_external_rollout(vo, rollout, case_name="open_drawer_rlbench_value_only")
    assert case["passed"] is True, case["hardMismatches"]
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    assert case["hardMismatches"] == []
    # The PASS rests on the terminal value, not accept-all: goal_satisfaction carries the
    # single HARD goal's support and agrees, while the deferred probes carry zero support.
    res = match(vo, extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    assert res.probe_agreement["goal_satisfaction"] is True
    assert res.probe_support["goal_satisfaction"] == 1
    for deferred in ("event_presence", "event_order", "articulation_transitions"):
        assert res.probe_support[deferred] == 0, deferred


# ---------------------------------------------------------------------------
# PRESERVED NEGATIVE — the gold target still FAILs all 9, leakage-clean
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _RERUN_PATHS, ids=_RERUN_IDS)
def test_rerun_demo_fails_gold_target_leakage_clean(path):
    # Result A must hold for every fresh demo: the real RLBench drawer opens past the gold
    # goal+tolerance and lacks the gold's CONTACT_BEGIN->ARTICULATION_CHANGE order. An
    # accidental PASS here would mean the seam silently drifted permissive.
    case = verify_external_rollout(load_json(_GOLD_TARGET), load_json(path), case_name="open_drawer")
    assert case["passed"] is False
    assert case["leakageClean"] is True
    assert case["physicalValidity"] is None
    assert set(case["hardMismatches"]) == {"event_order", "goal_satisfaction"}


# ---------------------------------------------------------------------------
# OFF-TASK — no real demo matches any off-task gold target
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _RERUN_PATHS, ids=_RERUN_IDS)
def test_rerun_demo_matches_no_off_task_gold_target(path):
    conf = external_confusion_report(load_json(path), load_gold_targets(_GOLD_DIR), expected_case="open_drawer")
    # The pilot's single biggest risk is a too-easy off-task PASS; this pins it shut on the
    # real evidence (not just the synthetic stand-in) for every fresh demo.
    assert conf["unexpectedOffTaskPasses"] == []
    assert conf["passes"] == []  # the real drawer trace matches NO gold target at all
    for name, passed in conf["results"].items():
        if name != "open_drawer":
            assert passed is False, name


# ---------------------------------------------------------------------------
# KINEMATIC MUTATIONS — wrong terminal value FAILs goal_satisfaction (leakage-clean)
# ---------------------------------------------------------------------------

_KINEMATIC_MUTATIONS = [
    # below the window floor (0.184): the drawer barely opens.
    ("below_window", lambda r: _ramp_articulation(r, 0.18)),
    # above the window ceiling (0.284): the drawer over-extends.
    ("above_window", lambda r: _ramp_articulation(r, 0.30)),
    # never opens: a flat, closed drawer.
    ("never_opens", lambda r: _flatten_articulation(r, 0.0)),
    # opens then closes: transient peak above the value, terminal back to 0.
    ("opens_then_closes", lambda r: _reverse_articulation(r, 0.235)),
]


@pytest.mark.parametrize("name,mutate", _KINEMATIC_MUTATIONS, ids=[n for n, _ in _KINEMATIC_MUTATIONS])
def test_kinematic_mutation_fails_value_only_goal_satisfaction(name, mutate):
    vo = load_json(_VALUE_ONLY_TARGET)
    # Contrast guard: the unmutated trace PASSes, so each FAIL below is the mutation's doing.
    assert verify_external_rollout(vo, load_json(_RERUN_PATHS[0]), case_name="vo")["passed"] is True
    for path in _RERUN_PATHS:
        bad = mutate(load_json(path))
        # only float articulation values changed -> still leakage-clean, so the FAIL is a
        # genuine matcher verdict, not a gate rejection sneaking in.
        assert_rollout_leakage_clean(bad)
        case = verify_external_rollout(vo, bad, case_name="open_drawer_rlbench_value_only")
        assert case["passed"] is False, (name, path.name)
        assert case["leakageClean"] is True, (name, path.name)
        assert "goal_satisfaction" in case["hardMismatches"], (name, path.name, case["hardMismatches"])


# ---------------------------------------------------------------------------
# LEAKAGE MUTATIONS — smuggled authoring is rejected BEFORE the matcher can PASS
# ---------------------------------------------------------------------------

_REPRESENTATIVE = _RERUN_FIXTURE_DIR / "open_drawer_bottom_demo00.rollout.json"

_LEAKAGE_MUTATIONS = [
    # forbidden top-level target-authoring keys (the leakage_report set).
    ("targetCsg", lambda r: r.__setitem__("targetCsg", {"leaked": "target authoring"}), "forbidden"),
    ("plannerView", lambda r: r.__setitem__("plannerView", {"stages": []}), "forbidden"),
    # non-neutral objectIdMap: an external trace has no target identity to map.
    ("objectIdMap", lambda r: r.__setitem__("objectIdMap", {"h_drawer": "body_000"}), "objectIdMap"),
    # per-frame articulation keyed by an RLBench joint name instead of body_NNN.
    ("frame_articulation_key",
     lambda r: r["frames"][0].__setitem__("articulation", {"drawer_joint_bottom": 0.1}), "articulation"),
    # non-whitelisted body field that could encode the target's identity.
    ("body_field", lambda r: r["sceneBodies"][0].__setitem__("categoryLabel", "drawer"), "non-whitelisted"),
]


@pytest.mark.parametrize("name,mutate,match_re", _LEAKAGE_MUTATIONS, ids=[n for n, _, _ in _LEAKAGE_MUTATIONS])
def test_leakage_mutation_is_rejected_before_matcher_success(name, mutate, match_re):
    base = load_json(_REPRESENTATIVE)
    # The unmutated representative is clean and PASSes value-only, so the rejection below
    # is the injected leak — not a pre-existing fixture problem.
    assert_rollout_leakage_clean(base)
    assert verify_external_rollout(load_json(_VALUE_ONLY_TARGET), base, case_name="vo")["passed"] is True

    bad = copy.deepcopy(base)
    mutate(bad)
    # First line of defence: the rollout-level gate names the specific leak.
    with pytest.raises(ExternalTraceLeakage, match=match_re):
        assert_rollout_leakage_clean(bad)
    # The full external-entry path refuses it too, BEFORE the matcher can return PASS.
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(load_json(_VALUE_ONLY_TARGET), bad, case_name="vo")
    # Confusion is likewise gated — it must never run on a leaky trace.
    with pytest.raises(ExternalTraceLeakage):
        external_confusion_report(bad, load_gold_targets(_GOLD_DIR), expected_case="open_drawer")


# ---------------------------------------------------------------------------
# TARGET CALIBRATION — the 0.234 m value is load-bearing, not accept-anything
# ---------------------------------------------------------------------------


def _retargeted_value_only(target_value):
    vo = copy.deepcopy(load_json(_VALUE_ONLY_TARGET))
    vo["plannerView"]["stages"][0]["goalConstraints"][0]["articulation"]["targetJointValue"] = target_value
    return vo


def test_value_only_retargeted_off_rlbench_value_fails_all_real_demos():
    # A value-only target retargeted to the gold 0.18 m (which the real ~0.234 m demos do
    # not satisfy) FAILs all 9 on goal_satisfaction: the 0.234 m acceptance is the RLBench
    # calibration, not an accept-anything goal.
    off = _retargeted_value_only(0.18)
    for path in _RERUN_PATHS:
        case = verify_external_rollout(off, load_json(path), case_name="vo_off")
        assert case["passed"] is False, path.name
        assert "goal_satisfaction" in case["hardMismatches"], path.name


def test_value_only_calibrated_value_passes_all_real_demos():
    # The flip side of the calibration: only the committed 0.234 m value accepts the demos.
    vo = load_json(_VALUE_ONLY_TARGET)
    goal = vo["plannerView"]["stages"][0]["goalConstraints"][0]["articulation"]
    assert goal["targetJointValue"] == _RLBENCH_VALUE
    for path in _RERUN_PATHS:
        case = verify_external_rollout(vo, load_json(path), case_name="vo")
        assert case["passed"] is True, (path.name, case["hardMismatches"])


def test_enforced_articulation_tolerance_defines_the_window_this_suite_relies_on():
    # Every "below/above window" mutation is calibrated to the GLOBALLY-enforced tolerance.
    # Pin it so a future MatcherConfig change that widened the window (turning the boundary
    # mutations into false PASSes) fails HERE, at the constant the suite depends on.
    assert MatcherConfig().articulation_tol == _ENFORCED_TOL
    lo, hi = _RLBENCH_VALUE - _ENFORCED_TOL, _RLBENCH_VALUE + _ENFORCED_TOL
    assert (lo, hi) == pytest.approx((0.184, 0.284))
    # The two boundary mutations (0.18, 0.30) sit OUTSIDE the window, so they must FAIL.
    assert 0.18 < lo and 0.30 > hi
