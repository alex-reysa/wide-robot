#!/usr/bin/env python3
"""Tests for the cross-source "One Task, Four Worlds" report (experiments/cross_source_oic).

Every assertion is pinned to FRESHLY REGENERATED values (the four legs recomputed live through
the frozen verifier core), never to hand-copied numbers — so the test fails if the verifier's
behaviour drifts. Pure-Python: the whole report builds with no MuJoCo / RLBench / cv2 installed
(if any were required, importing/running the legs here would fail), which this suite exercises.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

import csg.matcher
import pilots.external_verify as ev
from experiments.cross_source_oic.legs import mujoco_leg, rlbench_leg, sony_leg, rh20t_leg, all_legs
from experiments.cross_source_oic.target_equivalence import build_target_equivalence
from scripts.build_cross_source_report import build_record

REPO = Path(__file__).resolve().parents[1]
EXP_DIR = REPO / "experiments" / "cross_source_oic"


# --------------------------------------------------------------------------------------
# the frozen core is genuinely shared
# --------------------------------------------------------------------------------------

def test_external_and_internal_share_the_same_frozen_match():
    # the external driver imports the exact csg.matcher.match the internal benchmark uses.
    assert ev.match is csg.matcher.match
    assert ev.extract_robot_csg.__module__ == "csg.rollout_extract"


# --------------------------------------------------------------------------------------
# per-world headline (regenerated live)
# --------------------------------------------------------------------------------------

def test_mujoco_leg_passes_with_real_physical_validity():
    leg = mujoco_leg()
    a = leg["aggregate"]
    assert a["nSuccess"] == 1 and a["successStructuredCertify"] == 1
    assert a["physicalValidity"] is True            # physics genuinely re-checked
    assert a["nativeGoldPass"] is True              # full pick-place gold target PASSes too
    corpus = a["acceptanceCorpus"]
    assert corpus["sabotagesFailed"] == 4 and corpus["successPass"] == 1
    assert corpus["allMatchExpected"] is True       # matcher matches committed expected.json
    assert a["failureStructuredFalsePass"] == 0


def test_rlbench_leg_nine_of_nine_partitioned():
    leg = rlbench_leg()
    a = leg["aggregate"]
    assert a["nSuccess"] == 9 and a["successTerminalPass"] == 9
    assert a["successStructuredCertify"] == 9
    assert a["relationEventPass"] == 3 and a["placedFromOutsidePass"] == 6
    assert a["wrongTierRejections"] == 9            # real discrimination on a success-only world
    assert a["physicalValidity"] is None            # external trace, physics-unverified
    assert a["failureStructuredFalsePass"] == 0


def test_sony_leg_zero_structured_false_pass():
    leg = sony_leg()
    a = leg["aggregate"]
    assert a["nClips"] == 78
    assert a["nSuccess"] == 38 and a["nFailure"] == 40
    assert a["successStructuredCertify"] == 27 and a["successUncertain"] == 5
    # the headline safety number: no non-success clip is structured-certified.
    assert a["failureStructuredFalsePass"] == 0
    # born-inside controls pass ONLY the weak terminal tier — that is why the structured tier exists.
    assert a["failureTerminalPass"] == 3
    assert a["physicalValidity"] is None
    assert leg["evidenceThresholds"] == {"max_consecutive_missing": 30, "max_dropout_frac": 0.35}


def test_rh20t_leg_positive_passes_negative_fails():
    leg = rh20t_leg()
    a = leg["aggregate"]
    assert a["nSuccess"] == 1 and a["nFailure"] == 1
    assert a["successStructuredCertify"] == 1        # real pen->holder put-in
    assert a["failureCorrectlyRejected"] == 1        # derived near-not-inside negative FAILs
    assert a["failureStructuredFalsePass"] == 0
    assert a["physicalValidity"] is None


# --------------------------------------------------------------------------------------
# the cross-source claim
# --------------------------------------------------------------------------------------

def test_zero_false_pass_in_every_world():
    for leg in all_legs():
        assert leg["aggregate"]["failureStructuredFalsePass"] == 0, leg["worldKey"]
        assert leg["aggregate"]["noLeakageViolation"] is True, leg["worldKey"]


def test_totals_no_false_pass_and_leakage_clean():
    rec = build_record()
    t = rec["totals"]
    assert t["nonSuccessStructuredFalsePass"] == 0
    assert t["noLeakageViolationAnyWorld"] is True
    # exactly one world reports physics-validated; the other three are physics-unverified (null).
    physics_true = [L for L in rec["worlds"] if L["aggregate"].get("physicalValidity") is True]
    assert [L["worldKey"] for L in physics_true] == ["mujoco"]


# --------------------------------------------------------------------------------------
# same semantic task, not the same file
# --------------------------------------------------------------------------------------

def test_per_source_target_cards_share_one_enforced_core():
    te = build_target_equivalence()
    assert te["allExternalTiersIdentical"] is True
    for tier, t in te["perTier"].items():
        assert t["allIdentical"] is True, tier
        assert t["distinctSignatureCount"] == 1, tier
        assert t["pairwiseDiffs"] == [], tier
    ts = te["tierStrengthening"]
    assert all(ts.values()), ts
    assert te["internalSim"]["goalCoreMatchesSharedCore"] is True


def test_target_cards_are_actually_distinct_files():
    # guard the honesty point: the cards are NOT one shared file — different graphIds.
    import csg.common as common
    gids = set()
    for src in ("rlbench", "real_camera", "rh20t"):
        p = REPO / "pilots" / src / "targets" / "object_inside_container_terminal_only.json"
        gids.add(common.load_json(p)["graphId"])
    assert len(gids) == 3  # three distinct graphIds, one shared enforced core


def _base_card():
    import csg.common as common
    return common.load_json(REPO / "pilots" / "real_camera" / "targets"
                            / "object_inside_container_relation_event.json")


def test_signature_catches_every_enforced_field_mutation():
    # The over-strip guard: mutating ANY matcher-enforced field must change the canonical
    # signature, so the equivalence proof would CATCH a real difference (never launder it).
    from experiments.cross_source_oic.target_equivalence import canonical_signature, _sig_key
    base = _base_card()
    base_key = _sig_key(canonical_signature(base))

    def mutate(fn):
        import copy
        t = copy.deepcopy(base)
        fn(t)
        return _sig_key(canonical_signature(t))

    def flip_hard(t):
        t["plannerView"]["stages"][0]["goalConstraints"][0]["hard"] = False

    def change_relation(t):
        t["plannerView"]["stages"][0]["goalConstraints"][0]["relation"]["desiredRelation"] = "NEAR"

    def add_second_hard_goal(t):
        t["plannerView"]["stages"][0]["goalConstraints"].append(
            {"constraintId": "g2", "kind": "OBJECT_RELATION_GOAL", "hard": True, "confidence": 0.9,
             "relation": {"subjectObjectId": "h_cube", "objectObjectId": "h_tray", "desiredRelation": "ON_TOP_OF"}})

    def add_contact(t):
        t["contacts"] = [{"objectIds": ["h_cube", "h_tray"], "contactWord": "touch"}]

    def add_second_stage(t):
        t["plannerView"]["stages"].append(
            {"stageId": "s2", "confidence": 0.9, "goalConstraints": [
                {"constraintId": "g9", "kind": "OBJECT_RELATION_GOAL", "hard": True, "confidence": 0.9,
                 "relation": {"subjectObjectId": "h_cube", "objectObjectId": "h_tray", "desiredRelation": "FAR_FROM"}}]})

    for name, fn in [("hard", flip_hard), ("desiredRelation", change_relation),
                     ("secondHardGoal", add_second_hard_goal), ("contact", add_contact),
                     ("secondStage", add_second_stage)]:
        assert mutate(fn) != base_key, f"signature did not change when {name} was mutated (over-strip hole)"


def test_guard_raises_when_card_outgrows_assumptions():
    # The proof must self-disable (raise) rather than silently launder equivalence for a card
    # it cannot fully model.
    import copy
    from experiments.cross_source_oic.target_equivalence import _assert_within_simplifying_assumptions
    base = _base_card()
    # within assumptions -> returns a scope dict, no raise
    scope = _assert_within_simplifying_assumptions(base, "base")
    assert scope["nHardRelationGoals"] == 1 and scope["stageCount"] == 1

    for mutate in (
        lambda t: t["plannerView"]["stages"].append({"stageId": "s2", "goalConstraints": []}),
        lambda t: t["plannerView"]["stages"][0]["goalConstraints"].append(
            {"kind": "OBJECT_RELATION_GOAL", "hard": True,
             "relation": {"subjectObjectId": "h_cube", "objectObjectId": "h_tray", "desiredRelation": "NEAR"}}),
        lambda t: t.__setitem__("contacts", [{"objectIds": ["h_cube", "h_tray"]}]),
    ):
        t = copy.deepcopy(base)
        mutate(t)
        with pytest.raises(AssertionError):
            _assert_within_simplifying_assumptions(t, "mutated")


# --------------------------------------------------------------------------------------
# committed artifact is current + csg frozen
# --------------------------------------------------------------------------------------

def test_committed_report_reproduces_live_build():
    committed = json.loads((EXP_DIR / "cross_source_report.json").read_text())
    fresh = build_record()
    assert committed["totals"] == fresh["totals"]
    assert {w["worldKey"]: w["aggregate"] for w in committed["worlds"]} \
        == {w["worldKey"]: w["aggregate"] for w in fresh["worlds"]}


def test_csg_is_byte_frozen():
    try:
        out = subprocess.run(["git", "diff", "--name-only", "--", "csg"],
                             cwd=REPO, capture_output=True, text=True, timeout=30)
    except (FileNotFoundError, subprocess.SubprocessError):
        pytest.skip("git not available")
    assert out.returncode == 0
    assert out.stdout.strip() == "", f"csg/ must stay byte-frozen, changed: {out.stdout}"
