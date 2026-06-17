#!/usr/bin/env python3
"""Tests for the baseline_counterexamples experiment.

These recompute the naive ladder and the frozen-verifier verdicts LIVE from the
committed ``real_camera.tracks.v0`` clips (no raw mp4, no OpenCV) and assert:

  * the rim flagship: B1 center-predicate PASSes while the structured verifier
    FAILs with LEFT_ON_RIM (the "aha");
  * born-inside: a terminal predicate (and the verifier's weak terminal_only
    target) PASS, but the structured transition target FAILs on initial_state;
  * occlusion: every baseline certifies it, the verifier renders UNCERTAIN;
  * aggregate: >=1 naive false-PASS and 0 structured false-PASS on the human
    non-success clips;
  * the committed results_table.csv and per-case JSON faithfully match a fresh
    recompute (so the artifact cannot silently drift or be fabricated).
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from csg.common import load_json
from csg.predicates import DEFAULT as PRED_DEFAULT
from experiments.baseline_counterexamples.baseline_predicates import (
    INDEPENDENT_CONSTANTS,
    LADDER,
    evaluate_clip,
)
from scripts.build_baseline_counterexamples import (
    EXP_DIR,
    PLACED_FROM_OUTSIDE,
    RELATION_EVENT,
    TERMINAL_ONLY,
    TRACKS_DIR,
    build_row,
    independent_geometry_check,
    load_expected_classes,
    load_verdict_rows,
    parse_stem,
    reproducibility_check,
    rim_perturbation_table,
    run_all_targets,
)

RIM = "oic_fail_on_rim_001__iphone_top"
BORN_INSIDE = "oic_control_inside_to_inside_001__sony_front"
OCCLUSION = "oic_success_005__iphone_top"


def _tracks(stem: str) -> dict:
    return load_json(TRACKS_DIR / f"{stem}.tracks.json")


# --------------------------------------------------------------------------- #
# Flagship: rim — single-condition terminal predicate vs. structured verifier
# --------------------------------------------------------------------------- #


def test_b1_center_predicate_passes_rim():
    """A center-in-footprint terminal predicate calls the rim clip a success."""
    r = evaluate_clip(_tracks(RIM))
    assert r["B1_center_in_footprint"] is True
    assert r["B2_footprint_overlap"] is True


def test_b3_b4_b5_reject_rim():
    """The footprint-containment baselines AND the maximal 3D terminal predicate
    (B5) reject the rim clip — so the rim is a 2D-vs-3D lesson, not terminal-vs-
    structured. This is the steelman defense made executable."""
    r = evaluate_clip(_tracks(RIM))
    assert r["B3_full_inner_containment"] is False
    assert r["B4_full_containment_started_outside"] is False
    assert r["B5_terminal_3d_containment"] is False


def test_b5_passes_born_inside_and_occlusion_but_rejects_rim():
    """B5 is the strongest single-frame terminal predicate. It must close the rim
    (3D) yet STILL certify born-inside and occluded successes — proving the
    residual gap (transition + evidence) is genuinely structural, not 2D-vs-3D."""
    assert evaluate_clip(_tracks(RIM))["B5_terminal_3d_containment"] is False
    assert evaluate_clip(_tracks(BORN_INSIDE))["B5_terminal_3d_containment"] is True
    assert evaluate_clip(_tracks(OCCLUSION))["B5_terminal_3d_containment"] is True
    assert evaluate_clip(_tracks("oic_success_001__iphone_top"))["B5_terminal_3d_containment"] is True


def test_wide_robot_rim_fails_left_on_rim():
    """Every wide-robot target FAILs the rim clip with LEFT_ON_RIM — including the
    weakest (terminal_only), because the cube ends ON_TOP_OF, not INSIDE."""
    rec = run_all_targets(_tracks(RIM))
    for name, t in rec.items():
        assert t["status"] == "FAIL", name
        assert t["cameraFailureClass"] == "LEFT_ON_RIM", name
    # And the evidence is clean — the disagreement is about the definition of
    # "inside", not tracking quality.
    assert rec[TERMINAL_ONLY]["trackingMetrics"]["minPoseConfidence"] == 1.0
    assert rec[TERMINAL_ONLY]["leakageClean"] is True
    assert rec[TERMINAL_ONLY]["physicalValidity"] is None


# --------------------------------------------------------------------------- #
# Born-inside: terminal containment ignores the transition
# --------------------------------------------------------------------------- #


def test_born_inside_terminal_passes_but_structured_fails():
    rec = run_all_targets(_tracks(BORN_INSIDE))
    # The terminal predicate AND the verifier's weak terminal_only target pass...
    assert rec[TERMINAL_ONLY]["status"] == "PASS"
    assert evaluate_clip(_tracks(BORN_INSIDE))["B1_center_in_footprint"] is True
    # ...but the structured transition targets reject it on the initial state.
    for name in (RELATION_EVENT, PLACED_FROM_OUTSIDE):
        t = rec[name]
        assert t["status"] == "FAIL", name
        assert t["cameraFailureClass"] == "BORN_INSIDE_NO_TRANSITION", name
    assert rec[RELATION_EVENT]["hardMismatches"] == ["initial_state"]


# --------------------------------------------------------------------------- #
# Occlusion: pose predicates cannot see evidence quality
# --------------------------------------------------------------------------- #


def test_occlusion_uncertain_while_all_baselines_certify():
    rec = run_all_targets(_tracks(OCCLUSION))
    for name, t in rec.items():
        assert t["status"] == "UNCERTAIN", name
        assert t["failureClass"] == "extractor_uncertainty", name
    # Every baseline (even B4) certifies it — they only read first + last frame.
    b = evaluate_clip(_tracks(OCCLUSION))
    assert all(b[k["key"]] is True for k in LADDER)


# --------------------------------------------------------------------------- #
# Aggregate claim + committed-artifact fidelity
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def all_rows():
    expected = load_expected_classes()
    rows = []
    for tp in sorted(TRACKS_DIR.glob("*.tracks.json")):
        parsed = parse_stem(tp)
        if not parsed:
            continue
        episode_id, camera, _ = parsed
        exp = expected.get((episode_id, camera))
        if exp is None:
            continue
        rows.append(build_row(episode_id, camera, load_json(tp), exp))
    return rows


def test_aggregate_naive_false_pass_and_zero_structured_false_pass(all_rows):
    """The central claim, over all human non-success clips: naive predicates
    raise at least one false PASS; the structured verifier raises none."""
    non_success = [r for r in all_rows if not r["humanSuccess"]]
    assert non_success, "expected some human non-success clips"

    naive_false_pass = [r for r in non_success if r["anyNaivePass"] is True]
    b1_false_pass = [r for r in non_success if r["B1_center_in_footprint"] is True]
    # Even the MAXIMAL single-frame terminal predicate (B5) false-passes some
    # genuine failures (born-inside) — the residue a terminal check cannot reach.
    b5_false_pass = [r for r in non_success if r["B5_terminal_3d_containment"] is True]
    structured_false_pass = [r for r in non_success if r["wrStructuredCertifies"]]

    assert len(naive_false_pass) >= 1
    assert len(b1_false_pass) >= 1
    assert len(b5_false_pass) >= 1
    assert len(structured_false_pass) == 0


def _csv_cell(value) -> str:
    """How csv.DictWriter renders a Python value (None -> '', bool -> 'True')."""
    return "" if value is None else str(value)


def test_committed_csv_matches_recompute(all_rows):
    committed = list(csv.DictReader((EXP_DIR / "results_table.csv").open()))
    by_key = {(r["episodeId"], r["camera"]): r for r in committed}
    assert len(committed) == len(all_rows)

    stable_cols = [
        "expectedClass", "humanSuccess",
        "B1_center_in_footprint", "B2_footprint_overlap",
        "B3_full_inner_containment", "B4_full_containment_started_outside",
        "B5_terminal_3d_containment", "overlapFrac", "anyNaivePass",
        "wr_terminal_only_status", "wr_relation_event_status",
        "wr_placed_from_outside_status", "wrStructuredCertifies",
    ]
    for r in all_rows:
        c = by_key[(r["episodeId"], r["camera"])]
        for col in stable_cols:
            assert c[col] == _csv_cell(r[col]), f"{r['episodeId']}__{r['camera']}:{col}"


def test_committed_case_files_match_recompute():
    """Each featured case folder's JSON matches a fresh recompute, and its overlay
    PNG exists (checked as a file — no OpenCV needed to read it)."""
    cases = {
        "rim_edge": RIM,
        "born_inside": BORN_INSIDE,
        "occlusion_uncertain": OCCLUSION,
        "control_success": "oic_success_001__iphone_top",
    }
    for case_dir, stem in cases.items():
        d = EXP_DIR / "cases" / case_dir
        for fname in ("source_info.json", "naive_predicate_results.json",
                      "wide_robot_report.json", "overlay_final_frame.png"):
            assert (d / fname).exists(), f"{case_dir}/{fname} missing"
        png = d / "overlay_final_frame.png"
        assert png.stat().st_size > 1000, f"{case_dir} overlay looks empty"

        tracks = _tracks(stem)
        # naive results match
        naive = load_json(d / "naive_predicate_results.json")
        fresh = evaluate_clip(tracks)
        for k in LADDER:
            assert naive["results"][k["key"]] == fresh[k["key"]], f"{case_dir}:{k['key']}"
        # wide-robot headline matches
        wr = load_json(d / "wide_robot_report.json")
        rec = run_all_targets(tracks)
        for short, full in (("terminal_only", TERMINAL_ONLY),
                            ("relation_event", RELATION_EVENT),
                            ("placed_from_outside", PLACED_FROM_OUTSIDE)):
            assert wr["headline"][short]["status"] == rec[full]["status"], f"{case_dir}:{short}"


def test_rim_rejection_robust_under_calibration_perturbation():
    """Across 14 calibration perturbations of the rim clip, the wide-robot
    terminal_only verdict is NEVER PASS and B5 never certifies — the rejection is
    calibration-robust (cube sits ~26 mm above the rim). The committed table
    matches a fresh recompute."""
    fresh = rim_perturbation_table(_tracks(RIM))
    assert fresh["wr_terminal_only_ever_PASS"] is False
    for row in fresh["rows"]:
        assert row["wr_terminal_only_status"] != "PASS", row["perturbation"]
        assert row["B5_terminal_3d_containment"] is False, row["perturbation"]
    # B1 (the knife-edge naive PASS) does flip under at least one ~10 mm shift.
    assert len(fresh["b1_flips_under"]) >= 1

    committed = load_json(EXP_DIR / "cases" / "rim_edge" / "robustness_perturbation.json")
    assert committed["wr_terminal_only_ever_PASS"] is False
    assert committed["b1_flips_under"] == fresh["b1_flips_under"]
    assert len(committed["rows"]) == len(fresh["rows"])


def test_reproducibility_matches_stored_verdicts(all_rows):
    """Our recompute reproduces the verdicts stored in the committed dataset on all
    78 clips (a regression/reproducibility check — SAME verifier, honestly scoped)."""
    repro = reproducibility_check(all_rows, load_verdict_rows())
    assert repro["nClips"] == 78
    assert repro["agreeTerminal"] == 78
    assert repro["agreeRelation"] == 78
    assert repro["agreePlaced"] == 78
    assert repro["decisionFieldDisagreements"] == []

    committed = load_json(EXP_DIR / "reproducibility_check.json")
    for k in ("agreeTerminal", "agreeRelation", "agreePlaced"):
        assert committed[k] == 78
    assert committed["decisionFieldDisagreements"] == []


def test_independent_geometry_constants_pinned_to_csg():
    """The from-scratch reimplementation's thresholds must equal csg.predicates.DEFAULT,
    so a future csg retune is caught rather than silently diverging."""
    assert INDEPENDENT_CONSTANTS == {
        "inside_footprint_margin_m": PRED_DEFAULT.inside_footprint_margin_m,
        "inside_rim_slack_m": PRED_DEFAULT.inside_rim_slack_m,
        "on_top_eps_m": PRED_DEFAULT.on_top_eps_m,
        "near_gap_m": PRED_DEFAULT.near_gap_m,
        "min_xy_overlap_frac": PRED_DEFAULT.min_xy_overlap_frac,
    }


def test_independent_geometry_corroborates_verifier():
    """A from-scratch reimplementation of the containment geometry (no csg.predicates
    logic) reproduces the verifier's extracted terminal relation on every clip where
    the verifier emits one — genuine second-implementation agreement, not a snapshot."""
    indep = independent_geometry_check()
    assert indep["constantsMatchCsgDefault"] is True
    assert indep["clipsCompared"] >= 50          # most clips yield an extracted relation
    assert indep["agree"] == indep["clipsCompared"]
    assert indep["disagreements"] == []

    committed = load_json(EXP_DIR / "independent_geometry_check.json")
    assert committed["agree"] == committed["clipsCompared"]
    assert committed["disagreements"] == []
    assert committed["constantsMatchCsgDefault"] is True
