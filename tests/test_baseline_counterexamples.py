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
    EVIDENCE_THRESHOLDS,
    INDEPENDENT_CONSTANTS,
    LADDER,
    evaluate_clip,
)
from experiments.baseline_counterexamples.fixtures import (
    FIXTURE_EXPECTATIONS,
    TRACKS_FIXTURES,
    evaluate_fixture_suite,
)
from experiments.baseline_counterexamples.cross_task import (
    cross_task_report,
    engine_identity,
    verify_open_drawer_demos,
)
from scripts.build_baseline_counterexamples import (
    B6_KEY,
    ENGINEERING_COST_TABLE,
    EXP_DIR,
    PLACED_FROM_OUTSIDE,
    REAL_VIDEO_THRESHOLDS,
    RELATION_EVENT,
    TERMINAL_ONLY,
    TRACKS_DIR,
    b6_vs_structured_diff,
    build_row,
    independent_geometry_check,
    load_expected_classes,
    load_verdict_rows,
    parse_stem,
    per_baseline_scoreboard,
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


def test_occlusion_uncertain_while_single_and_two_frame_baselines_certify():
    rec = run_all_targets(_tracks(OCCLUSION))
    for name, t in rec.items():
        assert t["status"] == "UNCERTAIN", name
        assert t["failureClass"] == "extractor_uncertainty", name
    # Every GEOMETRY baseline (B1..B5, incl. two-frame B4) certifies it — they read
    # only the clean first + last visible frames and never see the mid occlusion.
    b = evaluate_clip(_tracks(OCCLUSION))
    for k in ("B1_center_in_footprint", "B2_footprint_overlap", "B3_full_inner_containment",
              "B4_full_containment_started_outside", "B5_terminal_3d_containment"):
        assert b[k] is True, k
    # B6 = B4 + the verifier's OWN evidence gate -> it refuses, exactly like the verifier.
    assert b["evidenceOk"] is False
    assert b["evidenceFailureClass"] == "extractor_uncertainty"
    assert b["B6_contained_started_outside_evidence_gated"] is False


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


def test_per_baseline_scoreboard_tradeoff(all_rows):
    """Lock the honest per-baseline scoreboard, including the tradeoff a critic would
    look for: B4 (a two-frame started-outside predicate) AND the structured verifier
    BOTH reach 0 false-PASS — so the claim is 'terminal predicates (incl. maximal B5)
    are insufficient', NOT 'against the strongest fair baseline'. B4 even out-certifies
    the verifier on successes (it does not fail-close on occlusion)."""
    board = {s["predicate"]: s for s in per_baseline_scoreboard(all_rows)}
    # single-frame terminal predicates false-PASS born-inside; even maximal B5 = 10.
    assert board["B1 center-in-footprint"]["falsePass"] == 11
    assert board["B5 terminal-3D-containment"]["falsePass"] == 10
    # the two predicates that reach 0 false-PASS: two-frame B4 and the structured verifier.
    assert board["B4 contained+started-outside"]["falsePass"] == 0
    assert board["wr structured (rel OR placed)"]["falsePass"] == 0
    # the disclosed tradeoff: B4 out-recalls the structured verifier on successes.
    assert board["B4 contained+started-outside"]["successCert"] == 32
    assert board["wr structured (rel OR placed)"]["successCert"] == 27
    assert (board["B4 contained+started-outside"]["successCert"]
            > board["wr structured (rel OR placed)"]["successCert"])
    # B6 (the engineered steelman) TIES the structured verifier's whole scoreboard.
    b6 = board["B6 contained+started-outside+evidence-gated"]
    struct = board["wr structured (rel OR placed)"]
    assert b6["falsePass"] == 0
    assert b6["successCert"] == 27
    assert b6["successCert"] == struct["successCert"]
    assert b6["falsePass"] == struct["falsePass"]
    assert b6["kind"] == "engineered (B4 + evidence gate)"


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
        "B5_terminal_3d_containment", "B6_contained_started_outside_evidence_gated",
        "overlapFrac", "evidenceOk", "anyNaivePass",
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


# --------------------------------------------------------------------------- #
# B6 (engineered steelman) vs the structured verifier — honest tie, not identity
# --------------------------------------------------------------------------- #


def test_b6_ties_structured_on_scoreboard_but_disagrees_clip_level(all_rows):
    """B6 = B4 + the verifier's own evidence gate ties the structured verifier's
    AGGREGATE scoreboard (same success-cert, same 0 false-PASS) but is NOT
    clip-for-clip identical — they disagree on 4 successes that cancel out. The
    honest framing depends on stating that, so it is pinned."""
    succ = [r for r in all_rows if r["humanSuccess"]]
    nons = [r for r in all_rows if not r["humanSuccess"]]
    b6_cert = sum(1 for r in succ if r[B6_KEY] is True)
    b6_fp = sum(1 for r in nons if r[B6_KEY] is True)
    st_cert = sum(1 for r in succ if r["wrStructuredCertifies"])
    st_fp = sum(1 for r in nons if r["wrStructuredCertifies"])
    assert (b6_cert, b6_fp) == (27, 0)
    assert (st_cert, st_fp) == (27, 0)  # tie on the scoreboard

    diff = b6_vs_structured_diff(all_rows)
    assert diff["tieOnAggregateScoreboard"] is True
    assert diff["nDisagreements"] == 4  # NOT a clip-for-clip identity
    assert len(diff["b6CertifiesVerifierDoesNot"]) == len(diff["verifierCertifiesB6DoesNot"]) == 2
    # the disagreements are real, nameable causes (obstruction false-neg vs stricter footprint)
    assert all("obstruction" in d["clip"] for d in diff["b6CertifiesVerifierDoesNot"])
    assert all("success_016" in d["clip"] for d in diff["verifierCertifiesB6DoesNot"])

    committed = load_json(EXP_DIR / "b6_vs_structured.json")
    assert committed["tieOnAggregateScoreboard"] is True
    assert committed["nDisagreements"] == 4


def test_b6_evidence_thresholds_pinned_to_build_thresholds():
    """B6's evidence gate must see the SAME thresholds the structured verifier saw on
    this dataset, or the B6-vs-verifier comparison would be rigged. baseline_predicates
    claims a test pins this — this IS that test."""
    assert EVIDENCE_THRESHOLDS == REAL_VIDEO_THRESHOLDS


def test_engineering_cost_table_well_formed():
    """The 'what each predicate must know' table escalates B1->wide-robot and ends
    with wide-robot declaring the same assumptions as data (the thesis)."""
    preds = [e["predicate"] for e in ENGINEERING_COST_TABLE]
    assert preds[0].startswith("B1") and preds[-1] == "wide-robot"
    for e in ENGINEERING_COST_TABLE:
        assert e["mustKnow"] and e["reimplements"] and e["form"]
    # only wide-robot reaches the "declarative graph + reusable verifier" form.
    assert "declarative" in ENGINEERING_COST_TABLE[-1]["form"]
    assert all("declarative" not in e["form"] for e in ENGINEERING_COST_TABLE[:-1])
    committed = load_json(EXP_DIR / "engineering_cost.json")
    assert [r["predicate"] for r in committed["rows"]] == preds


# --------------------------------------------------------------------------- #
# Deterministic fixtures — calibration-free semantics
# --------------------------------------------------------------------------- #


def test_fixture_suite_matches_asserted_semantics():
    """Every hand-authored fixture produces EXACTLY its asserted ladder + verifier
    verdicts. Calibration is not a question (round-number geometry), so any drift
    is a real semantic regression."""
    res = evaluate_fixture_suite()
    for fid, _builder, _human in TRACKS_FIXTURES:
        exp = FIXTURE_EXPECTATIONS[fid]
        rec = res[fid]
        assert rec["ladder"] == exp["ladder"], fid
        assert rec["occupancyStrawman"] == exp["occupancyStrawman"], fid
        assert rec["structuredCertifies"] == exp["structuredCertifies"], fid
        for tgt, status in exp["wr"].items():
            assert rec["wr"][tgt] == status, f"{fid}:{tgt}"
        if "wrClass" in exp:
            assert rec["wrClass"][RELATION_EVENT] == exp["wrClass"], fid


def test_fixture_rim_separates_b1_from_3d_containment():
    """The calibration-free rim fixture: B1 (center) certifies, 3D containment
    (B3/B5/B6) and the verifier reject — the dimensionality lesson, no calibration."""
    rec = evaluate_fixture_suite()["fx_rim_partial"]
    assert rec["ladder"]["B1"] is True
    assert rec["ladder"]["B3"] is False and rec["ladder"]["B5"] is False
    assert rec["wr"][TERMINAL_ONLY] == "FAIL"
    assert rec["wrClass"][RELATION_EVENT] == "LEFT_ON_RIM"


def test_fixture_occlusion_and_leakage_are_verifier_only_gates():
    """Occlusion: B1..B5 certify, B6 + verifier refuse (evidence). Leakage: the same
    rollout PASSes clean but is refused once a source name leaks in."""
    res = evaluate_fixture_suite()
    occ = res["fx_occluded_uncertain"]
    assert all(occ["ladder"][k] for k in ("B1", "B2", "B3", "B4", "B5"))
    assert occ["ladder"]["B6"] is False and occ["evidenceOk"] is False
    assert occ["wr"][TERMINAL_ONLY] == "UNCERTAIN"
    leak = res["fx_leaky_metadata"]
    assert leak["cleanStatus"] == "PASS"
    assert leak["leakyStatus"] == "UNCERTAIN"
    assert leak["leakyFailureClass"] == "leakage_violation"


def test_fixture_wrong_object_defeats_occupancy_but_not_identity_bound():
    """A decoy in the tray fools the identity-blind occupancy strawman; the
    cube-bound ladder and the verifier (judging the moving cube) reject."""
    rec = evaluate_fixture_suite()["fx_wrong_object"]
    assert rec["occupancyStrawman"] is True            # identity-blind check fooled
    assert all(rec["ladder"][k] is False for k in rec["ladder"])  # cube-bound: not fooled
    assert rec["structuredCertifies"] is False


def test_committed_fixture_results_match_recompute():
    """The committed fixtures/fixture_results.json matches a fresh recompute."""
    committed = load_json(EXP_DIR / "fixtures" / "fixture_results.json")["results"]
    fresh = evaluate_fixture_suite()
    for fid, _b, _h in TRACKS_FIXTURES:
        assert committed[fid]["ladder"] == fresh[fid]["ladder"], fid
        assert committed[fid]["wr"] == fresh[fid]["wr"], fid


# --------------------------------------------------------------------------- #
# Cross-task — one frozen engine, task = target graph
# --------------------------------------------------------------------------- #


def test_cross_task_engine_is_one_shared_function_object():
    """object_inside_container and open_drawer call the IDENTICAL verify_external_rollout."""
    ident = engine_identity()
    assert ident["realCameraImportIsSameObject"] is True
    assert ident["rlbenchImportIsSameObject"] is True
    assert ident["fn"] == "pilots.external_verify.verify_external_rollout"


def test_cross_task_open_drawer_passes_via_same_engine():
    """The committed live RLBench drawer demos PASS the open_drawer target through
    the shared engine — leakage-clean, non-vacuous, articulation probes supported."""
    od = verify_open_drawer_demos()
    assert od["nDemos"] == 9
    assert od["allPass"] and od["allLeakageClean"] and od["allNonVacuous"] and od["allProbesSupported"]


def test_cross_task_target_is_the_task_and_baseline_is_inapplicable():
    """Same engine + same drawer rollout: open_drawer PASSes, object_inside_container
    FAILs — the target defines the task. The cube/tray ladder has no inputs on a drawer."""
    ct = cross_task_report()
    tdt = ct["targetDefinesTask"]
    assert tdt["open_drawer_target"]["status"] == "PASS"
    assert tdt["object_inside_container_target"]["status"] == "FAIL"
    bi = ct["baselineInapplicable"]
    assert bi["ladderApplicable"] is False
    assert bi["drawerRolloutHas"]["anyContainerBody"] is False
    committed = load_json(EXP_DIR / "cross_task" / "cross_task_report.json")
    assert committed["openDrawerDemos"]["allPass"] is True
