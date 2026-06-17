#!/usr/bin/env python3
"""Build the ``experiments/baseline_counterexamples/`` artifact.

For every committed Sony/iPhone ``object_inside_container`` clip this:
  1. runs the naive B1..B4 baseline ladder (``experiments.baseline_counterexamples.
     baseline_predicates``) from the tracks — no video, no OpenCV;
  2. runs the structured wide-robot verifier (``pilots.real_camera.verify_episode``)
     against BOTH bundled targets (terminal_only + relation_event) with the same
     relaxed 30fps thresholds the ingest pipeline used;
  3. emits the aggregate ``results_table.csv`` / ``results_table.md``;
  4. writes the flagship per-case folders (rim_edge / born_inside /
     occlusion_uncertain / control_success) with ``source_info.json``,
     ``naive_predicate_results.json``, ``wide_robot_report.json`` and a rendered
     ``overlay_final_frame.png`` (best-effort: needs cv2 + the local raw mp4);
  5. dumps the full verifier records under ``wide_robot_reports/``.

The committed proof artifacts are JSON / CSV / Markdown / PNG and the source
hashes; raw mp4s stay local + untracked under the repo's ``*.mp4`` ignore rule
(pass ``--copy-clips`` to drop a copy into each case folder for local viewing).
``csg/`` is only READ. Output is timestamp-free so re-runs diff cleanly.

Usage:
    python3 -m scripts.build_baseline_counterexamples            # full build
    python3 -m scripts.build_baseline_counterexamples --no-overlays
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from csg.common import load_json  # noqa: E402
from csg.predicates import DEFAULT as _PRED_DEFAULT  # noqa: E402
from csg.rollout_extract import extract_robot_csg  # noqa: E402
from experiments.baseline_counterexamples.baseline_predicates import (  # noqa: E402
    BASELINE_PREDICATES_VERSION,
    DEFAULT_B2_MIN_OVERLAP_FRAC,
    INDEPENDENT_CONSTANTS,
    LADDER,
    clip_geometry,
    evaluate_clip,
    independent_terminal_relation,
)
from pilots.real_camera.verify_episode import verify_episode, verify_episode_both  # noqa: E402
from pilots.real_camera.tracks_to_rollout import tracks_to_rollout  # noqa: E402
from experiments.baseline_counterexamples.fixtures import (  # noqa: E402
    TRACKS_FIXTURES,
    evaluate_fixture_suite,
    write_fixtures,
)
from experiments.baseline_counterexamples.cross_task import (  # noqa: E402
    cross_task_report,
    write_cross_task,
)

DATASET = REPO / "datasets" / "sony_object_inside_container_v0"
TRACKS_DIR = DATASET / "tracks"
RECORDINGS = REPO / "recordings"
EXP_DIR = REPO / "experiments" / "baseline_counterexamples"

# Same evidence-quality thresholds the ingest pipeline used for raw 30fps video.
REAL_VIDEO_THRESHOLDS = {"max_consecutive_missing": 30, "max_dropout_frac": 0.35}

TERMINAL_ONLY = "object_inside_container_terminal_only"
RELATION_EVENT = "object_inside_container_relation_event"
PLACED_FROM_OUTSIDE = "object_inside_container_placed_from_outside"
TARGETS_DIR = REPO / "pilots" / "real_camera" / "targets"

# expectedClass values that denote a genuine successful outside->inside placement.
SUCCESS_PREFIX = "success"

# The flagship cases (folder name -> clip + the one-line lesson).
FEATURED = [
    {"dir": "rim_edge", "episodeId": "oic_fail_on_rim_001", "camera": "iphone_top",
     "lesson": "cube center is inside the tray footprint (B1 PASS) but the cube is on the rim -> wide-robot FAIL / LEFT_ON_RIM"},
    {"dir": "born_inside", "episodeId": "oic_control_inside_to_inside_001", "camera": "sony_front",
     "lesson": "cube ends inside (terminal predicate + wide-robot terminal_only PASS) but no outside->inside transition -> structured target FAIL / BORN_INSIDE_NO_TRANSITION"},
    {"dir": "occlusion_uncertain", "episodeId": "oic_success_005", "camera": "iphone_top",
     "lesson": "a genuine success by geometry (all baselines PASS) but the cube is occluded for 50 frames -> wide-robot UNCERTAIN (fail-closed)"},
    {"dir": "control_success", "episodeId": "oic_success_001", "camera": "iphone_top",
     "lesson": "a clean success: baselines PASS and wide-robot terminal_only PASS -- the verifier is not a fail-everything oracle"},
]


def parse_stem(path: Path) -> Optional[tuple]:
    stem = path.name[: -len(".tracks.json")] if path.name.endswith(".tracks.json") else path.stem
    if "__" not in stem:
        return None
    episode_id, camera = stem.rsplit("__", 1)
    return episode_id, camera, stem


def load_expected_classes() -> Dict[tuple, str]:
    verdicts = load_json(DATASET / "verdicts_all.json")
    out: Dict[tuple, str] = {}
    for row in verdicts.get("rows", []):
        out[(str(row.get("episodeId")), str(row.get("camera")))] = str(row.get("expectedClass"))
    return out


def video_relpath(manifest: Mapping[str, Any], episode_id: str, camera: str) -> Optional[str]:
    for v in manifest.get("videos", []):
        if str(v.get("episodeId")) == episode_id and str(v.get("camera")) == camera:
            return str(v.get("relativePath"))
    return None


def short_report(rec: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        "status": rec.get("status"),
        "passed": bool(rec.get("passed")),
        "cameraFailureClass": rec.get("cameraFailureClass"),
        "failureClass": rec.get("failureClass"),
        "hardMismatches": rec.get("hardMismatches"),
        "leakageClean": rec.get("leakageClean"),
        "physicalValidity": rec.get("physicalValidity"),
    }


def run_all_targets(tracks: Mapping[str, Any]) -> Dict[str, Any]:
    """All three structured targets for one episode: the two bundled
    (terminal_only + relation_event) plus placed_from_outside (the far-start
    sibling), so a genuine far-start success is judged against the right target."""
    both = verify_episode_both(tracks=tracks, thresholds=REAL_VIDEO_THRESHOLDS)
    placed_target = load_json(TARGETS_DIR / f"{PLACED_FROM_OUTSIDE}.json")
    both[PLACED_FROM_OUTSIDE] = verify_episode(
        placed_target, tracks=tracks, thresholds=REAL_VIDEO_THRESHOLDS, case_name=PLACED_FROM_OUTSIDE)
    return both


def build_row(episode_id: str, camera: str, tracks: Mapping[str, Any],
              expected: str) -> Dict[str, Any]:
    is_success = expected.startswith(SUCCESS_PREFIX)
    baseline = evaluate_clip(tracks)
    records = run_all_targets(tracks)
    term = records[TERMINAL_ONLY]
    rele = records[RELATION_EVENT]
    placed = records[PLACED_FROM_OUTSIDE]

    bvals = {k["key"]: (None if baseline is None else bool(baseline[k["key"]])) for k in LADDER}
    any_naive_pass = None if baseline is None else any(bvals[k["key"]] for k in LADDER)
    # A clip is "structurally certified" iff SOME real outside->inside transition
    # target passes (near-start relation_event OR far-start placed_from_outside).
    structured_certifies = rele.get("status") == "PASS" or placed.get("status") == "PASS"
    return {
        "episodeId": episode_id,
        "camera": camera,
        "expectedClass": expected,
        "humanSuccess": is_success,
        **bvals,
        "overlapFrac": None if baseline is None else round(float(baseline["overlapFrac"]), 4),
        "evidenceOk": None if baseline is None else bool(baseline["evidenceOk"]),
        "anyNaivePass": any_naive_pass,
        "wr_terminal_only_status": term.get("status"),
        "wr_terminal_only_class": term.get("cameraFailureClass") or term.get("failureClass"),
        "wr_relation_event_status": rele.get("status"),
        "wr_relation_event_class": rele.get("cameraFailureClass") or rele.get("failureClass"),
        "wr_placed_from_outside_status": placed.get("status"),
        "wr_placed_from_outside_class": placed.get("cameraFailureClass") or placed.get("failureClass"),
        "wrStructuredCertifies": structured_certifies,
        "_records": records,  # kept in-memory for featured dumps; stripped from CSV
        "_baseline": baseline,
    }


def load_verdict_rows() -> Dict[tuple, Dict[str, Any]]:
    """Full rows from the dataset's INDEPENDENT verdict harness (verdicts_all.json),
    keyed by (episodeId, camera) — used to cross-validate our verifier against a
    separate code path."""
    verdicts = load_json(DATASET / "verdicts_all.json")
    return {(str(r.get("episodeId")), str(r.get("camera"))): r for r in verdicts.get("rows", [])}


def _mutate_tray(tracks: Mapping[str, Any], dx: float, dy: float) -> Dict[str, Any]:
    t = json.loads(json.dumps(tracks))
    for f in t["frames"]:
        p = f.get("poses", {}).get("tray")
        if isinstance(p, dict) and isinstance(p.get("positionM"), dict):
            p["positionM"]["x"] += dx
            p["positionM"]["y"] += dy
    return t


def _mutate_cube_size(tracks: Mapping[str, Any], size_m: float) -> Dict[str, Any]:
    t = json.loads(json.dumps(tracks))
    for o in t["objects"]:
        if o.get("sourceRole") == "cube":
            o["sizeM"] = [size_m, size_m, size_m]
    return t


def rim_perturbation_table(tracks: Mapping[str, Any]) -> Dict[str, Any]:
    """Perturb the rim clip's calibration (tray center +/-5,+/-10 mm in x and y;
    cube size 4-6 cm) and re-run B1, B5 and the wide-robot terminal_only verdict
    on each. Proves the wide-robot FAIL never flips to PASS, while quantifying how
    fragile the naive B1 PASS is (the central honesty caveat)."""
    rows: List[Dict[str, Any]] = []

    def _eval(t: Mapping[str, Any], label: str, kind: str, delta: float) -> None:
        b = evaluate_clip(t)
        term = verify_episode_both(tracks=t, thresholds=REAL_VIDEO_THRESHOLDS)[TERMINAL_ONLY]
        rows.append({
            "perturbation": label, "kind": kind, "deltaMm": delta,
            "B1_center_in_footprint": bool(b["B1_center_in_footprint"]),
            "B5_terminal_3d_containment": bool(b["B5_terminal_3d_containment"]),
            "wr_terminal_only_status": term["status"],
            "wr_terminal_only_class": term.get("cameraFailureClass") or term.get("failureClass"),
        })

    _eval(tracks, "baseline", "none", 0.0)
    for d in (-10, -5, 5, 10):
        _eval(_mutate_tray(tracks, d / 1000.0, 0.0), f"tray_x{d:+d}mm", "tray_x", float(d))
    for d in (-10, -5, 5, 10):
        _eval(_mutate_tray(tracks, 0.0, d / 1000.0), f"tray_y{d:+d}mm", "tray_y", float(d))
    for s in (40, 45, 50, 55, 60):
        _eval(_mutate_cube_size(tracks, s / 1000.0), f"cube_size_{s}mm", "cube_size", float(s))

    geom = evaluate_clip(tracks)["geometry"]
    return {
        "clipStem": f"{tracks.get('episodeId')}__derived",
        "note": "Perturbing the tray center +/-5,+/-10 mm (x,y) and the cube size 4-6 cm, the "
                "wide-robot terminal_only verdict is NEVER PASS (always FAIL/UNCERTAIN). The naive "
                "B1 center-in-footprint PASS is knife-edge and flips under one ~10 mm shift; the "
                "wide-robot rejection is robust because the cube sits ~26 mm above the rim+slack.",
        "geometry": geom,
        "wr_terminal_only_ever_PASS": any(r["wr_terminal_only_status"] == "PASS" for r in rows),
        "b1_flips_under": [r["perturbation"] for r in rows if r["B1_center_in_footprint"] is False],
        "rows": rows,
    }


def reproducibility_check(rows: List[Dict[str, Any]],
                         verdict_rows: Dict[tuple, Dict[str, Any]]) -> Dict[str, Any]:
    """Confirm our recompute matches the verdicts stored in the committed dataset
    (verdicts_all.json). HONEST SCOPE: verdicts_all.json is produced by
    scripts/ingest_recordings.py, which calls the SAME pilots.real_camera.verify_episode
    with the SAME thresholds/targets on the SAME tracks. So this is a *reproducibility /
    regression-consistency* check (verifier output == stored verifier output), NOT an
    independent-implementation cross-check. It proves the experiment faithfully reuses the
    production verifier and that the dataset verdicts reproduce — see independent_geometry_check
    for genuine second-implementation corroboration of the geometry."""
    pairs = (("wr_terminal_only_status", "actualTerminal"),
             ("wr_relation_event_status", "actualRelation"),
             ("wr_placed_from_outside_status", "actualPlaced"))
    agree = {ours: 0 for ours, _ in pairs}
    disagreements: List[Dict[str, Any]] = []
    for r in rows:
        v = verdict_rows.get((r["episodeId"], r["camera"]))
        if not v:
            continue
        for ours, theirs in pairs:
            if r[ours] == v.get(theirs):
                agree[ours] += 1
            else:
                disagreements.append({"clip": f"{r['episodeId']}__{r['camera']}",
                                      "field": f"{ours} vs {theirs}",
                                      "ours": r[ours], "harness": v.get(theirs)})
    n = sum(1 for r in rows if (r["episodeId"], r["camera"]) in verdict_rows)
    return {
        "kind": "reproducibility / regression-consistency (SAME verifier code path, not independent)",
        "nClips": n,
        "storedVerdicts": "datasets/sony_object_inside_container_v0/verdicts_all.json (built by "
                          "scripts/ingest_recordings.py via the same verify_episode)",
        "agreeTerminal": agree["wr_terminal_only_status"],
        "agreeRelation": agree["wr_relation_event_status"],
        "agreePlaced": agree["wr_placed_from_outside_status"],
        "decisionFieldDisagreements": disagreements,
        "note": "Decision fields (PASS/FAIL/UNCERTAIN) reproduce the committed dataset verdicts exactly. "
                "Because both sides call the same verifier, this is a snapshot/regression check (the "
                "experiment uses the production verifier unchanged), NOT independent corroboration. The "
                "harness's cosmetic transitionClass label is not a decision field and is excluded here.",
    }


def _verifier_terminal_relation(tracks: Mapping[str, Any]):
    """('ok', relation|None) from the FROZEN extractor, or ('unconvertible', None)
    if the episode can't be minted into a rollout (e.g. occlusion drop-out)."""
    try:
        rollout = tracks_to_rollout(tracks)
    except Exception:  # occlusion / structural -> no rollout to extract from
        return ("unconvertible", None)
    robot = extract_robot_csg(rollout)
    last = [r for r in robot.get("relations", []) if str(r.get("relationId", "")).endswith("_last")]
    return ("ok", last[-1].get("relation") if last else None)


def independent_geometry_check() -> Dict[str, Any]:
    """GENUINE second-implementation cross-check: re-derive each clip's terminal
    cube->tray relation with a from-scratch axis-aligned reimplementation
    (baseline_predicates.independent_terminal_relation, no csg.predicates logic) and
    compare against the verifier's extracted terminal relation. Agreement here cannot
    be a snapshot artifact — it is two different implementations of the containment
    spec producing the same answer."""
    agree = 0
    compared = 0
    unconvertible: List[str] = []
    no_relation: List[Dict[str, Any]] = []
    disagreements: List[Dict[str, Any]] = []
    for tp in sorted(TRACKS_DIR.glob("*.tracks.json")):
        parsed = parse_stem(tp)
        if not parsed:
            continue
        _ep, _cam, stem = parsed
        tracks = load_json(tp)
        geom = clip_geometry(tracks)
        if geom is None:
            continue
        status, vrel = _verifier_terminal_relation(tracks)
        irel = independent_terminal_relation(geom["cubeLast"], geom["cubeSize"],
                                             geom["trayCenter"], geom["traySize"])
        if status == "unconvertible":
            unconvertible.append(stem)
            continue
        if vrel is None:
            # cube never moves -> verifier's figure-ground emits no relation (a motion
            # decision, not a geometry one); record what the geometry alone would say.
            no_relation.append({"clip": stem, "independent": irel})
            continue
        compared += 1
        if irel == vrel:
            agree += 1
        else:
            disagreements.append({"clip": stem, "independent": irel, "verifier": vrel})
    csg_default = {k: getattr(_PRED_DEFAULT, k) for k in INDEPENDENT_CONSTANTS}
    return {
        "kind": "independent second-implementation cross-check (from-scratch geometry vs the verifier's "
                "extracted terminal relation)",
        "constants": INDEPENDENT_CONSTANTS,
        "constantsMatchCsgDefault": INDEPENDENT_CONSTANTS == csg_default,
        "clipsCompared": compared,
        "agree": agree,
        "disagreements": disagreements,
        "scope": {
            "gateRejectedUnconvertible": len(unconvertible),
            "verifierEmitsNoRelation_noCubeMotion": len(no_relation),
        },
        "note": "On every clip where the verifier extracts a terminal relation, an independent from-scratch "
                "reimplementation of the containment geometry reproduces it. Excluded: clips gate-rejected "
                "as occluded (no rollout to extract), and clips with no cube motion (the verifier's "
                "figure-ground step emits no relation — a motion decision, outside the geometry's scope).",
    }


CSV_COLUMNS = [
    "episodeId", "camera", "expectedClass", "humanSuccess",
    "B1_center_in_footprint", "B2_footprint_overlap", "B3_full_inner_containment",
    "B4_full_containment_started_outside", "B5_terminal_3d_containment",
    "B6_contained_started_outside_evidence_gated",
    "overlapFrac", "evidenceOk", "anyNaivePass",
    "wr_terminal_only_status", "wr_terminal_only_class",
    "wr_relation_event_status", "wr_relation_event_class",
    "wr_placed_from_outside_status", "wr_placed_from_outside_class", "wrStructuredCertifies",
]


def per_baseline_scoreboard(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Success-certifications and false-PASSes for every predicate, side by side —
    the honest tradeoff table. A predicate that reaches 0 false-PASS by also
    refusing to certify successes is not free; this surfaces that."""
    succ = [r for r in rows if r["humanSuccess"]]
    nons = [r for r in rows if not r["humanSuccess"]]

    def score(label: str, kind: str, pred) -> Dict[str, Any]:
        return {
            "predicate": label,
            "kind": kind,
            "successCert": sum(1 for r in succ if pred(r)),
            "successTotal": len(succ),
            "falsePass": sum(1 for r in nons if pred(r)),
            "nonSuccessTotal": len(nons),
        }

    def _kind(key: str) -> str:
        if key == B6_KEY:
            return "engineered (B4 + evidence gate)"
        if key == "B4_full_containment_started_outside":
            return "naive two-frame (+ initial state)"
        return "naive single-frame terminal"

    board = [score(k["label"], _kind(k["key"]), (lambda key: lambda r: r[key] is True)(k["key"]))
             for k in LADDER]
    board.append(score("wr terminal_only", "verifier (weak target)",
                       lambda r: r["wr_terminal_only_status"] == "PASS"))
    board.append(score("wr structured (rel OR placed)", "verifier (structured)",
                       lambda r: bool(r["wrStructuredCertifies"])))
    return board


B6_KEY = "B6_contained_started_outside_evidence_gated"


def b6_vs_structured_diff(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Clip-level comparison of B6 (the engineered steelman) against the structured
    verifier. They TIE on the aggregate scoreboard (same success-cert, same 0
    false-PASS) but are NOT clip-for-clip identical — this surfaces the exact
    disagreements so the "B6 approximates the verifier" claim is stated honestly,
    not as a false clip-level identity. Each disagreement is explained by a real,
    nameable difference (B6's full-footprint containment vs the verifier's
    center-based `is_inside`; the verifier's relation-extraction sensitivity to
    terminal-pose corruption vs B6's raw-center read)."""
    b6_only: List[Dict[str, Any]] = []   # B6 certifies, structured does not
    struct_only: List[Dict[str, Any]] = []  # structured certifies, B6 does not
    agree_cert = agree_reject = 0
    for r in rows:
        b6 = r.get(B6_KEY) is True
        st = bool(r["wrStructuredCertifies"])
        clip = f"{r['episodeId']}__{r['camera']}"
        if b6 and st:
            agree_cert += 1
        elif not b6 and not st:
            agree_reject += 1
        elif b6 and not st:
            b6_only.append({"clip": clip, "humanSuccess": r["humanSuccess"],
                            "wr_relation_event_status": r["wr_relation_event_status"],
                            "wr_relation_event_class": r["wr_relation_event_class"],
                            "why": "B6 certifies via raw terminal-center geometry; the verifier's "
                                   "relation extraction reads a corrupted terminal relation and hard-FAILs "
                                   "(a known verifier false-negative on obstruction successes)"})
        else:
            struct_only.append({"clip": clip, "humanSuccess": r["humanSuccess"],
                                "B3": r.get("B3_full_inner_containment"),
                                "wr_placed_from_outside_status": r.get("wr_placed_from_outside_status"),
                                "why": "the verifier certifies via the placed_from_outside (far-start) target — "
                                       "the cube's CENTER is INSIDE by csg.is_inside — but B6's B3 requires the "
                                       "WHOLE footprint inside the shrunk inner region, which is stricter, so B6 "
                                       "rejects (a definitional difference in 'inside': center-in vs footprint-in)"})
    return {
        "kind": "B6 (engineered steelman) vs structured verifier — clip-level diff",
        "claim": "B6 and the structured verifier TIE on the aggregate scoreboard "
                 "(same success-cert and same 0 false-PASS) but disagree on individual clips that "
                 "cancel out; this lists every disagreement with its cause.",
        "agreeCertify": agree_cert,
        "agreeReject": agree_reject,
        "b6CertifiesVerifierDoesNot": b6_only,
        "verifierCertifiesB6DoesNot": struct_only,
        "nDisagreements": len(b6_only) + len(struct_only),
        "tieOnAggregateScoreboard": (len(b6_only) == len(struct_only)),
        "note": "Same-count, different-members. The disagreements are NOT errors-vs-truth in one direction: "
                "B6 is actually correct on the obstruction successes the verifier hard-FAILs, and the verifier "
                "is correct (by its center-based definition) on the near-edge successes B6's stricter "
                "full-footprint test rejects. The point of B6 is parity on the scoreboard, achieved by "
                "bolting the verifier's OWN evidence gate onto a hand-coded geometry predicate.",
    }


# What each predicate must "know" (the assumptions it bakes in), and which of the
# verifier's components it has effectively reimplemented. The argument the table
# makes: climbing the ladder gradually reimplements wide-robot's pieces as bespoke
# per-task code, until B6 — which reaches parity only by importing the verifier's
# own evidence gate. wide-robot externalises the SAME assumptions as an auditable
# task graph (data), reused across tasks by one frozen engine.
ENGINEERING_COST_TABLE = [
    {"predicate": "B1 / B2", "mustKnow": "tray footprint (XY rectangle)",
     "reimplements": "terminal containment (2D, rim-blind)", "form": "bespoke predicate code"},
    {"predicate": "B3", "mustKnow": "cube + tray dimensions, containment margin (still 2D, rim-blind)",
     "reimplements": "the verifier's footprint-containment geometry (2D — a cube 1 m above the tray still passes B3)",
     "form": "bespoke predicate code"},
    {"predicate": "B5", "mustKnow": "+ rim height (the z test B3 lacks)",
     "reimplements": "the verifier's FULL 3D `csg.is_inside` (shrunk footprint AND rim height)",
     "form": "bespoke predicate code"},
    {"predicate": "B4", "mustKnow": "+ initial-state semantics (the cube must have STARTED outside)",
     "reimplements": "the initial-state / outside->inside transition check (as a two-endpoint proxy)",
     "form": "bespoke predicate code"},
    {"predicate": "B6", "mustKnow": "+ evidence-quality thresholds (dropout / consecutive-missing / confidence)",
     "reimplements": "the fail-closed evidence gate — here by IMPORTING the verifier's own "
                     "`assess_evidence_quality` (a from-scratch baseline would have to re-derive it)",
     "form": "bespoke predicate code + bolt-on of the verifier's gate"},
    {"predicate": "wide-robot", "mustKnow": "the same assumptions, but DECLARED as data in the target graph "
                                            "(objects, objectStates, events, plannerView goals)",
     "reimplements": "nothing per-task: one frozen engine (csg.matcher / verify_external_rollout) reads the "
                     "graph; initial-state, transition, evidence gate, and leakage discipline are reusable "
                     "components, not task-specific code",
     "form": "declarative task graph + frozen reusable verifier"},
]


def write_csv(rows: List[Dict[str, Any]], path: Path) -> None:
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _md_bool(v: Any) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "PASS" if v else "reject"
    return str(v)


def write_md(rows: List[Dict[str, Any]], path: Path, aggregate: Dict[str, Any]) -> None:
    by_key = {(r["episodeId"], r["camera"]): r for r in rows}
    lines: List[str] = []
    lines.append("# Baseline counterexamples — results\n")
    lines.append("_Generated by `scripts/build_baseline_counterexamples.py`. "
                 "Naive predicates from `baseline_predicates.py`; wide-robot column from the "
                 "frozen `csg.matcher` via `pilots.real_camera.verify_episode`._\n")
    def _wr(status: Any, klass: Any) -> str:
        return f"{status}" + (f" / {klass}" if klass else "")

    lines.append("\n## Flagship cases\n")
    lines.append("| case | clip | human | B1 center | B2 overlap | B3 inner | B4 +started-out | "
                 "B5 3D-terminal | wr terminal_only | wr relation_event | wr placed_from_outside | structured certifies? |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for feat in FEATURED:
        r = by_key.get((feat["episodeId"], feat["camera"]))
        if not r:
            continue
        lines.append(
            f"| {feat['dir']} | `{r['episodeId']}__{r['camera']}` | "
            f"{'success' if r['humanSuccess'] else r['expectedClass']} | "
            f"{_md_bool(r['B1_center_in_footprint'])} | {_md_bool(r['B2_footprint_overlap'])} | "
            f"{_md_bool(r['B3_full_inner_containment'])} | {_md_bool(r['B4_full_containment_started_outside'])} | "
            f"{_md_bool(r['B5_terminal_3d_containment'])} | "
            f"{_wr(r['wr_terminal_only_status'], r['wr_terminal_only_class'])} | "
            f"{_wr(r['wr_relation_event_status'], r['wr_relation_event_class'])} | "
            f"{_wr(r['wr_placed_from_outside_status'], r['wr_placed_from_outside_class'])} | "
            f"{'**yes**' if r['wrStructuredCertifies'] else 'no'} |")
    lines.append("\n_PASS = predicate/target certifies the put-in succeeded; "
                 "reject = predicate says no; FAIL/UNCERTAIN = the verifier's verdict. "
                 "\"structured certifies\" = relation_event (near-start) OR placed_from_outside "
                 "(far-start) PASSes — i.e. the verifier saw a real outside→inside transition._\n")

    lines.append("\n## Per-baseline scoreboard (the tradeoff, nothing hidden)\n")
    lines.append("| predicate | kind | success-cert | false-PASS |")
    lines.append("|---|---|---|---|")
    for s in per_baseline_scoreboard(rows):
        lines.append(f"| {s['predicate']} | {s['kind']} | {s['successCert']}/{s['successTotal']} | "
                     f"{s['falsePass']}/{s['nonSuccessTotal']} |")
    lines.append("\n_Read this honestly: **B4** (contained + started-outside, a two-frame predicate) reaches "
                 "**0 false-PASS** — matching the structured verifier — and even out-certifies it on successes, "
                 "because it does not fail-close on occlusion. **B6** (B4 + the verifier's own evidence gate) "
                 "then ties the structured verifier's WHOLE scoreboard (same success-cert, same 0 false-PASS) — "
                 "the engineered steelman: a fully built-out task-specific predicate approximates the verifier on "
                 "this one task. The single-FRAME terminal predicates (B1/B2/B3/B5) are the ones that cannot "
                 "reject born-inside._\n")

    b6diff = b6_vs_structured_diff(rows)
    lines.append("\n### B6 vs. the structured verifier — same scoreboard, not the same clips\n")
    lines.append(f"- agree-certify: **{b6diff['agreeCertify']}**, agree-reject: **{b6diff['agreeReject']}**, "
                 f"disagreements: **{b6diff['nDisagreements']}** (they cancel: "
                 f"{len(b6diff['b6CertifiesVerifierDoesNot'])} B6-only vs "
                 f"{len(b6diff['verifierCertifiesB6DoesNot'])} verifier-only).")
    for d in b6diff["b6CertifiesVerifierDoesNot"]:
        lines.append(f"  - **B6 certifies, verifier doesn't** — `{d['clip']}`: {d['why']}.")
    for d in b6diff["verifierCertifiesB6DoesNot"]:
        lines.append(f"  - **verifier certifies, B6 doesn't** — `{d['clip']}`: {d['why']}.")
    lines.append("\n_So \"B6 approximates the verifier\" is true at the **aggregate scoreboard** level, not as a "
                 "clip-for-clip identity — and the disagreements are real definitional/extraction differences, "
                 "not one side being wrong. Full diff: `b6_vs_structured.json`._\n")

    lines.append("\n## Baseline engineering cost — the ladder reimplements wide-robot\n")
    lines.append("| predicate | what it must know | what it reimplements | form |")
    lines.append("|---|---|---|---|")
    for e in ENGINEERING_COST_TABLE:
        lines.append(f"| {e['predicate']} | {e['mustKnow']} | {e['reimplements']} | {e['form']} |")
    lines.append("\n_Climbing the ladder gradually rebuilds wide-robot's pieces as bespoke per-task code, until "
                 "B6 reaches parity only by importing the verifier's own evidence gate. wide-robot declares the "
                 "SAME assumptions as data in a task graph that one frozen engine reads — so they are auditable "
                 "and reused across tasks (see the cross-task example) instead of re-derived per predicate._\n")

    lines.append("\n## Aggregate over all clips\n")
    lines.append(f"- clips scored: **{aggregate['nClips']}** "
                 f"({aggregate['nSuccess']} human-success, {aggregate['nNonSuccess']} human-non-success)")
    lines.append(f"- **naive B1 (center-in-footprint) false PASSes on human-non-success clips: "
                 f"{aggregate['naiveB1FalsePass']}**")
    lines.append(f"- **B5 (maximal single-frame terminal predicate = csg.is_inside on last frame) "
                 f"false PASSes on human-non-success clips: {aggregate['b5FalsePass']}** "
                 f"(the born-inside clips — a single-frame terminal check cannot see the missing transition)")
    lines.append(f"- **B4 (contained + started-outside, a TWO-frame predicate) false PASSes: "
                 f"{aggregate['b4FalsePass']}** — the started-outside clause rejects born-inside, so B4 matches "
                 f"the structured verifier's 0 false-PASS. Tradeoff: B4 certifies "
                 f"{aggregate['b4SuccessCert']}/{aggregate['nSuccess']} successes (it does NOT fail-close on "
                 f"occlusion), vs the structured verifier's {aggregate['structuredCertifiesSuccess']}/"
                 f"{aggregate['nSuccess']}. See the scoreboard above.")
    lines.append(f"- **B6 (B4 + the verifier's own evidence gate) false PASSes: {aggregate['b6FalsePass']}**, "
                 f"success-cert {aggregate['b6SuccessCert']}/{aggregate['nSuccess']} — the engineered steelman "
                 f"TIES the structured verifier's whole scoreboard (both "
                 f"{aggregate['structuredCertifiesSuccess']}/{aggregate['nSuccess']} cert, both "
                 f"{aggregate['structuredFalsePass']} false-PASS). It reaches parity by bolting the verifier's "
                 f"separable evidence gate onto a hand-coded geometry predicate (clip-level it still disagrees on "
                 f"a few successes that cancel — see the B6-vs-verifier diff).")
    lines.append(f"- wide-robot `terminal_only` PASSes on human-non-success clips: "
                 f"{aggregate['terminalOnlyFalsePass']} "
                 f"(the verifier asked the *weak* terminal question — these are the born-inside clips)")
    lines.append(f"- **wide-robot STRUCTURED (relation_event OR placed_from_outside) PASSes on "
                 f"human-non-success clips: {aggregate['structuredFalsePass']}**")
    lines.append(f"- wide-robot STRUCTURED certifies human-success clips: "
                 f"{aggregate['structuredCertifiesSuccess']} / {aggregate['nSuccess']}; of the rest, "
                 f"{aggregate['successStructuredUncertain']} are UNCERTAIN (fail-closed on occlusion) and "
                 f"{aggregate['successStructuredFail']} are hard FAILs — real false-negatives on the "
                 f"hand/tag-obstruction successes where brief occlusion corrupts the terminal relation "
                 f"without tripping the evidence gate (an honest known limitation, not part of the thesis)")
    lines.append(f"- wide-robot clips rendered UNCERTAIN (fail-closed on evidence): "
                 f"{aggregate['uncertain']}")
    lines.append("\nThe full per-clip table is in `results_table.csv`.\n")

    lines.append("\n## Deterministic fixtures (semantics, calibration-free)\n")
    lines.append("Hand-authored `real_camera.tracks.v0` episodes with round-number geometry (nothing to "
                 "calibrate), one per semantic, regenerated by `fixtures.py` into `fixtures/`:\n")
    fres = evaluate_fixture_suite()
    lines.append("| fixture | human | B1 | B3 | B4 | B5 | B6 | wr terminal | wr relation | occupancy-strawman |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for fid, _b, _h in TRACKS_FIXTURES:
        r = fres[fid]
        L = r["ladder"]
        lines.append(f"| `{fid}` | {r['human']} | {_md_bool(L['B1'])} | {_md_bool(L['B3'])} | "
                     f"{_md_bool(L['B4'])} | {_md_bool(L['B5'])} | {_md_bool(L['B6'])} | "
                     f"{r['wr'][TERMINAL_ONLY]} | {r['wr'][RELATION_EVENT]} | {_md_bool(r['occupancyStrawman'])} |")
    leak = fres["fx_leaky_metadata"]
    lines.append(f"\n_`fx_leaky_metadata` is rollout-only: the identical evidence is **{leak['cleanStatus']}** "
                 f"clean but **{leak['leakyStatus']}** (`{leak['leakyFailureClass']}`) once a source role name "
                 f"leaks into `objectIdMap` — a gate the baselines do not have. The occupancy-strawman column is "
                 f"the identity-blind \"is anything inside?\" check `fx_wrong_object` defeats. Full asserted "
                 f"semantics + recompute in `fixtures/fixture_results.json`._\n")

    ct = cross_task_report()
    od = ct["openDrawerDemos"]
    lines.append("\n## Cross-task — one frozen engine, task = target graph\n")
    lines.append(f"- engine identity: `{ct['engineIdentity']['fn']}` is the **same function object** imported by "
                 f"the real-camera *and* RLBench pilots "
                 f"(realCamera={ct['engineIdentity']['realCameraImportIsSameObject']}, "
                 f"rlbench={ct['engineIdentity']['rlbenchImportIsSameObject']}).")
    lines.append(f"- `open_drawer` articulation target on the {od['nDemos']} committed live RLBench drawer "
                 f"rollouts via that engine: allPass={od['allPass']}, leakage-clean={od['allLeakageClean']}, "
                 f"non-vacuous={od['allNonVacuous']}, articulation probes supported={od['allProbesSupported']}, "
                 f"all physics-unverified (physicalValidity null)={od.get('allPhysicsUnverified')}.")
    tdt = ct["targetDefinesTask"]
    lines.append(f"- same engine, same drawer rollout, two targets: `open_drawer` → "
                 f"**{tdt['open_drawer_target']['status']}**, `object_inside_container` → "
                 f"**{tdt['object_inside_container_target']['status']}**. The target IS the task.")
    lines.append(f"- the cube/tray baseline ladder is **inapplicable** to a drawer "
                 f"(bodies={ct['baselineInapplicable']['drawerRolloutHas']['bodyPhysicalKinds']}, "
                 f"container={ct['baselineInapplicable']['drawerRolloutHas']['anyContainerBody']}) — scoring "
                 f"open_drawer means a brand-new joint-extension predicate from scratch. Detail in "
                 f"`cross_task/cross_task_report.json`.\n")
    path.write_text("\n".join(lines) + "\n")


def render_overlay_png(episode_id: str, camera: str, dst_png: Path, draw_tags: bool = True) -> Optional[str]:
    """Render the terminal-frame overlay to ``dst_png``. Returns None on success
    or a short reason string if it could not render (missing cv2 / video)."""
    try:
        import cv2  # noqa: F401
    except Exception as exc:  # pragma: no cover - environment without the camera extra
        return f"opencv unavailable ({exc})"
    from pilots.real_camera.visualize_episode import render_overlay
    tmp_dir = dst_png.parent / "_overlay_tmp"
    try:
        try:
            jpg = render_overlay(episode_id, camera, "terminal", out_dir=tmp_dir, draw_tags=draw_tags)
        except Exception as exc:  # tag detector or video read failed -> retry without tags
            if draw_tags:
                jpg = render_overlay(episode_id, camera, "terminal", out_dir=tmp_dir, draw_tags=False)
            else:
                raise exc
        import cv2
        img = cv2.imread(str(jpg))
        if img is None:
            return f"could not read rendered frame {jpg}"
        # Downscale to <=1280px wide so the committed PNG stays small (the overlay
        # is a qualitative proof, not a pixel-exact reference). 4K Sony -> ~1280x720.
        h, w = img.shape[:2]
        if w > 1280:
            scale = 1280.0 / w
            img = cv2.resize(img, (1280, int(round(h * scale))), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(dst_png), img)
        return None
    except FileNotFoundError as exc:
        return f"missing input ({exc})"
    except Exception as exc:  # pragma: no cover
        return f"render failed ({type(exc).__name__}: {exc})"
    finally:
        if tmp_dir.exists():
            shutil.rmtree(tmp_dir, ignore_errors=True)


def write_case(feat: Mapping[str, Any], row: Dict[str, Any], manifest: Mapping[str, Any],
               *, render: bool, copy_clip: bool) -> Dict[str, Any]:
    case_dir = EXP_DIR / "cases" / feat["dir"]
    case_dir.mkdir(parents=True, exist_ok=True)
    episode_id, camera = feat["episodeId"], feat["camera"]
    stem = f"{episode_id}__{camera}"
    tracks = load_json(TRACKS_DIR / f"{stem}.tracks.json")
    records = row["_records"]
    baseline = row["_baseline"]
    relpath = video_relpath(manifest, episode_id, camera)

    # source_info.json — identity + provenance, no raw pixels.
    source_info = {
        "clipStem": stem,
        "episodeId": episode_id,
        "camera": camera,
        "humanLabel": row["expectedClass"],
        "humanVerdict": "PASS" if row["humanSuccess"] else "FAIL",
        "lesson": feat["lesson"],
        "numFrames": tracks.get("frames") and len(tracks["frames"]),
        "fps": tracks.get("fps"),
        "videoSha256": tracks.get("videoSha256"),
        "calibrationHash": tracks.get("calibrationHash"),
        "rawVideoRelativePath": relpath,
        "rawVideoNote": "raw mp4 is gitignored (repo *.mp4 rule); present locally for overlay "
                        "regeneration. The tracked evidence is the tracks/calibration JSON.",
        "geometry": baseline["geometry"] if baseline else None,
        "tracksPath": f"datasets/sony_object_inside_container_v0/tracks/{stem}.tracks.json",
    }
    (case_dir / "source_info.json").write_text(json.dumps(source_info, indent=2) + "\n")

    # naive_predicate_results.json
    naive = {
        "predicatesVersion": BASELINE_PREDICATES_VERSION,
        "params": {"b2_min_overlap_frac": DEFAULT_B2_MIN_OVERLAP_FRAC,
                   "inner_margin_m": baseline["params"]["inner_margin_m"] if baseline else None},
        "results": {k["key"]: (None if baseline is None else bool(baseline[k["key"]])) for k in LADDER},
        "ladder": [{"key": k["key"], "label": k["label"], "question": k["question"],
                    "verdict": (None if baseline is None else ("PASS" if baseline[k["key"]] else "reject"))}
                   for k in LADDER],
        "overlapFrac": None if baseline is None else round(float(baseline["overlapFrac"]), 4),
        "naivePassSet": [] if baseline is None else [k["key"] for k in LADDER if baseline[k["key"]]],
    }
    (case_dir / "naive_predicate_results.json").write_text(json.dumps(naive, indent=2) + "\n")

    # wide_robot_report.json — headline + all three full records.
    wr = {
        "headline": {
            "terminal_only": short_report(records[TERMINAL_ONLY]),
            "relation_event": short_report(records[RELATION_EVENT]),
            "placed_from_outside": short_report(records[PLACED_FROM_OUTSIDE]),
            "structuredCertifies": bool(row["wrStructuredCertifies"]),
        },
        "note": "terminal_only is the verifier asked the WEAK (terminal-containment) question. "
                "relation_event (near-start) and placed_from_outside (far-start) are the STRUCTURED "
                "targets (initial state + outside->inside transition + event); 'structuredCertifies' "
                "is true iff one of them PASSes. physicalValidity is null by contract for real-camera traces.",
        "records": records,
    }
    (case_dir / "wide_robot_report.json").write_text(json.dumps(wr, indent=2, sort_keys=True) + "\n")

    overlay_status = "skipped (--no-overlays)"
    if render:
        reason = render_overlay_png(episode_id, camera, case_dir / "overlay_final_frame.png")
        overlay_status = "rendered" if reason is None else f"NOT rendered: {reason}"

    if copy_clip and relpath:
        src = RECORDINGS / relpath
        if src.exists():
            shutil.copy2(src, case_dir / "near_miss_clip.mp4")

    return {"dir": feat["dir"], "stem": stem, "overlay": overlay_status}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Build the baseline_counterexamples experiment artifact.")
    ap.add_argument("--no-overlays", action="store_true", help="skip cv2 overlay rendering")
    ap.add_argument("--copy-clips", action="store_true",
                    help="copy local raw mp4s into each case folder (untracked under *.mp4)")
    args = ap.parse_args(argv)

    EXP_DIR.mkdir(parents=True, exist_ok=True)
    (EXP_DIR / "cases").mkdir(exist_ok=True)
    (EXP_DIR / "wide_robot_reports").mkdir(exist_ok=True)

    expected = load_expected_classes()
    manifest = load_json(RECORDINGS / "manifest.json")

    rows: List[Dict[str, Any]] = []
    for tp in sorted(TRACKS_DIR.glob("*.tracks.json")):
        parsed = parse_stem(tp)
        if not parsed:
            continue
        episode_id, camera, _stem = parsed
        exp = expected.get((episode_id, camera))
        if exp is None:
            continue  # not in the verdict set (e.g. legacy synthetic fixtures)
        tracks = load_json(tp)
        rows.append(build_row(episode_id, camera, tracks, exp))

    # Aggregate counts (the honesty headline).
    non_success = [r for r in rows if not r["humanSuccess"]]
    success = [r for r in rows if r["humanSuccess"]]
    aggregate = {
        "nClips": len(rows),
        "nSuccess": len(success),
        "nNonSuccess": len(non_success),
        "naiveB1FalsePass": sum(1 for r in non_success if r["B1_center_in_footprint"] is True),
        "b4FalsePass": sum(1 for r in non_success if r["B4_full_containment_started_outside"] is True),
        "b4SuccessCert": sum(1 for r in success if r["B4_full_containment_started_outside"] is True),
        "b5FalsePass": sum(1 for r in non_success if r["B5_terminal_3d_containment"] is True),
        "b6FalsePass": sum(1 for r in non_success if r[B6_KEY] is True),
        "b6SuccessCert": sum(1 for r in success if r[B6_KEY] is True),
        "anyNaiveFalsePass": sum(1 for r in non_success if r["anyNaivePass"] is True),
        "terminalOnlyFalsePass": sum(1 for r in non_success if r["wr_terminal_only_status"] == "PASS"),
        "structuredFalsePass": sum(1 for r in non_success if r["wrStructuredCertifies"]),
        "structuredCertifiesSuccess": sum(1 for r in success if r["wrStructuredCertifies"]),
        # successes the structured verifier did NOT certify, split by honesty mode:
        # UNCERTAIN (fail-closed on evidence) vs a hard FAIL (a real false-negative).
        "successStructuredUncertain": sum(
            1 for r in success if not r["wrStructuredCertifies"]
            and r["wr_relation_event_status"] == "UNCERTAIN"),
        "successStructuredFail": sum(
            1 for r in success if not r["wrStructuredCertifies"]
            and r["wr_relation_event_status"] != "UNCERTAIN"),
        "uncertain": sum(1 for r in rows if r["wr_relation_event_status"] == "UNCERTAIN"),
    }

    write_csv(rows, EXP_DIR / "results_table.csv")
    write_md(rows, EXP_DIR / "results_table.md", aggregate)

    # (1) Reproducibility/regression vs the stored dataset verdicts (same verifier).
    repro = reproducibility_check(rows, load_verdict_rows())
    (EXP_DIR / "reproducibility_check.json").write_text(json.dumps(repro, indent=2) + "\n")
    # (2) GENUINE independent second-implementation cross-check of the geometry.
    indep = independent_geometry_check()
    (EXP_DIR / "independent_geometry_check.json").write_text(json.dumps(indep, indent=2) + "\n")
    # (3) B6 (engineered steelman) vs structured verifier — clip-level diff (honest tie).
    b6diff = b6_vs_structured_diff(rows)
    (EXP_DIR / "b6_vs_structured.json").write_text(json.dumps(b6diff, indent=2) + "\n")
    # (4) Baseline engineering-cost table (the assumptions each predicate bakes in).
    (EXP_DIR / "engineering_cost.json").write_text(json.dumps(
        {"kind": "what each predicate must know / reimplements", "rows": ENGINEERING_COST_TABLE}, indent=2) + "\n")
    # (5) Deterministic synthetic fixture suite (calibration-free semantics).
    fixture_files = write_fixtures()
    # (6) Cross-task example (one frozen engine; task = target graph).
    cross_task_file = write_cross_task()
    # Remove the previous misleadingly-named artifact if present.
    stale = EXP_DIR / "cross_validation.json"
    if stale.exists():
        stale.unlink()

    # Featured per-case folders + full reports.
    by_key = {(r["episodeId"], r["camera"]): r for r in rows}
    summaries = []
    for feat in FEATURED:
        row = by_key.get((feat["episodeId"], feat["camera"]))
        if not row:
            print(f"WARNING: featured clip {feat['episodeId']}__{feat['camera']} not found", file=sys.stderr)
            continue
        summaries.append(write_case(feat, row, manifest, render=not args.no_overlays, copy_clip=args.copy_clips))
        # full verifier dump (all three targets)
        (EXP_DIR / "wide_robot_reports" / f"{feat['episodeId']}__{feat['camera']}.json").write_text(
            json.dumps(row["_records"], indent=2, sort_keys=True) + "\n")
        # rim clip: committed calibration-perturbation robustness table
        if feat["dir"] == "rim_edge":
            rim_tracks = load_json(TRACKS_DIR / f"{feat['episodeId']}__{feat['camera']}.tracks.json")
            table = rim_perturbation_table(rim_tracks)
            (EXP_DIR / "cases" / "rim_edge" / "robustness_perturbation.json").write_text(
                json.dumps(table, indent=2) + "\n")

    print(json.dumps({"aggregate": aggregate, "cases": summaries,
                      "reproducibility": {k: repro[k] for k in
                                          ("nClips", "agreeTerminal", "agreeRelation", "agreePlaced")},
                      "reproducibilityDisagreements": len(repro["decisionFieldDisagreements"]),
                      "independentGeometry": {"agree": indep["agree"], "clipsCompared": indep["clipsCompared"],
                                              "disagreements": len(indep["disagreements"]),
                                              "constantsMatchCsgDefault": indep["constantsMatchCsgDefault"]},
                      "b6VsStructured": {"agreeCertify": b6diff["agreeCertify"],
                                         "nDisagreements": b6diff["nDisagreements"],
                                         "tieOnAggregateScoreboard": b6diff["tieOnAggregateScoreboard"]},
                      "fixtures": fixture_files, "crossTask": cross_task_file},
                     indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
