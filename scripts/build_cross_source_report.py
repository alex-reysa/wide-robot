#!/usr/bin/env python3
"""Build the ``experiments/cross_source_oic/`` artifact — the "One Task, Four Worlds" report.

Runs the SAME semantic task, ``object_inside_container`` (*did the object end up inside the
container, having been put there?*), through the SAME frozen verifier core across four worlds —
MuJoCo internal sim, RLBench external sim, Sony/iPhone real camera, RH20T real-robot video —
and emits one cross-source PASS / FAIL / leakage / validity report. Every verdict is recomputed
LIVE from committed inputs; nothing in ``csg/`` is modified or re-derived. Output is timestamp-free
so re-runs diff cleanly, and reproduces from a clean clone with NO MuJoCo / RLBench / cv2 installed.

Emits into ``experiments/cross_source_oic/``:
  * ``cross_source_report.md``     — public-facing narrative + master table + caveats
  * ``cross_source_report.json``   — the full structured record
  * ``summary.csv``                — one row per clip across all four worlds
  * ``leakage_report.json``        — per-world leakage cleanliness
  * ``target_equivalence.json``    — proof the per-source target cards share one enforced core
  * ``source_manifest.json``       — provenance + committed input paths + counts per world
  * ``per_world/<key>.json``       — per-leg detail dumps

Usage:
    python3 -m scripts.build_cross_source_report
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.cross_source_oic.legs import all_legs  # noqa: E402
from experiments.cross_source_oic.target_equivalence import build_target_equivalence  # noqa: E402

EXP_DIR = REPO / "experiments" / "cross_source_oic"

CSV_COLUMNS = [
    "world", "clipId", "kind", "groundTruth", "expectedClass",
    "terminal", "relationEvent", "placed", "structuredCertifies",
    "leakageClean", "physicalValidity", "terminalClass", "structuredClass",
]


def _success_breakdown(leg: Mapping[str, Any]) -> Dict[str, int]:
    """Split a world's successes into certify / UNCERTAIN / hard-FAIL (false-negative)."""
    a = leg["aggregate"]
    cert = a["successStructuredCertify"]
    unc = a["successUncertain"]
    hard_fail = a["nSuccess"] - cert - unc
    return {"certify": cert, "uncertain": unc, "hardFail": hard_fail, "total": a["nSuccess"]}


def build_record() -> Dict[str, Any]:
    legs = all_legs()
    te = build_target_equivalence()

    totals = {
        "nClips": sum(L["aggregate"]["nClips"] for L in legs),
        "nSuccess": sum(L["aggregate"]["nSuccess"] for L in legs),
        "nNonSuccess": sum(L["aggregate"]["nFailure"] for L in legs),
        "successStructuredCertify": sum(L["aggregate"]["successStructuredCertify"] for L in legs),
        "successUncertain": sum(L["aggregate"]["successUncertain"] for L in legs),
        "nonSuccessStructuredFalsePass": sum(L["aggregate"]["failureStructuredFalsePass"] for L in legs),
        "noLeakageViolationAnyWorld": all(L["aggregate"]["noLeakageViolation"] for L in legs),
    }
    totals["successHardFail"] = totals["nSuccess"] - totals["successStructuredCertify"] - totals["successUncertain"]

    return {
        "task": "object_inside_container",
        "question": "Did the object end up inside the container, having been put there "
                    "(a real outside→inside transition), not merely born inside?",
        "frozenCore": "csg.matcher.match + csg.rollout_extract.extract_robot_csg (the SAME functions "
                      "verify_external_rollout and csg.benchmark.run_one call). csg/ is byte-frozen.",
        "claim": "The same semantic task, instantiated per source as a target card and judged by the "
                 "same frozen verifier core, PASSes genuine successes, rejects non-successes (0 false "
                 "PASSes in every world), stays leakage-clean, and reports physical validity honestly "
                 "(true only for the internal sim, where physics was actually re-checked; null for the "
                 "three external worlds).",
        "worlds": legs,
        "totals": totals,
        "targetEquivalence": te,
    }


# --------------------------------------------------------------------------------------
# writers
# --------------------------------------------------------------------------------------

def write_summary_csv(legs: List[Dict[str, Any]], path: Path) -> None:
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for leg in legs:
            for c in leg["clips"]:
                w.writerow({"world": leg["worldKey"], **c})


def _validity_label(mode: str) -> str:
    return "**true** (physics re-checked)" if "true" in mode else "null (physics-unverified)"


def write_md(record: Mapping[str, Any], path: Path) -> None:
    legs = record["worlds"]
    totals = record["totals"]
    te = record["targetEquivalence"]
    L: List[str] = []

    L.append("# object_inside_container — One Task, Four Worlds\n")
    L.append("_Generated by `scripts/build_cross_source_report.py`. Every verdict is recomputed live "
             "from committed inputs through the frozen verifier core (`csg.matcher.match` + "
             "`csg.rollout_extract.extract_robot_csg`); `csg/` is byte-frozen. Reproduces from a clean "
             "clone with no MuJoCo / RLBench / cv2 installed._\n")
    L.append(f"\n**The task.** {record['question']}\n")
    L.append(f"\n**The claim.** {record['claim']}\n")

    # ---- master table ----
    L.append("\n## Cross-source scoreboard\n")
    L.append("| World | Verifier path | Successes (certified / total) | UNCERTAIN | hard-FAIL (false-neg) | "
             "Non-success false-PASS | Leakage | physicalValidity |")
    L.append("|---|---|---|---|---|---|---|---|")
    for leg in legs:
        a = leg["aggregate"]
        sb = _success_breakdown(leg)
        leak = "clean" if a["noLeakageViolation"] else "**LEAK**"
        L.append(
            f"| {leg['world']} | `{leg['verifierPath'].split(' ')[0]}` | "
            f"**{sb['certify']}/{sb['total']}** | {sb['uncertain']} | {sb['hardFail']} | "
            f"**{a['failureStructuredFalsePass']}/{a['nFailure']}** | {leak} | "
            f"{_validity_label(leg['physicalValidityMode'])} |")
    L.append(
        f"\n_Across all four worlds: **{totals['successStructuredCertify']}** successes certified, "
        f"**{totals['successUncertain']}** UNCERTAIN (fail-closed on weak evidence), "
        f"**{totals['successHardFail']}** hard-FAIL false-negatives (a known real-camera occlusion limit, "
        f"below), and **{totals['nonSuccessStructuredFalsePass']}** false-PASSes on "
        f"**{totals['nNonSuccess']}** non-success clips. Leakage-clean in every world: "
        f"**{totals['noLeakageViolationAnyWorld']}**._\n")
    L.append("\n_\"Certified\" = the STRUCTURED tier PASSes (a real outside→inside put-in: "
             "`relation_event` near-start OR `placed_from_outside` far-start). \"false-PASS\" = a "
             "non-success clip the structured tier wrongly certifies. The weak `terminal_only` tier "
             "(\"did it merely END inside?\") is intentionally easier — it is what a born-inside episode "
             "passes, which is exactly why the structured tier exists. The UNCERTAIN column counts "
             "SUCCESS-clip abstentions only; non-success clips that are UNCERTAIN are counted as correctly "
             "rejected (they do not certify), not as false-PASSes._\n")

    # ---- per-world detail ----
    L.append("\n## Per world\n")
    for leg in legs:
        a = leg["aggregate"]
        L.append(f"\n### {leg['world']}\n")
        L.append(f"- **source:** {leg['source']}")
        L.append(f"- **verifier path:** `{leg['verifierPath']}`")
        L.append(f"- **physical validity:** {_validity_label(leg['physicalValidityMode'])}")
        if leg["worldKey"] == "mujoco":
            ng = leg["nativeGold"]
            ac = a["acceptanceCorpus"]
            L.append(f"- the genuine MuJoCo solver rollout PASSes `terminal_only` AND the structured tier, "
                     f"with **physicalValidity = true** (5/5 applicable physics checks pass — "
                     f"articulation_limits is N/A, no articulated joints). It also PASSes its native full "
                     f"pick-place gold target `{ng['graphId']}` (a SUPERSET adding contact / carry / "
                     f"temporal-order probes: passed={ng['passed']}).")
            L.append(f"- internal acceptance corpus (committed `gold_tests/put_cube_in_tray/` robot CSGs): "
                     f"the frozen matcher PASSes the success and FAILs all **{ac['sabotagesFailed']}** "
                     f"sabotaged variants (missing-contact / removed-at-end / wrong-order / wrong-relation) — "
                     f"every verdict matches the committed `expected.json` ({ac['allMatchExpected']}).")
        elif leg["worldKey"] == "rlbench":
            L.append(f"- all **{a['nSuccess']}** live demos PASS `terminal_only`; the structured tier "
                     f"certifies every one (**{a['relationEventPass']}** near-start via `relation_event`, "
                     f"**{a['placedFromOutsidePass']}** far-start via `placed_from_outside`).")
            L.append(f"- in-data discrimination (a success-only world): each demo FAILs the WRONG-precondition "
                     f"structured tier (**{a['wrongTierRejections']}/{a['nSuccess']}** wrong-tier rejections) — "
                     f"the verifier is reading the demonstrated start state, not rubber-stamping.")
        elif leg["worldKey"] == "sony":
            sb = _success_breakdown(leg)
            L.append(f"- **{sb['certify']}/{sb['total']}** genuine successes certified; **{sb['uncertain']}** "
                     f"UNCERTAIN (fail-closed on occlusion/low-confidence — correct abstentions, not wrong "
                     f"FAILs); **{sb['hardFail']}** hard-FAIL false-negatives where brief hand/tag obstruction "
                     f"corrupts the terminal relation without tripping the evidence gate (an honest known "
                     f"limitation, not part of the claim).")
            L.append(f"- **{a['failureStructuredFalsePass']}/{a['nFailure']}** structured false-PASSes on "
                     f"non-success clips. The **{a['failureTerminalPass']}** non-success clips that pass the "
                     f"WEAK `terminal_only` tier are object-already-inside controls (born-inside / "
                     f"inside-to-inside) — they correctly FAIL the structured tier (no outside→inside "
                     f"transition), which is the whole point of having it.")
            L.append(f"- evidence-quality thresholds (relaxed 30fps): `{leg['evidenceThresholds']}` "
                     f"(the same `EVIDENCE_THRESHOLDS` the ingest pipeline + baseline experiment use).")
        elif leg["worldKey"] == "rh20t":
            L.append(f"- the real positive episode (pen→holder) PASSes `terminal_only` AND `relation_event` "
                     f"(non-vacuous); the derived negative (pen placed beside the holder) FAILs both. "
                     f"0 false-PASS.")
            L.append("- source-blind: the rollout carries only a neutral `RH20T` label + a one-way episode "
                     "hash + the archive digest — no task id, description, or scene path reaches the verifier.")

    # ---- target equivalence ----
    L.append("\n## Same task, not the same file — target-card equivalence\n")
    L.append("Each world is verified against its OWN target card (different `graphId`, object labels, "
             "geometry provenance, captions). They are NOT one shared file. But every card reduces to "
             "the SAME verifier-enforced semantic core per tier, with all ids canonicalised to roles and "
             "authoring-only fields stripped:\n")
    L.append("| Tier | Sources compared | Enforced cores identical? |")
    L.append("|---|---|---|")
    for tier, t in te["perTier"].items():
        L.append(f"| `{tier}` | {', '.join(t['sources'])} | **{t['allIdentical']}** "
                 f"({t['distinctSignatureCount']} distinct signature) |")
    ts = te["tierStrengthening"]
    L.append(f"\n- the tiers are nested: `terminal_only` enforces only the INSIDE goal "
             f"({ts['terminalOnlyHasNoRelationsOrEvents']}); `relation_event` adds the authored "
             f"NEAR→INSIDE endpoints + the containment event ({ts['relationEventAddsEndpointsAndEvent']}); "
             f"the INSIDE goal is unchanged across all tiers ({ts['terminalGoalUnchangedAcrossTiers']}).")
    L.append(f"- `placed_from_outside` differs from `relation_event` ONLY in the initial authored "
             f"relation endpoint (r0: NEAR vs FAR_FROM) — the single fact `initial_state` reads to tell a "
             f"near-start put-in from a far-start one "
             f"({ts['placedDiffersFromRelationEventOnlyInStartEndpoint']}).")
    inn = te["internalSim"]
    L.append(f"- the MuJoCo internal world is judged against its native gold pick-place target "
             f"(`{inn['mujocoGoldGraphId']}`), a SUPERSET; its INSIDE goal core matches the shared core "
             f"({inn['goalCoreMatchesSharedCore']}) — the internal world is judged at a STRONGER tier, "
             f"never a weaker one. Full proof: `target_equivalence.json`.\n")

    L.append("\n## Honest caveats\n")
    L.append("- **Two verifier paths, one core.** The three external worlds go through "
             "`verify_external_rollout` (the leakage door + the frozen core); MuJoCo goes through the frozen "
             "core directly (`extract_robot_csg → match → leakage_report`, == `csg.benchmark.run_one`) "
             "because its legitimate internal `objectIdMap` would (correctly) be rejected by the external door. "
             "The matcher and relation extractor are the identical frozen functions in both paths.")
    L.append("- **physicalValidity is honest, not uniform.** `true` ONLY for MuJoCo (physics genuinely "
             "re-checked at capture); `null` for the three external worlds (a recorded trace cannot be "
             "physics-revalidated). A `null` PASS is labelled *physics-unverified*, never *valid*.")
    L.append("- **Failure data is uneven across worlds.** Sony (40 non-success clips) and RH20T (1 derived "
             "negative) carry explicit failures; the two sim worlds are success-only in their committed live "
             "data, so their discrimination is shown via the wrong-tier rejection (RLBench) and the committed "
             "sabotage corpus (MuJoCo).")
    L.append("- **Sony hard-FAIL false-negatives are real.** A handful of genuine successes hard-FAIL when "
             "brief obstruction corrupts the terminal relation without tripping the evidence gate. They are "
             "reported, not hidden; they are a perception limit of marker-only 3A capture, not a verifier claim.\n")

    path.write_text("\n".join(L) + "\n")


def write_artifacts(record: Mapping[str, Any]) -> Dict[str, Any]:
    EXP_DIR.mkdir(parents=True, exist_ok=True)
    (EXP_DIR / "per_world").mkdir(exist_ok=True)
    legs = record["worlds"]

    (EXP_DIR / "cross_source_report.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    write_summary_csv(legs, EXP_DIR / "summary.csv")
    write_md(record, EXP_DIR / "cross_source_report.md")

    # leakage report (per world)
    leak = {
        "kind": "per-world leakage cleanliness through the frozen leakage gate(s)",
        "worlds": {leg["worldKey"]: {
            "noLeakageViolation": leg["aggregate"]["noLeakageViolation"],
            "leakageCleanCount": leg["aggregate"]["leakageCleanCount"],
            "nClips": leg["aggregate"]["nClips"],
            "note": "UNCERTAIN clips report leakageClean=n/a (no rollout minted); none is a violation.",
        } for leg in legs},
        "noLeakageViolationAnyWorld": record["totals"]["noLeakageViolationAnyWorld"],
    }
    (EXP_DIR / "leakage_report.json").write_text(json.dumps(leak, indent=2) + "\n")

    # target equivalence
    (EXP_DIR / "target_equivalence.json").write_text(json.dumps(record["targetEquivalence"], indent=2) + "\n")

    # source manifest (provenance + committed inputs)
    manifest = {
        "task": record["task"],
        "frozenCore": record["frozenCore"],
        "worlds": {leg["worldKey"]: {
            "world": leg["world"],
            "source": leg["source"],
            "verifierPath": leg["verifierPath"],
            "physicalValidityMode": leg["physicalValidityMode"],
            "nClips": leg["aggregate"]["nClips"],
            "tiersRun": leg["tiersRun"],
        } for leg in legs},
        "committedInputs": {
            "mujoco": "experiments/cross_source_oic/mujoco_fixture/ (+ gold_tests/put_cube_in_tray/)",
            "rlbench": "pilots/rlbench/fixtures/live_runpod_20260616_put_item/",
            "sony": "datasets/sony_object_inside_container_v0/{tracks,verdicts_all.json}",
            "rh20t": "datasets/rh20t_object_inside_container_v0/rollouts/",
            "targets": "pilots/{rlbench,real_camera,rh20t}/targets/object_inside_container_*.json",
        },
    }
    (EXP_DIR / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    # per-world detail dumps
    for leg in legs:
        (EXP_DIR / "per_world" / f"{leg['worldKey']}.json").write_text(
            json.dumps(leg, indent=2, sort_keys=True) + "\n")

    return {
        "totals": record["totals"],
        "perWorld": {leg["worldKey"]: leg["aggregate"] for leg in legs},
        "allExternalTiersIdentical": record["targetEquivalence"]["allExternalTiersIdentical"],
    }


def main() -> int:
    record = build_record()
    summary = write_artifacts(record)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
