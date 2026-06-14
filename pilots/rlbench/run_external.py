#!/usr/bin/env python3
"""Run an external ``csg.rollout.v0`` trace through the FROZEN csg verifier.

This is the pilot's whole point in one function. The benchmark's ``run_one`` does
``solve(target) -> extract_robot_csg -> match -> leakage_report``; here the solver
is replaced by an external trace (an RLBench demo, via
``pilots.rlbench.adapter``), and the *same* frozen
``extract_robot_csg -> match -> leakage_report`` runs unchanged. Nothing in ``csg``
is imported in an altered form — the verifier does not know the trace came from
RLBench, which is exactly what makes a clean PASS meaningful.

PASS criterion is identical to ``csg.benchmark.run_one``: the hard-probe matcher
passes, the trace is leakage-clean, and physical validity ``is not False`` (an
external kinematic trace reports ``null`` → *physics-unverified*, an honest PASS
label, never *valid*).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

from csg.common import Json, load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg
from csg.benchmark import leakage_report, classify_failure, KNOWN_EQUIVALENT_TASKS

from .adapter import assert_rollout_leakage_clean


def verify_external_rollout(
    target: Mapping[str, Any],
    rollout: Mapping[str, Any],
    *,
    matcher_cfg: Optional[MatcherConfig] = None,
    case_name: str = "external",
    base_case: Optional[str] = None,
) -> Json:
    """Verify one external rollout against ``target`` with the frozen verifier.

    Raises :class:`pilots.rlbench.adapter.ExternalTraceLeakage` if the rollout
    carries target-authored keys or non-whitelisted body fields — a malformed/
    cheating external trace fails at the door, before the matcher ever runs.
    """
    assert_rollout_leakage_clean(rollout)  # first line of defence (rollout-level)

    robot = extract_robot_csg(rollout)
    result = match(target, robot, matcher_cfg or MatcherConfig())
    leak = leakage_report(robot)                                   # second line (robot-CSG level)

    diag = rollout.get("diagnostics", {}) or {}
    validity = diag.get("physicalValidity", None)
    passed = result.passed and leak["clean"] and validity is not False
    case: Json = {
        "case": case_name,
        "baseCase": base_case or case_name,
        "traceSource": rollout.get("backend"),
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "matcherPassed": result.passed,
        "leakageClean": leak["clean"],
        "physicalValidity": validity,                              # None ⇒ physics-unverified (honest)
        "physicalValidityReason": diag.get("physicalValidityReason"),
        "vacuous": result.vacuous,
        "distance": result.distance,
        "probeAgreement": result.probe_agreement,
        "probeSupport": result.probe_support,
        "hardMismatches": [p for p in result.hard_probes if not result.probe_agreement[p]],
        "leakage": leak,
        "objectMapping": result.object_mapping,
        "objectOrbitAmbiguous": result.object_orbit_ambiguous,
    }
    case["failureClassification"] = classify_failure(case)
    return case


def load_gold_targets(gold_dir: str | Path) -> Dict[str, Json]:
    """Map ``{task_name: target_json}`` from ``gold_dir/<task>/target.json``.

    Task names are the gold directory names (``open_drawer``, ``put_cube_in_tray``,
    …) — the same neutral case names ``csg.benchmark`` uses, never RLBench labels.
    """
    root = Path(gold_dir)
    out: Dict[str, Json] = {}
    for target_path in sorted(root.glob("*/target.json")):
        out[target_path.parent.name] = load_json(target_path)
    return out


def external_confusion_report(
    rollout: Mapping[str, Any],
    gold_targets: Mapping[str, Json],
    *,
    expected_case: str,
    matcher_cfg: Optional[MatcherConfig] = None,
    equivalent_tasks: Sequence[frozenset] = KNOWN_EQUIVALENT_TASKS,
) -> Json:
    """Cross-task confusion for ONE external rollout: match its robot CSG against
    every gold target.

    The mirror of ``csg.benchmark.confusion_matrix`` for the external seam, but 1×N
    (one external rollout vs N gold targets). A meaningful external PASS requires the
    rollout PASS its *own* task's target (``expected_case``) AND FAIL every
    non-equivalent target — evidence the verifier is testing the demonstrated
    behaviour, not csg-solver-specific trajectory shape. ``equivalent_tasks`` reuses
    the benchmark's documented quotient equivalences so an off-task PASS that is
    *expected* (e.g. insert_object ~ put_cube_in_tray) is not flagged.
    """
    assert_rollout_leakage_clean(rollout)  # confusion never runs on a leaky trace
    robot = extract_robot_csg(rollout)
    cfg = matcher_cfg or MatcherConfig()

    def _equiv(a: str, b: str) -> bool:
        return a == b or any({a, b} <= set(eq) for eq in equivalent_tasks)

    results = {name: bool(match(target, robot, cfg).passed) for name, target in gold_targets.items()}
    passes = sorted(n for n, p in results.items() if p)
    unexpected = sorted(n for n in passes if not _equiv(n, expected_case))
    expected_present = expected_case in gold_targets
    missed = expected_case if (expected_present and not results.get(expected_case, False)) else None
    return {
        "expectedCase": expected_case,
        "expectedCasePresent": expected_present,
        "results": results,
        "passes": passes,
        "expectedPass": sorted(n for n in gold_targets if _equiv(n, expected_case)),
        "unexpectedOffTaskPasses": unexpected,
        "missedExpected": missed,
        "confusionClean": not unexpected and missed is None and expected_present,
        "knownEquivalentTasks": sorted(sorted(eq) for eq in equivalent_tasks),
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run an external csg.rollout.v0 trace through the frozen csg verifier.")
    parser.add_argument("--target", required=True, help="target CSG json (e.g. gold_tests/open_drawer/target.json)")
    parser.add_argument("--rollout", required=True, help="external csg.rollout.v0 json (from the RLBench adapter)")
    parser.add_argument("--json", action="store_true", help="print the full case record")
    parser.add_argument("--confusion", action="store_true",
                        help="also match this rollout against every gold target (must PASS its own "
                             "task and FAIL non-equivalent ones)")
    parser.add_argument("--gold-dir", default="gold_tests",
                        help="directory of gold tasks (<task>/target.json) for the confusion check")
    parser.add_argument("--case-name", default=None,
                        help="expected task name for the confusion diagonal "
                             "(default: the parent directory name of --target, e.g. open_drawer "
                             "for gold_tests/open_drawer/target.json)")
    args = parser.parse_args(argv)

    target = load_json(Path(args.target))
    rollout = load_json(Path(args.rollout))
    case_name = args.case_name or Path(args.target).resolve().parent.name or Path(args.rollout).stem
    case = verify_external_rollout(target, rollout, case_name=case_name)

    confusion: Optional[Json] = None
    if args.confusion:
        gold = load_gold_targets(args.gold_dir)
        confusion = external_confusion_report(rollout, gold, expected_case=case_name)
        case["confusion"] = confusion

    if args.json:
        print(json.dumps(case, indent=2, sort_keys=True))
    else:
        print(f"external-verify status={case['status']} matcher={case['matcherPassed']} "
              f"leakageClean={case['leakageClean']} physicalValidity={case['physicalValidity']} "
              f"traceSource={case['traceSource']}")
        if case["hardMismatches"]:
            print(f"  hard-probe mismatches: {case['hardMismatches']}")
        if confusion is not None:
            verdict = "CLEAN" if confusion["confusionClean"] else "DIRTY"
            print(f"  confusion[{confusion['expectedCase']}] {verdict}: "
                  f"passes={confusion['passes']}")
            if confusion["unexpectedOffTaskPasses"]:
                print(f"    UNEXPECTED off-task passes: {confusion['unexpectedOffTaskPasses']}")
            if confusion["missedExpected"]:
                print(f"    MISSED expected diagonal: {confusion['missedExpected']}")

    ok = case["passed"] and (confusion is None or confusion["confusionClean"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
