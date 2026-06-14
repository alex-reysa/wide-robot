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
from typing import Any, Mapping, Optional

from csg.common import Json, load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg
from csg.benchmark import leakage_report, classify_failure

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


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run an external csg.rollout.v0 trace through the frozen csg verifier.")
    parser.add_argument("--target", required=True, help="target CSG json (e.g. gold_tests/open_drawer/target.json)")
    parser.add_argument("--rollout", required=True, help="external csg.rollout.v0 json (from the RLBench adapter)")
    parser.add_argument("--json", action="store_true", help="print the full case record")
    args = parser.parse_args(argv)

    target = load_json(Path(args.target))
    rollout = load_json(Path(args.rollout))
    case = verify_external_rollout(target, rollout, case_name=Path(args.rollout).stem)

    if args.json:
        print(json.dumps(case, indent=2, sort_keys=True))
    else:
        print(f"external-verify status={case['status']} matcher={case['matcherPassed']} "
              f"leakageClean={case['leakageClean']} physicalValidity={case['physicalValidity']} "
              f"traceSource={case['traceSource']}")
        if case["hardMismatches"]:
            print(f"  hard-probe mismatches: {case['hardMismatches']}")
    return 0 if case["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
