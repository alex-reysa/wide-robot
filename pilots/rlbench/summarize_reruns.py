#!/usr/bin/env python3
"""Aggregate a directory of external RLBench rollouts into reproducibility rates.

The single-rollout seam is :mod:`pilots.rlbench.run_external`; this is its N-rollout
rollup, for a *deliberate* rerun (e.g. 3 demos x bottom/middle/top = 9 traces). It
answers the reproducibility question the first single trace could not: across many
fresh demos, does the value-only target PASS every time, does the gold target FAIL
every time leakage-clean, and does the trace never accidentally match an off-task
target?

For each ``*.rollout.json`` it runs the **frozen** verifier three ways and records:
  * value-only target  -> expected PASS   (``valueOnlyPass``)
  * gold open_drawer    -> expected FAIL   (``goldFail`` + ``goldLeakageClean``)
  * 1xN confusion       -> expected no off-task pass (``offTaskClean``)

A leaky rollout is recorded as a failure across the board (and never reaches the
matcher), not a crash — one bad demo in nine must not abort the summary.

``strongResult`` is the headline: N>0 demos, every one leakage-clean, value-only
PASS on all, gold FAIL-leakage-clean on all, and off-task-clean on all. That is the
"9/9 value-only PASS, 9/9 gold FAIL leakage-clean" bar, made executable.

This module imports no RLBench and touches no ``csg`` internals; it consumes only the
frozen verifier through ``run_external``. Run it on the committed 9-demo rerun fixtures
today (the 9/9 strong result, reproducible with no RLBench):

    python3 -m pilots.rlbench.summarize_reruns \
      --rollouts-dir pilots/rlbench/fixtures/live_runpod_20260614_rerun
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import Json, load_json
from csg.matcher import MatcherConfig

from .adapter import ExternalTraceLeakage
from .run_external import external_confusion_report, load_gold_targets, verify_external_rollout

# The committed value-only diagnostic target (Result B). Resolved relative to this
# file so the default works from any CWD.
DEFAULT_VALUE_ONLY_TARGET = Path(__file__).resolve().parent / "targets" / "open_drawer_rlbench_value_only.json"
DEFAULT_EXPECTED_CASE = "open_drawer"


def _summarize_one(
    path: Path,
    rollout: Mapping[str, Any],
    *,
    gold_target: Mapping[str, Any],
    value_only_target: Mapping[str, Any],
    gold_targets: Mapping[str, Json],
    expected_case: str,
    matcher_cfg: MatcherConfig,
) -> Dict[str, Any]:
    """One rollout's verdicts against gold, value-only, and off-task confusion."""
    record: Dict[str, Any] = {
        "rollout": str(path),
        "leakageClean": False,
        "valueOnlyPass": False,
        "goldFail": False,
        "goldLeakageClean": False,
        "offTaskClean": False,
        "offTaskPasses": [],
        "goldHardMismatches": [],
        "error": None,
    }
    try:
        gold_case = verify_external_rollout(gold_target, rollout, case_name=expected_case, matcher_cfg=matcher_cfg)
        vo_case = verify_external_rollout(
            value_only_target, rollout, case_name=f"{expected_case}_value_only", matcher_cfg=matcher_cfg)
        confusion = external_confusion_report(
            rollout, gold_targets, expected_case=expected_case, matcher_cfg=matcher_cfg)
    except ExternalTraceLeakage as exc:
        # Leaky trace: the verifier slams the door before matching. Record it as a
        # clean failure of every criterion (it cannot count toward a strong result).
        record["error"] = f"leakage: {exc}"
        return record

    record["leakageClean"] = bool(gold_case["leakageClean"])
    record["valueOnlyPass"] = bool(vo_case["passed"])
    record["goldFail"] = not bool(gold_case["passed"])
    record["goldLeakageClean"] = bool(gold_case["leakageClean"])
    record["goldHardMismatches"] = list(gold_case["hardMismatches"])
    record["offTaskPasses"] = list(confusion["unexpectedOffTaskPasses"])
    record["offTaskClean"] = confusion["unexpectedOffTaskPasses"] == []
    return record


def summarize_reruns(
    rollout_paths: Sequence[Path],
    *,
    gold_target: Mapping[str, Any],
    value_only_target: Mapping[str, Any],
    gold_targets: Mapping[str, Json],
    expected_case: str = DEFAULT_EXPECTED_CASE,
    matcher_cfg: Optional[MatcherConfig] = None,
) -> Json:
    """Verify each rollout three ways and aggregate reproducibility rates.

    ``gold_targets`` is the confusion set (``{task: target}``); ``gold_target`` is the
    expected-case gold target (normally ``gold_targets[expected_case]``).
    """
    cfg = matcher_cfg or MatcherConfig()
    per_rollout = [
        _summarize_one(
            path, load_json(path),
            gold_target=gold_target, value_only_target=value_only_target,
            gold_targets=gold_targets, expected_case=expected_case, matcher_cfg=cfg,
        )
        for path in rollout_paths
    ]
    n = len(per_rollout)
    n_value_only = sum(1 for r in per_rollout if r["valueOnlyPass"])
    n_gold_fail_clean = sum(1 for r in per_rollout if r["goldFail"] and r["goldLeakageClean"])
    n_off_task_clean = sum(1 for r in per_rollout if r["offTaskClean"])
    n_leak_clean = sum(1 for r in per_rollout if r["leakageClean"])
    strong = (
        n > 0
        and n_value_only == n
        and n_gold_fail_clean == n
        and n_off_task_clean == n
        and n_leak_clean == n
    )
    return {
        "expectedCase": expected_case,
        "nRollouts": n,
        "rates": {
            "valueOnlyPass": [n_value_only, n],
            "goldFailLeakageClean": [n_gold_fail_clean, n],
            "offTaskClean": [n_off_task_clean, n],
            "leakageClean": [n_leak_clean, n],
        },
        "strongResult": strong,
        "perRollout": per_rollout,
    }


def discover_rollouts(rollouts_dir: str | Path) -> List[Path]:
    """Sorted ``*.rollout.json`` directly under ``rollouts_dir`` (the recorder writes
    them flat). Summary sidecars (``*.summary.json``) are deliberately excluded."""
    return sorted(Path(rollouts_dir).glob("*.rollout.json"))


def _rate(pair: Sequence[int]) -> str:
    k, n = pair
    return f"{k}/{n}"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Aggregate a directory of external RLBench rollouts into reproducibility rates "
                    "(value-only PASS / gold FAIL-leakage-clean / off-task-clean).")
    parser.add_argument("--rollouts-dir", required=True,
                        help="directory of *.rollout.json (e.g. a record_open_drawer --out-dir)")
    parser.add_argument("--gold-dir", default="gold_tests",
                        help="directory of gold tasks (<task>/target.json) for gold + confusion")
    parser.add_argument("--value-only-target", default=str(DEFAULT_VALUE_ONLY_TARGET),
                        help="value-only diagnostic target json (Result B)")
    parser.add_argument("--expected-case", default=DEFAULT_EXPECTED_CASE,
                        help="gold task the demos demonstrate (default: open_drawer)")
    parser.add_argument("--json", action="store_true", help="print the full summary record")
    args = parser.parse_args(argv)

    rollout_paths = discover_rollouts(args.rollouts_dir)
    if not rollout_paths:
        print(f"no *.rollout.json found under {args.rollouts_dir}", file=sys.stderr)
        return 2

    gold_targets = load_gold_targets(args.gold_dir)
    if args.expected_case not in gold_targets:
        print(f"expected case {args.expected_case!r} not in gold dir {args.gold_dir}", file=sys.stderr)
        return 2
    gold_target = gold_targets[args.expected_case]
    value_only_target = load_json(Path(args.value_only_target))

    summary = summarize_reruns(
        rollout_paths,
        gold_target=gold_target, value_only_target=value_only_target,
        gold_targets=gold_targets, expected_case=args.expected_case,
    )

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(f"rerun summary [{summary['expectedCase']}]: {summary['nRollouts']} rollout(s)")
        for r in summary["perRollout"]:
            name = Path(r["rollout"]).name
            if r["error"]:
                print(f"  {name}: ERROR {r['error']}")
                continue
            vo = "PASS" if r["valueOnlyPass"] else "FAIL"
            gold = "FAIL" if r["goldFail"] else "PASS"
            off = "clean" if r["offTaskClean"] else f"OFF-TASK {r['offTaskPasses']}"
            print(f"  {name}: value-only={vo} gold={gold} leakage={'clean' if r['leakageClean'] else 'DIRTY'} "
                  f"confusion={off}")
        rates = summary["rates"]
        print(f"  rates: value-only PASS {_rate(rates['valueOnlyPass'])} | "
              f"gold FAIL-leakage-clean {_rate(rates['goldFailLeakageClean'])} | "
              f"off-task-clean {_rate(rates['offTaskClean'])} | "
              f"leakage-clean {_rate(rates['leakageClean'])}")
        print(f"  STRONG RESULT: {'YES' if summary['strongResult'] else 'NO'}")

    return 0 if summary["strongResult"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
