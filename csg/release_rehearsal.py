#!/usr/bin/env python3
"""Run or print the Phase 2E release rehearsal command sequence."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any, Dict, List

from .common import Json, write_json


def _s(value: str | Path) -> str:
    return str(value)


def _step(name: str, argv: List[str]) -> Json:
    return {"name": name, "argv": argv}


def build_rehearsal_plan(
    *,
    out_root: str | Path,
    python: str | Path = "python3",
    sim_python: str | Path = ".venv-sim/bin/python",
    seeds: int = 30,
    require_final_metadata: bool = False,
    project_root: str | Path = ".",
) -> Json:
    out = Path(out_root)
    py = _s(python)
    sim = _s(sim_python)
    symbolic = out / "symbolic"
    mujoco = out / "mujoco"
    randomized = out / f"mujoco_randomized_{seeds}"
    comparison = out / "comparison"
    invalid = out / "invalid_fixtures"

    audit = [
        py, "-m", "csg.release_audit",
        "--symbolic", str(symbolic),
        "--mujoco", str(mujoco),
        "--randomized", str(randomized),
        "--comparison", str(comparison),
        "--invalid-fixtures", str(invalid),
        "--seeds", str(seeds),
    ]
    if require_final_metadata:
        audit += ["--require-final-metadata", "--project-root", _s(project_root)]

    return {
        "schemaVersion": "csg.release_rehearsal_plan.v1",
        "outRoot": str(out),
        "seeds": int(seeds),
        "strictFinalMetadata": bool(require_final_metadata),
        "outputs": {
            "symbolic": str(symbolic),
            "mujoco": str(mujoco),
            "randomized": str(randomized),
            "comparison": str(comparison),
            "invalidFixtures": str(invalid),
        },
        "steps": [
            _step("core_tests", [py, "-m", "pytest", "tests/", "-q"]),
            _step("symbolic_gold", [
                py, "-m", "csg.benchmark", "gold_tests", "--confusion",
                "--require-pass", "--out", str(symbolic),
            ]),
            _step("mujoco_tests", [sim, "-m", "pytest", "tests/", "-q"]),
            _step("mujoco_gold", [
                sim, "-m", "csg.benchmark", "gold_tests", "--backend", "mujoco",
                "--confusion", "--require-pass", "--out", str(mujoco),
            ]),
            _step("mujoco_randomized", [
                sim, "-m", "csg.benchmark", "gold_tests", "--backend", "mujoco",
                "--confusion", "--randomized", "--seeds", str(seeds),
                "--require-pass", "--out", str(randomized),
            ]),
            _step("backend_comparison", [
                sim, "-m", "csg.benchmark", "gold_tests", "--compare-backends",
                "symbolic,noop,mujoco", "--confusion", "--require-pass", "--out",
                str(comparison),
            ]),
            _step("invalid_fixtures", [
                sim, "-m", "csg.benchmark", "--invalid-fixtures", "gold_invalid",
                "--require-pass", "--out", str(invalid),
            ]),
            _step("release_audit", audit),
        ],
    }


def run_rehearsal(plan: Json, *, cwd: str | Path = ".") -> Json:
    results: List[Json] = []
    for step in plan["steps"]:
        proc = subprocess.run(step["argv"], cwd=cwd, text=True, capture_output=True)
        result = {
            "name": step["name"],
            "argv": step["argv"],
            "returncode": proc.returncode,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
        }
        results.append(result)
        if proc.returncode != 0:
            break
    return {
        "schemaVersion": "csg.release_rehearsal_result.v1",
        "ok": bool(results) and all(r["returncode"] == 0 for r in results) and len(results) == len(plan["steps"]),
        "plan": plan,
        "results": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run or print the Phase 2E release rehearsal.")
    parser.add_argument("--out", "--out-root", dest="out_root", default="phase2e_release_out")
    parser.add_argument("--python", default="python3")
    parser.add_argument("--sim-python", default=".venv-sim/bin/python")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--require-final-metadata", action="store_true")
    parser.add_argument("--project-root", default=".")
    parser.add_argument("--dry-run", action="store_true", help="print the command plan without running it")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    plan = build_rehearsal_plan(
        out_root=args.out_root,
        python=args.python,
        sim_python=args.sim_python,
        seeds=args.seeds,
        require_final_metadata=args.require_final_metadata,
        project_root=args.project_root,
    )
    if args.dry_run:
        if args.json:
            print(json.dumps(plan, indent=2, sort_keys=True))
        else:
            for step in plan["steps"]:
                print(f"{step['name']}: {' '.join(step['argv'])}")
        return 0

    result = run_rehearsal(plan)
    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    write_json(out_root / "release_rehearsal_result.json", result)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"release rehearsal ok={result['ok']} steps={len(result['results'])}/{len(plan['steps'])}")
        for step in result["results"]:
            print(f"  {step['name']}: rc={step['returncode']}")
            if step["returncode"] != 0:
                break
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
