#!/usr/bin/env python3
"""Validate Phase 2E release report artifacts."""
from __future__ import annotations

import argparse
import json
import tomllib
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Mapping

from .common import Json, get_any


def _load_json(path: Path) -> Json | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _check(checks: List[Json], name: str, ok: bool, message: str) -> None:
    checks.append({"name": name, "ok": bool(ok), "message": message})


def _require_file(checks: List[Json], path: Path, name: str) -> bool:
    ok = path.is_file()
    _check(checks, name, ok, f"{path} {'present' if ok else 'missing'}")
    return ok


def _has_source_provenance(report: Mapping[str, Any]) -> bool:
    return get_any(report.get("sourceProvenance", {}) or {}, "schemaVersion", default=None) == "csg.source_provenance.v1"


def _source_provenance_kind(report: Mapping[str, Any]) -> str:
    return str(get_any(report.get("sourceProvenance", {}) or {}, "kind", default=""))


def _audit_benchmark_dir(
    checks: List[Json],
    label: str,
    path: Path,
    *,
    total: int,
    passed: int,
    physical: Mapping[str, int],
    randomized: bool = False,
    seeds: int = 30,
    require_git_provenance: bool = False,
) -> None:
    required = ["report.json", "report.md", "summary.csv", "failure_classification.json"]
    for filename in required:
        _require_file(checks, path / filename, f"{label}:{filename}")
    report = _load_json(path / "report.json")
    if report is None:
        return

    summary = report.get("summary", {}) or {}
    _check(checks, f"{label}:schema", report.get("schemaVersion") == "csg.benchmark_report.v2",
           f"{label} report schema {report.get('schemaVersion')}")
    _check(checks, f"{label}:sourceProvenance", _has_source_provenance(report),
           f"{label} sourceProvenance present")
    if require_git_provenance:
        kind = _source_provenance_kind(report)
        _check(checks, f"{label}:sourceProvenance:git", kind == "git",
               f"{label} sourceProvenance kind expected git got {kind}")
    for key, expected in {"total": total, "passed": passed, "failed": total - passed}.items():
        actual = summary.get(key)
        _check(checks, f"{label}:summary:{key}", actual == expected,
               f"{label} summary {key} expected {expected} got {actual}")
    for key, expected in physical.items():
        actual = (summary.get("physicalValidity", {}) or {}).get(key)
        _check(checks, f"{label}:physical:{key}", actual == expected,
               f"{label} physicalValidity {key} expected {expected} got {actual}")
    leakage = summary.get("leakage", {}) or {}
    _check(checks, f"{label}:leakage:clean", leakage.get("clean") == total,
           f"{label} leakage clean expected {total} got {leakage.get('clean')}")
    _check(checks, f"{label}:leakage:dirty", leakage.get("dirty") == 0,
           f"{label} leakage dirty expected 0 got {leakage.get('dirty')}")
    confusion = report.get("confusion", {}) or {}
    _check(checks, f"{label}:confusion:unexpected", not confusion.get("unexpectedOffDiagonalPasses"),
           f"{label} unexpected off-diagonal passes {confusion.get('unexpectedOffDiagonalPasses', [])}")
    _check(checks, f"{label}:confusion:diagonal", not confusion.get("missedDiagonal"),
           f"{label} missed diagonal {confusion.get('missedDiagonal', [])}")

    sidecar = _load_json(path / "failure_classification.json")
    if sidecar is not None:
        _check(checks, f"{label}:failure_sidecar:sourceProvenance", _has_source_provenance(sidecar),
               f"{label} failure sidecar sourceProvenance present")
        _check(checks, f"{label}:failure_sidecar:physical", sidecar.get("physicalValiditySummary") == dict(physical),
               f"{label} failure sidecar physical summary expected {dict(physical)} got {sidecar.get('physicalValiditySummary')}")

    if randomized:
        random = report.get("randomized", {}) or {}
        seed_values = random.get("seeds", []) or []
        _check(checks, f"{label}:randomized:enabled", random.get("enabled") is True,
               f"{label} randomized enabled {random.get('enabled')}")
        _check(checks, f"{label}:randomized:seeds", len(seed_values) == seeds,
               f"{label} seed count expected {seeds} got {len(seed_values)}")
        layouts: Dict[str, set[str]] = defaultdict(set)
        for case in report.get("cases", []) or []:
            base = str(case.get("baseCase") or case.get("case"))
            layouts[base].add(json.dumps(case.get("sampledLayout"), sort_keys=True))
        for base in ["insert_object", "open_drawer", "place_on_top", "push_object", "put_cube_in_tray"]:
            count = len(layouts.get(base, set()))
            _check(checks, f"{label}:randomized:{base}:layouts", count == seeds,
                   f"{label} {base} distinct sampled layouts expected {seeds} got {count}")


def _audit_comparison_dir(checks: List[Json], path: Path, *, require_git_provenance: bool = False) -> None:
    _require_file(checks, path / "comparison_report.json", "comparison:comparison_report.json")
    report = _load_json(path / "comparison_report.json")
    if report is None:
        return
    _check(checks, "comparison:schema", report.get("schemaVersion") == "csg.benchmark_comparison.v1",
           f"comparison schema {report.get('schemaVersion')}")
    _check(checks, "comparison:sourceProvenance", _has_source_provenance(report),
           "comparison sourceProvenance present")
    if require_git_provenance:
        kind = _source_provenance_kind(report)
        _check(checks, "comparison:sourceProvenance:git", kind == "git",
               f"comparison sourceProvenance kind expected git got {kind}")
    baselines = report.get("baselines", {}) or {}
    for name, physical in {"symbolic": {"unverified": 5}, "mujoco": {"valid": 5}}.items():
        baseline = baselines.get(name, {}) or {}
        summary = baseline.get("summary", {}) or {}
        _check(checks, f"comparison:{name}:passed", summary.get("passed") == 5,
               f"comparison {name} passed expected 5 got {summary.get('passed')}")
        _check(checks, f"comparison:{name}:failed", summary.get("failed") == 0,
               f"comparison {name} failed expected 0 got {summary.get('failed')}")
        _check(checks, f"comparison:{name}:physical", summary.get("physicalValidity") == physical,
               f"comparison {name} physical expected {physical} got {summary.get('physicalValidity')}")
        confusion = baseline.get("confusion", {}) or {}
        _check(checks, f"comparison:{name}:confusion", not confusion.get("unexpectedOffDiagonalPasses") and not confusion.get("missedDiagonal"),
               f"comparison {name} confusion unexpected={confusion.get('unexpectedOffDiagonalPasses', [])} missed={confusion.get('missedDiagonal', [])}")
    noop = baselines.get("noop", {}) or {}
    noop_summary = noop.get("summary", {}) or {}
    noop_classes = noop_summary.get("failureClassification", {}) or {}
    _check(checks, "comparison:noop:expectedFailure", noop.get("expectedFailure") is True,
           f"comparison noop expectedFailure expected true got {noop.get('expectedFailure')}")
    _check(checks, "comparison:noop:failed", noop_summary.get("failed") == 5 and noop_summary.get("passed") == 0,
           f"comparison noop expected failed=5 passed=0 got failed={noop_summary.get('failed')} passed={noop_summary.get('passed')}")
    _check(checks, "comparison:noop:physical", noop_summary.get("physicalValidity") == {"unverified": 5},
           f"comparison noop physical expected {{'unverified': 5}} got {noop_summary.get('physicalValidity')}")
    _check(checks, "comparison:noop:classes", bool(noop_classes) and set(noop_classes) != {"passed"},
           f"comparison noop expected non-passed failure classes got {noop_classes}")
    noop_confusion = noop.get("confusion", {}) or {}
    _check(checks, "comparison:noop:confusion", not noop_confusion.get("unexpectedOffDiagonalPasses"),
           f"comparison noop unexpected off-diagonal passes {noop_confusion.get('unexpectedOffDiagonalPasses', [])}")


def _audit_invalid_dir(checks: List[Json], path: Path, *, require_git_provenance: bool = False) -> None:
    _require_file(checks, path / "invalid_fixtures_report.json", "invalid:invalid_fixtures_report.json")
    report = _load_json(path / "invalid_fixtures_report.json")
    if report is None:
        return
    _check(checks, "invalid:schema", report.get("schemaVersion") == "csg.invalid_fixture_report.v1",
           f"invalid schema {report.get('schemaVersion')}")
    _check(checks, "invalid:sourceProvenance", _has_source_provenance(report),
           "invalid sourceProvenance present")
    if require_git_provenance:
        kind = _source_provenance_kind(report)
        _check(checks, "invalid:sourceProvenance:git", kind == "git",
               f"invalid sourceProvenance kind expected git got {kind}")
    summary = report.get("summary", {}) or {}
    _check(checks, "invalid:matched", summary == {"total": 9, "matched": 9, "mismatched": 0},
           f"invalid summary expected total/matched/mismatched 9/9/0 got {summary}")
    categories = Counter(
        get_any((fixture.get("result", {}) or {}).get("failureClassification", {}) or {}, "category", default="")
        for fixture in report.get("fixtures", []) or []
    )
    expected = {
        "physical_invalidity": 6,
        "contact_missing": 1,
        "relation_not_achieved": 1,
        "event_order_wrong": 1,
    }
    _check(checks, "invalid:categories", dict(categories) == expected,
           f"invalid categories expected {expected} got {dict(categories)}")


def _pyproject_has_license(path: Path) -> bool:
    if not path.is_file():
        return False
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    return "license" in (data.get("project", {}) or {})


def _audit_final_metadata(checks: List[Json], project_root: Path) -> None:
    git_path = project_root / ".git"
    _check(checks, "final_metadata:git_dir", git_path.exists(),
           "final metadata requires .git directory")
    license_path = project_root / "LICENSE"
    _check(checks, "final_metadata:license_file", license_path.is_file(),
           "final metadata requires LICENSE")
    pyproject_path = project_root / "pyproject.toml"
    _check(checks, "final_metadata:pyproject_license", _pyproject_has_license(pyproject_path),
           "final metadata requires pyproject.toml license metadata")


def audit_release_artifacts(
    symbolic_dir: str | Path,
    mujoco_dir: str | Path,
    randomized_dir: str | Path,
    comparison_dir: str | Path,
    invalid_fixtures_dir: str | Path,
    *,
    seeds: int = 30,
    require_final_metadata: bool = False,
    project_root: str | Path = ".",
) -> Json:
    checks: List[Json] = []
    _audit_benchmark_dir(checks, "symbolic", Path(symbolic_dir), total=5, passed=5,
                         physical={"unverified": 5}, require_git_provenance=require_final_metadata)
    _audit_benchmark_dir(checks, "mujoco", Path(mujoco_dir), total=5, passed=5,
                         physical={"valid": 5}, require_git_provenance=require_final_metadata)
    _audit_benchmark_dir(checks, "randomized", Path(randomized_dir), total=5 * seeds, passed=5 * seeds,
                         physical={"valid": 5 * seeds}, randomized=True, seeds=seeds,
                         require_git_provenance=require_final_metadata)
    # Honest scope: the MuJoCo gold + randomized + comparison-baseline checks above
    # ASSERT the published summaries equal the expected all-green constants. They do
    # NOT — and cannot, from these reports alone — independently verify the physics:
    # the per-case distances are machine-dependent floats that are never re-run here,
    # so a report that keeps an all-green summary while gutting/forging its per-case
    # physics evidence still satisfies this audit. Binding the MuJoCo numbers requires
    # the CI build-provenance attestation (csg.verify_release: ATTESTED_TAGS); this
    # audit is a consistency assertion against expected constants, not that proof.
    _check(checks, "mujoco:assertion_scope", True,
           "MuJoCo gold/randomized/comparison evidence is ASSERTED against expected all-green "
           "constants here, NOT independently verified (per-case physics floats are not re-run); "
           "binding the physics numbers requires the CI attestation in csg.verify_release")
    _audit_comparison_dir(checks, Path(comparison_dir), require_git_provenance=require_final_metadata)
    _audit_invalid_dir(checks, Path(invalid_fixtures_dir), require_git_provenance=require_final_metadata)
    if require_final_metadata:
        _audit_final_metadata(checks, Path(project_root))
    failed = [check for check in checks if not check["ok"]]
    return {
        "schemaVersion": "csg.release_audit.v1",
        "ok": not failed,
        "summary": {"checksTotal": len(checks), "checksPassed": len(checks) - len(failed), "checksFailed": len(failed)},
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit Phase 2E release report artifacts.")
    parser.add_argument("--symbolic", required=True, help="symbolic gold benchmark output directory")
    parser.add_argument("--mujoco", required=True, help="MuJoCo gold benchmark output directory")
    parser.add_argument("--randomized", required=True, help="30-seed MuJoCo randomized benchmark output directory")
    parser.add_argument("--comparison", required=True, help="symbolic-vs-MuJoCo comparison output directory")
    parser.add_argument("--invalid-fixtures", required=True, help="invalid fixture output directory")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--project-root", default=".", help="project root for final metadata checks")
    parser.add_argument("--require-final-metadata", action="store_true",
                        help="also require Git-backed report provenance plus LICENSE and pyproject license metadata")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    report = audit_release_artifacts(
        args.symbolic,
        args.mujoco,
        args.randomized,
        args.comparison,
        args.invalid_fixtures,
        seeds=args.seeds,
        require_final_metadata=args.require_final_metadata,
        project_root=args.project_root,
    )
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        summary = report["summary"]
        print(f"release audit ok={report['ok']} checks={summary['checksPassed']}/{summary['checksTotal']}")
        for check in report["checks"]:
            if not check["ok"]:
                print(f"  FAIL {check['name']}: {check['message']}")
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
