#!/usr/bin/env python3
"""Run the frozen compiler-verifier loop and report probe-vector PASS.

For each target CSG:
    target -> compile_scene -> solve -> rollout frames
           -> extract_robot_csg (frames only) -> match(target, robot)

A case PASSes iff:
    * every HARD probe agrees (match.passed), AND
    * the robot CSG is leakage-clean (no TaskSpec, sim-only provenance).

The acceptance criterion is the probe-agreement vector, not a scalar threshold.
The scalar distance is reported as a secondary (curriculum) signal.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import subprocess
import traceback
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from .common import Json, get_any, load_json, write_json
from .matcher import MatcherConfig, match
from .rollout_extract import extract_robot_csg
from .solver import SolverConfig, solve

LEAKAGE_FORBIDDEN_KEYS = ("plannerView", "planner_view", "targetCsg", "target_csg", "solverMetadata", "solver_metadata")
ALLOWED_ESTIMATORS = {"SIM_STATE_EXTRACTION"}

# Task pairs that genuinely share an observable quotient class: a PASS of one
# task's target against the other task's rollout is the *correct* quotient
# semantics, not solver hardcoding. insert_object and put_cube_in_tray are
# both "move a rigid object INSIDE a container"; the demonstrations differ in
# labels, sizes and container parts, none of which are quotient facts
# (physical_quotient.md). The confusion matrix asserts these mutually PASS
# while every other off-diagonal cell FAILs.
KNOWN_EQUIVALENT_TASKS = (frozenset({"insert_object", "put_cube_in_tray"}),)
EXPECTED_FAILURE_BASELINES = {"noop"}

SOURCE_PROVENANCE_GLOBS = (
    "pyproject.toml",
    "README.md",
    "roadmap.md",
    "physical_quotient.md",
    "Causal_Skill_Graph_V0.md",
    "csg/**/*.py",
    "csg/**/*.md",
    "docs/**/*.md",
    "gold_tests/**/*.json",
    "gold_invalid/**/*.json",
    "tests/**/*.py",
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _source_snapshot(root: Optional[Path] = None) -> Json:
    root = (root or _repo_root()).resolve()
    paths: List[Path] = []
    seen: set[str] = set()
    for pattern in SOURCE_PROVENANCE_GLOBS:
        for path in root.glob(pattern):
            if not path.is_file() or "__pycache__" in path.parts:
                continue
            rel = path.relative_to(root).as_posix()
            if rel not in seen:
                seen.add(rel)
                paths.append(path)

    file_entries: List[Json] = []
    aggregate = hashlib.sha256()
    for path in sorted(paths, key=lambda p: p.relative_to(root).as_posix()):
        rel = path.relative_to(root).as_posix()
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        aggregate.update(rel.encode("utf-8"))
        aggregate.update(b"\0")
        aggregate.update(digest.encode("ascii"))
        aggregate.update(b"\n")
        file_entries.append({"path": rel, "sha256": digest})
    return {
        "algorithm": "sha256",
        "digest": aggregate.hexdigest(),
        "fileCount": len(file_entries),
        "files": file_entries,
    }


def _git_provenance(root: Optional[Path] = None) -> Optional[Json]:
    root = (root or _repo_root()).resolve()
    if not (root / ".git").exists():
        return None
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=root,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.splitlines()
    except (OSError, subprocess.CalledProcessError):
        return None
    return {"commit": commit, "dirty": bool(status), "statusPorcelain": status}


def source_provenance(root: Optional[Path] = None) -> Json:
    root = (root or _repo_root()).resolve()
    git = _git_provenance(root)
    return {
        "schemaVersion": "csg.source_provenance.v1",
        "kind": "git" if git else "source_snapshot",
        "root": str(root),
        "git": git,
        "snapshot": _source_snapshot(root),
    }


def classify_failure(case: Json) -> Json:
    """Classify a benchmark case from already-reported verifier evidence."""
    if case.get("status") == "PASS":
        return {"category": "passed", "evidence": {"status": case.get("status")}}

    if case.get("leakageClean") is False or get_any(case.get("leakage", {}) or {}, "clean", default=True) is False:
        return {
            "category": "target_leakage_detected",
            "evidence": {
                "leakageClean": case.get("leakageClean"),
                "leakage": case.get("leakage"),
            },
        }

    if case.get("physicalValidity") is False:
        evidence = {"physicalValidity": False}
        if "physicalValidityReason" in case:
            evidence["physicalValidityReason"] = case.get("physicalValidityReason")
        if "physicalValidityReport" in case:
            evidence["physicalValidityReport"] = case.get("physicalValidityReport")
        return {"category": "physical_invalidity", "evidence": evidence}

    if case.get("status") == "ERROR":
        return {
            "category": "solver_error",
            "evidence": {"status": case.get("status"), "error": case.get("error")},
        }

    hard_mismatches = set(case.get("hardMismatches", []) or [])
    probe_agreement = case.get("probeAgreement", {}) or {}
    mismatched_probes = hard_mismatches
    evidence = {
        "matcherPassed": case.get("matcherPassed"),
        "hardMismatches": sorted(hard_mismatches),
        "probeAgreement": probe_agreement,
    }

    if {"contact_word", "contact_evidence"} & mismatched_probes:
        return {"category": "contact_missing", "evidence": evidence}
    if "event_order" in mismatched_probes:
        return {"category": "event_order_wrong", "evidence": evidence}
    if {
        "goal_satisfaction",
        "relation_transitions",
        "articulation_transitions",
    } & mismatched_probes:
        return {"category": "relation_not_achieved", "evidence": evidence}

    return {"category": "verifier_mismatch", "evidence": evidence}


def _failure_classification_summary(cases: Sequence[Json]) -> Dict[str, int]:
    summary: Dict[str, int] = {}
    for case in cases:
        category = str(get_any(case.get("failureClassification", {}) or {}, "category", default="verifier_mismatch"))
        summary[category] = summary.get(category, 0) + 1
    return dict(sorted(summary.items()))


def _physical_validity_summary(cases: Sequence[Json]) -> Dict[str, int]:
    summary = {"valid": 0, "invalid": 0, "unverified": 0}
    for case in cases:
        validity = case.get("physicalValidity")
        if validity is True:
            summary["valid"] += 1
        elif validity is False:
            summary["invalid"] += 1
        else:
            summary["unverified"] += 1
    return {k: v for k, v in summary.items() if v}


def _leakage_summary(cases: Sequence[Json]) -> Dict[str, int]:
    summary = {"clean": 0, "dirty": 0}
    for case in cases:
        if case.get("leakageClean") is True:
            summary["clean"] += 1
        else:
            summary["dirty"] += 1
    return summary


def leakage_report(robot_csg: Json) -> Dict[str, Any]:
    forbidden = [k for k in LEAKAGE_FORBIDDEN_KEYS if k in robot_csg]
    estimators = sorted({str(get_any(e, "estimator", default="")).upper() for e in robot_csg.get("evidence", [])})
    # Every fact needs sim-extraction provenance; an evidence-free or
    # empty-estimator CSG is NOT clean (it proves nothing about its origin).
    bad_estimators = [e for e in estimators if e not in ALLOWED_ESTIMATORS]
    has_evidence = bool(robot_csg.get("evidence"))
    read_target = bool(get_any(get_any(robot_csg, "extractionMetadata", default={}) or {}, "readTargetCsg", default=False))
    clean = has_evidence and not forbidden and not bad_estimators and not read_target
    return {"clean": clean, "forbiddenKeys": forbidden, "badEstimators": bad_estimators,
            "hasEvidence": has_evidence, "readTargetCsg": read_target}


def _target_base_name(target_path: Path, idx: int) -> str:
    stem = target_path.stem.replace(".target", "")
    return (target_path.parent.name if stem == "target" else stem) or f"case_{idx:03d}"


def _solver_config_json(cfg: SolverConfig) -> Json:
    return {k: v for k, v in asdict(cfg).items() if v is not None}


def run_one(target_path: Path, out_dir: Path, idx: int, solver_cfg: SolverConfig, matcher_cfg: MatcherConfig,
            *, case_name: Optional[str] = None, base_case: Optional[str] = None, seed: Optional[int] = None) -> Json:
    base = base_case or _target_base_name(target_path, idx)
    name = case_name or base
    cdir = out_dir / name
    cdir.mkdir(parents=True, exist_ok=True)
    try:
        target = load_json(target_path)
        run = solve(target, solver_cfg)
        robot = extract_robot_csg(run.rollout)
        result = match(target, robot, matcher_cfg)
        leak = leakage_report(robot)

        write_json(cdir / "rollout.json", run.rollout)
        write_json(cdir / "robot_csg.json", robot)
        write_json(cdir / "matcher_report.json", result.to_json())

        # Reporting contract (csg/validity.md): None = backend cannot check
        # (symbolic) -> "interface-valid, physics-unverified"; an explicit
        # False from a physics backend fails the case outright.
        diag = run.rollout.get("diagnostics", {}) or {}
        validity = diag.get("physicalValidity", None)
        passed = result.passed and leak["clean"] and validity is not False
        status = "PASS" if passed else "FAIL"
        case = {
            "case": name, "baseCase": base, "target": str(target_path), "status": status,
            "seed": seed,
            "solverConfig": _solver_config_json(solver_cfg),
            "passed": passed, "matcherPassed": result.passed, "leakageClean": leak["clean"],
            "physicalValidity": validity,
            "physicalValidityReason": diag.get("physicalValidityReason"),
            "sampledLayout": diag.get("sampledLayout"),
            "vacuous": result.vacuous,
            "distance": result.distance,
            "probeAgreement": result.probe_agreement,
            "probeSupport": result.probe_support,
            "hardMismatches": [p for p in result.hard_probes if not result.probe_agreement[p]],
            "leakage": leak,
            "objectMapping": result.object_mapping,
            "objectOrbitAmbiguous": result.object_orbit_ambiguous,
            "outDir": str(cdir),
        }
        if run.validity_report is not None:
            case["physicalValidityReport"] = run.validity_report
            write_json(cdir / "validity_report.json", run.validity_report)
        case["failureClassification"] = classify_failure(case)
        return case
    except Exception as exc:  # noqa: BLE001
        (cdir / "error.log").write_text(traceback.format_exc(), encoding="utf-8")
        case = {
            "case": name, "baseCase": base, "target": str(target_path), "status": "ERROR",
            "seed": seed, "solverConfig": _solver_config_json(solver_cfg),
            "passed": False, "error": repr(exc), "outDir": str(cdir),
        }
        case["failureClassification"] = classify_failure(case)
        return case


def discover_targets(paths: Sequence[str], target_dir: Optional[str]) -> List[Path]:
    out: List[Path] = []
    for p in paths:
        pp = Path(p)
        if pp.is_dir():
            out.extend(sorted(pp.rglob("target.json")))
            out.extend(sorted(pp.rglob("*.target.json")))
        else:
            out.append(pp)
    if target_dir:
        root = Path(target_dir)
        out.extend(sorted(root.rglob("target.json")))
        out.extend(sorted(root.rglob("*.target.json")))
    seen: set = set()
    uniq: List[Path] = []
    for p in out:
        r = str(p.resolve())
        if r not in seen:
            seen.add(r)
            uniq.append(p)
    return uniq


def _expected_equivalent(a: str, b: str) -> bool:
    return a == b or any({a, b} <= eq for eq in KNOWN_EQUIVALENT_TASKS)


def confusion_matrix(cases: Sequence[Json], matcher_cfg: MatcherConfig) -> Json:
    """Cross-task confusion: match every case's *target* against every case's
    rollout-derived *robot CSG*. The diagonal must PASS; an off-diagonal PASS
    means either a documented quotient equivalence (KNOWN_EQUIVALENT_TASKS) or
    a red flag: an under-constrained target / hardcoded solver trajectory."""
    usable = [c for c in cases if c.get("status") != "ERROR" and (Path(c["outDir"]) / "robot_csg.json").is_file()]
    robots = {c["case"]: load_json(Path(c["outDir"]) / "robot_csg.json") for c in usable}
    base_names = {c["case"]: str(c.get("baseCase") or c["case"]) for c in usable}
    matrix: Dict[str, Dict[str, bool]] = {}
    for c in usable:
        target = load_json(c["target"])
        matrix[c["case"]] = {name: bool(match(target, robot, matcher_cfg).passed) for name, robot in robots.items()}
    off_diag = [[t, r] for t, row in matrix.items() for r, p in row.items() if p and t != r]
    return {
        "matrix": matrix,
        "missedDiagonal": sorted(t for t, row in matrix.items() if not row.get(t, False)),
        "offDiagonalPasses": sorted(off_diag),
        "unexpectedOffDiagonalPasses": sorted(
            p for p in off_diag
            if not _expected_equivalent(base_names.get(p[0], p[0]), base_names.get(p[1], p[1]))
        ),
        "knownEquivalentTasks": sorted(sorted(eq) for eq in KNOWN_EQUIVALENT_TASKS),
    }


def _seed_values(randomized: bool, seeds: int | Sequence[int]) -> List[int]:
    if not randomized:
        return []
    if isinstance(seeds, int):
        if seeds <= 0:
            raise ValueError("--seeds must be positive when --randomized is set")
        return list(range(seeds))
    out = [int(s) for s in seeds]
    if not out:
        raise ValueError("at least one seed is required when randomized=True")
    return out


def run_benchmark(targets: Sequence[Path], out_dir: str | Path, solver_cfg: Optional[SolverConfig] = None, matcher_cfg: Optional[MatcherConfig] = None, confusion: bool = False, randomized: bool = False, seeds: int | Sequence[int] = 1) -> Json:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    solver_cfg = solver_cfg or SolverConfig()
    matcher_cfg = matcher_cfg or MatcherConfig()
    provenance = source_provenance()
    seed_values = _seed_values(randomized, seeds)
    if seed_values:
        cases = [
            run_one(
                t, out, i, replace(solver_cfg, seed=seed), matcher_cfg,
                case_name=f"{_target_base_name(t, i)}__seed_{seed:03d}",
                base_case=_target_base_name(t, i),
                seed=seed,
            )
            for seed in seed_values
            for i, t in enumerate(targets)
        ]
    else:
        cases = [run_one(t, out, i, solver_cfg, matcher_cfg) for i, t in enumerate(targets)]
    passed = sum(1 for c in cases if c.get("status") == "PASS")
    failure_summary = _failure_classification_summary(cases)
    physical_summary = _physical_validity_summary(cases)
    leakage_summary = _leakage_summary(cases)
    report = {
        "schemaVersion": "csg.benchmark_report.v2",
        "summary": {"total": len(cases), "passed": passed, "failed": len(cases) - passed,
                    "failureClassification": failure_summary,
                    "physicalValidity": physical_summary,
                    "leakage": leakage_summary,
                    "criterion": ("all HARD probes agree (non-vacuously) AND robot CSG leakage-clean "
                                  "AND physical validity not false (None = physics-unverified)")},
        "sourceProvenance": provenance,
        "randomized": {"enabled": bool(seed_values), "seeds": seed_values},
        "failureClassificationSummary": failure_summary,
        "cases": cases,
    }
    if confusion:
        report["confusion"] = confusion_matrix(cases, matcher_cfg)
    write_json(out / "report.json", report)
    write_json(out / "failure_classification.json", {
        "schemaVersion": "csg.benchmark_failure_classification.v1",
        "sourceProvenance": provenance,
        "summary": failure_summary,
        "physicalValiditySummary": physical_summary,
        "leakageSummary": leakage_summary,
        "cases": [
            {
                "case": c.get("case"),
                "baseCase": c.get("baseCase"),
                "seed": c.get("seed"),
                "status": c.get("status"),
                "failureClassification": c.get("failureClassification"),
            }
            for c in cases
        ],
    })
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["case", "baseCase", "seed", "status", "failureClassification", "distance", "matcherPassed", "leakageClean", "physicalValidity", "vacuous", "objectOrbitAmbiguous", "hardMismatches"])
        w.writeheader()
        for c in cases:
            row = {k: c.get(k) for k in w.fieldnames}
            row["failureClassification"] = get_any(c.get("failureClassification", {}) or {}, "category", default="")
            w.writerow(row)

    def _validity_label(v: Any) -> str:
        if v is None:
            return "unverified"
        return "valid" if v else "INVALID"

    lines = ["# CSG Loop Benchmark", "", f"passed: {passed}/{len(cases)}",
             f"source: {provenance['kind']} {provenance['snapshot']['algorithm']}:{provenance['snapshot']['digest']}",
             "physical validity: " + ", ".join(f"{k}:{v}" for k, v in physical_summary.items()),
             "leakage: " + ", ".join(f"{k}:{v}" for k, v in leakage_summary.items()),
             "", "PASS with validity 'unverified' means interface-valid, physics-unverified.",
             "An 'orbit' marker means the object mapping is one representative of a symmetry",
             "orbit (interchangeable identical objects), not a unique identity.", "",
             "Failure classes: " + ", ".join(f"{k}:{v}" for k, v in failure_summary.items()), "",
             "| case | status | class | distance | hard mismatches | leakage | validity | orbit | support |", "|---|---|---|---:|---|---|---|---|---|"]
    for c in cases:
        supp = c.get("probeSupport", {}) or {}
        supp_s = "vacuous" if c.get("vacuous") else ",".join(f"{k}:{v}" for k, v in sorted(supp.items()) if v)
        failure_class = get_any(c.get("failureClassification", {}) or {}, "category", default="verifier_mismatch")
        lines.append(f"| {c['case']} | {c['status']} | {failure_class} | {c.get('distance')} | {','.join(c.get('hardMismatches', [])) or '-'} | "
                     f"{'clean' if c.get('leakageClean') else 'LEAK'} | {_validity_label(c.get('physicalValidity'))} | "
                     f"{'orbit' if c.get('objectOrbitAmbiguous') else 'unique'} | {supp_s or '-'} |")

    if confusion:
        conf = report["confusion"]
        names = sorted(conf["matrix"])
        lines += ["", "## Cross-task confusion (rows: targets, columns: rollouts)", "",
                  "A PASS off the diagonal is correct only for documented quotient-equivalent",
                  f"tasks ({'; '.join(' ~ '.join(eq) for eq in conf['knownEquivalentTasks']) or 'none'});",
                  "any other off-diagonal PASS indicates an under-constrained target or a",
                  "hardcoded solver trajectory.", "",
                  "| target \\ rollout | " + " | ".join(names) + " |",
                  "|---|" + "---|" * len(names)]
        for t in names:
            row = conf["matrix"][t]
            lines.append(f"| {t} | " + " | ".join("PASS" if row.get(r) else "." for r in names) + " |")
        if conf["unexpectedOffDiagonalPasses"]:
            lines += ["", "**UNEXPECTED off-diagonal passes:** "
                      + "; ".join(f"{t} vs {r}" for t, r in conf["unexpectedOffDiagonalPasses"])]
        if conf["missedDiagonal"]:
            lines += ["", "**Missed diagonal:** " + ", ".join(conf["missedDiagonal"])]
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def run_benchmark_comparison(targets: Sequence[Path], out_dir: str | Path, baselines: Mapping[str, SolverConfig],
                             matcher_cfg: Optional[MatcherConfig] = None, confusion: bool = False,
                             randomized: bool = False, seeds: int | Sequence[int] = 1) -> Json:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    matcher_cfg = matcher_cfg or MatcherConfig()
    baseline_reports: Dict[str, Json] = {}
    for name, cfg in baselines.items():
        bdir = out / name
        report = run_benchmark(
            targets,
            bdir,
            solver_cfg=cfg,
            matcher_cfg=matcher_cfg,
            confusion=confusion,
            randomized=randomized,
            seeds=seeds,
        )
        summary = dict(report["summary"])
        summary["physicalValidity"] = _physical_validity_summary(report["cases"])
        baseline_report: Json = {
            "baseline": name,
            "backend": cfg.resolved_backend(),
            "expectedFailure": name in EXPECTED_FAILURE_BASELINES or cfg.resolved_backend() in EXPECTED_FAILURE_BASELINES,
            "solverConfig": _solver_config_json(cfg),
            "summary": summary,
            "randomized": report.get("randomized", {"enabled": False, "seeds": []}),
            "failureClassificationSummary": report.get("failureClassificationSummary", {}),
            "cases": report["cases"],
            "outDir": str(bdir),
        }
        if "confusion" in report:
            baseline_report["confusion"] = report["confusion"]
        baseline_reports[name] = baseline_report

    comparison: Json = {
        "schemaVersion": "csg.benchmark_comparison.v1",
        "sourceProvenance": source_provenance(),
        "baselineOrder": list(baselines.keys()),
        "baselines": baseline_reports,
    }
    write_json(out / "comparison_report.json", comparison)
    return comparison


def _comparison_has_failures(report: Mapping[str, Any]) -> bool:
    for baseline in (report.get("baselines", {}) or {}).values():
        summary = baseline.get("summary", {}) or {}
        expected_failure = bool(baseline.get("expectedFailure"))
        failed = int(summary.get("failed", 0) or 0)
        if expected_failure:
            classes = summary.get("failureClassification", {}) or {}
            if failed <= 0 or set(classes) == {"passed"}:
                return True
        elif failed > 0:
            return True
        confusion = baseline.get("confusion", {}) or {}
        if confusion.get("unexpectedOffDiagonalPasses"):
            return True
    return False


def discover_invalid_fixtures(paths: Sequence[str | Path]) -> List[Path]:
    out: List[Path] = []
    for p in paths:
        pp = Path(p)
        if pp.is_dir():
            out.extend(sorted(pp.rglob("*.json")))
        else:
            out.append(pp)
    return [p for p in out if p.is_file()]


def _resolve_fixture_target(manifest_path: Path, manifest: Mapping[str, Any]) -> Path:
    target = str(get_any(manifest, "target", default=""))
    if not target:
        raise ValueError(f"{manifest_path}: missing target")
    target_path = Path(target)
    if not target_path.is_absolute():
        target_path = manifest_path.parent / target_path
    return target_path.resolve()


def _apply_invalid_target_mutation(target: Mapping[str, Any], mutation: str) -> Json:
    """Build a deliberately invalid target CSG for semantic fixture checks.

    Mutations are benchmark-only fixture generation; they do not change the
    solver, extractor, matcher, or leakage gate. The generated target is written
    under the fixture output directory before the normal run path consumes it.
    """
    mutated = copy.deepcopy(dict(target))
    kind = str(mutation or "")
    if kind == "release_before_relation":
        events = mutated.get("events", []) or []
        relation_event = None
        release_event = None
        for event in events:
            ek = str(get_any(event, "eventKind", "event_kind", default=""))
            if ek in {"CONTAINMENT_CHANGE", "SUPPORT_CHANGE", "RELATION_CHANGE", "ARTICULATION_CHANGE"} and relation_event is None:
                relation_event = event
            if ek == "RELEASE_INFERRED" and release_event is None:
                release_event = event
        if relation_event is None or release_event is None:
            raise ValueError("targetMutation release_before_relation requires relation/articulation and release events")
        relation_span = copy.deepcopy(get_any(relation_event, "timeSpan", "time_span", default={}) or {})
        release_span = copy.deepcopy(get_any(release_event, "timeSpan", "time_span", default={}) or {})
        relation_event["timeSpan"] = release_span
        release_event["timeSpan"] = relation_span
        return mutated
    raise ValueError(f"unknown targetMutation: {mutation}")


def _failed_validity_checks(case: Mapping[str, Any]) -> List[str]:
    report = case.get("physicalValidityReport", {}) or {}
    checks = report.get("checks", {}) or {}
    return sorted(
        name for name, check in checks.items()
        if bool(get_any(check, "applicable", default=False)) and not bool(get_any(check, "passed", default=True))
    )


def _invalid_fixture_mismatches(case: Mapping[str, Any], expected: Mapping[str, Any], failed_checks: Sequence[str]) -> List[str]:
    mismatches: List[str] = []
    if "passed" in expected and bool(case.get("passed")) != bool(expected["passed"]):
        mismatches.append(f"passed expected {expected['passed']} got {case.get('passed')}")
    if "physicalValidity" in expected and case.get("physicalValidity") is not expected["physicalValidity"]:
        mismatches.append(f"physicalValidity expected {expected['physicalValidity']} got {case.get('physicalValidity')}")
    expected_category = get_any(expected, "failureCategory", default=None)
    actual_category = get_any(case.get("failureClassification", {}) or {}, "category", default=None)
    if expected_category and actual_category != expected_category:
        mismatches.append(f"failureCategory expected {expected_category} got {actual_category}")
    expected_check = get_any(expected, "failedValidityCheck", default=None)
    if expected_check and expected_check not in failed_checks:
        mismatches.append(f"failedValidityCheck expected {expected_check} got {list(failed_checks)}")
    expected_probe = get_any(expected, "hardMismatch", default=None)
    if expected_probe and expected_probe not in (case.get("hardMismatches", []) or []):
        mismatches.append(f"hardMismatch expected {expected_probe} got {case.get('hardMismatches', [])}")
    return mismatches


def run_invalid_fixture(manifest_path: str | Path, out_dir: str | Path, idx: int = 0,
                        matcher_cfg: Optional[MatcherConfig] = None) -> Json:
    manifest_path = Path(manifest_path)
    manifest = load_json(manifest_path)
    fixture_id = str(get_any(manifest, "fixtureId", "fixture_id", default=manifest_path.stem))
    task = str(get_any(manifest, "task", default=fixture_id))
    target_path = _resolve_fixture_target(manifest_path, manifest)
    mutation = get_any(manifest, "targetMutation", "target_mutation", default=None)
    if mutation:
        mutated_target = _apply_invalid_target_mutation(load_json(target_path), str(mutation))
        generated_dir = Path(out_dir) / "_generated_targets"
        generated_dir.mkdir(parents=True, exist_ok=True)
        target_path = generated_dir / f"{fixture_id}.json"
        write_json(target_path, mutated_target)
    cfg = SolverConfig(**dict(get_any(manifest, "solverConfig", "solver_config", default={}) or {}))
    case = run_one(
        target_path,
        Path(out_dir),
        idx,
        cfg,
        matcher_cfg or MatcherConfig(),
        case_name=fixture_id,
        base_case=task,
        seed=cfg.seed,
    )
    expected = dict(get_any(manifest, "expected", default={}) or {})
    failed_checks = _failed_validity_checks(case)
    mismatches = _invalid_fixture_mismatches(case, expected, failed_checks)
    return {
        "fixtureId": fixture_id,
        "manifest": str(manifest_path),
        "task": task,
        "target": str(target_path),
        "solverConfig": _solver_config_json(cfg),
        "expected": expected,
        "result": case,
        "failedValidityChecks": failed_checks,
        "expectedFailureMatched": not mismatches,
        "mismatches": mismatches,
    }


def run_invalid_fixtures(fixtures: str | Path | Sequence[str | Path], out_dir: str | Path,
                         matcher_cfg: Optional[MatcherConfig] = None) -> Json:
    paths = discover_invalid_fixtures([fixtures] if isinstance(fixtures, (str, Path)) else fixtures)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    results = [run_invalid_fixture(path, out, idx, matcher_cfg) for idx, path in enumerate(paths)]
    matched = sum(1 for r in results if r["expectedFailureMatched"])
    report = {
        "schemaVersion": "csg.invalid_fixture_report.v1",
        "sourceProvenance": source_provenance(),
        "summary": {"total": len(results), "matched": matched, "mismatched": len(results) - matched},
        "fixtures": results,
    }
    write_json(out / "invalid_fixtures_report.json", report)
    return report


def main() -> int:
    p = argparse.ArgumentParser(description="Run the CSG compiler-verifier loop benchmark.")
    p.add_argument("targets", nargs="*", help="target.json files or directories")
    p.add_argument("--target-dir", default=None)
    p.add_argument("--out", "--out-dir", dest="out", default="csg_benchmark_out")
    p.add_argument("--json", action="store_true")
    p.add_argument("--require-pass", action="store_true")
    p.add_argument("--backend", "--engine", dest="backend", default="symbolic")
    p.add_argument("--compare-backends", default=None,
                   help="comma-separated backend comparison, e.g. symbolic,mujoco; writes comparison_report.json")
    p.add_argument("--invalid-fixtures", nargs="+", default=None,
                   help="invalid fixture manifest files or directories; writes invalid_fixtures_report.json")
    p.add_argument("--randomized", action="store_true",
                   help="run each target once per deterministic seed (for MuJoCo randomized benchmark sweeps)")
    p.add_argument("--seeds", type=int, default=1,
                   help="number of seeds to run when --randomized is set; seeds are 0..N-1")
    p.add_argument("--confusion", action="store_true",
                   help="also match every target against every other task's rollout (solver-hardcoding check)")
    args = p.parse_args()
    if args.invalid_fixtures:
        report = run_invalid_fixtures(args.invalid_fixtures, args.out)
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            summary = report["summary"]
            print(f"invalid fixtures matched={summary['matched']}/{summary['total']}")
            for fixture in report["fixtures"]:
                print(
                    f"  {fixture['fixtureId']}: "
                    f"{'MATCH' if fixture['expectedFailureMatched'] else 'MISMATCH'} "
                    f"checks={fixture['failedValidityChecks']}"
                )
            print(f"invalid fixture report written to {Path(args.out) / 'invalid_fixtures_report.json'}")
        if args.require_pass and report["summary"]["mismatched"]:
            return 2
        return 0
    targets = discover_targets(args.targets, args.target_dir)
    if not targets:
        raise SystemExit("No target.json files found.")
    if args.compare_backends:
        baseline_names = [b.strip() for b in str(args.compare_backends).split(",") if b.strip()]
        if not baseline_names:
            raise SystemExit("--compare-backends requires at least one backend name")
        report = run_benchmark_comparison(
            targets,
            args.out,
            {name: SolverConfig(backend=name) for name in baseline_names},
            confusion=args.confusion,
            randomized=args.randomized,
            seeds=args.seeds,
        )
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print("baseline comparison:")
            for name in report["baselineOrder"]:
                baseline = report["baselines"][name]
                summary = baseline["summary"]
                print(
                    f"  {name}: passed={summary['passed']}/{summary['total']} "
                    f"validity={summary.get('physicalValidity', {})} "
                    f"classes={summary.get('failureClassification', {})}"
                )
                confusion = baseline.get("confusion", {}) or {}
                if confusion:
                    print(
                        f"    confusion unexpected={len(confusion.get('unexpectedOffDiagonalPasses', []))} "
                        f"missed={len(confusion.get('missedDiagonal', []))}"
                    )
            print(f"comparison report written to {Path(args.out) / 'comparison_report.json'}")
        if args.require_pass and _comparison_has_failures(report):
            return 2
        return 0
    report = run_benchmark(
        targets, args.out,
        solver_cfg=SolverConfig(backend=args.backend),
        confusion=args.confusion,
        randomized=args.randomized,
        seeds=args.seeds,
    )
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"passed={report['summary']['passed']}/{report['summary']['total']}")
        cases = report["cases"]
        if len(cases) > 25:
            print(f"  cases: {len(cases)} total; details written to {Path(args.out) / 'report.json'}")
            failures = [c for c in cases if c.get("status") != "PASS"]
            for c in failures[:20]:
                print(f"  {c['case']}: {c['status']} dist={c.get('distance')} {c.get('hardMismatches', [])}")
            if len(failures) > 20:
                print(f"  ... {len(failures) - 20} more failures")
        else:
            for c in cases:
                print(f"  {c['case']}: {c['status']} dist={c.get('distance')} {c.get('hardMismatches', [])}")
        if args.confusion:
            conf = report["confusion"]
            names = sorted(conf["matrix"])
            if len(names) > 20:
                print(f"confusion: {len(names)}x{len(names)} matrix written to {Path(args.out) / 'report.md'}")
                print(f"  missed diagonal: {len(conf['missedDiagonal'])}")
                print(f"  unexpected off-diagonal passes: {len(conf['unexpectedOffDiagonalPasses'])}")
                print(f"  off-diagonal passes: {len(conf['offDiagonalPasses'])}")
            else:
                width = max((len(n) for n in names), default=8) + 2
                print("confusion (rows: targets, cols: rollouts):")
                print(" " * width + "".join(n.rjust(width) for n in names))
                for t in names:
                    print(t.rjust(width) + "".join(("PASS" if conf["matrix"][t].get(r) else ".").rjust(width) for r in names))
            if conf["unexpectedOffDiagonalPasses"]:
                print(f"UNEXPECTED off-diagonal passes: {conf['unexpectedOffDiagonalPasses']}")
            if conf["missedDiagonal"]:
                print(f"missed diagonal: {conf['missedDiagonal']}")
    if args.require_pass and report["summary"]["failed"]:
        return 2
    if args.require_pass and report.get("confusion", {}).get("unexpectedOffDiagonalPasses"):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
