import json

from csg.release_audit import audit_release_artifacts


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _provenance(*, kind="source_snapshot"):
    return {
        "schemaVersion": "csg.source_provenance.v1",
        "kind": kind,
        "git": {"commit": "a" * 40, "dirty": False, "statusPorcelain": []} if kind == "git" else None,
        "snapshot": {"algorithm": "sha256", "digest": "0" * 64, "fileCount": 3, "files": []},
    }


def _write_benchmark_dir(path, *, total, passed, physical, leakage, randomized=False, provenance_kind="source_snapshot"):
    path.mkdir(parents=True, exist_ok=True)
    base_cases = ["insert_object", "open_drawer", "place_on_top", "push_object", "put_cube_in_tray"]
    cases = []
    seeds = list(range(30)) if randomized else [None]
    for seed in seeds:
        for base in base_cases[: total if not randomized else 5]:
            case = {"baseCase": base, "case": base, "physicalValidity": True, "leakageClean": True}
            if randomized:
                case["seed"] = seed
                case["case"] = f"{base}__seed_{seed:03d}"
                case["sampledLayout"] = {"body": [seed, base, 0]}
            cases.append(case)
    summary = {
        "total": total,
        "passed": passed,
        "failed": total - passed,
        "failureClassification": {"passed": passed},
        "physicalValidity": physical,
        "leakage": leakage,
    }
    _write_json(path / "report.json", {
        "schemaVersion": "csg.benchmark_report.v2",
        "summary": summary,
        "sourceProvenance": _provenance(kind=provenance_kind),
        "randomized": {"enabled": randomized, "seeds": list(range(30)) if randomized else []},
        "cases": cases,
        "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
    })
    _write_json(path / "failure_classification.json", {
        "schemaVersion": "csg.benchmark_failure_classification.v1",
        "sourceProvenance": _provenance(kind=provenance_kind),
        "summary": {"passed": passed},
        "physicalValiditySummary": physical,
        "leakageSummary": leakage,
        "cases": [],
    })
    (path / "report.md").write_text("# report\n", encoding="utf-8")
    (path / "summary.csv").write_text("case,status\n", encoding="utf-8")


def _write_comparison_dir(path, *, provenance_kind="source_snapshot"):
    _write_json(path / "comparison_report.json", {
        "schemaVersion": "csg.benchmark_comparison.v1",
        "sourceProvenance": _provenance(kind=provenance_kind),
        "baselineOrder": ["symbolic", "noop", "mujoco"],
        "baselines": {
            "symbolic": {
                "summary": {"total": 5, "passed": 5, "failed": 0, "physicalValidity": {"unverified": 5}},
                "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
            },
            "noop": {
                "expectedFailure": True,
                "summary": {
                    "total": 5,
                    "passed": 0,
                    "failed": 5,
                    "physicalValidity": {"unverified": 5},
                    "failureClassification": {"contact_missing": 1, "event_order_wrong": 4},
                },
                "confusion": {"missedDiagonal": ["put_cube_in_tray"], "unexpectedOffDiagonalPasses": []},
            },
            "mujoco": {
                "summary": {"total": 5, "passed": 5, "failed": 0, "physicalValidity": {"valid": 5}},
                "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
            },
        },
    })


def _write_invalid_dir(path, *, provenance_kind="source_snapshot"):
    categories = ["physical_invalidity"] * 6 + ["contact_missing", "relation_not_achieved", "event_order_wrong"]
    _write_json(path / "invalid_fixtures_report.json", {
        "schemaVersion": "csg.invalid_fixture_report.v1",
        "sourceProvenance": _provenance(kind=provenance_kind),
        "summary": {"total": 9, "matched": 9, "mismatched": 0},
        "fixtures": [{"result": {"failureClassification": {"category": category}}} for category in categories],
    })


def test_release_audit_accepts_complete_phase_2e_artifacts(tmp_path):
    symbolic = tmp_path / "symbolic"
    mujoco = tmp_path / "mujoco"
    randomized = tmp_path / "randomized"
    comparison = tmp_path / "comparison"
    invalid = tmp_path / "invalid"
    _write_benchmark_dir(symbolic, total=5, passed=5, physical={"unverified": 5}, leakage={"clean": 5, "dirty": 0})
    _write_benchmark_dir(mujoco, total=5, passed=5, physical={"valid": 5}, leakage={"clean": 5, "dirty": 0})
    _write_benchmark_dir(randomized, total=150, passed=150, physical={"valid": 150}, leakage={"clean": 150, "dirty": 0}, randomized=True)
    _write_comparison_dir(comparison)
    _write_invalid_dir(invalid)

    report = audit_release_artifacts(symbolic, mujoco, randomized, comparison, invalid, seeds=30)

    assert report["schemaVersion"] == "csg.release_audit.v1"
    assert report["ok"] is True
    assert report["summary"]["checksFailed"] == 0
    check_names = {check["name"] for check in report["checks"]}
    assert "comparison:noop:failed" in check_names
    assert "comparison:noop:classes" in check_names


def test_release_audit_reports_missing_or_failing_artifacts(tmp_path):
    symbolic = tmp_path / "symbolic"
    _write_benchmark_dir(symbolic, total=5, passed=4, physical={"unverified": 5}, leakage={"clean": 4, "dirty": 1})

    report = audit_release_artifacts(
        symbolic,
        tmp_path / "missing_mujoco",
        tmp_path / "missing_randomized",
        tmp_path / "missing_comparison",
        tmp_path / "missing_invalid",
        seeds=30,
    )

    assert report["ok"] is False
    messages = "\n".join(check["message"] for check in report["checks"] if not check["ok"])
    assert "symbolic summary passed expected 5 got 4" in messages
    assert "missing_mujoco/report.json missing" in messages


def test_release_audit_final_metadata_mode_requires_git_and_license(tmp_path):
    symbolic = tmp_path / "symbolic"
    mujoco = tmp_path / "mujoco"
    randomized = tmp_path / "randomized"
    comparison = tmp_path / "comparison"
    invalid = tmp_path / "invalid"
    _write_benchmark_dir(symbolic, total=5, passed=5, physical={"unverified": 5}, leakage={"clean": 5, "dirty": 0})
    _write_benchmark_dir(mujoco, total=5, passed=5, physical={"valid": 5}, leakage={"clean": 5, "dirty": 0})
    _write_benchmark_dir(randomized, total=150, passed=150, physical={"valid": 150}, leakage={"clean": 150, "dirty": 0}, randomized=True)
    _write_comparison_dir(comparison)
    _write_invalid_dir(invalid)

    report = audit_release_artifacts(
        symbolic,
        mujoco,
        randomized,
        comparison,
        invalid,
        seeds=30,
        require_final_metadata=True,
        project_root=tmp_path,
    )

    assert report["ok"] is False
    messages = "\n".join(check["message"] for check in report["checks"] if not check["ok"])
    assert "final metadata requires .git directory" in messages
    assert "final metadata requires LICENSE" in messages
    assert "final metadata requires pyproject.toml license metadata" in messages
    assert "symbolic sourceProvenance kind expected git got source_snapshot" in messages


def test_release_audit_final_metadata_mode_accepts_git_and_license(tmp_path):
    symbolic = tmp_path / "symbolic"
    mujoco = tmp_path / "mujoco"
    randomized = tmp_path / "randomized"
    comparison = tmp_path / "comparison"
    invalid = tmp_path / "invalid"
    for path, physical, total in [
        (symbolic, {"unverified": 5}, 5),
        (mujoco, {"valid": 5}, 5),
    ]:
        _write_benchmark_dir(path, total=total, passed=total, physical=physical, leakage={"clean": total, "dirty": 0}, provenance_kind="git")
    _write_benchmark_dir(randomized, total=150, passed=150, physical={"valid": 150}, leakage={"clean": 150, "dirty": 0}, randomized=True, provenance_kind="git")
    _write_comparison_dir(comparison, provenance_kind="git")
    _write_invalid_dir(invalid, provenance_kind="git")
    (tmp_path / ".git").mkdir()
    (tmp_path / "LICENSE").write_text("Example license\n", encoding="utf-8")
    (tmp_path / "pyproject.toml").write_text('[project]\nlicense = "MIT"\n', encoding="utf-8")

    report = audit_release_artifacts(
        symbolic,
        mujoco,
        randomized,
        comparison,
        invalid,
        seeds=30,
        require_final_metadata=True,
        project_root=tmp_path,
    )

    assert report["ok"] is True
    assert not [check for check in report["checks"] if not check["ok"]]
