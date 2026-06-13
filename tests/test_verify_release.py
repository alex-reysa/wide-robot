import json
import tarfile

import pytest

from csg import verify_release as vr

COMMIT = "a" * 40


def _prov(*, commit=COMMIT, kind="git", dirty=False):
    git = {"commit": commit, "dirty": dirty, "statusPorcelain": []} if kind == "git" else None
    return {
        "schemaVersion": "csg.source_provenance.v1",
        "kind": kind,
        "git": git,
        "snapshot": {"algorithm": "sha256", "digest": "0" * 64, "fileCount": 0, "files": []},
    }


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _benchmark_dir(path, *, total, physical, randomized=False, commit=COMMIT, kind="git"):
    base_cases = ["insert_object", "open_drawer", "place_on_top", "push_object", "put_cube_in_tray"]
    cases = []
    seeds = list(range(30)) if randomized else [None]
    for seed in seeds:
        for base in base_cases:
            case = {"baseCase": base, "case": base, "physicalValidity": True, "leakageClean": True}
            if randomized:
                case["seed"] = seed
                case["case"] = f"{base}__seed_{seed:03d}"
                case["sampledLayout"] = {"body": [seed, base, 0]}
            cases.append(case)
    summary = {
        "total": total,
        "passed": total,
        "failed": 0,
        "failureClassification": {"passed": total},
        "physicalValidity": physical,
        "leakage": {"clean": total, "dirty": 0},
    }
    _write_json(path / "report.json", {
        "schemaVersion": "csg.benchmark_report.v2",
        "summary": summary,
        "sourceProvenance": _prov(commit=commit, kind=kind),
        "randomized": {"enabled": randomized, "seeds": list(range(30)) if randomized else []},
        "cases": cases,
        "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
    })
    _write_json(path / "failure_classification.json", {
        "schemaVersion": "csg.benchmark_failure_classification.v1",
        "sourceProvenance": _prov(commit=commit, kind=kind),
        "summary": {"passed": total},
        "physicalValiditySummary": physical,
        "leakageSummary": {"clean": total, "dirty": 0},
        "cases": [],
    })
    (path / "report.md").write_text("# report\n", encoding="utf-8")
    (path / "summary.csv").write_text("case,status\n", encoding="utf-8")


def _comparison_dir(path, *, commit=COMMIT, kind="git"):
    _write_json(path / "comparison_report.json", {
        "schemaVersion": "csg.benchmark_comparison.v1",
        "sourceProvenance": _prov(commit=commit, kind=kind),
        "baselineOrder": ["symbolic", "noop", "mujoco"],
        "baselines": {
            "symbolic": {
                "summary": {"total": 5, "passed": 5, "failed": 0, "physicalValidity": {"unverified": 5}},
                "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
            },
            "noop": {
                "expectedFailure": True,
                "summary": {
                    "total": 5, "passed": 0, "failed": 5,
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


def _invalid_dir(path, *, commit=COMMIT, kind="git"):
    categories = ["physical_invalidity"] * 6 + ["contact_missing", "relation_not_achieved", "event_order_wrong"]
    _write_json(path / "invalid_fixtures_report.json", {
        "schemaVersion": "csg.invalid_fixture_report.v1",
        "sourceProvenance": _prov(commit=commit, kind=kind),
        "summary": {"total": 9, "matched": 9, "mismatched": 0},
        "fixtures": [{"result": {"failureClassification": {"category": c}}} for c in categories],
    })


def _build_reports(root, *, commit=COMMIT, kind="git"):
    _benchmark_dir(root / "symbolic", total=5, physical={"unverified": 5}, commit=commit, kind=kind)
    _benchmark_dir(root / "mujoco", total=5, physical={"valid": 5}, commit=commit, kind=kind)
    _benchmark_dir(root / "mujoco_randomized_30", total=150, physical={"valid": 150},
                   randomized=True, commit=commit, kind=kind)
    _comparison_dir(root / "comparison", commit=commit, kind=kind)
    _invalid_dir(root / "invalid_fixtures", commit=commit, kind=kind)


def _make_tarball(src_root, dest_tgz):
    with tarfile.open(dest_tgz, "w:gz") as tar:
        for path in sorted(src_root.rglob("*")):
            if path.is_file():
                tar.add(path, arcname=path.relative_to(src_root).as_posix())


def _build_asset_dir(tmp_path, *, commit=COMMIT, kind="git"):
    reports = tmp_path / "reports_src"
    _build_reports(reports, commit=commit, kind=kind)
    assets = tmp_path / "assets"
    assets.mkdir()
    tarball = assets / "phase2e-report-artifacts.tar.gz"
    _make_tarball(reports, tarball)
    wheel = assets / "csg-9.9.9-py3-none-any.whl"
    wheel.write_bytes(b"not really a wheel")
    sums = "".join(
        f"{vr.sha256_file(p)}  {p.name}\n"
        for p in [tarball, wheel]
    )
    (assets / "RELEASE_SHA256SUMS").write_text(sums, encoding="utf-8")
    return assets


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def test_sha256_file_matches_hashlib(tmp_path):
    import hashlib
    payload = b"hello reproducibility"
    target = tmp_path / "f.bin"
    target.write_bytes(payload)
    assert vr.sha256_file(target) == hashlib.sha256(payload).hexdigest()


def test_parse_sha256sums_handles_standard_and_binary_marker():
    text = (
        f"{'a' * 64}  one.tar.gz\n"
        f"{'B' * 64} *two.whl\n"
        "\n"
        f"{'c' * 64}  ./three.tar.gz\n"
    )
    parsed = vr.parse_sha256sums(text)
    assert parsed == {"one.tar.gz": "a" * 64, "two.whl": "b" * 64, "three.tar.gz": "c" * 64}


def test_parse_sha256sums_rejects_malformed():
    with pytest.raises(ValueError):
        vr.parse_sha256sums("not a checksum line\n")


def test_verify_checksums_flags_mismatch_and_missing(tmp_path):
    good = tmp_path / "good.bin"
    good.write_bytes(b"good")
    sums = {"good.bin": vr.sha256_file(good), "bad.bin": "f" * 64, "missing.bin": "e" * 64}
    (tmp_path / "bad.bin").write_bytes(b"tampered")
    checks = {c["name"]: c["ok"] for c in vr.verify_checksums(tmp_path, sums)}
    assert checks["checksum:good.bin"] is True
    assert checks["checksum:bad.bin"] is False
    assert checks["checksum:missing.bin"] is False


def test_locate_report_dirs_reads_seed_count_from_name(tmp_path):
    (tmp_path / "mujoco_randomized_7").mkdir()
    dirs = vr.locate_report_dirs(tmp_path, seeds=30)
    assert dirs["seeds"] == 7
    assert dirs["randomized_dir"].name == "mujoco_randomized_7"
    assert dirs["symbolic_dir"] == tmp_path / "symbolic"


def test_verify_report_commits_pass_and_fail(tmp_path):
    _build_reports(tmp_path)
    ok = vr.verify_report_commits(tmp_path, COMMIT)
    assert all(c["ok"] for c in ok)
    # one file per report (symbolic/mujoco/randomized: report + failure_classification; comparison; invalid)
    assert len(ok) == 8

    wrong = vr.verify_report_commits(tmp_path, "b" * 40)
    assert all(not c["ok"] for c in wrong)


def test_verify_report_commits_rejects_source_snapshot(tmp_path):
    _build_reports(tmp_path, kind="source_snapshot")
    checks = vr.verify_report_commits(tmp_path, COMMIT)
    assert all(not c["ok"] for c in checks)


# ---------------------------------------------------------------------------
# Orchestration (offline via asset_dir + injected expected_commit)
# ---------------------------------------------------------------------------


def test_verify_release_accepts_a_good_release(tmp_path):
    assets = _build_asset_dir(tmp_path)
    report = vr.verify_release("v9.9.9", asset_dir=assets, expected_commit=COMMIT, seeds=30)
    assert report["schemaVersion"] == "csg.verify_release.v1"
    assert report["ok"] is True
    assert report["summary"]["checksFailed"] == 0
    names = {c["name"] for c in report["checks"]}
    assert "checksum:phase2e-report-artifacts.tar.gz" in names
    assert "provenance:symbolic/report.json" in names


def test_verify_release_detects_tampered_asset(tmp_path):
    assets = _build_asset_dir(tmp_path)
    (assets / "csg-9.9.9-py3-none-any.whl").write_bytes(b"tampered after sums were written")
    report = vr.verify_release("v9.9.9", asset_dir=assets, expected_commit=COMMIT, seeds=30)
    assert report["ok"] is False
    failed = {c["name"] for c in report["checks"] if not c["ok"]}
    assert "checksum:csg-9.9.9-py3-none-any.whl" in failed


def test_verify_release_detects_commit_mismatch(tmp_path):
    assets = _build_asset_dir(tmp_path)
    report = vr.verify_release("v9.9.9", asset_dir=assets, expected_commit="b" * 40, seeds=30)
    assert report["ok"] is False
    failed = {c["name"] for c in report["checks"] if not c["ok"]}
    assert any(name.startswith("provenance:") for name in failed)


def test_main_returns_0_on_good_release(tmp_path, monkeypatch):
    assets = _build_asset_dir(tmp_path)
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: COMMIT)
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(assets)])
    assert rc == 0


def test_main_returns_2_on_bad_release(tmp_path, monkeypatch):
    assets = _build_asset_dir(tmp_path)
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: "b" * 40)
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(assets)])
    assert rc == 2
