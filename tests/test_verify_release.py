"""Tests for csg.verify_release.

These exercise the *real* trust-binding layer — git archive, tar round-trips,
source-snapshot recompute, and the wheel/sdist source check — not just hand-built
fixtures. The synthetic helpers build a release whose reports' source snapshot
and whose distributions' csg/*.py are genuinely derived from a source tree, so a
tamper anywhere breaks a binding.
"""
import io
import json
import os
import subprocess
import tarfile
import zipfile
from pathlib import Path

import pytest

from csg import release_manifest as rm
from csg import verify_release as vr

COMMIT = "a" * 40

# A minimal source tree whose files match SOURCE_PROVENANCE_GLOBS.
SRC_FILES = {
    "pyproject.toml": b"[project]\nname = \"csg\"\nversion = \"9.9.9\"\n",
    "README.md": b"# csg test tree\n",
    "csg/__init__.py": b"__version__ = \"9.9.9\"\n",
    "csg/solver.py": b"def solve():\n    return 42\n",
    "csg/matcher.py": b"def match(a, b):\n    return a == b\n",
}


# ---------------------------------------------------------------------------
# Builders
# ---------------------------------------------------------------------------


def _write_tree(root: Path, files=SRC_FILES) -> Path:
    for rel, data in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    return root


def _prov(snapshot, *, commit=COMMIT, kind="git", dirty=False):
    git = {"commit": commit, "dirty": dirty, "statusPorcelain": []} if kind == "git" else None
    return {
        "schemaVersion": "csg.source_provenance.v1",
        "kind": kind,
        "root": "/x",
        "git": git,
        "snapshot": snapshot,
    }


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _benchmark_dir(path, *, total, physical, snapshot, randomized=False, commit=COMMIT, kind="git"):
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
        "total": total, "passed": total, "failed": 0,
        "failureClassification": {"passed": total},
        "physicalValidity": physical,
        "leakage": {"clean": total, "dirty": 0},
    }
    _write_json(path / "report.json", {
        "schemaVersion": "csg.benchmark_report.v2",
        "summary": summary,
        "sourceProvenance": _prov(snapshot, commit=commit, kind=kind),
        "randomized": {"enabled": randomized, "seeds": list(range(30)) if randomized else []},
        "cases": cases,
        "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
    })
    _write_json(path / "failure_classification.json", {
        "schemaVersion": "csg.benchmark_failure_classification.v1",
        "sourceProvenance": _prov(snapshot, commit=commit, kind=kind),
        "summary": {"passed": total},
        "physicalValiditySummary": physical,
        "leakageSummary": {"clean": total, "dirty": 0},
        "cases": [],
    })
    (path / "report.md").write_text("# report\n", encoding="utf-8")
    (path / "summary.csv").write_text("case,status\n", encoding="utf-8")


def _comparison_dir(path, *, snapshot, commit=COMMIT, kind="git"):
    _write_json(path / "comparison_report.json", {
        "schemaVersion": "csg.benchmark_comparison.v1",
        "sourceProvenance": _prov(snapshot, commit=commit, kind=kind),
        "baselineOrder": ["symbolic", "noop", "mujoco"],
        "baselines": {
            "symbolic": {
                "summary": {"total": 5, "passed": 5, "failed": 0, "physicalValidity": {"unverified": 5}},
                "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
            },
            "noop": {
                "expectedFailure": True,
                "summary": {"total": 5, "passed": 0, "failed": 5,
                            "physicalValidity": {"unverified": 5},
                            "failureClassification": {"contact_missing": 1, "event_order_wrong": 4}},
                "confusion": {"missedDiagonal": ["put_cube_in_tray"], "unexpectedOffDiagonalPasses": []},
            },
            "mujoco": {
                "summary": {"total": 5, "passed": 5, "failed": 0, "physicalValidity": {"valid": 5}},
                "confusion": {"missedDiagonal": [], "unexpectedOffDiagonalPasses": []},
            },
        },
    })


def _invalid_dir(path, *, snapshot, commit=COMMIT, kind="git"):
    categories = ["physical_invalidity"] * 6 + ["contact_missing", "relation_not_achieved", "event_order_wrong"]
    _write_json(path / "invalid_fixtures_report.json", {
        "schemaVersion": "csg.invalid_fixture_report.v1",
        "sourceProvenance": _prov(snapshot, commit=commit, kind=kind),
        "summary": {"total": 9, "matched": 9, "mismatched": 0},
        "fixtures": [{"result": {"failureClassification": {"category": c}}} for c in categories],
    })


def _build_reports(root, *, snapshot, commit=COMMIT, kind="git"):
    _benchmark_dir(root / "symbolic", total=5, physical={"unverified": 5}, snapshot=snapshot, commit=commit, kind=kind)
    _benchmark_dir(root / "mujoco", total=5, physical={"valid": 5}, snapshot=snapshot, commit=commit, kind=kind)
    _benchmark_dir(root / "mujoco_randomized_30", total=150, physical={"valid": 150},
                   randomized=True, snapshot=snapshot, commit=commit, kind=kind)
    _comparison_dir(root / "comparison", snapshot=snapshot, commit=commit, kind=kind)
    _invalid_dir(root / "invalid_fixtures", snapshot=snapshot, commit=commit, kind=kind)


def _make_tarball(src_root, dest_tgz):
    with tarfile.open(dest_tgz, "w:gz") as tar:
        for path in sorted(src_root.rglob("*")):
            if path.is_file():
                tar.add(path, arcname=path.relative_to(src_root).as_posix())


def _build_wheel(dest_whl, source_tree, *, files=None):
    """A wheel is a zip with csg/* at the top level + a dist-info."""
    with zipfile.ZipFile(dest_whl, "w") as zf:
        if files is None:
            for p in sorted((source_tree / "csg").rglob("*.py")):
                zf.writestr(p.relative_to(source_tree).as_posix(), p.read_bytes())
        else:
            for name, data in files.items():
                zf.writestr(name, data)
        zf.writestr("csg-9.9.9.dist-info/METADATA", "Name: csg\nVersion: 9.9.9\n")


def _build_sdist(dest_tgz, source_tree, *, files=None):
    """An sdist nests the package under <name>-<version>/."""
    with tarfile.open(dest_tgz, "w:gz") as tar:
        items = (
            {f"csg-9.9.9/{p.relative_to(source_tree).as_posix()}": p.read_bytes()
             for p in sorted((source_tree / "csg").rglob("*.py"))}
            if files is None else files
        )
        for name, data in items.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))


def build_release(tmp_path, *, snapshot, source_tree, reports_mutator=None):
    """Build a complete, internally-consistent release asset directory."""
    reports = tmp_path / "reports_src"
    _build_reports(reports, snapshot=snapshot)
    if reports_mutator is not None:
        reports_mutator(reports)

    assets = tmp_path / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    _make_tarball(reports, assets / "phase2e-report-artifacts.tar.gz")
    _build_wheel(assets / "csg-9.9.9-py3-none-any.whl", source_tree)
    _build_sdist(assets / "csg-9.9.9.tar.gz", source_tree)
    rm.write_sha256sums(assets)
    manifest = rm.build_manifest(
        tag="v9.9.9", commit=COMMIT, asset_dir=assets, reports_root=reports, version="9.9.9", seeds=30,
    )
    _write_json(assets / "release_manifest.json", manifest)
    return assets


@pytest.fixture
def release(tmp_path):
    source_tree = _write_tree(tmp_path / "source")
    snapshot = vr.compute_source_snapshot(source_tree)
    assets = build_release(tmp_path, snapshot=snapshot, source_tree=source_tree)
    return {"assets": assets, "source_tree": source_tree, "snapshot": snapshot, "tmp": tmp_path}


def _verify(release, **kw):
    return vr.verify_release(
        "v9.9.9", asset_dir=str(release["assets"]), expected_commit=COMMIT,
        source_tree=str(release["source_tree"]), seeds=30, **kw,
    )


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


def test_parse_sha256sums_rejects_duplicate_filename():
    # F3: a decoy line must not silently shadow the enforced one.
    text = f"{'a' * 64}  pkg.whl\n{'b' * 64}  pkg.whl\n"
    with pytest.raises(ValueError, match="duplicate"):
        vr.parse_sha256sums(text)


def test_parse_sha256sums_rejects_unsafe_filename():
    for bad in (f"{'a' * 64}  ../escape.whl\n", f"{'a' * 64}  /etc/hosts\n", f"{'a' * 64}  sub/dir.whl\n"):
        with pytest.raises(ValueError, match="unsafe"):
            vr.parse_sha256sums(bad)


def test_parse_sha256sums_rejects_tab_and_crlf():
    with pytest.raises(ValueError):
        vr.parse_sha256sums(f"{'a' * 64}\tpkg.whl\n")
    with pytest.raises(ValueError):
        vr.parse_sha256sums(f"{'a' * 64}  pkg.whl\r\n")


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


def test_origin_repo_handles_ssh_https_and_nested(monkeypatch):
    cases = {
        "git@github.com:alex-reysa/wide-robot.git": "alex-reysa/wide-robot",
        "https://github.com/alex-reysa/wide-robot.git": "alex-reysa/wide-robot",
        "https://github.com/alex-reysa/wide-robot": "alex-reysa/wide-robot",
        "https://gitlab.com/group/subgroup/repo.GIT": "group/subgroup/repo",
        "ssh://git@host:22/owner/repo.git": "owner/repo",
    }
    for url, expected in cases.items():
        monkeypatch.setattr(vr.subprocess, "run", lambda *a, _u=url, **k: type("P", (), {"stdout": _u})())
        assert vr.origin_repo() == expected


def test_aggregate_snapshot_digest_matches_benchmark(tmp_path):
    tree = _write_tree(tmp_path / "src")
    snap = vr.compute_source_snapshot(tree)
    assert vr._aggregate_snapshot_digest(snap["files"]) == snap["digest"]
    assert vr._aggregate_snapshot_digest("not a list") is None


# ---------------------------------------------------------------------------
# verify_report_commits (snapshot binding)
# ---------------------------------------------------------------------------


def test_verify_report_commits_pass_and_fail(tmp_path):
    snap = {"algorithm": "sha256", "digest": "0" * 64, "fileCount": 0, "files": []}
    _build_reports(tmp_path, snapshot=snap)
    ok = vr.verify_report_commits(tmp_path, COMMIT)
    assert all(c["ok"] for c in ok)
    assert len(ok) == 8  # provenance only (no snapshot binding requested)
    wrong = vr.verify_report_commits(tmp_path, "b" * 40)
    assert all(not c["ok"] for c in wrong)


def test_verify_report_commits_binds_snapshot(tmp_path):
    source_tree = _write_tree(tmp_path / "source")
    snap = vr.compute_source_snapshot(source_tree)
    _build_reports(tmp_path / "good", snapshot=snap)
    checks = vr.verify_report_commits(tmp_path / "good", COMMIT, expected_snapshot=snap)
    assert all(c["ok"] for c in checks)
    assert any(c["name"].startswith("snapshot:") for c in checks)

    # A swapped digest is caught.
    bad = dict(snap, digest="deadbeef" * 8)
    _build_reports(tmp_path / "bad", snapshot=bad)
    checks = vr.verify_report_commits(tmp_path / "bad", COMMIT, expected_snapshot=snap)
    assert any(not c["ok"] and c["name"].startswith("snapshot:") for c in checks)


def test_verify_report_commits_rejects_lying_files_table(tmp_path):
    source_tree = _write_tree(tmp_path / "source")
    snap = vr.compute_source_snapshot(source_tree)
    # Keep the (correct) digest but lie in the per-file table → internal
    # inconsistency must be detected.
    lying = dict(snap, files=[{"path": "csg/solver.py", "sha256": "00" * 32}])
    _build_reports(tmp_path / "lie", snapshot=lying)
    checks = vr.verify_report_commits(tmp_path / "lie", COMMIT, expected_snapshot=snap)
    assert any(not c["ok"] and c["name"].startswith("snapshot:") for c in checks)


# ---------------------------------------------------------------------------
# Orchestration (offline, synthetic source tree)
# ---------------------------------------------------------------------------


def test_verify_release_accepts_a_good_release(release):
    report = _verify(release)
    assert report["ok"] is True, [c for c in report["checks"] if not c["ok"]]
    names = {c["name"] for c in report["checks"]}
    assert "checksum:phase2e-report-artifacts.tar.gz" in names
    assert "provenance:symbolic/report.json" in names
    assert "snapshot:symbolic/report.json" in names
    assert "source_dist:csg-9.9.9-py3-none-any.whl" in names
    assert "manifest:commit" in names


def test_verify_release_detects_tampered_asset(release):
    (release["assets"] / "csg-9.9.9.tar.gz").write_bytes(b"tampered after sums were written")
    report = _verify(release)
    assert report["ok"] is False
    failed = {c["name"] for c in report["checks"] if not c["ok"]}
    assert "checksum:csg-9.9.9.tar.gz" in failed


def test_verify_release_detects_commit_mismatch(release):
    report = vr.verify_release(
        "v9.9.9", asset_dir=str(release["assets"]), expected_commit="b" * 40,
        source_tree=str(release["source_tree"]), seeds=30,
    )
    assert report["ok"] is False
    failed = {c["name"] for c in report["checks"] if not c["ok"]}
    assert any(name.startswith("provenance:") for name in failed)


def test_verify_release_defeats_trojan_wheel(release):
    # F1: the trojan keeps a valid checksum + manifest entry but its csg source
    # diverges from the tagged tree.
    trojan = b"def solve():\n    steal_secrets()\n"
    _build_wheel(release["assets"] / "csg-9.9.9-py3-none-any.whl", release["source_tree"],
                 files={"csg/__init__.py": SRC_FILES["csg/__init__.py"],
                        "csg/solver.py": trojan,
                        "csg/matcher.py": SRC_FILES["csg/matcher.py"]})
    rm.write_sha256sums(release["assets"])
    manifest = rm.build_manifest(tag="v9.9.9", commit=COMMIT, asset_dir=release["assets"],
                                 reports_root=release["tmp"] / "reports_src", version="9.9.9", seeds=30)
    _write_json(release["assets"] / "release_manifest.json", manifest)
    report = _verify(release)
    assert report["ok"] is False
    failed = {c["name"] for c in report["checks"] if not c["ok"]}
    assert "source_dist:csg-9.9.9-py3-none-any.whl" in failed


def test_verify_release_flags_extra_csg_module_in_wheel(release):
    _build_wheel(release["assets"] / "csg-9.9.9-py3-none-any.whl", release["source_tree"],
                 files={**{p.relative_to(release["source_tree"]).as_posix(): p.read_bytes()
                           for p in (release["source_tree"] / "csg").rglob("*.py")},
                        "csg/backdoor.py": b"# injected\n"})
    rm.write_sha256sums(release["assets"])
    manifest = rm.build_manifest(tag="v9.9.9", commit=COMMIT, asset_dir=release["assets"],
                                 reports_root=release["tmp"] / "reports_src", version="9.9.9", seeds=30)
    _write_json(release["assets"] / "release_manifest.json", manifest)
    report = _verify(release)
    assert report["ok"] is False


def test_verify_release_rejects_wheel_with_smuggled_top_level_module(release):
    # F1 residual: a backdoor placed OUTSIDE csg/ (intact csg source) must still
    # fail — the canonical wheel ships only the csg package + metadata.
    src = release["source_tree"]
    _build_wheel(release["assets"] / "csg-9.9.9-py3-none-any.whl", src,
                 files={**{p.relative_to(src).as_posix(): p.read_bytes()
                           for p in (src / "csg").rglob("*.py")},
                        "evilpkg/__init__.py": b"import os  # backdoor\n"})
    rm.write_sha256sums(release["assets"])
    manifest = rm.build_manifest(tag="v9.9.9", commit=COMMIT, asset_dir=release["assets"],
                                 reports_root=release["tmp"] / "reports_src", version="9.9.9", seeds=30)
    _write_json(release["assets"] / "release_manifest.json", manifest)
    report = _verify(release)
    assert report["ok"] is False
    bad = next(c for c in report["checks"] if c["name"] == "source_dist:csg-9.9.9-py3-none-any.whl")
    assert "evilpkg" in bad["message"]


def test_verify_release_rejects_decoy_report_tarball(release):
    # F2: a second report tarball must not mask the real one.
    import shutil
    shutil.copy(release["assets"] / "phase2e-report-artifacts.tar.gz",
                release["assets"] / "0000-report-artifacts.tar.gz")
    report = _verify(release)
    assert report["ok"] is False
    assert any(c["name"] == "reports:tarball" and not c["ok"] for c in report["checks"])


def test_verify_release_flags_unlisted_extra_asset(release):
    # F4: an asset not in the manifest must fail.
    (release["assets"] / "evil-extra.bin").write_bytes(b"surprise")
    report = _verify(release)
    assert report["ok"] is False
    assert any(c["name"] == "manifest:asset_set" and not c["ok"] for c in report["checks"])


def test_verify_release_reconciles_manifest_commit(release):
    manifest = json.loads((release["assets"] / "release_manifest.json").read_text())
    manifest["commit"] = "f" * 40
    _write_json(release["assets"] / "release_manifest.json", manifest)
    report = _verify(release)
    assert report["ok"] is False
    assert any(c["name"] == "manifest:commit" and not c["ok"] for c in report["checks"])


# ---------------------------------------------------------------------------
# Exit-code contract (F6 / F10): hostile bytes → exit 2, never a traceback
# ---------------------------------------------------------------------------


def _inject_source_tree(monkeypatch, source_tree):
    def fake(commit, dest, *, cwd="."):
        import shutil
        shutil.copytree(source_tree, dest, dirs_exist_ok=True)
        return Path(dest)
    monkeypatch.setattr(vr, "resolve_source_tree", fake)
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: COMMIT)


def test_main_returns_0_on_good_release(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 0


def test_main_returns_2_on_commit_mismatch(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: "b" * 40)
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 2


def test_main_returns_2_on_malformed_sums(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    (release["assets"] / "RELEASE_SHA256SUMS").write_text("totally not a checksums file\n")
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 2


def test_main_returns_2_on_truncated_tarball(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    tgz = release["assets"] / "phase2e-report-artifacts.tar.gz"
    tgz.write_bytes(tgz.read_bytes()[: 100])  # truncate
    rm.write_sha256sums(release["assets"])  # keep checksum honest so we hit the extractor
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 2


def test_main_returns_2_on_corrupt_report_json(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    # Re-pack reports with a corrupt report.json.
    import shutil
    work = release["tmp"] / "corrupt"
    with tarfile.open(release["assets"] / "phase2e-report-artifacts.tar.gz") as t:
        t.extractall(work, filter="data")
    (work / "symbolic" / "report.json").write_text("{ this is not json")
    (release["assets"] / "phase2e-report-artifacts.tar.gz").unlink()
    _make_tarball(work, release["assets"] / "phase2e-report-artifacts.tar.gz")
    rm.write_sha256sums(release["assets"])
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 2


def test_main_returns_3_when_tag_unresolved(release, monkeypatch):
    def boom(tag, **kw):
        raise vr.VerifyReleaseError("could not resolve commit for tag")
    monkeypatch.setattr(vr, "resolve_tag_commit", boom)
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download"])
    assert rc == 3


# ---------------------------------------------------------------------------
# Malicious tarball members (F7) — symlink write-through must not escape
# ---------------------------------------------------------------------------


def test_safe_extract_rejects_symlink_member(tmp_path):
    tgz = tmp_path / "evil.tar.gz"
    escape = tmp_path / "ESCAPE_TARGET"
    with tarfile.open(tgz, "w:gz") as tar:
        link = tarfile.TarInfo("x")
        link.type = tarfile.SYMTYPE
        link.linkname = str(escape)
        tar.addfile(link)
        data = b"pwned"
        f = tarfile.TarInfo("x/pwned")
        f.size = len(data)
        tar.addfile(f, io.BytesIO(data))
    with tarfile.open(tgz) as tar:
        with pytest.raises(vr.ReleaseContentError):
            vr._safe_extract(tar, tmp_path / "dest")
    assert not (escape / "pwned").exists()


def test_safe_extract_rejects_path_traversal(tmp_path):
    tgz = tmp_path / "trav.tar.gz"
    with tarfile.open(tgz, "w:gz") as tar:
        data = b"x"
        info = tarfile.TarInfo("../../../../tmp/csg_pwned_escape")
        info.size = len(data)
        tar.addfile(info, io.BytesIO(data))
    with tarfile.open(tgz) as tar:
        with pytest.raises(vr.ReleaseContentError):
            vr._safe_extract(tar, tmp_path / "dest")


# ---------------------------------------------------------------------------
# Real git / gh layer (F9) — no monkeypatching of the git archive path
# ---------------------------------------------------------------------------


def _git(repo, *args):
    subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True,
                   env={**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
                        "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"})


def test_end_to_end_against_a_real_git_repo(tmp_path, monkeypatch):
    """Exercises resolve_tag_commit + resolve_source_tree (git archive) + the
    real tar round-trip + snapshot recompute + source-dist check, with nothing
    in the git layer monkeypatched. Also drives the flagship CLI (main)."""
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init", "-q")
    _write_tree(repo)
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "release")
    _git(repo, "tag", "-a", "vtest", "-m", "vtest")
    commit = vr.resolve_tag_commit("vtest", cwd=repo)

    # Build the source tree + snapshot exactly as the verifier will derive them.
    src = vr.resolve_source_tree(commit, tmp_path / "arch", cwd=repo)
    snapshot = vr.compute_source_snapshot(src)

    reports = tmp_path / "reports_src"
    _build_reports(reports, snapshot=snapshot, commit=commit)
    assets = tmp_path / "assets"
    assets.mkdir()
    _make_tarball(reports, assets / "phase2e-report-artifacts.tar.gz")
    _build_wheel(assets / "csg-9.9.9-py3-none-any.whl", src)
    _build_sdist(assets / "csg-9.9.9.tar.gz", src)
    rm.write_sha256sums(assets)
    _write_json(assets / "release_manifest.json",
                rm.build_manifest(tag="vtest", commit=commit, asset_dir=assets,
                                  reports_root=reports, version="9.9.9", seeds=30))

    report = vr.verify_release("vtest", asset_dir=str(assets), download=False, seeds=30, cwd=str(repo))
    assert report["ok"] is True, [c for c in report["checks"] if not c["ok"]]
    assert report["expectedCommit"] == commit

    # Drive the actual CLI entrypoint (main) against the real release: this is
    # the flagship tool, exercised end to end (no git-layer monkeypatching).
    monkeypatch.chdir(repo)
    rc = vr.main(["--tag", "vtest", "--asset-dir", str(assets), "--no-download"])
    assert rc == 0

    # A trojan in the committed-vs-distributed source is caught end to end.
    _build_wheel(assets / "csg-9.9.9-py3-none-any.whl", src,
                 files={"csg/solver.py": b"def solve():\n    return 'evil'\n"})
    rm.write_sha256sums(assets)
    _write_json(assets / "release_manifest.json",
                rm.build_manifest(tag="vtest", commit=commit, asset_dir=assets,
                                  reports_root=reports, version="9.9.9", seeds=30))
    report = vr.verify_release("vtest", asset_dir=str(assets), download=False, seeds=30, cwd=str(repo))
    assert report["ok"] is False


def test_download_release_assets_uses_fake_gh(tmp_path, monkeypatch):
    """resolve via a fake gh on PATH (F9: the download shim was untested)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    payload = tmp_path / "payload"
    payload.mkdir()
    (payload / "RELEASE_SHA256SUMS").write_text("x\n")
    (payload / "phase2e-report-artifacts.tar.gz").write_bytes(b"tgz")
    fake = bindir / "gh"
    fake.write_text(
        "#!/usr/bin/env bash\n"
        "# args: release download <tag> --dir <dir> --clobber ...\n"
        "dir=\"\"; prev=\"\"\n"
        "for a in \"$@\"; do if [ \"$prev\" = \"--dir\" ]; then dir=\"$a\"; fi; prev=\"$a\"; done\n"
        f"cp {payload}/* \"$dir\"/\n"
    )
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    out = vr.download_release_assets("vX", tmp_path / "dl", repo="owner/repo")
    assert set(out) == {"RELEASE_SHA256SUMS", "phase2e-report-artifacts.tar.gz"}


def test_download_release_assets_reports_missing_gh(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path / "empty-nonexistent-bin"))
    with pytest.raises(vr.VerifyReleaseError, match="gh executable not found"):
        vr.download_release_assets("vX", tmp_path / "dl")


def test_resolve_remote_tag_commit_with_fake_gh(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    sha = "c" * 40
    fake = bindir / "gh"
    fake.write_text(
        "#!/usr/bin/env bash\n"
        f"echo '{json.dumps({'object': {'sha': sha, 'type': 'commit'}})}'\n"
    )
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    assert vr.resolve_remote_tag_commit("owner/repo", "vX") == sha


def test_resolve_remote_tag_commit_peels_annotated_tag(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    peeled = "d" * 40
    fake = bindir / "gh"
    # First call: refs/tags/* → {object:{sha:<tagobj>,type:tag}}.
    # Second call: git/tags/<tagobj> → {object:{sha:<peeled commit>}}.
    fake.write_text(
        "#!/usr/bin/env bash\n"
        "case \"$*\" in\n"
        "  *refs/tags/*) echo '{\"object\": {\"sha\": \"tagobj123\", \"type\": \"tag\"}}' ;;\n"
        f"  *git/tags/*) echo '{{\"object\": {{\"sha\": \"{peeled}\"}}}}' ;;\n"
        "esac\n"
    )
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    assert vr.resolve_remote_tag_commit("owner/repo", "vAnnotated") == peeled


def test_empty_download_is_operational_exit_3(tmp_path, monkeypatch):
    # F10: a gh download that writes zero assets is operational (exit 3), not a
    # silent "release is bad" with no assets fetched.
    bindir = tmp_path / "bin"
    bindir.mkdir()
    fake = bindir / "gh"
    fake.write_text("#!/usr/bin/env bash\nexit 0\n")  # "succeeds", writes nothing
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: COMMIT)
    monkeypatch.setattr(vr, "resolve_remote_tag_commit", lambda *a, **k: None)
    rc = vr.main(["--tag", "v9.9.9", "--work-dir", str(tmp_path / "wd")])
    assert rc == 3


def test_resolve_tag_commit_unresolved_raises(tmp_path):
    repo = tmp_path / "r"
    repo.mkdir()
    _git(repo, "init", "-q")
    _write_tree(repo)
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "c")
    with pytest.raises(vr.VerifyReleaseError):
        vr.resolve_tag_commit("no-such-tag", cwd=repo)


def test_main_json_output_smoke(release, monkeypatch, capsys):
    _inject_source_tree(monkeypatch, release["source_tree"])
    rc = vr.main(["--tag", "v9.9.9", "--asset-dir", str(release["assets"]), "--no-download", "--json"])
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] is True and out["schemaVersion"] == "csg.verify_release.v1"


def test_remote_commit_mismatch_fires_check(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    report = vr.verify_release(
        "v9.9.9", asset_dir=str(release["assets"]), download=False,
        source_tree=str(release["source_tree"]), expected_commit=COMMIT,
        remote_commit="9" * 40, seeds=30,
    )
    assert any(c["name"] == "identity:remote_commit" and not c["ok"] for c in report["checks"])


# ---------------------------------------------------------------------------
# F8 — identity / root of trust pinning
# ---------------------------------------------------------------------------


def test_repo_override_requires_allow_flag(release, monkeypatch):
    _inject_source_tree(monkeypatch, release["source_tree"])
    report = vr.verify_release(
        "v9.9.9", repo="attacker/wide-robot", asset_dir=str(release["assets"]),
        download=False, source_tree=str(release["source_tree"]), expected_commit=COMMIT, seeds=30,
    )
    assert any(c["name"] == "identity:repo" and not c["ok"] for c in report["checks"])
    report_ok = vr.verify_release(
        "v9.9.9", repo="attacker/wide-robot", allow_untrusted_repo=True,
        asset_dir=str(release["assets"]), download=False,
        source_tree=str(release["source_tree"]), expected_commit=COMMIT, seeds=30,
    )
    assert any(c["name"] == "identity:repo" and c["ok"] for c in report_ok["checks"])


def test_known_tag_commit_pin_catches_forged_tag(release, monkeypatch):
    # A forged local tag for a *known* release must be caught: the expected
    # commit comes from the pinned map, and the local resolution is checked.
    _inject_source_tree(monkeypatch, release["source_tree"])
    monkeypatch.setitem(vr.KNOWN_TAG_COMMITS, "v9.9.9", COMMIT)
    monkeypatch.setattr(vr, "resolve_tag_commit", lambda tag, **kw: "f" * 40)  # forged local tag
    report = vr.verify_release(
        "v9.9.9", asset_dir=str(release["assets"]), download=False,
        source_tree=str(release["source_tree"]), seeds=30,
    )
    assert report["expectedCommit"] == COMMIT  # uses the pin, not the forged tag
    assert any(c["name"] == "identity:tag_commit" and not c["ok"] for c in report["checks"])


# ---------------------------------------------------------------------------
# Real published release (local maintainer machine only — assets are gitignored)
# ---------------------------------------------------------------------------

_REAL_ASSETS = Path(__file__).resolve().parents[1] / "release_v0.3.1_out" / "assets"


@pytest.mark.skipif(not _REAL_ASSETS.is_dir(), reason="published v0.3.1 assets not present (gitignored)")
def test_real_v0_3_1_release_verifies_against_pinned_commit():
    repo_root = Path(__file__).resolve().parents[1]
    if not (repo_root / ".git").is_dir():
        pytest.skip("not a git checkout")
    have = subprocess.run(["git", "cat-file", "-e", f"{vr.KNOWN_TAG_COMMITS['v0.3.1']}^{{commit}}"],
                          cwd=repo_root, capture_output=True)
    if have.returncode != 0:
        pytest.skip("pinned v0.3.1 commit not in this clone")
    report = vr.verify_release("v0.3.1", asset_dir=str(_REAL_ASSETS), download=False,
                               seeds=30, cwd=str(repo_root))
    assert report["ok"] is True, [c for c in report["checks"] if not c["ok"]]
    assert report["expectedCommit"] == vr.KNOWN_TAG_COMMITS["v0.3.1"]
