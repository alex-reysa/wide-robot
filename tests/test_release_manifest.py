import json

from csg import release_manifest as rm
from csg import verify_release as vr


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _reports(root, *, seeds=30):
    _write(root / "symbolic" / "report.json",
           {"summary": {"total": 5, "passed": 5, "failed": 0,
                        "physicalValidity": {"unverified": 5}, "leakage": {"clean": 5, "dirty": 0}}})
    _write(root / "mujoco" / "report.json",
           {"summary": {"total": 5, "passed": 5, "failed": 0,
                        "physicalValidity": {"valid": 5}, "leakage": {"clean": 5, "dirty": 0}}})
    n = 5 * seeds
    _write(root / f"mujoco_randomized_{seeds}" / "report.json",
           {"summary": {"total": n, "passed": n, "failed": 0,
                        "physicalValidity": {"valid": n}, "leakage": {"clean": n, "dirty": 0}}})
    _write(root / "comparison" / "comparison_report.json",
           {"baselineOrder": ["symbolic", "noop", "mujoco"],
            "baselines": {"noop": {"expectedFailure": True, "summary": {"passed": 0, "failed": 5}}}})
    _write(root / "invalid_fixtures" / "invalid_fixtures_report.json",
           {"summary": {"total": 9, "matched": 9, "mismatched": 0}})


def test_expected_benchmark_summaries_reads_from_reports(tmp_path):
    _reports(tmp_path)
    summaries = rm.expected_benchmark_summaries(tmp_path, seeds=30)
    assert summaries["symbolic"]["physicalValidity"] == {"unverified": 5}
    assert summaries["mujoco"]["physicalValidity"] == {"valid": 5}
    assert summaries["randomized"]["total"] == 150
    assert summaries["randomized"]["leakage"] == {"clean": 150, "dirty": 0}
    assert summaries["invalid"] == {"total": 9, "matched": 9, "mismatched": 0}
    assert summaries["comparison"]["baselineOrder"] == ["symbolic", "noop", "mujoco"]


def test_expected_benchmark_summaries_honours_seeds(tmp_path):
    _reports(tmp_path, seeds=2)
    summaries = rm.expected_benchmark_summaries(tmp_path, seeds=2)
    assert summaries["seeds"] == 2
    assert summaries["randomized"]["total"] == 10


def test_write_sha256sums_format_and_exclusions(tmp_path):
    (tmp_path / "one.tar.gz").write_bytes(b"a")
    (tmp_path / "two.whl").write_bytes(b"bb")
    (tmp_path / "release_manifest.json").write_text("{}", encoding="utf-8")
    out = rm.write_sha256sums(tmp_path)
    text = out.read_text(encoding="utf-8")
    assert "RELEASE_SHA256SUMS" not in text  # never lists itself
    assert "release_manifest.json" not in text  # nor the manifest
    for line in text.splitlines():
        digest, name = line.split("  ", 1)  # exactly two spaces
        assert len(digest) == 64 and name
    assert set(vr.parse_sha256sums(text)) == {"one.tar.gz", "two.whl"}


def test_build_manifest_pins_assets_and_summaries(tmp_path):
    reports = tmp_path / "reports"
    _reports(reports)
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "csg-1.2.3.tar.gz").write_bytes(b"sdist-bytes")

    manifest = rm.build_manifest(
        tag="v1.2.3", commit="a" * 40, asset_dir=assets, reports_root=reports, version="1.2.3", seeds=30,
    )
    assert manifest["schemaVersion"] == "csg.release_manifest.v1"
    assert (manifest["tag"], manifest["commit"], manifest["version"]) == ("v1.2.3", "a" * 40, "1.2.3")
    assert manifest["checksumsFile"] == "RELEASE_SHA256SUMS"
    assert manifest["expectedBenchmarkSummaries"]["randomized"]["total"] == 150

    asset = next(a for a in manifest["assets"] if a["name"] == "csg-1.2.3.tar.gz")
    assert asset["sha256"] == rm.sha256_file(assets / "csg-1.2.3.tar.gz")
    assert asset["bytes"] == len(b"sdist-bytes")


def test_exact_commands_are_canonical():
    cmds = rm.exact_commands("v0.3.1", seeds=30)
    assert cmds["verify_release"] == "python3 -m csg.verify_release --tag v0.3.1"
    assert "mujoco_randomized_30" in cmds["mujoco_randomized"]
    assert "--require-pass" in cmds["symbolic_gold"]
    assert cmds["clean_clone_rehearsal"] == "bash scripts/clean_clone_rehearsal.sh v0.3.1"


def test_pyproject_version_is_readable():
    version = rm._pyproject_version()
    assert isinstance(version, str) and version
