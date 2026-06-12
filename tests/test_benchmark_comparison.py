import json

from csg.benchmark import run_benchmark_comparison
from csg.solver import SolverConfig
from conftest import GOLD


def test_benchmark_comparison_groups_baselines_and_writes_report(tmp_path):
    targets = [
        GOLD / "put_cube_in_tray" / "target.json",
        GOLD / "open_drawer" / "target.json",
    ]
    report = run_benchmark_comparison(
        targets,
        tmp_path,
        {
            "symbolic": SolverConfig(backend="symbolic"),
            "symbolic_repeat": SolverConfig(backend="symbolic"),
        },
        confusion=True,
    )

    assert report["schemaVersion"] == "csg.benchmark_comparison.v1"
    assert report["baselineOrder"] == ["symbolic", "symbolic_repeat"]
    assert set(report["baselines"]) == {"symbolic", "symbolic_repeat"}
    for name, baseline in report["baselines"].items():
        assert baseline["summary"]["passed"] == 2, name
        assert baseline["summary"]["failed"] == 0, name
        assert baseline["summary"]["physicalValidity"] == {"unverified": 2}
        assert baseline["summary"]["failureClassification"] == {"passed": 2}
        assert baseline["confusion"]["unexpectedOffDiagonalPasses"] == []

    sidecar = json.loads((tmp_path / "comparison_report.json").read_text(encoding="utf-8"))
    assert sidecar["baselineOrder"] == ["symbolic", "symbolic_repeat"]
    assert sidecar["sourceProvenance"]["schemaVersion"] == "csg.source_provenance.v1"
    assert sidecar["sourceProvenance"]["snapshot"]["algorithm"] == "sha256"
    assert len(sidecar["sourceProvenance"]["snapshot"]["digest"]) == 64
    assert (tmp_path / "symbolic" / "report.json").is_file()
    assert (tmp_path / "symbolic_repeat" / "report.json").is_file()


def test_benchmark_comparison_includes_deliberately_dumb_noop_baseline(tmp_path):
    targets = [
        GOLD / "put_cube_in_tray" / "target.json",
        GOLD / "push_object" / "target.json",
    ]
    report = run_benchmark_comparison(
        targets,
        tmp_path,
        {
            "scripted_symbolic": SolverConfig(backend="symbolic"),
            "noop": SolverConfig(backend="noop"),
        },
        confusion=False,
    )

    assert report["baselineOrder"] == ["scripted_symbolic", "noop"]
    scripted = report["baselines"]["scripted_symbolic"]
    noop = report["baselines"]["noop"]

    assert scripted["summary"]["failureClassification"] == {"passed": 2}
    assert noop["backend"] == "noop"
    assert noop["expectedFailure"] is True
    assert noop["summary"]["passed"] == 0
    assert noop["summary"]["failed"] == 2
    assert noop["summary"]["failureClassification"]
    assert set(noop["summary"]["failureClassification"]) != {"passed"}
    assert all(case["status"] == "FAIL" for case in noop["cases"])
    assert all(case["failureClassification"]["category"] != "passed" for case in noop["cases"])
