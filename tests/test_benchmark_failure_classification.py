import json

from csg.benchmark import classify_failure, run_benchmark
from conftest import GOLD


def test_classify_failure_uses_existing_evidence_precedence():
    assert classify_failure({"status": "PASS", "passed": True})["category"] == "passed"
    assert classify_failure({"status": "FAIL", "passed": True})["category"] == "verifier_mismatch"

    assert classify_failure({
        "status": "FAIL",
        "passed": False,
        "leakageClean": False,
        "physicalValidity": False,
        "hardMismatches": ["goal_satisfaction"],
    })["category"] == "target_leakage_detected"

    assert classify_failure({
        "status": "FAIL",
        "passed": False,
        "leakageClean": True,
        "physicalValidity": False,
        "physicalValidityReason": "penetration",
        "hardMismatches": ["goal_satisfaction"],
    }) == {
        "category": "physical_invalidity",
        "evidence": {
            "physicalValidity": False,
            "physicalValidityReason": "penetration",
        },
    }

    assert classify_failure({"status": "ERROR", "passed": False, "error": "boom"})["category"] == "solver_error"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["contact_word"]})["category"] == "contact_missing"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["event_order"]})["category"] == "event_order_wrong"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["goal_satisfaction"]})["category"] == "relation_not_achieved"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["relation_transitions"]})["category"] == "relation_not_achieved"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["articulation_transitions"]})["category"] == "relation_not_achieved"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["initial_state"]})["category"] == "verifier_mismatch"
    assert classify_failure({"status": "FAIL", "hardMismatches": ["terminal_state"]})["category"] == "verifier_mismatch"
    assert classify_failure({
        "status": "FAIL",
        "matcherPassed": False,
        "hardMismatches": [],
        "probeAgreement": {"contact_word": False},
    })["category"] == "verifier_mismatch"
    assert classify_failure({"status": "FAIL", "matcherPassed": False, "hardMismatches": []})["category"] == "verifier_mismatch"


def test_run_benchmark_writes_failure_classification_sidecar(tmp_path):
    targets = [
        GOLD / "put_cube_in_tray" / "target.json",
        GOLD / "open_drawer" / "target.json",
    ]

    report = run_benchmark(targets, tmp_path)

    assert report["failureClassificationSummary"] == {"passed": 2}
    assert report["summary"]["failureClassification"] == {"passed": 2}
    assert report["summary"]["physicalValidity"] == {"unverified": 2}
    assert report["summary"]["leakage"] == {"clean": 2, "dirty": 0}
    for case in report["cases"]:
        assert case["failureClassification"]["category"] == "passed"

    sidecar = json.loads((tmp_path / "failure_classification.json").read_text(encoding="utf-8"))
    assert sidecar["sourceProvenance"]["schemaVersion"] == "csg.source_provenance.v1"
    assert sidecar["summary"] == {"passed": 2}
    assert sidecar["physicalValiditySummary"] == {"unverified": 2}
    assert sidecar["leakageSummary"] == {"clean": 2, "dirty": 0}
    assert [c["case"] for c in sidecar["cases"]] == ["put_cube_in_tray", "open_drawer"]
    assert all(c["failureClassification"]["category"] == "passed" for c in sidecar["cases"])


def test_run_benchmark_records_source_provenance(tmp_path):
    report = run_benchmark([GOLD / "put_cube_in_tray" / "target.json"], tmp_path)

    provenance = report["sourceProvenance"]
    assert provenance["schemaVersion"] == "csg.source_provenance.v1"
    assert provenance["snapshot"]["algorithm"] == "sha256"
    assert len(provenance["snapshot"]["digest"]) == 64
    assert provenance["snapshot"]["fileCount"] > 0
    paths = {entry["path"] for entry in provenance["snapshot"]["files"]}
    assert "csg/benchmark.py" in paths
    assert "pyproject.toml" in paths

    sidecar = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert sidecar["sourceProvenance"] == provenance
