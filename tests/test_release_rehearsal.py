from pathlib import Path

from csg.release_rehearsal import build_rehearsal_plan


def test_release_rehearsal_plan_matches_phase_2e_checklist(tmp_path):
    plan = build_rehearsal_plan(
        out_root=tmp_path / "phase2e",
        python="python3",
        sim_python=".venv-sim/bin/python",
        seeds=30,
        require_final_metadata=True,
    )

    assert [step["name"] for step in plan["steps"]] == [
        "core_tests",
        "symbolic_gold",
        "mujoco_tests",
        "mujoco_gold",
        "mujoco_randomized",
        "backend_comparison",
        "invalid_fixtures",
        "release_audit",
    ]
    by_name = {step["name"]: step["argv"] for step in plan["steps"]}
    assert by_name["core_tests"] == ["python3", "-m", "pytest", "tests/", "-q"]
    assert by_name["symbolic_gold"][-2:] == ["--out", str(tmp_path / "phase2e" / "symbolic")]
    assert by_name["mujoco_gold"][0] == ".venv-sim/bin/python"
    assert by_name["mujoco_randomized"] == [
        ".venv-sim/bin/python",
        "-m",
        "csg.benchmark",
        "gold_tests",
        "--backend",
        "mujoco",
        "--confusion",
        "--randomized",
        "--seeds",
        "30",
        "--require-pass",
        "--out",
        str(tmp_path / "phase2e" / "mujoco_randomized_30"),
    ]
    assert by_name["backend_comparison"] == [
        ".venv-sim/bin/python",
        "-m",
        "csg.benchmark",
        "gold_tests",
        "--compare-backends",
        "symbolic,noop,mujoco",
        "--confusion",
        "--require-pass",
        "--out",
        str(tmp_path / "phase2e" / "comparison"),
    ]
    assert "--require-final-metadata" in by_name["release_audit"]


def test_release_rehearsal_plan_defaults_to_non_strict_audit(tmp_path):
    plan = build_rehearsal_plan(out_root=tmp_path, sim_python=Path(".venv-sim/bin/python"))

    audit_argv = plan["steps"][-1]["argv"]
    assert audit_argv[0] == "python3"
    assert "--require-final-metadata" not in audit_argv
    assert "--randomized" in audit_argv
    assert str(tmp_path / "mujoco_randomized_30") in audit_argv
