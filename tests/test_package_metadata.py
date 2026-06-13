import tomllib


def _pyproject():
    return tomllib.loads(open("pyproject.toml", "rb").read().decode("utf-8"))


def test_pyproject_exposes_command_line_entry_points():
    metadata = _pyproject()

    assert metadata["project"]["scripts"] == {
        "csg-benchmark": "csg.benchmark:main",
        "csg-matcher": "csg.matcher:main",
        "csg-release-audit": "csg.release_audit:main",
        "csg-release-manifest": "csg.release_manifest:main",
        "csg-release-rehearsal": "csg.release_rehearsal:main",
        "csg-rollout-extract": "csg.rollout_extract:main",
        "csg-skills": "csg.skills:main",
        "csg-solver": "csg.solver:main",
        "csg-to-sim": "csg.to_sim:main",
        "csg-verify-release": "csg.verify_release:main",
    }


def test_pyproject_declares_public_package_metadata_with_mit_license():
    project = _pyproject()["project"]

    assert project["readme"] == "README.md"
    assert project["license"] == "MIT"
    assert project["license-files"] == ["LICENSE"]
    assert "robotics" in project["keywords"]
    assert "csg" in project["keywords"]
    assert "Programming Language :: Python :: 3" in project["classifiers"]
    assert "Operating System :: OS Independent" in project["classifiers"]
