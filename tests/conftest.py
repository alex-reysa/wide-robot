"""Shared pytest fixtures and helpers for the CSG test suite."""
import copy
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

GOLD = REPO / "gold_tests"

from csg.common import load_json  # noqa: E402


def rename(graph, mapping):
    """Crude whole-graph id rename for building robot variants in tests."""
    s = json.dumps(graph)
    for k, v in mapping.items():
        s = s.replace(k, v)
    return json.loads(s)


def to_robot(target):
    """Rename a human target into a robot-id, gripper-effector graph and strip
    the TaskSpec, as an honest rollout extractor would (no plannerView)."""
    r = rename(target, {"h_cube": "r_cube", "h_tray": "r_tray", "h_drawer": "r_drawer",
                        "right_hand": "robot_gripper", "RIGHT_HAND": "ROBOT_GRIPPER"})
    r.pop("plannerView", None)
    r.pop("temporalEdges", None)
    return r


@pytest.fixture
def cube_target():
    return load_json(GOLD / "put_cube_in_tray" / "target.json")


@pytest.fixture
def cube_robot_success():
    return load_json(GOLD / "put_cube_in_tray" / "robot_success.json")


@pytest.fixture
def drawer_target():
    return load_json(GOLD / "open_drawer" / "target.json")
