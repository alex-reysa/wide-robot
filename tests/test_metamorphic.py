"""Metamorphic invariants: transforms that MUST preserve PASS / distance 0.

Each case directly targets an audit finding that previously produced a nonzero
distance for an honest, semantically-identical robot CSG.
"""
import copy
import json

from csg.matcher import match
from conftest import to_robot


def _rescale_time(graph, k):
    g = copy.deepcopy(graph)

    def walk(o):
        if isinstance(o, dict):
            for key, v in o.items():
                if key in ("startTimeNs", "endTimeNs", "timeNs") and isinstance(v, (str, int)):
                    o[key] = str(int(int(v) * k))
                else:
                    walk(v)
        elif isinstance(o, list):
            for x in o:
                walk(x)
    walk(g)
    return g


def test_identity_passes(cube_target):
    assert match(cube_target, cube_target).passed


def test_rename_copy_passes(cube_target):
    assert match(cube_target, to_robot(cube_target)).passed


def test_no_plannerview_passes(cube_target):
    """Honest rollout has no TaskSpec — must still PASS (was 0.208 before)."""
    robot = to_robot(cube_target)
    assert "plannerView" not in robot
    assert match(cube_target, robot).passed


def test_confidence_jitter_passes(cube_target):
    """Confidence is a mask, not a weight (was 0.056 before)."""
    robot = json.loads(json.dumps(to_robot(cube_target)))

    def jitter(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == "confidence" and isinstance(v, (int, float)):
                    o[k] = min(1.0, max(0.51, v + 0.04))
                else:
                    jitter(v)
        elif isinstance(o, list):
            for x in o:
                jitter(x)
    jitter(robot)
    assert match(cube_target, robot).passed


def test_time_rescale_passes(cube_target):
    assert match(cube_target, _rescale_time(to_robot(cube_target), 3)).passed


def test_equal_timestamp_permutation_deterministic(cube_target):
    """Equal-timestamp events in different array order must score identically."""
    robot = to_robot(cube_target)
    robot["events"][0]["timeSpan"] = robot["events"][1]["timeSpan"]
    perm = copy.deepcopy(robot)
    perm["events"][0], perm["events"][1] = perm["events"][1], perm["events"][0]
    assert match(cube_target, robot).distance == match(cube_target, perm).distance


def test_converse_relation_phrasing_passes(cube_target):
    """CONTAINS(tray,cube) is INSIDE(cube,tray) — must PASS (was 0.317 before)."""
    robot = to_robot(cube_target)
    rt = robot["events"][2]["observedDeltas"][0]["relationTransition"]
    rt["subjectObjectId"], rt["objectObjectId"] = "r_tray", "r_cube"
    rt["fromRelation"], rt["toRelation"] = "NEAR", "CONTAINS"
    assert match(cube_target, robot).passed


def test_symmetry(cube_target):
    robot = to_robot(cube_target)
    assert match(cube_target, robot).distance == match(robot, cube_target).distance


def test_articulated_copy_passes(drawer_target):
    """Regression for the TOPO_ART mapping bug (was 0.125 before)."""
    assert match(drawer_target, to_robot(drawer_target)).passed
