"""Physical-validity reporting contract + goal-pose admissibility (audit A8/A9b).

The symbolic backend cannot check physics, so it must report
``physicalValidity: None`` (never true), the benchmark must surface it, and —
critically — the geometry the loop compiles must make the goal *physically
reachable*: a container compiled as a solid box made INSIDE satisfiable only
by interpenetration, so the future MuJoCo backend (roadmap Phase 2C) could
never honestly PASS. These tests are the gate Phase 2C builds on.
"""
import math

import pytest

from csg import predicates as P
from csg.common import load_json, pose_xyz, get_any
from csg.solver import solve
from csg.to_sim import compile_scene, scene_to_mujoco_xml
from csg.benchmark import run_benchmark
from conftest import GOLD

PENETRATION_TOL_M = 0.005
TASKS = ["put_cube_in_tray", "open_drawer"]


def _collision_boxes(body, center):
    """World-frame collision boxes for a (sanitized) scene body — a single box,
    or floor + 4 walls for containers. Mirrors csg.to_sim._body_geoms."""
    sx, sy, sz = [float(v) for v in body["sizeM"]]
    cav = body.get("containerCavity")
    if not isinstance(cav, dict):
        return [P.box_from(center, (sx, sy, sz))]
    wall = min(float(cav["wallThicknessM"]), sx / 4, sy / 4)
    floor = min(float(cav["floorThicknessM"]), sz / 2)
    cx, cy, cz = center
    wall_h = sz - floor
    wz = cz + floor / 2
    return [
        P.box_from((cx, cy, cz - (sz - floor) / 2), (sx, sy, floor)),
        P.box_from((cx - (sx - wall) / 2, cy, wz), (wall, sy, wall_h)),
        P.box_from((cx + (sx - wall) / 2, cy, wz), (wall, sy, wall_h)),
        P.box_from((cx, cy - (sy - wall) / 2, wz), (sx - 2 * wall, wall, wall_h)),
        P.box_from((cx, cy + (sy - wall) / 2, wz), (sx - 2 * wall, wall, wall_h)),
    ]


@pytest.mark.parametrize("task", TASKS)
def test_symbolic_rollout_reports_validity_none(task):
    run = solve(load_json(GOLD / task / "target.json"))
    diag = run.rollout["diagnostics"]
    assert "physicalValidity" in diag
    assert diag["physicalValidity"] is None  # never silently true


def test_benchmark_surfaces_validity_and_support(tmp_path):
    targets = [GOLD / t / "target.json" for t in TASKS]
    report = run_benchmark(targets, tmp_path)
    assert report["summary"]["failed"] == 0
    for case in report["cases"]:
        assert case["physicalValidity"] is None
        assert case["vacuous"] is False
        assert any(v > 0 for v in case["probeSupport"].values())
    md = (tmp_path / "report.md").read_text()
    assert "physics-unverified" in md


@pytest.mark.parametrize("task", TASKS)
def test_goal_pose_admissible(task):
    """At the rollout's final frame, no object penetrates another body's
    collision geometry by more than PENETRATION_TOL_M. This is the test that
    would have caught audit A8 (INSIDE goal 3 cm inside a solid tray box)."""
    run = solve(load_json(GOLD / task / "target.json"))
    rollout = run.rollout
    assert rollout["frames"], "solver produced no frames"
    bodies = {b["objectId"]: b for b in rollout["sceneBodies"]}
    last = rollout["frames"][-1]["objectPoses"]
    ids = [oid for oid in bodies if oid in last]
    for i, a_id in enumerate(ids):
        for b_id in ids[i + 1:]:
            for abox in _collision_boxes(bodies[a_id], pose_xyz(last[a_id])):
                for bbox in _collision_boxes(bodies[b_id], pose_xyz(last[b_id])):
                    gap = P.box_gap(abox, bbox)
                    assert gap >= -PENETRATION_TOL_M, (
                        f"{a_id} penetrates {b_id} by {-gap:.4f} m at final pose")


def test_inside_goal_reaches_true_containment():
    """The cavity must not break the INSIDE predicate: at the final frame the
    cube is inside the tray's *outer* box per the shared grammar."""
    run = solve(load_json(GOLD / "put_cube_in_tray" / "target.json"))
    rollout = run.rollout
    bodies = {b["objectId"]: b for b in rollout["sceneBodies"]}
    cube_id = next(o for o, b in bodies.items() if b.get("physicalKind") == "RIGID_OBJECT")
    tray_id = next(o for o, b in bodies.items() if b.get("isContainer"))
    last = rollout["frames"][-1]["objectPoses"]
    cube = P.box_from(pose_xyz(last[cube_id]), tuple(float(v) for v in bodies[cube_id]["sizeM"]))
    tray = P.box_from(pose_xyz(last[tray_id]), tuple(float(v) for v in bodies[tray_id]["sizeM"]))
    assert P.is_inside(cube, tray)


def test_container_compiles_to_open_cavity():
    scene = compile_scene(load_json(GOLD / "put_cube_in_tray" / "target.json"))
    containers = [b for b in scene["bodies"] if b.get("isContainer")]
    assert len(containers) == 1
    assert "containerCavity" in containers[0]
    xml = scene_to_mujoco_xml(scene)
    body_id = containers[0]["bodyId"]
    section = xml.split(f"name='{body_id}'", 1)[1].split("</body>", 1)[0]
    assert section.count("<geom") == 5, "container must be floor + 4 walls, not a solid box"
