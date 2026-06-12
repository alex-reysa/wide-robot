"""Backend dispatch contract for the MuJoCo backend (roadmap Phase 2C).

These run WITHOUT mujoco installed: they pin the optional-import behaviour so
the frozen verifier suite never depends on the physics extra. The heavy
end-to-end checks live in ``test_mujoco_backend.py`` (gated by importorskip).
"""
import pytest

from csg.backends import mujoco as mjbackend
from csg.solver import solve, SolverConfig
from csg.common import load_json
from csg.predicates import DEFAULT as PRED, box_from, box_gap
from csg.skills import choose_primary_program, generate_skill_skeletons
from csg.to_sim import compile_scene
from csg.backends.mujoco.scene_mjcf import build_arm_scene_xml, deconflict_layout
from conftest import GOLD

HAS_MUJOCO = mjbackend.mujoco_available()


def test_backend_package_imports_without_mujoco():
    # Importing the backend package must never import mujoco itself.
    assert hasattr(mjbackend, "run_skill")
    assert isinstance(mjbackend.MUJOCO_AVAILABLE, bool)


def test_validity_module_is_stdlib_only():
    # The validity checks must be importable with no physics dependency.
    from csg.backends.mujoco import validity, trace  # noqa: F401
    assert hasattr(validity, "check_validity")
    assert hasattr(trace, "SimTrace")


@pytest.mark.skipif(HAS_MUJOCO, reason="mujoco installed: error path not exercised")
def test_mujoco_backend_errors_clearly_when_absent():
    target = load_json(GOLD / "put_cube_in_tray" / "target.json")
    with pytest.raises(RuntimeError) as exc:
        solve(target, SolverConfig(backend="mujoco"))
    assert "mujoco" in str(exc.value).lower()


def test_symbolic_backend_unchanged_validity_none():
    target = load_json(GOLD / "put_cube_in_tray" / "target.json")
    run = solve(target)
    assert run.rollout["diagnostics"]["physicalValidity"] is None
    assert run.validity_report is None


def _scene_program(task):
    scene = compile_scene(load_json(GOLD / task / "target.json"), backend="mujoco")
    prog = choose_primary_program(generate_skill_skeletons(scene))
    return scene, prog


def _layout_gap(scene, layout, a, b):
    bodies = {body["objectId"]: body for body in scene["bodies"]}
    return box_gap(box_from(layout[a], bodies[a]["sizeM"]), box_from(layout[b], bodies[b]["sizeM"]))


def test_pick_family_layout_uses_skill_specific_initial_gap_without_mujoco():
    place_scene, place_prog = _scene_program("place_on_top")
    _, place_layout = build_arm_scene_xml(place_scene, program=place_prog)
    place_gap = _layout_gap(place_scene, place_layout, place_prog["manipulatedObjectId"], place_prog["targetObjectId"])
    assert place_gap > PRED.near_gap_m

    put_scene, put_prog = _scene_program("put_cube_in_tray")
    _, put_layout = build_arm_scene_xml(put_scene, program=put_prog)
    put_gap = _layout_gap(put_scene, put_layout, put_prog["manipulatedObjectId"], put_prog["targetObjectId"])
    assert put_gap <= PRED.near_gap_m


def test_deconflict_layout_prefers_program_target_and_centers_manipulated_mover():
    bodies = [
        {"objectId": "static_a", "mobility": "STATIC", "sizeM": [0.08, 0.08, 0.04]},
        {"objectId": "target_static", "mobility": "STATIC", "sizeM": [0.10, 0.10, 0.04]},
        {"objectId": "other_mover", "mobility": "MOVABLE", "sizeM": [0.03, 0.03, 0.03]},
        {"objectId": "manipulated", "mobility": "MOVABLE", "sizeM": [0.04, 0.04, 0.04]},
    ]
    layout = deconflict_layout(bodies, program={
        "skillType": "pick_place",
        "manipulatedObjectId": "manipulated",
        "targetObjectId": "target_static",
    })

    assert layout["target_static"][:2] == (0.46, 0.0)
    assert layout["static_a"][1] != 0.0
    assert layout["manipulated"][1] == 0.0
    assert layout["other_mover"][1] != 0.0
