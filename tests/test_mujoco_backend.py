"""End-to-end MuJoCo backend tests (roadmap Phase 2C), gated on mujoco.

These actually run the physics: build the arm scene, simulate scripted tasks,
extract a robot CSG from the frames, and check it PASSes the frozen verifier
with a *real* physicalValidity verdict — and that a sabotaged rollout does not.
Skipped automatically when mujoco is not installed, so the core suite stays
dependency-free.
"""
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip("mujoco")

from csg.common import load_json, get_any, pose_xyz
from csg import predicates as P
from csg.to_sim import compile_scene, sanitize_bodies_for_rollout
from csg.skills import generate_skill_skeletons, choose_primary_program
from csg.solver import solve, SolverConfig
from csg.rollout_extract import extract_robot_csg
from csg.matcher import match
from csg.benchmark import run_benchmark, run_benchmark_comparison, leakage_report
from csg.backends.mujoco.runner import run_skill, _Runner
from csg.backends.mujoco.scene_mjcf import build_arm_scene_xml
from csg.backends.mujoco.arm import arm_mjcf
from conftest import GOLD

TARGET = GOLD / "put_cube_in_tray" / "target.json"
PUSH_TARGET = GOLD / "push_object" / "target.json"
OPEN_TARGET = GOLD / "open_drawer" / "target.json"
ALL_GOLD_TARGETS = tuple(sorted(GOLD.glob("*/target.json")))
PICK_FAMILY_TARGETS = {
    "put_cube_in_tray": TARGET,
    "insert_object": GOLD / "insert_object" / "target.json",
    "place_on_top": GOLD / "place_on_top" / "target.json",
}
PICK_FAMILY_RELATIONS = {
    "put_cube_in_tray": ("NEAR", "INSIDE"),
    "insert_object": ("NEAR", "INSIDE"),
    "place_on_top": ("FAR_FROM", "ON_TOP_OF"),
}


def _scene_program(target):
    scene = compile_scene(target, backend="mujoco")
    prog = choose_primary_program(generate_skill_skeletons(scene))
    return scene, prog


def _rollout_from(scene, prog, res):
    return {
        "schemaVersion": "csg.rollout.v0", "backend": "mujoco",
        "robotEffectorId": "robot_gripper",
        "objectIdMap": dict(get_any(scene, "objectIdMap", default={}) or {}),
        "sceneBodies": sanitize_bodies_for_rollout(scene["bodies"]),
        "skillProgram": prog, "frames": res.frames, "success": True,
        "failures": res.failures, "diagnostics": {},
    }


def _max_touch_gap_for_contact(rollout, contact):
    bodies = {b["objectId"]: b for b in rollout["sceneBodies"]}
    object_id = contact["b"]["id"]
    size = bodies[object_id]["sizeM"]
    start_ns = int(contact["timeSpan"]["startTimeNs"])
    end_ns = int(contact["timeSpan"]["endTimeNs"])
    gaps = []
    for frame in rollout["frames"]:
        t_ns = int(frame["timeNs"])
        if not (start_ns <= t_ns <= end_ns):
            continue
        obj_pose = frame["objectPoses"].get(object_id)
        if obj_pose is None:
            continue
        eff = pose_xyz(frame["effectorPose"])
        obj = pose_xyz(obj_pose)
        gaps.append(P.point_to_box_distance(eff, P.box_from(obj, size)))
    return max(gaps) if gaps else float("inf")


def _contact_frame_indices(rollout, contact):
    start_ns = int(contact["timeSpan"]["startTimeNs"])
    end_ns = int(contact["timeSpan"]["endTimeNs"])
    return [
        i for i, frame in enumerate(rollout["frames"])
        if start_ns <= int(frame["timeNs"]) <= end_ns
    ]


# ---- scene structure --------------------------------------------------------


def test_arm_scene_loads_and_tray_has_open_cavity():
    scene = compile_scene(load_json(TARGET), backend="mujoco")
    xml, layout = build_arm_scene_xml(scene)
    model = mujoco.MjModel.from_xml_string(xml)  # must compile
    tray_id = next(b["objectId"] for b in scene["bodies"] if b.get("isContainer"))
    bid = model.body(tray_id).id
    ngeom = sum(1 for g in range(model.ngeom) if model.geom_bodyid[g] == bid)
    assert ngeom == 5, "container must compile to floor + 4 walls (audit A8)"
    # the arm and its actuators are present
    assert model.site("grasp_site").id >= 0
    assert model.nu == 8  # 6 arm joints + 2 finger actuators


def test_no_spawn_penetration_after_preroll():
    scene = compile_scene(load_json(TARGET), backend="mujoco")
    xml, _ = build_arm_scene_xml(scene)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    _, _, _, spec = arm_mjcf()
    for j, v in spec.home.items():
        data.qpos[model.jnt_qposadr[model.joint(j).id]] = v
    for j in spec.joint_names:
        data.ctrl[model.actuator("act_" + j).id] = spec.home[j]
    mujoco.mj_forward(model, data)
    for _ in range(250):
        mujoco.mj_step(model, data)
    worst = max([0.0] + [-float(data.contact[i].dist) for i in range(data.ncon)])
    assert worst <= 0.005, f"spawn penetration {worst*1000:.1f}mm after settle"


# ---- end-to-end honest loop -------------------------------------------------


@pytest.mark.parametrize("task_name,target_path", PICK_FAMILY_TARGETS.items())
def test_pick_family_end_to_end_passes_with_real_validity(task_name, target_path):
    target = load_json(target_path)
    run = solve(target, SolverConfig(backend="mujoco"))
    diag = run.rollout["diagnostics"]
    assert diag["physicalValidity"] is True, diag["physicalValidityReason"]
    assert run.rollout["frames"], "no frames produced"

    robot = extract_robot_csg(run.rollout)
    relations = [r["relation"] for r in robot["relations"]]
    assert relations == list(PICK_FAMILY_RELATIONS[task_name]), task_name
    assert [c["mode"] for c in robot["contacts"]] == ["GRASP_LIKELY"], f"{task_name}: expected a single grasp contact"
    cm = [c["contactEvidence"]["motionCorrelation"] for c in robot["contacts"]]
    assert all(x >= 0.6 for x in cm), f"{task_name}: co-motion below threshold: {cm}"

    result = match(target, robot)
    assert result.passed, f"{task_name}: hard mismatches: {result.hard_probes}"
    assert all(result.probe_agreement[p] for p in result.hard_probes)
    assert leakage_report(robot)["clean"], task_name


def test_push_object_end_to_end_passes_with_touching_contact():
    target = load_json(PUSH_TARGET)
    run = solve(target, SolverConfig(backend="mujoco"))
    diag = run.rollout["diagnostics"]
    assert diag["physicalValidity"] is True, diag["physicalValidityReason"]
    assert run.rollout["frames"], "no frames produced"
    assert {f["gripperClosed"] for f in run.rollout["frames"]} == {False}

    robot = extract_robot_csg(run.rollout)
    assert [r["relation"] for r in robot["relations"]] == ["FAR_FROM", "NEAR"]
    assert [c["mode"] for c in robot["contacts"]] == ["TOUCHING_LIKELY"]
    assert "GRASP_LIKELY" not in {c["mode"] for c in robot["contacts"]}
    assert "RELEASE_INFERRED" not in {e["eventKind"] for e in robot["events"]}
    assert _max_touch_gap_for_contact(run.rollout, robot["contacts"][0]) <= P.DEFAULT.touching_gap_m

    result = match(target, robot)
    assert result.passed, f"hard mismatches: {result.hard_probes}"
    assert all(result.probe_agreement[p] for p in result.hard_probes)
    assert leakage_report(robot)["clean"]

    checks = run.validity_report["checks"]
    for name in ("non_penetration", "pose_continuity", "workspace_reachability"):
        assert checks[name]["applicable"] is True, name
        assert checks[name]["passed"] is True, checks[name]["detail"]
    for name in ("gripper_feasibility", "quasi_static_support_at_release"):
        assert checks[name]["applicable"] is False, name
        assert checks[name]["passed"] is True, checks[name]["detail"]


def test_push_object_touching_contact_uses_open_aperture_wider_than_puck():
    target = load_json(PUSH_TARGET)
    scene, prog = _scene_program(target)
    runner = _Runner(scene, prog, SimpleNamespace(seed=None))
    res = runner.run()
    rollout = _rollout_from(scene, prog, res)
    assert len(runner.steps) == len(rollout["frames"])
    assert res.physical_validity is True, res.physical_validity_reason
    assert {f["gripperClosed"] for f in rollout["frames"]} == {False}

    robot = extract_robot_csg(rollout)
    assert [r["relation"] for r in robot["relations"]] == ["FAR_FROM", "NEAR"]
    assert [c["mode"] for c in robot["contacts"]] == ["TOUCHING_LIKELY"]
    assert "GRASP_LIKELY" not in {c["mode"] for c in robot["contacts"]}
    assert "RELEASE_INFERRED" not in {e["eventKind"] for e in robot["events"]}
    assert _max_touch_gap_for_contact(rollout, robot["contacts"][0]) <= P.DEFAULT.touching_gap_m

    contact_frames = _contact_frame_indices(rollout, robot["contacts"][0])
    assert contact_frames, "no frames in extracted push contact interval"
    puck_id = robot["contacts"][0]["b"]["id"]
    puck_width = min(next(b["sizeM"] for b in rollout["sceneBodies"] if b["objectId"] == puck_id)[:2])
    apertures = [runner.steps[i].gripper_aperture_m for i in contact_frames]
    assert min(apertures) >= puck_width + 0.002

    result = match(target, robot)
    assert result.passed, f"hard mismatches: {result.hard_probes}"
    assert leakage_report(robot)["clean"]

    checks = res.validity_report["checks"]
    for name in ("non_penetration", "pose_continuity", "workspace_reachability"):
        assert checks[name]["applicable"] is True, name
        assert checks[name]["passed"] is True, checks[name]["detail"]
    for name in ("gripper_feasibility", "quasi_static_support_at_release"):
        assert checks[name]["applicable"] is False, name
        assert checks[name]["passed"] is True, checks[name]["detail"]


def test_open_drawer_end_to_end_passes_with_articulation():
    target = load_json(OPEN_TARGET)
    run = solve(target, SolverConfig(backend="mujoco"))
    diag = run.rollout["diagnostics"]
    assert diag["physicalValidity"] is True, diag["physicalValidityReason"]
    assert run.rollout["frames"], "no frames produced"

    drawer_id = run.rollout["skillProgram"]["manipulatedObjectId"]
    art_vals = [
        float(f["articulation"][drawer_id])
        for f in run.rollout["frames"]
        if drawer_id in f.get("articulation", {})
    ]
    assert art_vals, "drawer articulation was not recorded"
    assert 0.015 <= art_vals[0] <= 0.025
    assert art_vals[-1] >= 0.18
    assert max(art_vals) <= 0.22
    assert art_vals[-1] - art_vals[0] > 0.05
    assert all(b + 0.002 >= a for a, b in zip(art_vals, art_vals[1:]))

    robot = extract_robot_csg(run.rollout)
    result = match(target, robot)
    assert result.passed, f"hard mismatches: {result.hard_probes}"
    assert all(result.probe_agreement[p] for p in result.hard_probes)
    assert leakage_report(robot)["clean"]

    contacts = [c for c in robot["contacts"] if c["b"]["id"] == drawer_id]
    assert "GRASP_LIKELY" in {c["mode"] for c in contacts}
    assert "RELEASE_INFERRED" in {
        e["eventKind"] for e in robot["events"]
        if drawer_id in e.get("involvedObjectIds", [])
    }
    art_events = [
        e for e in robot["events"]
        if e["eventKind"] == "ARTICULATION_CHANGE"
        and drawer_id in e.get("involvedObjectIds", [])
    ]
    assert art_events, "missing drawer articulation change event"
    transition = art_events[0]["observedDeltas"][0]["articulationTransition"]
    assert transition["fromState"]["jointKind"] == "PRISMATIC"
    assert transition["toState"]["jointKind"] == "PRISMATIC"
    assert transition["fromState"]["valueKind"] == "EXTENSION_M"
    assert transition["toState"]["valueKind"] == "EXTENSION_M"
    assert transition["toState"]["jointValue"] - transition["fromState"]["jointValue"] > 0.05

    checks = run.validity_report["checks"]
    for name in (
        "non_penetration", "pose_continuity", "gripper_feasibility",
        "workspace_reachability", "articulation_limits",
    ):
        assert checks[name]["applicable"] is True, name
        assert checks[name]["passed"] is True, checks[name]["detail"]
    assert checks["quasi_static_support_at_release"]["applicable"] is False
    assert checks["quasi_static_support_at_release"]["passed"] is True


def test_validity_report_sidecar_present():
    run = solve(load_json(TARGET), SolverConfig(backend="mujoco"))
    rep = run.validity_report
    assert rep is not None and "checks" in rep
    assert set(rep["checks"]) >= {
        "non_penetration", "pose_continuity", "quasi_static_support_at_release",
        "gripper_feasibility", "workspace_reachability", "articulation_limits"}
    assert rep["checks"]["workspace_reachability"]["passed"]


def test_deterministic_rollouts():
    import json
    a = solve(load_json(TARGET), SolverConfig(backend="mujoco")).rollout
    b = solve(load_json(TARGET), SolverConfig(backend="mujoco")).rollout
    assert json.dumps(a["frames"]) == json.dumps(b["frames"])


def test_seeded_layouts_are_reproducible_and_distinct():
    scene, prog = _scene_program(load_json(TARGET))
    _xml_a, layout_a = build_arm_scene_xml(scene, seed=7, program=prog)
    _xml_b, layout_b = build_arm_scene_xml(scene, seed=7, program=prog)
    _xml_c, layout_c = build_arm_scene_xml(scene, seed=8, program=prog)
    assert layout_a == layout_b
    assert layout_a != layout_c


def test_seeded_rollouts_same_seed_are_deterministic():
    import json
    target = load_json(TARGET)
    a = solve(target, SolverConfig(backend="mujoco", seed=7)).rollout
    b = solve(target, SolverConfig(backend="mujoco", seed=7)).rollout
    assert json.dumps(a["frames"]) == json.dumps(b["frames"])
    assert a["diagnostics"].get("seed") == 7
    assert a["diagnostics"].get("sampledLayout")


# ---- benchmark gate ---------------------------------------------------------


@pytest.fixture(scope="module")
def mujoco_benchmark_report(tmp_path_factory):
    out = tmp_path_factory.mktemp("mujoco_benchmark")
    return run_benchmark(
        ALL_GOLD_TARGETS,
        out,
        solver_cfg=SolverConfig(backend="mujoco"),
        confusion=True,
    )


def test_benchmark_mujoco_passes_all_gold_targets(mujoco_benchmark_report):
    report = mujoco_benchmark_report
    assert len(ALL_GOLD_TARGETS) == 5
    assert report["summary"]["failed"] == 0
    assert {c["case"] for c in report["cases"]} == {p.parent.name for p in ALL_GOLD_TARGETS}
    for case in report["cases"]:
        assert case["status"] == "PASS", case["case"]
        assert case["physicalValidity"] is True, case["case"]
        assert case["leakageClean"] is True, case["case"]


def test_benchmark_mujoco_confusion_matrix_all_gold_targets(mujoco_benchmark_report):
    conf = mujoco_benchmark_report["confusion"]
    expected_off_diagonal = [
        ["insert_object", "put_cube_in_tray"],
        ["put_cube_in_tray", "insert_object"],
    ]
    assert conf["missedDiagonal"] == []
    assert conf["unexpectedOffDiagonalPasses"] == []
    assert conf["offDiagonalPasses"] == expected_off_diagonal


def test_benchmark_mujoco_randomized_seeded_smoke(tmp_path):
    report = run_benchmark(
        ALL_GOLD_TARGETS,
        tmp_path,
        solver_cfg=SolverConfig(backend="mujoco"),
        confusion=True,
        randomized=True,
        seeds=2,
    )
    assert report["summary"]["total"] == 10
    assert report["summary"]["failed"] == 0
    assert report["randomized"] == {"enabled": True, "seeds": [0, 1]}
    assert report["failureClassificationSummary"] == {"passed": 10}
    assert {c["seed"] for c in report["cases"]} == {0, 1}
    assert all(c["physicalValidity"] is True for c in report["cases"])
    assert all(c["failureClassification"]["category"] == "passed" for c in report["cases"])
    assert all(c.get("sampledLayout") for c in report["cases"])
    conf = report["confusion"]
    assert conf["missedDiagonal"] == []
    assert conf["unexpectedOffDiagonalPasses"] == []


def test_benchmark_push_randomized_seeds_sample_distinct_layouts(tmp_path):
    report = run_benchmark(
        [PUSH_TARGET],
        tmp_path,
        solver_cfg=SolverConfig(backend="mujoco"),
        randomized=True,
        seeds=4,
    )
    assert report["summary"]["total"] == 4
    assert report["summary"]["failed"] == 0
    layouts = {
        tuple((oid, tuple(xyz)) for oid, xyz in sorted(case["sampledLayout"].items()))
        for case in report["cases"]
    }
    assert len(layouts) == 4


def test_benchmark_comparison_symbolic_baseline_vs_scripted_mujoco(tmp_path):
    report = run_benchmark_comparison(
        ALL_GOLD_TARGETS,
        tmp_path,
        {
            "symbolic_baseline": SolverConfig(backend="symbolic"),
            "scripted_mujoco": SolverConfig(backend="mujoco"),
        },
        confusion=True,
    )
    assert report["baselineOrder"] == ["symbolic_baseline", "scripted_mujoco"]
    symbolic = report["baselines"]["symbolic_baseline"]
    mujoco_report = report["baselines"]["scripted_mujoco"]
    assert symbolic["summary"]["passed"] == 5
    assert symbolic["summary"]["physicalValidity"] == {"unverified": 5}
    assert mujoco_report["summary"]["passed"] == 5
    assert mujoco_report["summary"]["physicalValidity"] == {"valid": 5}
    assert mujoco_report["confusion"]["unexpectedOffDiagonalPasses"] == []


# ---- sabotage: the verifier must NOT rubber-stamp ---------------------------


def test_early_release_fails_loop():
    target = load_json(TARGET)
    scene, prog = _scene_program(target)
    res = run_skill(scene, prog, SimpleNamespace(seed=None, sabotage="early_release"))
    robot = extract_robot_csg(_rollout_from(scene, prog, res))
    loop_passes = res.physical_validity and match(target, robot).passed
    assert not loop_passes, "dropping the cube mid-air must not PASS the loop"
