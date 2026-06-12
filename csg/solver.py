#!/usr/bin/env python3
"""Symbolic/kinematic solver: compiled scene -> rollout frames.

The rollout is the honest artifact the extractor consumes. It carries ONLY:
  * scene bodies (geometry/identity the sim instantiated),
  * the selected skill program,
  * continuous frames (effector + object poses + gripper + articulation),
  * honest diagnostics.

It does NOT carry the target's observation graph nor a ``targetCsg`` copy, so
the extractor cannot read the answer key. Object poses are interpolated so
there are no teleports and the extractor observes real co-motion during grasp.

Goal poses use the same geometric constants as ``predicates`` so that the
extractor agrees the goal relation was reached (e.g. INSIDE means the object
center sits below the container rim, not on top of it).
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .common import Json, ROBOT_GRIPPER_ID, as_list, copy_json, enum_name, get_any, load_json, make_pose, pose_xyz, s_to_ns, write_json
from .predicates import DEFAULT as PRED
from .skills import choose_primary_program, generate_skill_skeletons
from .to_sim import compile_scene, sanitize_bodies_for_rollout, write_scene_outputs

Vec3 = Tuple[float, float, float]


@dataclass
class SolverConfig:
    backend: str = "symbolic"
    robot_effector_id: str = ROBOT_GRIPPER_ID
    lift_m: float = 0.10
    steps_per_segment: int = 4
    preserve_object_ids: bool = False
    engine: Optional[str] = None  # back-compat alias
    seed: Optional[int] = None  # backend-specific (e.g. randomized MuJoCo rollouts)
    sabotage: Optional[str] = None  # backend-specific invalid-fixture injection

    def resolved_backend(self) -> str:
        return str(self.engine or self.backend or "symbolic")


@dataclass
class SolverRun:
    rollout: Json
    success: bool
    selected_program: Json = field(default_factory=dict)
    failures: List[str] = field(default_factory=list)
    validity_report: Optional[Json] = None  # sidecar detail (mujoco backend); never in the rollout

    def to_json(self) -> Json:
        return {"rollout": self.rollout, "success": self.success,
                "selectedProgram": self.selected_program, "failures": self.failures}


def _coerce_config(config: Any, backend: Optional[str]) -> SolverConfig:
    if isinstance(config, SolverConfig):
        cfg = SolverConfig(**{k: getattr(config, k) for k in SolverConfig.__dataclass_fields__})
    elif isinstance(config, Mapping):
        allowed = set(SolverConfig.__dataclass_fields__)
        cfg = SolverConfig(**{str(k): v for k, v in config.items() if str(k) in allowed})
    elif config is None:
        cfg = SolverConfig()
    else:
        cfg = SolverConfig(**{k: getattr(config, k) for k in SolverConfig.__dataclass_fields__ if hasattr(config, k)})
    if backend is not None:
        cfg.backend, cfg.engine = backend, None
    return cfg


def _body(scene: Mapping[str, Any], oid: str) -> Json:
    for b in as_list(get_any(scene, "bodies", default=[])):
        if str(get_any(b, "objectId", "bodyId", default="")) == oid:
            return dict(b)
    return {}


def _size(body: Mapping[str, Any]) -> Vec3:
    s = get_any(body, "sizeM", "size_m", default=[0.04, 0.04, 0.04])
    s = list(s) + [0.04, 0.04, 0.04]
    return (float(s[0]), float(s[1]), float(s[2]))


def _init_xyz(body: Mapping[str, Any]) -> Vec3:
    return pose_xyz(get_any(body, "initialPose", "initial_pose", default=make_pose("world", (0.0, 0.0, 0.03))))


def _goal_xyz(subj_body: Mapping[str, Any], ref_body: Mapping[str, Any], rel: str) -> Vec3:
    osz = _size(subj_body)
    sx, sy, sz = _init_xyz(subj_body)
    if not ref_body:
        return (sx + 0.10, sy, max(osz[2] / 2, 0.02))
    tx, ty, tz = _init_xyz(ref_body)
    rsz = _size(ref_body)
    rel = enum_name(rel)
    if rel in {"INSIDE", "CONTAINS"}:
        # Rest on the container's *cavity floor*: center below the rim (true
        # containment) without interpenetrating the floor slab (audit A8).
        cavity = get_any(ref_body, "containerCavity", default=None) or {}
        floor_th = float(get_any(cavity, "floorThicknessM", "floor_thickness_m", default=0.0) or 0.0)
        return (tx, ty, (tz - rsz[2] / 2) + floor_th + osz[2] / 2)
    if rel in {"ON_TOP_OF", "SUPPORTED_BY", "ABOVE_3D"}:
        return (tx, ty, tz + rsz[2] / 2 + osz[2] / 2)
    if rel == "ALIGNED_WITH":
        return (tx, ty, sz)
    if rel == "NEAR":
        return (tx + rsz[0] / 2 + osz[0] / 2 + PRED.near_gap_m * 0.5, ty, max(osz[2] / 2, 0.02))
    return (tx, ty, tz + rsz[2] / 2 + osz[2] / 2)


def _lerp(a: Vec3, b: Vec3, f: float) -> Vec3:
    return tuple(a[i] + (b[i] - a[i]) * f for i in range(3))  # type: ignore[return-value]


@dataclass
class Key:
    phase: str
    eff: Vec3
    objects: Dict[str, Vec3]
    closed: bool
    articulation: Dict[str, float] = field(default_factory=dict)


def _interpolate(keys: List[Key], all_objs: Dict[str, Vec3], times: Sequence[float], steps: int) -> List[Json]:
    frames: List[Json] = []
    n = len(keys)
    for k in range(n - 1):
        a, b = keys[k], keys[k + 1]
        t0 = times[min(k, len(times) - 1)]
        t1 = times[min(k + 1, len(times) - 1)]
        sub = steps if k < n - 1 else 1
        for s in range(sub):
            f = s / sub
            t = t0 + (t1 - t0) * f
            obj_poses: Dict[str, Json] = {}
            for oid, base in all_objs.items():
                pa = a.objects.get(oid, base)
                pb = b.objects.get(oid, base)
                obj_poses[oid] = make_pose("world", _lerp(pa, pb, f))
            art = {oid: a.articulation.get(oid, 0.0) + (b.articulation.get(oid, a.articulation.get(oid, 0.0)) - a.articulation.get(oid, 0.0)) * f for oid in set(a.articulation) | set(b.articulation)}
            frames.append({
                "timeS": float(t), "timeNs": s_to_ns(float(t)), "phase": a.phase,
                "effectorPose": make_pose("world", _lerp(a.eff, b.eff, f)),
                "gripperClosed": bool(a.closed),
                "objectPoses": obj_poses,
                "articulation": art,
            })
    # Final key as a settled frame.
    last = keys[-1]
    t = times[-1]
    frames.append({
        "timeS": float(t), "timeNs": s_to_ns(float(t)), "phase": last.phase,
        "effectorPose": make_pose("world", last.eff),
        "gripperClosed": bool(last.closed),
        "objectPoses": {oid: make_pose("world", last.objects.get(oid, base)) for oid, base in all_objs.items()},
        "articulation": dict(last.articulation),
    })
    return frames


def _times(n: int) -> List[float]:
    return [float(i) for i in range(n)]


def _frames_pick_place(scene: Mapping[str, Any], program: Mapping[str, Any], cfg: SolverConfig) -> List[Json]:
    subj = str(get_any(program, "manipulatedObjectId", default=""))
    ref = str(get_any(program, "targetObjectId", default=""))
    rel = enum_name(get_any(program, "relationGoal", default="NEAR"))
    sb, rb = _body(scene, subj), _body(scene, ref)
    start = _init_xyz(sb)
    goal = _goal_xyz(sb, rb, rel)
    lift = cfg.lift_m
    all_objs = {str(get_any(b, "objectId", default="")): _init_xyz(b) for b in as_list(get_any(scene, "bodies", default=[]))}
    above_start = (start[0], start[1], start[2] + lift)
    above_goal = (goal[0], goal[1], goal[2] + lift)

    def with_subj(p: Vec3) -> Dict[str, Vec3]:
        d = dict(all_objs)
        if subj:
            d[subj] = p
        return d

    keys = [
        Key("approach", above_start, with_subj(start), False),
        Key("pregrasp", start, with_subj(start), False),
        Key("grasp", start, with_subj(start), True),
        Key("lift", above_start, with_subj(above_start), True),
        Key("transport", above_goal, with_subj(above_goal), True),
        Key("lower", goal, with_subj(goal), True),
        Key("release", goal, with_subj(goal), False),
        Key("retreat", above_goal, with_subj(goal), False),
    ]
    return _interpolate(keys, all_objs, _times(len(keys)), cfg.steps_per_segment)


def _frames_push(scene: Mapping[str, Any], program: Mapping[str, Any], cfg: SolverConfig) -> List[Json]:
    subj = str(get_any(program, "manipulatedObjectId", default=""))
    ref = str(get_any(program, "targetObjectId", default=""))
    rel = enum_name(get_any(program, "relationGoal", default="NEAR"))
    sb, rb = _body(scene, subj), _body(scene, ref)
    start = _init_xyz(sb)
    goal = _goal_xyz(sb, rb, rel)
    osz = _size(sb)
    all_objs = {str(get_any(b, "objectId", default="")): _init_xyz(b) for b in as_list(get_any(scene, "bodies", default=[]))}
    behind = (start[0] - osz[0], start[1], start[2])
    contact = (start[0] - osz[0] / 2, start[1], start[2])
    push_end = (goal[0] - osz[0] / 2, goal[1], goal[2])

    def with_subj(p: Vec3) -> Dict[str, Vec3]:
        d = dict(all_objs)
        if subj:
            d[subj] = p
        return d

    keys = [
        Key("approach", behind, with_subj(start), False),
        Key("contact", contact, with_subj(start), False),
        Key("push", push_end, with_subj(goal), False),
        Key("retreat", (push_end[0] - osz[0], push_end[1], push_end[2] + 0.05), with_subj(goal), False),
    ]
    return _interpolate(keys, all_objs, _times(len(keys)), cfg.steps_per_segment)


def _frames_open(scene: Mapping[str, Any], program: Mapping[str, Any], cfg: SolverConfig) -> List[Json]:
    oid = str(get_any(program, "manipulatedObjectId", default=""))
    body = _body(scene, oid)
    art = get_any(body, "articulation", default={}) or {}
    goal = get_any(program, "articulationGoal", default={}) or {}
    start_val = float(get_any(art, "jointValue", "joint_value", default=0.0) or 0.0)
    target_val = float(get_any(goal, "targetJointValue", "target_joint_value", default=start_val + 0.16) or 0.0)
    pos = _init_xyz(body)
    osz = _size(body)
    all_objs = {str(get_any(b, "objectId", default="")): _init_xyz(b) for b in as_list(get_any(scene, "bodies", default=[]))}
    handle = (pos[0] + osz[0] / 2, pos[1], pos[2])
    pulled = (handle[0] + (target_val - start_val), pos[1], pos[2])

    keys = [
        Key("approach_handle", (handle[0] + 0.05, handle[1], handle[2]), dict(all_objs), False, {oid: start_val}),
        Key("grip_handle", handle, dict(all_objs), True, {oid: start_val}),
        # Hold: contact is established BEFORE the joint starts moving, so the
        # CONTACT_BEGIN event is disjoint-before ARTICULATION_CHANGE.
        Key("hold", handle, dict(all_objs), True, {oid: start_val}),
        Key("pull_open", pulled, dict(all_objs), True, {oid: target_val}),
        Key("release_handle", (pulled[0] + 0.04, pulled[1], pulled[2]), dict(all_objs), False, {oid: target_val}),
    ]
    return _interpolate(keys, all_objs, _times(len(keys)), cfg.steps_per_segment)


def _frames_noop(scene: Mapping[str, Any]) -> List[Json]:
    all_objs = {str(get_any(b, "objectId", default="")): _init_xyz(b) for b in as_list(get_any(scene, "bodies", default=[]))}
    object_poses = {oid: make_pose("world", xyz) for oid, xyz in all_objs.items()}
    effector = make_pose("world", (0.30, 0.0, 0.20))
    return [
        {
            "timeS": 0.0,
            "timeNs": s_to_ns(0.0),
            "phase": "noop_start",
            "effectorPose": effector,
            "gripperClosed": False,
            "objectPoses": copy_json(object_poses),
            "articulation": {},
        },
        {
            "timeS": 1.0,
            "timeNs": s_to_ns(1.0),
            "phase": "noop_end",
            "effectorPose": effector,
            "gripperClosed": False,
            "objectPoses": copy_json(object_poses),
            "articulation": {},
        },
    ]


def solve(target_csg: Mapping[str, Any], config: Any = None, *, backend: Optional[str] = None,
          scene: Optional[Mapping[str, Any]] = None, programs: Optional[Sequence[Mapping[str, Any]]] = None) -> SolverRun:
    cfg = _coerce_config(config, backend)
    be = cfg.resolved_backend()
    scene = copy_json(scene) if scene is not None else compile_scene(target_csg, backend=be, robot_effector_id=cfg.robot_effector_id, preserve_object_ids=cfg.preserve_object_ids)
    programs = list(programs or generate_skill_skeletons(scene))
    selected = choose_primary_program(programs)
    skill = enum_name(get_any(selected, "skillType", default="noop")).lower()

    failures: List[str] = []
    # Validity reporting contract (csg/validity.md): None = "backend cannot
    # check"; the symbolic backend must never claim true. The MuJoCo backend
    # sets a real verdict from contact dynamics.
    validity: Optional[bool] = None
    validity_reason = "symbolic backend has no contact dynamics; not checked"
    validity_report: Optional[Json] = None
    note = ("Symbolic kinematic backend: poses interpolated, no contact dynamics. "
            "Physical validity NOT checked (see csg/validity.md).")

    if be == "mujoco":
        from .backends.mujoco import run_skill  # lazy: raises if mujoco missing
        res = run_skill(scene, selected, cfg)
        frames = list(res.frames)
        failures = list(res.failures)
        validity = res.physical_validity
        validity_reason = res.physical_validity_reason
        validity_report = res.validity_report
        note = ("MuJoCo physics backend: fixed-base arm + parallel-jaw gripper, "
                "scripted pick-place; physical validity checked (see csg/validity.md).")
    elif be == "noop":
        frames = _frames_noop(scene)
        failures.append("noop_baseline_no_action")
        validity_reason = "noop baseline has no contact dynamics; not checked"
        note = ("No-op diagnostic baseline: scene is instantiated but the arm "
                "does not manipulate anything. Expected to fail task probes.")
    elif skill in {"pick_place", "insert", "place_on"}:
        frames = _frames_pick_place(scene, selected, cfg)
    elif skill == "push":
        frames = _frames_push(scene, selected, cfg)
    elif skill == "open":
        frames = _frames_open(scene, selected, cfg)
    else:
        frames = []
        failures.append(f"no_executable_skill_for:{skill}")

    plan_produced = bool(frames)
    diagnostics: Json = {
        "selectedProgramId": get_any(selected, "programId", default=""),
        "skill": skill,
        "planProduced": plan_produced,
        "numFrames": len(frames),
        "hiddenVariablesNotUsed": ["force", "torque", "mass", "friction", "stable_grasp_quality"],
        # Reporting contract (csg/validity.md): true / false / None.
        # None = "backend cannot check"; the symbolic backend must never
        # claim true. The MuJoCo backend (Phase 2C) sets a real verdict.
        "physicalValidity": validity,
        "physicalValidityReason": validity_reason,
        "note": note,
    }
    if be == "mujoco" and cfg.seed is not None:
        diagnostics["seed"] = int(cfg.seed)
        diagnostics["sampledLayout"] = getattr(res, "sampled_layout", {}) or {}

    rollout: Json = {
        "schemaVersion": "csg.rollout.v0",
        "backend": be,
        "robotEffectorId": cfg.robot_effector_id,
        "objectIdMap": dict(get_any(scene, "objectIdMap", default={}) or {}),
        "sceneBodies": sanitize_bodies_for_rollout(get_any(scene, "bodies", default=[])),
        "skillProgram": copy_json(selected),
        "frames": frames,
        "success": plan_produced,
        "failures": failures,
        "diagnostics": diagnostics,
    }
    return SolverRun(rollout=rollout, success=plan_produced, selected_program=copy_json(selected),
                     failures=failures, validity_report=validity_report)


def solve_to_files(target_path: str | Path, out_dir: str | Path, *, backend: str = "symbolic", config: Any = None) -> Dict[str, str]:
    target = load_json(target_path)
    cfg = _coerce_config(config, backend)
    scene = compile_scene(target, backend=cfg.resolved_backend(), robot_effector_id=cfg.robot_effector_id, preserve_object_ids=cfg.preserve_object_ids)
    programs = generate_skill_skeletons(scene)
    run = solve(target, cfg, scene=scene, programs=programs)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = write_scene_outputs(scene, out, basename="scene")
    paths["skill_skeletons"] = str(out / "skill_skeletons.json")
    paths["rollout"] = str(out / "rollout.json")
    write_json(paths["skill_skeletons"], {"schemaVersion": "csg.skill_skeletons.v0", "skeletons": programs})
    write_json(paths["rollout"], run.rollout)
    if run.validity_report is not None:
        paths["validity_report"] = str(out / "validity_report.json")
        write_json(paths["validity_report"], run.validity_report)
    return paths


def main() -> None:
    p = argparse.ArgumentParser(description="Solve a target CSG into rollout frames (no leakage).")
    p.add_argument("target_csg")
    p.add_argument("--out", "--out-dir", dest="out", default="csg_solve_out")
    p.add_argument("--backend", "--engine", dest="backend", default="symbolic")
    args = p.parse_args()
    paths = solve_to_files(args.target_csg, args.out, backend=args.backend)
    print(json.dumps(paths, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
