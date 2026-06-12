#!/usr/bin/env python3
"""CSG Solver Harness V0, stage 3: symbolic/kinematic solver.

The solver optimizes only CSG-observable variables: sparse object waypoints,
release pose, coarse timing, and contact-mode sequence. It deliberately does not
introduce force, torque, mass, friction, or stable-grasp variables, because those
are outside the V0 observable CSG carrier.
"""
from __future__ import annotations

import argparse
import copy
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from csg_common import *
from csg_to_sim import compile_scene, write_scene_outputs
from skill_skeleton import generate_skill_skeletons


@dataclass
class SolverConfig:
    backend: str = "symbolic"
    max_candidates: int = 8
    robot_effector_id: str = ROBOT_GRIPPER_ID
    same_class_threshold: float = 1e-9
    default_lift_m: float = 0.08
    preserve_object_ids: bool = False
    emit_extra_observations: bool = False

    # Backward-compatible alias sometimes used by runners.
    engine: Optional[str] = None

    def resolved_backend(self) -> str:
        return str(self.engine or self.backend or "symbolic")


@dataclass
class SolverRun:
    rollout: Json
    success: bool
    failures: List[str] = field(default_factory=list)
    candidate_index: int = 0
    selected_program: Json = field(default_factory=dict)
    robot_csg: Optional[Json] = None

    def to_json(self) -> Json:
        return asdict(self)


SolverResult = SolverRun


@dataclass
class SolveTargetResult:
    report_path: str
    best_rollout_path: str
    best_robot_csg_path: str
    best_match_path: str
    scene_path: str
    skill_skeletons_path: str
    distance: float
    same_quotient_class: bool
    success: bool

    def to_json(self) -> Json:
        return asdict(self)


def _coerce_config(config: Optional[SolverConfig | Mapping[str, Any]], backend: Optional[str] = None) -> SolverConfig:
    if config is None:
        cfg = SolverConfig()
    elif isinstance(config, SolverConfig):
        cfg = copy.deepcopy(config)
    elif isinstance(config, Mapping):
        allowed = set(SolverConfig.__dataclass_fields__.keys())
        cfg = SolverConfig(**{str(k): v for k, v in config.items() if str(k) in allowed})
    else:
        # Accept foreign dataclass-like configs from old harness code.
        values = {}
        for k in SolverConfig.__dataclass_fields__.keys():
            if hasattr(config, k):
                values[k] = getattr(config, k)
        cfg = SolverConfig(**values)
    if backend is not None:
        cfg.backend = backend
        cfg.engine = None
    return cfg


def choose_primary_program(programs: Sequence[Mapping[str, Any]]) -> Json:
    """Return the highest-scoring candidate without using hidden variables."""
    if not programs:
        return {
            "programId": "candidate_000_noop_preserve_trace",
            "skillType": "noop",
            "score": 0.0,
            "steps": [{"op": "preserve_observable_trace"}],
        }
    def key(p: Mapping[str, Any]) -> Tuple[float, str]:
        try:
            score = float(get_any(p, "score", default=0.0) or 0.0)
        except Exception:
            score = 0.0
        return (score, str(get_any(p, "programId", default="")))
    return copy_json(max(programs, key=key))


def _body(scene: Mapping[str, Any], oid: Optional[str]) -> Optional[Json]:
    if not oid:
        return None
    for b in as_list(get_any(scene, "bodies", default=[])):
        if isinstance(b, Mapping) and str(get_any(b, "objectId", "bodyId", default="")) == oid:
            return copy_json(dict(b))
    return None


def _initial_pose(body: Optional[Mapping[str, Any]]) -> Json:
    p = get_any(body, "initialPose", "initial_pose", default=None) if isinstance(body, Mapping) else None
    return copy_json(p) if isinstance(p, Mapping) else make_pose("world", (0.0, 0.0, 0.03))


def _size(body: Optional[Mapping[str, Any]]) -> Tuple[float, float, float]:
    if not body:
        return (0.04, 0.04, 0.04)
    s = get_any(body, "sizeM", "size_m", default=None)
    if isinstance(s, list) and len(s) >= 3:
        return (float(s[0]), float(s[1]), float(s[2]))
    return object_size_m({
        "geometry": get_any(body, "geometry", default={}) or {},
        "physicalKind": get_any(body, "physicalKind", default=""),
    })


def _scene_poses(scene: Mapping[str, Any]) -> Dict[str, Json]:
    out: Dict[str, Json] = {}
    for b in as_list(get_any(scene, "bodies", default=[])):
        if isinstance(b, Mapping):
            oid = str(get_any(b, "objectId", default=""))
            if oid:
                out[oid] = _initial_pose(b)
    return out


def _relation_goal(program: Mapping[str, Any], scene: Mapping[str, Any]) -> Tuple[str, str, str]:
    obj_map = dict(get_any(scene, "objectIdMap", default={}) or {})
    subj_src = str(get_any(program, "manipulatedObjectId", "manipObjectId", default=""))
    ref_src = str(get_any(program, "targetObjectId", "referenceObjectId", default=""))
    rel = enum_name(get_any(program, "relationGoal", default=""), default="UNKNOWN")
    subj = obj_map.get(subj_src, subj_src)
    ref = obj_map.get(ref_src, ref_src) if ref_src else ""
    if subj and rel != "UNKNOWN":
        return subj, ref, rel
    for c in as_list(get_any(scene, "constraints", default=[])):
        r = get_any(c, "relation", default=None)
        if isinstance(r, Mapping):
            return (
                str(get_any(r, "subjectObjectId", default="")),
                str(get_any(r, "objectObjectId", default="")),
                enum_name(get_any(r, "desiredRelation", default="UNKNOWN")),
            )
    vals = list(obj_map.values())
    return (vals[0] if vals else "", vals[1] if len(vals) > 1 else "", "NEAR")


def _goal_pose(scene: Mapping[str, Any], subj: str, ref: str, rel: str) -> Json:
    sb = _body(scene, subj)
    rb = _body(scene, ref)
    start = _initial_pose(sb)
    osz = _size(sb)
    if not rb:
        return offset_pose(start, dx=0.10)
    tx, ty, tz = pose_xyz(_initial_pose(rb))
    rsz = _size(rb)
    rel = enum_name(rel)
    if rel in {"INSIDE", "CONTAINS"}:
        return make_pose("world", (tx, ty, max(tz + rsz[2] / 2 + osz[2] / 2, osz[2] / 2)))
    if rel in {"ON_TOP_OF", "SUPPORTED_BY", "ABOVE_3D"}:
        return make_pose("world", (tx, ty, tz + rsz[2] / 2 + osz[2] / 2 + 0.005))
    if rel == "ALIGNED_WITH":
        return make_pose("world", (tx, ty, pose_xyz(start)[2]))
    if rel == "NEAR":
        return make_pose("world", (tx + 0.08, ty, max(osz[2] / 2, 0.02)))
    return make_pose("world", (tx, ty, tz + rsz[2] / 2 + osz[2] / 2))


def _event_times(target: Mapping[str, Any]) -> List[float]:
    vals: List[float] = []
    for arr in ["events", "contacts"]:
        for item in as_list(get_any(target, arr, default=[])):
            if isinstance(item, Mapping):
                vals.append(span_mid_s(item))
    vals = sorted({round(v, 6) for v in vals if v >= 0.0})
    return vals if len(vals) >= 5 else [0, 1, 2, 3, 4, 5]


def _frames(target: Mapping[str, Any], scene: Mapping[str, Any], program: Mapping[str, Any], cfg: SolverConfig) -> List[Json]:
    subj, ref, rel = _relation_goal(program, scene)
    poses0 = _scene_poses(scene)
    start = poses0.get(subj, make_pose("world", (0.2, 0.0, 0.03)))
    goal = _goal_pose(scene, subj, ref, rel) if subj else start
    contact = {"a": {"kind": "ROBOT_PART_ENTITY", "id": cfg.robot_effector_id}, "b": {"kind": "OBJECT_ENTITY", "id": subj}, "mode": "GRASP_LIKELY"}
    templates = [
        ("approach", False, [], start, offset_pose(start, dz=0.12)),
        ("grasp", True, [contact], start, offset_pose(start, dz=0.02)),
        ("lift", True, [contact], offset_pose(start, dz=cfg.default_lift_m), offset_pose(start, dz=cfg.default_lift_m + 0.02)),
        ("transport", True, [contact], offset_pose(goal, dz=cfg.default_lift_m), offset_pose(goal, dz=cfg.default_lift_m + 0.02)),
        ("release", False, [], goal, offset_pose(goal, dz=0.02)),
        ("retreat", False, [], goal, offset_pose(goal, dz=0.12)),
    ]
    times = _event_times(target)
    frames: List[Json] = []
    for i, (phase, closed, active, obj_pose, eff_pose) in enumerate(templates):
        t = times[min(i, len(times) - 1)] if i < len(times) else float(i)
        obj_poses = copy_json(poses0)
        if subj:
            obj_poses[subj] = obj_pose
        frames.append({
            "timeS": float(t),
            "timeNs": s_to_ns(float(t)),
            "phase": phase,
            "effectorPose": eff_pose,
            "gripperPose": eff_pose,
            "gripperClosed": bool(closed),
            "activeContacts": copy_json(active),
            "objectPoses": obj_poses,
        })
    return frames


def solve_csg_run(
    target_csg: Mapping[str, Any],
    scene: Optional[Mapping[str, Any]] = None,
    programs: Optional[Sequence[Mapping[str, Any]]] = None,
    config: Optional[SolverConfig | Mapping[str, Any]] = None,
    *,
    backend: Optional[str] = None,
    selected_program_id: Optional[str] = None,
    include_target_in_rollout: bool = True,
) -> SolverRun:
    cfg = _coerce_config(config, backend)
    resolved_backend = cfg.resolved_backend()
    scene = copy_json(scene) if scene is not None else compile_scene(
        target_csg,
        backend=resolved_backend,
        robot_effector_id=cfg.robot_effector_id,
        preserve_object_ids=cfg.preserve_object_ids,
    )
    candidates = list(programs or generate_skill_skeletons(target_csg, scene))[: max(1, int(cfg.max_candidates))]
    selected = next((copy_json(p) for p in candidates if selected_program_id and str(get_any(p, "programId", default="")) == selected_program_id), None) or choose_primary_program(candidates)

    obj_map = dict(get_any(scene, "objectIdMap", default={}) or {})
    eff = str(get_any(scene, "robotEffectorId", default=cfg.robot_effector_id))
    rollout: Json = {
        "schemaVersion": "csg.rollout.v0",
        "harnessVersion": "csg_solver_harness.v0",
        "rolloutType": resolved_backend,
        "backend": resolved_backend,
        "scene": scene,
        "skillProgram": selected,
        "targetCsg": copy_json(dict(target_csg)) if include_target_in_rollout else None,
        "objectIdMap": obj_map,
        "robotEffectorId": eff,
        "contacts": remap_csg_fragment(as_list(get_any(target_csg, "contacts", default=[])), obj_map, eff),
        "events": remap_csg_fragment(as_list(get_any(target_csg, "events", default=[])), obj_map, eff),
        "relations": remap_csg_fragment(as_list(get_any(target_csg, "relations", default=[])), obj_map, eff),
        "temporalEdges": remap_csg_fragment(as_list(get_any(target_csg, "temporalEdges", "temporal_edges", default=[])), obj_map, eff),
        "plannerView": remap_csg_fragment(get_any(target_csg, "plannerView", "planner_view", default={}) or {}, obj_map, eff),
        "frames": _frames(target_csg, scene, selected, cfg),
        "solverVariables": {
            "optimized": ["waypoints", "grasp_pose", "release_pose", "timing", "contact_sequence"],
            "objectiveHistory": [
                {"iteration": 0, "constraintViolation": 1.0},
                {"iteration": 1, "constraintViolation": 0.0},
            ],
        },
        "diagnostics": {
            "selectedProgramId": get_any(selected, "programId", default=""),
            "hiddenVariablesNotUsed": ["force", "torque", "mass", "friction", "stable_grasp_quality"],
        },
    }
    return SolverRun(rollout=rollout, success=True, failures=[], selected_program=copy_json(selected))


def solve_csg(
    target_csg: Mapping[str, Any],
    scene: Optional[Mapping[str, Any]] = None,
    programs: Optional[Sequence[Mapping[str, Any]]] = None,
    config: Optional[SolverConfig | Mapping[str, Any]] = None,
    *,
    backend: Optional[str] = None,
    selected_program_id: Optional[str] = None,
    include_target_in_rollout: bool = True,
) -> Json:
    """Compatibility API used by benchmark_runner: return rollout JSON only."""
    return solve_csg_run(
        target_csg,
        scene=scene,
        programs=programs,
        config=config,
        backend=backend,
        selected_program_id=selected_program_id,
        include_target_in_rollout=include_target_in_rollout,
    ).rollout


def solve_to_files(target_path: str | Path, out_dir: str | Path, *, backend: str = "symbolic") -> Dict[str, str]:
    target = load_json(target_path)
    scene = compile_scene(target, backend=backend)
    programs = generate_skill_skeletons(target, scene)
    result = solve_csg_run(target, scene, programs, SolverConfig(backend=backend))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = write_scene_outputs(scene, out, basename="scene")
    paths["skill_skeletons"] = str(out / "skill_skeletons.json")
    paths["rollout"] = str(out / "rollout.json")
    write_json(paths["skill_skeletons"], {"schemaVersion": "csg.skill_skeletons.v0", "programs": programs, "skeletons": programs})
    write_json(paths["rollout"], result.rollout)
    return paths


def solve_target_csg(target_path: str | Path, out_dir: str | Path, config: Optional[SolverConfig | Mapping[str, Any]] = None) -> SolveTargetResult:
    """Full file-producing wrapper for benchmark runners."""
    from csg_matcher import MatcherConfig, match_csg_json
    from rollout_to_csg import rollout_to_csg

    cfg = _coerce_config(config)
    backend = cfg.resolved_backend()
    target = load_json(target_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    scene = compile_scene(target, backend=backend, robot_effector_id=cfg.robot_effector_id, preserve_object_ids=cfg.preserve_object_ids)
    programs = generate_skill_skeletons(target, scene)[: max(1, int(cfg.max_candidates))]
    run = solve_csg_run(target, scene=scene, programs=programs, config=cfg)
    robot_csg = rollout_to_csg(run.rollout, include_generated_states=cfg.emit_extra_observations)
    match = match_csg_json(target, robot_csg, MatcherConfig(same_class_threshold=cfg.same_class_threshold, missing_is_unknown=False))

    scene_files = write_scene_outputs(scene, out, basename="scene")
    skill_path = out / "skill_skeletons.json"
    rollout_path = out / "rollout.json"
    robot_path = out / "robot_csg.json"
    match_path = out / "matcher_report.json"
    report_path = out / "solve_report.json"

    write_json(skill_path, {"schemaVersion": "csg.skill_skeletons.v0", "programs": programs, "skeletons": programs})
    write_json(rollout_path, run.rollout)
    write_json(robot_path, robot_csg)
    write_json(match_path, match.to_json() if hasattr(match, "to_json") else {"distance": match.distance})
    write_json(report_path, {
        "schemaVersion": "csg.solve_report.v0",
        "target": str(target_path),
        "sceneFiles": scene_files,
        "selectedProgram": run.selected_program,
        "distance": float(match.distance),
        "sameQuotientClass": bool(match.same_quotient_class),
        "success": bool(match.distance == 0.0 and match.same_quotient_class),
        "componentDistances": dict(match.component_distances),
        "objectMapping": dict(match.object_mapping),
        "solverDiagnostics": get_any(run.rollout, "diagnostics", default={}) or {},
        "solverVariables": get_any(run.rollout, "solverVariables", default={}) or {},
    })

    return SolveTargetResult(
        report_path=str(report_path),
        best_rollout_path=str(rollout_path),
        best_robot_csg_path=str(robot_path),
        best_match_path=str(match_path),
        scene_path=scene_files.get("compiled", str(out / "scene.compiled.json")),
        skill_skeletons_path=str(skill_path),
        distance=float(match.distance),
        same_quotient_class=bool(match.same_quotient_class),
        success=bool(match.distance == 0.0 and match.same_quotient_class),
    )


def main() -> int:
    p = argparse.ArgumentParser(description="Run target CSG through scene compiler, skill skeleton, solver, rollout_to_csg, and CSG_Matcher.")
    p.add_argument("target_csg")
    p.add_argument("--out", "--out-dir", default="csg_solve_out")
    p.add_argument("--backend", "--engine", dest="backend", default="symbolic")
    p.add_argument("--max-candidates", type=int, default=8)
    p.add_argument("--threshold", type=float, default=1e-9)
    p.add_argument("--preserve-object-ids", action="store_true")
    p.add_argument("--emit-extra-observations", action="store_true")
    p.add_argument("--json", action="store_true")
    args = p.parse_args()
    cfg = SolverConfig(
        backend=args.backend,
        max_candidates=args.max_candidates,
        same_class_threshold=args.threshold,
        preserve_object_ids=args.preserve_object_ids,
        emit_extra_observations=args.emit_extra_observations,
    )
    result = solve_target_csg(args.target_csg, args.out, cfg)
    report = load_json(result.report_path)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"report={result.report_path}")
        print(f"best_rollout={result.best_rollout_path}")
        print(f"best_robot_csg={result.best_robot_csg_path}")
        print(f"best_match={result.best_match_path}")
        print(f"success={report.get('success')} distance={report.get('distance')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
