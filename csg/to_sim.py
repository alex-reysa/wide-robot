#!/usr/bin/env python3
"""Compile a target CSG's TaskSpec (PlannerView) into a simulator scene.

The scene is the solver's *input*. It deliberately carries only what a planner
may legitimately consume:
  * bodies (geometry, size, initial pose, mobility),
  * the object-id map (target id -> robot id),
  * planner goal / path constraints and contact permissions.

It does NOT embed the target's observation graph (contacts / events /
relations) nor a ``targetCsg`` copy. That separation is what lets the rollout
extractor be independent: the extractor reads frames + bodies and can never
see the answer key. (Information-flow contract, enforced by tests/test_leakage.)
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from .common import (
    Json,
    ROBOT_GRIPPER_ID,
    as_list,
    category_label,
    enum_name,
    get_any,
    load_json,
    make_pose,
    norm_label,
    ns_to_s,
    object_id,
    object_size_m,
    size_is_approximate,
    write_json,
)


# Open-cavity container compilation (audit A8): a container compiled as a
# single solid box makes the INSIDE predicate satisfiable only by
# interpenetration, so a physics backend could never honestly PASS. Containers
# get a floor slab + four walls; the solver rests inserted objects on the
# cavity floor. Constants live here so compiler and solver cannot drift.
CONTAINER_WALL_M = 0.008
CONTAINER_FLOOR_M = 0.008


def _container_object_ids(csg: Mapping[str, Any], constraints: List[Json]) -> set:
    """Target object ids that must be compiled with an open cavity: objects
    with RIM/OPENING parts, plus the container side of any INSIDE goal or
    observed INSIDE transition."""
    out: set = set()
    for o in as_list(get_any(csg, "objects", default=[])):
        parts = {enum_name(get_any(p, "kind", "part_kind", default="")) for p in as_list(get_any(o, "parts", default=[]))}
        if {"RIM", "OPENING"} & parts:
            out.add(object_id(o))
    for c in constraints:
        rel = get_any(c, "relation", default=None)
        if isinstance(rel, Mapping):
            desired = enum_name(get_any(rel, "desiredRelation", "desired_relation", default=""))
            if desired == "INSIDE":
                out.add(str(get_any(rel, "objectObjectId", "object_object_id", default="")))
            elif desired == "CONTAINS":
                out.add(str(get_any(rel, "subjectObjectId", "subject_object_id", default="")))
    for ev in as_list(get_any(csg, "events", default=[])):
        for d in as_list(get_any(ev, "observed_deltas", "observedDeltas", default=[])):
            rt = get_any(d, "relation_transition", "relationTransition", default=None)
            if isinstance(rt, Mapping):
                to_rel = enum_name(get_any(rt, "toRelation", "to_relation", default=""))
                if to_rel == "INSIDE":
                    out.add(str(get_any(rt, "objectObjectId", "object_object_id", default="")))
                elif to_rel == "CONTAINS":
                    out.add(str(get_any(rt, "subjectObjectId", "subject_object_id", default="")))
    out.discard("")
    return out


def build_robot_object_map(csg: Mapping[str, Any], prefix: str = "body") -> Dict[str, str]:
    """Target id -> opaque robot id. Ids are deliberately neutral (body_000,
    body_001, ...): deriving them from target ids carried target-authored text
    into the rollout and the 'independently extracted' robot CSG (audit A4)."""
    out: Dict[str, str] = {}
    n = 0
    for o in as_list(get_any(csg, "objects", default=[])):
        oid = object_id(o)
        if not oid or oid in out:
            continue
        out[oid] = f"{prefix}_{n:03d}"
        n += 1
    return out


def _remap_entity(ent: Any, obj_map: Mapping[str, str], effector: str) -> Json:
    if not isinstance(ent, Mapping):
        return {"kind": "ROBOT_PART_ENTITY", "id": effector}
    kind = enum_name(get_any(ent, "kind", default=""))
    eid = str(get_any(ent, "id", default=""))
    if kind == "OBJECT_ENTITY" or eid in obj_map:
        return {"kind": "OBJECT_ENTITY", "id": obj_map.get(eid, eid)}
    return {"kind": "ROBOT_PART_ENTITY", "id": effector}


def _remap_refs(obj: Any, obj_map: Mapping[str, str], effector: str) -> Any:
    if isinstance(obj, list):
        return [_remap_refs(x, obj_map, effector) for x in obj]
    if not isinstance(obj, Mapping):
        return obj
    if "kind" in obj and "id" in obj:
        return _remap_entity(obj, obj_map, effector)
    id_keys = {
        "objectId", "object_id", "subjectObjectId", "subject_object_id", "objectObjectId",
        "object_object_id", "targetObjectId", "target_object_id", "articulatedObjectId", "articulated_object_id",
    }
    out: Json = {}
    for k, v in obj.items():
        if k in id_keys and isinstance(v, str):
            out[k] = obj_map.get(v, v)
        else:
            out[k] = _remap_refs(v, obj_map, effector)
    return out


def _earliest_pose(csg: Mapping[str, Any], oid: str, idx: int) -> Json:
    best: Optional[Json] = None
    best_t = float("inf")
    for st in as_list(get_any(csg, "object_states", "objectStates", default=[])):
        if str(get_any(st, "object_id", "objectId", default="")) != oid:
            continue
        p = get_any(st, "pose_3d", "pose3D", "pose3d", default=None)
        t = ns_to_s(get_any(st, "time_ns", "timeNs", default=0))
        if isinstance(p, Mapping) and t < best_t:
            best, best_t = copy.deepcopy(dict(p)), t
    if best is not None:
        return best
    # Invented layout, flagged downstream via "initialPoseApproximate".
    return make_pose("world", (0.22 + 0.12 * (idx % 3), -0.12 + 0.12 * (idx // 3), 0.03), confidence_value=0.5)


def _planner_constraints(csg: Mapping[str, Any]) -> List[Json]:
    out: List[Json] = []
    pv = get_any(csg, "planner_view", "plannerView", default={}) or {}
    for stage in as_list(get_any(pv, "stages", default=[])):
        for grp in ("preconditions", "path_constraints", "pathConstraints", "goal_constraints", "goalConstraints"):
            out.extend(copy.deepcopy(as_list(get_any(stage, grp, default=[]))))
    # Fallback: derive a relation goal from observed relation transitions.
    if not any(get_any(c, "relation", default=None) for c in out):
        for ev in as_list(get_any(csg, "events", default=[])):
            for d in as_list(get_any(ev, "observed_deltas", "observedDeltas", default=[])):
                rt = get_any(d, "relation_transition", "relationTransition", default=None)
                if isinstance(rt, Mapping):
                    out.append({
                        "constraintId": f"goal_{get_any(rt, 'subjectObjectId', 'subject_object_id', default='obj')}",
                        "kind": "OBJECT_RELATION_GOAL", "hard": True, "weight": 1.0,
                        "relation": {
                            "subjectObjectId": get_any(rt, "subjectObjectId", "subject_object_id", default=""),
                            "objectObjectId": get_any(rt, "objectObjectId", "object_object_id", default=""),
                            "desiredRelation": get_any(rt, "toRelation", "to_relation", default="NEAR"),
                        },
                    })
                at = get_any(d, "articulation_transition", "articulationTransition", default=None)
                if isinstance(at, Mapping):
                    to_state = get_any(at, "toState", "to_state", default={}) or {}
                    out.append({
                        "constraintId": f"goal_art_{get_any(at, 'articulatedObjectId', 'articulated_object_id', default='obj')}",
                        "kind": "ARTICULATION_GOAL", "hard": True, "weight": 1.0,
                        "articulation": {
                            "articulatedObjectId": get_any(at, "articulatedObjectId", "articulated_object_id", default=get_any(to_state, "articulatedObjectId", default="")),
                            "jointKind": get_any(to_state, "jointKind", "joint_kind", default="UNKNOWN_JOINT"),
                            "targetJointValue": get_any(to_state, "jointValue", "joint_value", default=0.0),
                            "valueKind": get_any(to_state, "valueKind", "value_kind", default="OPEN_FRACTION_0_TO_1"),
                        },
                    })
    return out


def _contact_permissions(csg: Mapping[str, Any]) -> List[Json]:
    out: List[Json] = []
    pv = get_any(csg, "planner_view", "plannerView", default={}) or {}
    for st in as_list(get_any(pv, "stages", default=[])):
        out.extend(copy.deepcopy(as_list(get_any(st, "contact_permissions", "contactPermissions", default=[]))))
    return out


def compile_scene(
    csg: Mapping[str, Any],
    *,
    backend: str = "symbolic",
    robot_effector_id: str = ROBOT_GRIPPER_ID,
    preserve_object_ids: bool = False,
    scene_id: Optional[str] = None,
) -> Json:
    obj_map = (
        {object_id(o): object_id(o) for o in as_list(get_any(csg, "objects", default=[])) if object_id(o)}
        if preserve_object_ids else build_robot_object_map(csg)
    )
    pv = get_any(csg, "planner_view", "plannerView", default={}) or {}
    planner_bodies = {str(get_any(b, "object_id", "objectId", default="")): b for b in as_list(get_any(pv, "bodies", default=[]))}
    constraints = _planner_constraints(csg)
    container_ids = _container_object_ids(csg, constraints)

    bodies: List[Json] = []
    for i, o in enumerate(as_list(get_any(csg, "objects", default=[]))):
        oid = object_id(o)
        if not oid:
            continue
        rid = obj_map.get(oid, oid)
        pb = planner_bodies.get(oid, {})
        pose = _earliest_pose(csg, oid, i)
        has_state_pose = any(str(get_any(st, "object_id", "objectId", default="")) == oid and get_any(st, "pose_3d", "pose3D", "pose3d", default=None) for st in as_list(get_any(csg, "object_states", "objectStates", default=[])))
        bodies.append({
            "bodyId": rid,
            "objectId": rid,
            "sourceObjectId": oid,
            "categoryLabel": category_label(o),
            "physicalKind": get_any(o, "physical_kind", "physicalKind", default="UNKNOWN_OBJECT_KIND"),
            "geometry": copy.deepcopy(get_any(o, "geometry", default={}) or {}),
            "parts": copy.deepcopy(as_list(get_any(o, "parts", default=[]))),
            "sizeM": list(object_size_m(o)),
            "sizeApproximate": size_is_approximate(o),
            "initialPose": pose,
            "initialPoseApproximate": not has_state_pose,
            "mobility": get_any(pb, "mobility", default="UNKNOWN_MOBILITY"),
            "articulation": copy.deepcopy(_object_articulation(csg, oid)),
            "isContainer": oid in container_ids,
            **({"containerCavity": {"wallThicknessM": CONTAINER_WALL_M, "floorThicknessM": CONTAINER_FLOOR_M}}
               if oid in container_ids else {}),
        })

    return {
        "schemaVersion": "csg.sim_scene.v0",
        "sceneId": scene_id or f"scene_{get_any(csg, 'graphId', 'graph_id', default='target')}",
        "backend": backend,
        "worldFrameId": get_any(pv, "worldFrameId", "world_frame_id", default="world"),
        "robot": {"robotId": "generic_parallel_gripper_arm", "effectorId": robot_effector_id},
        "robotEffectorId": robot_effector_id,
        "objectIdMap": obj_map,
        "bodies": bodies,
        "constraints": _remap_refs(constraints, obj_map, robot_effector_id),
        "contactPermissions": _remap_refs(_contact_permissions(csg), obj_map, robot_effector_id),
    }


# Information-flow contract for the rollout (audit A4): the extractor may see
# only what a simulator could observe about an instantiated body. Free-text
# carriers (categoryLabel, sourceObjectId, geometry notes, part labels, poses
# the frames already supersede) are stripped.
ROLLOUT_BODY_FIELDS = (
    "objectId", "bodyId", "physicalKind", "sizeM", "sizeApproximate",
    "mobility", "articulation", "isContainer", "containerCavity",
)


def sanitize_bodies_for_rollout(bodies: Any) -> List[Json]:
    """Whitelist scene bodies down to what a rollout may carry. The *scene*
    (solver input) legitimately knows the target; the rollout handed to the
    independent extractor must not."""
    out: List[Json] = []
    for b in as_list(bodies):
        if not isinstance(b, Mapping):
            continue
        sb = {k: copy.deepcopy(b[k]) for k in ROLLOUT_BODY_FIELDS if k in b}
        sb.setdefault("objectId", sb.get("bodyId", ""))
        sb.setdefault("bodyId", sb.get("objectId", ""))
        out.append(sb)
    return out


def _object_articulation(csg: Mapping[str, Any], oid: str) -> Json:
    """Initial articulation value for an articulated object, if observed."""
    for st in as_list(get_any(csg, "object_states", "objectStates", default=[])):
        if str(get_any(st, "object_id", "objectId", default="")) == oid:
            art = get_any(st, "articulation", default=None)
            if isinstance(art, Mapping):
                return dict(art)
    # Fallback: starting value from the earliest articulation transition.
    for ev in as_list(get_any(csg, "events", default=[])):
        for d in as_list(get_any(ev, "observed_deltas", "observedDeltas", default=[])):
            at = get_any(d, "articulation_transition", "articulationTransition", default=None)
            if isinstance(at, Mapping):
                fs = get_any(at, "fromState", "from_state", default={}) or {}
                if fs:
                    return dict(fs)
    return {}


def scene_to_mujoco_xml(scene: Mapping[str, Any]) -> str:
    import html

    from .common import pose_xyz

    lines = ["<mujoco model='csg_scene'>", "  <worldbody>", "    <body name='table' pos='0 0 0'><geom type='plane' size='1 1 .01'/></body>"]
    for b in as_list(get_any(scene, "bodies", default=[])):
        name = html.escape(str(get_any(b, "bodyId", default="body")))
        x, y, z = pose_xyz(get_any(b, "initialPose", default=make_pose()))
        sx, sy, sz = [float(v) for v in (as_list(get_any(b, "sizeM", default=[0.04, 0.04, 0.04])) + [0.04, 0.04, 0.04])[:3]]
        movable = enum_name(get_any(b, "mobility", default="UNKNOWN")) in {"MOVABLE", "ARTICULATED"}
        lines.append(f"    <body name='{name}' pos='{x:.6f} {y:.6f} {z:.6f}'>")
        if movable:
            lines.append("      <freejoint/>")
        lines.extend(f"      {g}" for g in _body_geoms(b, sx, sy, sz))
        lines.append("    </body>")
    lines += ["  </worldbody>", "</mujoco>"]
    return "\n".join(lines) + "\n"


def _body_geoms(body: Mapping[str, Any], sx: float, sy: float, sz: float) -> List[str]:
    """Collision geoms for one body, positioned relative to the body frame.
    Containers get an open cavity (floor slab + 4 walls) so INSIDE is
    physically reachable without interpenetration (audit A8)."""
    cavity = get_any(body, "containerCavity", default=None)
    if not isinstance(cavity, Mapping):
        return [f"<geom type='box' size='{sx/2:.6f} {sy/2:.6f} {sz/2:.6f}'/>"]
    wall = float(get_any(cavity, "wallThicknessM", default=CONTAINER_WALL_M) or CONTAINER_WALL_M)
    floor = float(get_any(cavity, "floorThicknessM", default=CONTAINER_FLOOR_M) or CONTAINER_FLOOR_M)
    wall = min(wall, sx / 4, sy / 4)
    floor = min(floor, sz / 2)
    wall_h = (sz - floor) / 2
    wall_zc = floor / 2  # walls sit on the floor slab, reaching the rim
    return [
        f"<geom type='box' size='{sx/2:.6f} {sy/2:.6f} {floor/2:.6f}' pos='0 0 {-(sz - floor)/2:.6f}'/>",
        f"<geom type='box' size='{wall/2:.6f} {sy/2:.6f} {wall_h:.6f}' pos='{-(sx - wall)/2:.6f} 0 {wall_zc:.6f}'/>",
        f"<geom type='box' size='{wall/2:.6f} {sy/2:.6f} {wall_h:.6f}' pos='{(sx - wall)/2:.6f} 0 {wall_zc:.6f}'/>",
        f"<geom type='box' size='{(sx - 2*wall)/2:.6f} {wall/2:.6f} {wall_h:.6f}' pos='0 {-(sy - wall)/2:.6f} {wall_zc:.6f}'/>",
        f"<geom type='box' size='{(sx - 2*wall)/2:.6f} {wall/2:.6f} {wall_h:.6f}' pos='0 {(sy - wall)/2:.6f} {wall_zc:.6f}'/>",
    ]


def write_scene_outputs(scene: Mapping[str, Any], out_dir: str | Path, basename: str = "scene") -> Dict[str, str]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = {
        "compiled": str(out / f"{basename}.compiled.json"),
        "internal": str(out / f"{basename}.compiled.json"),
        "mujoco": str(out / f"{basename}.mjcf.xml"),
    }
    write_json(paths["compiled"], scene)
    Path(paths["mujoco"]).write_text(scene_to_mujoco_xml(scene), encoding="utf-8")
    return paths


def main() -> None:
    p = argparse.ArgumentParser(description="Compile a target CSG into a simulator scene (no leakage).")
    p.add_argument("target_csg")
    p.add_argument("--out-dir", "--out", default="scene_out")
    p.add_argument("--backend", default="symbolic")
    p.add_argument("--preserve-object-ids", action="store_true")
    args = p.parse_args()
    scene = compile_scene(load_json(args.target_csg), backend=args.backend, preserve_object_ids=args.preserve_object_ids)
    print(json.dumps(write_scene_outputs(scene, args.out_dir), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
