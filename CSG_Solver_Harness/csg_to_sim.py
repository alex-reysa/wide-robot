#!/usr/bin/env python3
"""Compile PlannerView/observable CSG facts into simulator scene artifacts.

The internal scene is simulator-neutral. MuJoCo MJCF and Isaac JSON are exported
as runnable/ingestable stubs for downstream adapters.
"""
from __future__ import annotations

import argparse
import copy
import html
import json
from pathlib import Path
from typing import Any, Mapping

from csg_common import *


def _planner_bodies(csg: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    pv = get_any(csg, 'plannerView', 'planner_view', default={}) or {}
    out: dict[str, Mapping[str, Any]] = {}
    for b in as_list(get_any(pv, 'bodies', default=[])):
        if isinstance(b, Mapping):
            oid = str(get_any(b, 'objectId', 'object_id', default=''))
            if oid:
                out[oid] = b
    return out


def _relation_constraints(csg: Mapping[str, Any]) -> list[Json]:
    constraints: list[Json] = []
    pv = get_any(csg, 'plannerView', 'planner_view', default={}) or {}
    for stage in as_list(get_any(pv, 'stages', default=[])):
        if not isinstance(stage, Mapping):
            continue
        for group in ['preconditions', 'pathConstraints', 'path_constraints', 'goalConstraints', 'goal_constraints']:
            for c in as_list(get_any(stage, group, default=[])):
                if isinstance(c, Mapping):
                    constraints.append(copy.deepcopy(dict(c)))
    # Fallback from relation transitions when PlannerView is sparse.
    for ev in as_list(get_any(csg, 'events', default=[])):
        if not isinstance(ev, Mapping):
            continue
        for d in as_list(get_any(ev, 'observedDeltas', 'observed_deltas', default=[])):
            rt = get_any(d, 'relationTransition', 'relation_transition', default=None)
            if isinstance(rt, Mapping):
                constraints.append({
                    'constraintId': f"goal_{get_any(rt,'subjectObjectId','subject_object_id',default='obj')}_{enum_name(get_any(rt,'toRelation','to_relation',default='REL'))}",
                    'kind': 'OBJECT_RELATION_GOAL',
                    'hard': True,
                    'weight': 1.0,
                    'relation': {
                        'subjectObjectId': get_any(rt, 'subjectObjectId', 'subject_object_id', default=''),
                        'objectObjectId': get_any(rt, 'objectObjectId', 'object_object_id', default=''),
                        'desiredRelation': get_any(rt, 'toRelation', 'to_relation', default='NEAR'),
                    },
                    'confidence': get_any(d, 'confidence', default=get_any(ev, 'confidence', default=1.0)),
                })
    return constraints


def _contact_permissions(csg: Mapping[str, Any]) -> list[Json]:
    out: list[Json] = []
    pv = get_any(csg, 'plannerView', 'planner_view', default={}) or {}
    for st in as_list(get_any(pv, 'stages', default=[])):
        if isinstance(st, Mapping):
            out.extend(copy.deepcopy(as_list(get_any(st, 'contactPermissions', 'contact_permissions', default=[]))))
    return out


def compile_scene(
    csg: Mapping[str, Any],
    *,
    backend: str = 'symbolic',
    robot_effector_id: str = ROBOT_GRIPPER_ID,
    preserve_object_ids: bool = False,
    scene_id: str | None = None,
) -> Json:
    obj_map = {object_id(o): object_id(o) for o in as_list(get_any(csg, 'objects', default=[])) if isinstance(o, Mapping)} if preserve_object_ids else build_robot_object_map(csg)
    planner_bodies = _planner_bodies(csg)
    bodies: list[Json] = []
    for i, o in enumerate(as_list(get_any(csg, 'objects', default=[]))):
        if not isinstance(o, Mapping):
            continue
        oid = object_id(o)
        rid = obj_map.get(oid, oid)
        pb = planner_bodies.get(oid, {})
        body = {
            'bodyId': rid,
            'objectId': rid,
            'sourceObjectId': oid,
            'categoryLabel': category_label(o),
            'physicalKind': get_any(o, 'physicalKind', 'physical_kind', default='UNKNOWN_OBJECT_KIND'),
            'geometry': copy.deepcopy(get_any(o, 'geometry', default={}) or {}),
            'sizeM': list(object_size_m(o)),
            'initialPose': earliest_pose(csg, oid, i),
            'mobility': get_any(pb, 'mobility', default='UNKNOWN_MOBILITY'),
            'confidence': float(get_any(o, 'categoryConfidence', 'category_confidence', default=1.0) or 1.0),
            'rawObject': robotize_object(o, obj_map),
        }
        bodies.append(body)

    raw_constraints = _relation_constraints(csg)
    raw_permissions = _contact_permissions(csg)
    scene = {
        'schemaVersion': 'csg.sim_scene.v0',
        'sceneId': scene_id or f"scene_{get_any(csg, 'graphId', 'graph_id', default='target')}",
        'backend': backend,
        'worldFrameId': get_any(get_any(csg, 'plannerView', 'planner_view', default={}) or {}, 'worldFrameId', 'world_frame_id', default='world'),
        'robot': {'robotId': 'generic_parallel_gripper_arm', 'effectorId': robot_effector_id},
        'robotEffectorId': robot_effector_id,
        'objectIdMap': obj_map,
        'bodies': bodies,
        'constraints': remap_csg_fragment(raw_constraints, obj_map, robot_effector_id),
        'contactPermissions': remap_csg_fragment(raw_permissions, obj_map, robot_effector_id),
        'plannerView': remap_csg_fragment(get_any(csg, 'plannerView', 'planner_view', default={}) or {}, obj_map, robot_effector_id),
        'targetProbePolicy': target_probe_policy(csg),
        'targetCsg': copy.deepcopy(dict(csg)),
    }
    return scene

compile_csg_to_scene = compile_scene
compile_scene_from_csg = compile_scene


def scene_to_mujoco_xml(scene: Mapping[str, Any]) -> str:
    lines = ["<mujoco model='csg_scene'>", "  <worldbody>", "    <body name='table' pos='0 0 0'><geom type='plane' size='1 1 .01'/></body>"]
    for b in as_list(get_any(scene, 'bodies', default=[])):
        if not isinstance(b, Mapping):
            continue
        oid = html.escape(str(get_any(b, 'bodyId', default='body')))
        x, y, z = pose_xyz(get_any(b, 'initialPose', default=make_pose()))
        sx, sy, sz = as_list(get_any(b, 'sizeM', default=[0.04, 0.04, 0.04]))[:3]
        movable = enum_name(get_any(b, 'mobility', default='UNKNOWN')) == 'MOVABLE'
        lines.append(f"    <body name='{oid}' pos='{x:.6f} {y:.6f} {z:.6f}'>")
        if movable:
            lines.append("      <freejoint/>")
        lines.append(f"      <geom type='box' size='{float(sx)/2:.6f} {float(sy)/2:.6f} {float(sz)/2:.6f}'/>")
        lines.append("    </body>")
    lines.extend(["  </worldbody>", "</mujoco>"])
    return "\n".join(lines) + "\n"


def scene_to_isaac_json(scene: Mapping[str, Any]) -> Json:
    return {'schemaVersion': 'csg.isaac_scene.v0', 'scene': scene, 'notes': 'Adapter-ready Isaac/PhysX scene specification.'}


def write_scene_outputs(scene: Mapping[str, Any], out_dir: str | Path, basename: str = 'scene') -> dict[str, str]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = {
        'internal': str(out / f'{basename}.compiled.json'),
        'compiled': str(out / f'{basename}.compiled.json'),
        'mujoco': str(out / f'{basename}.mjcf.xml'),
        'isaac': str(out / f'{basename}.isaac.json'),
    }
    write_json(paths['internal'], scene)
    # Compatibility alias for older tools that looked for scene.internal.json.
    if basename == 'scene':
        write_json(out / 'scene.internal.json', scene)
    Path(paths['mujoco']).write_text(scene_to_mujoco_xml(scene), encoding='utf-8')
    write_json(paths['isaac'], scene_to_isaac_json(scene))
    return paths

write_scene_files = write_scene_outputs
export_scene_bundle = write_scene_outputs


def main() -> None:
    p = argparse.ArgumentParser(description='Compile CSG PlannerView into simulator scene artifacts.')
    p.add_argument('target_csg')
    p.add_argument('--out-dir', '--out', default='scene_out')
    p.add_argument('--backend', default='symbolic')
    p.add_argument('--preserve-object-ids', action='store_true')
    args = p.parse_args()
    scene = compile_scene(load_json(args.target_csg), backend=args.backend, preserve_object_ids=args.preserve_object_ids)
    print(json.dumps(write_scene_outputs(scene, args.out_dir), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
