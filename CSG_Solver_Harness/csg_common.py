#!/usr/bin/env python3
"""Shared helpers for the CSG Solver Harness V0."""
from __future__ import annotations

import copy
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

Json = Dict[str, Any]
ROBOT_GRIPPER_ID = "robot_gripper"


def snake_to_camel(s: str) -> str:
    parts = str(s).split('_')
    return parts[0] + ''.join(p[:1].upper() + p[1:] for p in parts[1:])


def camel_to_snake(s: str) -> str:
    return re.sub(r'(?<!^)(?=[A-Z])', '_', str(s)).lower()


def get_any(d: Any, *names: str, default: Any = None) -> Any:
    if not isinstance(d, Mapping):
        return default
    keys: List[str] = []
    for n in names:
        if n:
            keys.extend([n, snake_to_camel(n), camel_to_snake(n), n[:1].lower() + n[1:]])
    for k in keys:
        if k in d:
            return d[k]
    return default


def as_list(x: Any) -> List[Any]:
    if x is None:
        return []
    return x if isinstance(x, list) else [x]


def enum_name(x: Any, default: str = "UNKNOWN") -> str:
    s = str(x or default).strip() or default
    return re.sub(r'[^A-Za-z0-9_]+', '_', s).upper()


def norm_label(x: Any) -> str:
    return re.sub(r'[^a-z0-9]+', '_', str(x or 'unknown').lower()).strip('_') or 'unknown'


def safe_id(x: Any, prefix: str = 'id') -> str:
    s = re.sub(r'[^A-Za-z0-9_]+', '_', str(x or '')).strip('_') or prefix
    return f'{prefix}_{s}' if s[0].isdigit() else s


def load_json(path: str | Path) -> Json:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)

read_json = load_json


def write_json(path: str | Path, obj: Any) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write('\n')

save_json = write_json


def ns_to_s(x: Any) -> float:
    try:
        return float(x) / 1e9
    except Exception:
        return 0.0


def s_to_ns(x: float) -> str:
    return str(int(round(float(x) * 1e9)))


def time_span_json(a: float, b: Optional[float] = None) -> Json:
    if b is None:
        b = a
    return {"startTimeNs": s_to_ns(a), "endTimeNs": s_to_ns(b)}

make_timespan = time_span_json


def span_mid_s(obj: Mapping[str, Any]) -> float:
    ts = get_any(obj, 'timeSpan', 'time_span', default={}) or {}
    a = ns_to_s(get_any(ts, 'startTimeNs', 'start_time_ns', default=0))
    b = ns_to_s(get_any(ts, 'endTimeNs', 'end_time_ns', default=0))
    if a == 0 and b == 0:
        return ns_to_s(get_any(obj, 'timeNs', 'time_ns', default=0))
    return 0.5 * (a + max(a, b))

time_mid = span_mid_s


def confidence(obj: Any, default: float = 1.0) -> float:
    try:
        if isinstance(obj, Mapping):
            return float(get_any(obj, 'confidence', default=default))
    except Exception:
        pass
    return default


def vec3(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Json:
    return {"x": float(x), "y": float(y), "z": float(z)}


def make_pose(
    frame_id: str = "world",
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    confidence_value: float = 1.0,
) -> Json:
    return {
        "frameId": frame_id,
        "positionM": vec3(*xyz),
        "orientationWxyz": {"w": float(quat[0]), "x": float(quat[1]), "y": float(quat[2]), "z": float(quat[3])},
        "confidence": float(confidence_value),
    }


def pose_xyz(pose: Any) -> Tuple[float, float, float]:
    p = get_any(pose, 'positionM', 'position_m', 'position', default={}) if isinstance(pose, Mapping) else {}
    return (
        float(get_any(p, 'x', default=0.0) or 0.0),
        float(get_any(p, 'y', default=0.0) or 0.0),
        float(get_any(p, 'z', default=0.0) or 0.0),
    )


def pose_quat(pose: Any) -> Tuple[float, float, float, float]:
    q = get_any(pose, 'orientationWxyz', 'orientation_wxyz', default={}) if isinstance(pose, Mapping) else {}
    return (
        float(get_any(q, 'w', default=1.0) or 1.0),
        float(get_any(q, 'x', default=0.0) or 0.0),
        float(get_any(q, 'y', default=0.0) or 0.0),
        float(get_any(q, 'z', default=0.0) or 0.0),
    )


def pose_with_xyz(pose: Any, xyz: Tuple[float, float, float]) -> Json:
    return make_pose(
        str(get_any(pose, 'frameId', 'frame_id', default='world') if isinstance(pose, Mapping) else 'world'),
        xyz,
        pose_quat(pose),
        confidence(pose, 1.0),
    )


def offset_pose(pose: Any, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Json:
    x, y, z = pose_xyz(pose)
    return pose_with_xyz(pose, (x + dx, y + dy, z + dz))


def object_id(obj: Mapping[str, Any]) -> str:
    return str(get_any(obj, 'objectId', 'object_id', default=''))


def category_label(obj: Mapping[str, Any]) -> str:
    return str(get_any(obj, 'categoryLabel', 'category_label', default='object'))


def object_size_m(obj: Mapping[str, Any]) -> Tuple[float, float, float]:
    geom = get_any(obj, 'geometry', default={}) or {}
    ob = get_any(geom, 'orientedBox', 'oriented_box', default=None)
    if isinstance(ob, Mapping):
        s = get_any(ob, 'sizeM', 'size_m', default={}) or {}
        return (
            float(get_any(s, 'x', default=0.04) or 0.04),
            float(get_any(s, 'y', default=0.04) or 0.04),
            float(get_any(s, 'z', default=0.04) or 0.04),
        )
    if enum_name(get_any(obj, 'physicalKind', 'physical_kind', default='')) == 'STATIC_SCENE_SURFACE':
        return (0.24, 0.18, 0.03)
    return (0.04, 0.04, 0.04)


def earliest_pose(csg: Mapping[str, Any], oid: str, idx: int = 0) -> Json:
    best: Optional[Json] = None
    best_t = 1e99
    for st in as_list(get_any(csg, 'objectStates', 'object_states', default=[])):
        if not isinstance(st, Mapping):
            continue
        if str(get_any(st, 'objectId', 'object_id', default='')) != oid:
            continue
        p = get_any(st, 'pose3D', 'pose_3d', 'pose3d', default=None)
        t = ns_to_s(get_any(st, 'timeNs', 'time_ns', default=0))
        if isinstance(p, Mapping) and t < best_t:
            best = copy.deepcopy(dict(p)); best_t = t
    return best or make_pose('world', (0.22 + 0.10 * (idx % 3), -0.12 + 0.12 * (idx // 3), 0.03), confidence_value=0.75)


def build_robot_object_map(csg: Mapping[str, Any], prefix: str = 'r') -> Dict[str, str]:
    out: Dict[str, str] = {}
    used: set[str] = set()
    for i, o in enumerate(as_list(get_any(csg, 'objects', default=[]))):
        if not isinstance(o, Mapping):
            continue
        oid = object_id(o)
        if not oid:
            continue
        base = oid
        for p in ['human_', 'robot_', 'sim_', 'obj_', 'h_', 'r_']:
            if base.startswith(p):
                base = base[len(p):]
                break
        cand = safe_id(f'{prefix}_{base}', prefix)
        if cand in used:
            cand = safe_id(f'{prefix}_{norm_label(category_label(o))}_{i}', prefix)
        root = cand
        j = 2
        while cand in used:
            cand = f'{root}_{j}'
            j += 1
        out[oid] = cand
        used.add(cand)
    return out


def remap_entity(ent: Any, obj_map: Mapping[str, str], effector: str = ROBOT_GRIPPER_ID) -> Json:
    if not isinstance(ent, Mapping):
        return {"kind": "ROBOT_PART_ENTITY", "id": effector}
    kind = enum_name(get_any(ent, 'kind', default=''))
    eid = str(get_any(ent, 'id', default=''))
    if kind == 'OBJECT_ENTITY' or eid in obj_map:
        return {"kind": "OBJECT_ENTITY", "id": obj_map.get(eid, eid)}
    if kind in {'HUMAN_PART_ENTITY', 'ROBOT_PART_ENTITY'} or eid:
        return {"kind": "ROBOT_PART_ENTITY", "id": effector}
    return {"kind": kind, "id": eid}


def remap_csg_fragment(obj: Any, obj_map: Mapping[str, str], effector: str = ROBOT_GRIPPER_ID) -> Any:
    if isinstance(obj, list):
        return [remap_csg_fragment(x, obj_map, effector) for x in obj]
    if not isinstance(obj, Mapping):
        return obj
    if 'kind' in obj and 'id' in obj:
        return remap_entity(obj, obj_map, effector)
    id_keys = {
        'objectId', 'object_id', 'subjectObjectId', 'subject_object_id', 'objectObjectId', 'object_object_id',
        'targetObjectId', 'target_object_id', 'articulatedObjectId', 'articulated_object_id'
    }
    out: Json = {}
    for k, v in obj.items():
        if k in id_keys and isinstance(v, str):
            out[k] = obj_map.get(v, v)
        elif k in {'involvedObjectIds', 'involved_object_ids'} and isinstance(v, list):
            out[k] = [obj_map.get(str(x), str(x)) for x in v]
        elif k in {'involvedAgentPartIds', 'involved_agent_part_ids'} and isinstance(v, list):
            out[k] = [effector] if v else []
        elif k in {'agentPartId', 'agent_part_id'} and isinstance(v, str):
            out[k] = effector
        else:
            out[k] = remap_csg_fragment(v, obj_map, effector)
    return out

remap_csg_refs = remap_csg_fragment


def robotize_object(obj: Mapping[str, Any], obj_map: Mapping[str, str]) -> Json:
    o = copy.deepcopy(dict(obj))
    oid = object_id(obj)
    o['objectId'] = obj_map.get(oid, oid)
    o.pop('object_id', None)
    return o


def target_probe_policy(csg: Mapping[str, Any]) -> Json:
    return {
        'hasContacts': bool(as_list(get_any(csg, 'contacts', default=[]))),
        'hasEvents': bool(as_list(get_any(csg, 'events', default=[]))),
        'hasRelations': bool(as_list(get_any(csg, 'relations', default=[]))),
        'hasTemporalEdges': bool(as_list(get_any(csg, 'temporalEdges', 'temporal_edges', default=[]))),
        'hasPlannerView': bool(get_any(csg, 'plannerView', 'planner_view', default=None)),
        'hasObjectStates': bool(as_list(get_any(csg, 'objectStates', 'object_states', default=[]))),
    }


def robotize_csg(target_csg: Mapping[str, Any], obj_map: Optional[Mapping[str, str]] = None, effector: str = ROBOT_GRIPPER_ID) -> Json:
    """Preserve observable CSG probes while changing object ids/effector role.

    This is the strict grader-facing projection. It does not infer hidden force,
    friction, mass, material, or intent; it only remaps the observable graph.
    """
    target = copy.deepcopy(dict(target_csg or {}))
    mapping = dict(obj_map or build_robot_object_map(target))
    graph = remap_csg_fragment(target, mapping, effector)
    graph['schemaVersion'] = get_any(target, 'schemaVersion', 'schema_version', default='csg.v0')
    graph['graphId'] = f"robot_{get_any(target, 'graphId', 'graph_id', default='rollout')}"
    graph['objects'] = [robotize_object(o, mapping) for o in as_list(get_any(target, 'objects', default=[])) if isinstance(o, Mapping)]
    graph['agentParts'] = [{
        'agentPartId': effector,
        'agentKind': 'ROBOT',
        'partKind': 'ROBOT_GRIPPER',
        'label': effector,
    }]
    meta = dict(get_any(graph, 'solverMetadata', default={}) or {})
    meta.update({'objectIdMap': mapping, 'robotEffectorId': effector, 'robotizedFrom': get_any(target, 'graphId', 'graph_id', default='target')})
    graph['solverMetadata'] = meta
    return graph


# Backwards-compatible alias used by harness stages.
def copy_json(x: Any) -> Any:
    return copy.deepcopy(x)


def copy_json(obj: Any) -> Any:
    """Deep-copy a JSON-like value."""
    return copy.deepcopy(obj)
