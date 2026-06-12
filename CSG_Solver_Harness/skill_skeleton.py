#!/usr/bin/env python3
"""Infer candidate skill skeletons from CSG event/relation/contact words."""
from __future__ import annotations

import argparse
import copy
import json
from typing import Any, Mapping

from csg_common import *


def _object_parts(csg: Mapping[str, Any], oid: str) -> set[str]:
    for o in as_list(get_any(csg, 'objects', default=[])):
        if isinstance(o, Mapping) and object_id(o) == oid:
            return {enum_name(get_any(p, 'kind', default='')) for p in as_list(get_any(o, 'parts', default=[])) if isinstance(p, Mapping)}
    return set()


def _relation_goals(csg: Mapping[str, Any]) -> list[Json]:
    goals: list[Json] = []
    pv = get_any(csg, 'plannerView', 'planner_view', default={}) or {}
    for st in as_list(get_any(pv, 'stages', default=[])):
        if not isinstance(st, Mapping):
            continue
        for c in as_list(get_any(st, 'goalConstraints', 'goal_constraints', default=[])):
            rel = get_any(c, 'relation', default=None)
            if isinstance(rel, Mapping):
                goals.append({
                    'subjectObjectId': str(get_any(rel, 'subjectObjectId', 'subject_object_id', default='')),
                    'objectObjectId': str(get_any(rel, 'objectObjectId', 'object_object_id', default='')),
                    'relation': enum_name(get_any(rel, 'desiredRelation', 'desired_relation', default='NEAR')),
                    'source': 'planner_goal',
                    'confidence': confidence(c),
                })
    for ev in as_list(get_any(csg, 'events', default=[])):
        if not isinstance(ev, Mapping):
            continue
        for d in as_list(get_any(ev, 'observedDeltas', 'observed_deltas', default=[])):
            rt = get_any(d, 'relationTransition', 'relation_transition', default=None)
            if isinstance(rt, Mapping):
                goals.append({
                    'subjectObjectId': str(get_any(rt, 'subjectObjectId', 'subject_object_id', default='')),
                    'objectObjectId': str(get_any(rt, 'objectObjectId', 'object_object_id', default='')),
                    'relation': enum_name(get_any(rt, 'toRelation', 'to_relation', default='NEAR')),
                    'fromRelation': enum_name(get_any(rt, 'fromRelation', 'from_relation', default='UNKNOWN')),
                    'source': str(get_any(ev, 'eventId', 'event_id', default='event')),
                    'confidence': confidence(d, confidence(ev)),
                })
    # de-duplicate but keep first source.
    seen: set[tuple[str, str, str]] = set()
    out: list[Json] = []
    for g in goals:
        key = (g['subjectObjectId'], g['objectObjectId'], g['relation'])
        if g['subjectObjectId'] and key not in seen:
            out.append(g); seen.add(key)
    return out


def _articulation_goals(csg: Mapping[str, Any]) -> list[Json]:
    goals: list[Json] = []
    for ev in as_list(get_any(csg, 'events', default=[])):
        if not isinstance(ev, Mapping):
            continue
        for d in as_list(get_any(ev, 'observedDeltas', 'observed_deltas', default=[])):
            at = get_any(d, 'articulationTransition', 'articulation_transition', default=None)
            if isinstance(at, Mapping):
                after = get_any(at, 'toState', 'to_state', default={}) or {}
                goals.append({
                    'articulatedObjectId': str(get_any(at, 'articulatedObjectId', 'articulated_object_id', default=get_any(after, 'articulatedObjectId', default=''))),
                    'jointKind': enum_name(get_any(after, 'jointKind', default='UNKNOWN_JOINT')),
                    'targetJointValue': get_any(after, 'jointValue', 'joint_value', default=0.0),
                    'valueKind': enum_name(get_any(after, 'valueKind', default='OPEN_FRACTION_0_TO_1')),
                    'source': str(get_any(ev, 'eventId', 'event_id', default='event')),
                    'confidence': confidence(d, confidence(ev)),
                })
    return goals


def _has_grasp(csg: Mapping[str, Any], oid: str) -> bool:
    for c in as_list(get_any(csg, 'contacts', default=[])):
        if not isinstance(c, Mapping):
            continue
        ids = [str(get_any(get_any(c, side, default={}) or {}, 'id', default='')) for side in ['a', 'b']]
        if oid in ids and enum_name(get_any(c, 'mode', default='')) == 'GRASP_LIKELY':
            return True
    return False


def _classify_relation(csg: Mapping[str, Any], g: Mapping[str, Any]) -> str:
    rel = enum_name(get_any(g, 'relation', default='NEAR'))
    target = str(get_any(g, 'objectObjectId', default=''))
    parts = _object_parts(csg, target)
    if rel in {'INSIDE', 'CONTAINS'}:
        return 'insert' if {'OPENING', 'RIM'}.intersection(parts) else 'pick_place'
    if rel in {'ON_TOP_OF', 'SUPPORTED_BY', 'ABOVE_3D'}:
        return 'place_on'
    if rel in {'NEAR', 'ALIGNED_WITH'} and _has_grasp(csg, str(get_any(g, 'subjectObjectId', default=''))):
        return 'pick_place'
    return 'push'


def _steps_for(skill: str, subj: str, target: str, rel: str) -> list[Json]:
    if skill in {'pick_place', 'insert', 'place_on'}:
        return [
            {'op': 'move_effector_to_pregrasp', 'objectId': subj},
            {'op': 'close_gripper_until_contact', 'objectId': subj, 'contactMode': 'GRASP_LIKELY'},
            {'op': 'lift_object', 'objectId': subj},
            {'op': 'move_object_to_relation_target', 'objectId': subj, 'targetObjectId': target, 'relation': rel},
            {'op': 'open_gripper_release', 'objectId': subj},
        ]
    if skill == 'push':
        return [
            {'op': 'move_effector_to_push_pose', 'objectId': subj},
            {'op': 'maintain_touching_contact', 'objectId': subj, 'contactMode': 'TOUCHING_LIKELY'},
            {'op': 'push_until_relation', 'objectId': subj, 'targetObjectId': target, 'relation': rel},
        ]
    if skill == 'open':
        return [
            {'op': 'move_effector_to_handle', 'objectId': subj},
            {'op': 'maintain_touching_contact', 'objectId': subj, 'contactMode': 'TOUCHING_LIKELY'},
            {'op': 'move_along_articulation_axis', 'objectId': subj},
        ]
    return [{'op': 'preserve_observable_trace'}]


def generate_skill_skeletons(csg: Mapping[str, Any], scene: Mapping[str, Any] | None = None) -> list[Json]:
    programs: list[Json] = []
    for i, g in enumerate(_relation_goals(csg)):
        subj = str(get_any(g, 'subjectObjectId', default=''))
        target = str(get_any(g, 'objectObjectId', default=''))
        rel = enum_name(get_any(g, 'relation', default='NEAR'))
        skill = _classify_relation(csg, g)
        programs.append({
            'programId': f'candidate_{i:03d}_{skill}_{norm_label(subj)}_{rel.lower()}',
            'skillType': skill,
            'manipulatedObjectId': subj,
            'targetObjectId': target,
            'relationGoal': rel,
            'source': get_any(g, 'source', default='csg'),
            'score': 1.0 + (0.2 if _has_grasp(csg, subj) else 0.0),
            'steps': _steps_for(skill, subj, target, rel),
        })
    base = len(programs)
    for j, g in enumerate(_articulation_goals(csg)):
        oid = str(get_any(g, 'articulatedObjectId', default=''))
        programs.append({
            'programId': f'candidate_{base+j:03d}_open_{norm_label(oid)}',
            'skillType': 'open',
            'manipulatedObjectId': oid,
            'targetObjectId': oid,
            'articulationGoal': copy.deepcopy(g),
            'score': 1.1,
            'steps': _steps_for('open', oid, oid, 'ARTICULATION_GOAL'),
        })
    if not programs:
        objs = [object_id(o) for o in as_list(get_any(csg, 'objects', default=[])) if isinstance(o, Mapping) and object_id(o)]
        programs.append({'programId': 'candidate_000_noop_preserve_trace', 'skillType': 'noop', 'manipulatedObjectId': objs[0] if objs else '', 'score': 0.0, 'steps': _steps_for('noop', '', '', '')})
    programs.sort(key=lambda p: float(get_any(p, 'score', default=0.0) or 0.0), reverse=True)
    return programs

infer_skill_skeletons = generate_skill_skeletons
infer_skill_candidates = generate_skill_skeletons


def main() -> None:
    p = argparse.ArgumentParser(description='Infer candidate skill skeletons from a target CSG.')
    p.add_argument('target_csg')
    p.add_argument('--out', default='skill_skeletons.json')
    args = p.parse_args()
    programs = generate_skill_skeletons(load_json(args.target_csg))
    write_json(args.out, {'schemaVersion': 'csg.skill_skeletons.v0', 'skeletons': programs})
    print(json.dumps({'out': args.out, 'num_skeletons': len(programs)}, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()


def choose_primary_program(programs):
    """Return the highest-scoring candidate skill skeleton."""
    candidates = list(programs or [])
    if not candidates:
        return {'programId': 'candidate_000_noop_preserve_trace', 'skillType': 'noop', 'score': 0.0, 'steps': [{'op': 'preserve_observable_trace'}]}
    return max(candidates, key=lambda p: float(get_any(p, 'score', default=0.0) or 0.0))

# Compatibility helper for csg_solver.py
def choose_primary_program(programs):
    programs = list(programs or [])
    if not programs:
        return {"programId": "candidate_0_noop", "skillType": "noop", "score": 0.0, "steps": []}
    return sorted(programs, key=lambda p: (-float(get_any(p, "score", default=0.0) or 0.0), str(get_any(p, "programId", default=""))))[0]
