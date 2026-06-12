#!/usr/bin/env python3
"""Convert simulated rollout traces back into grader-facing CSG JSON."""
from __future__ import annotations

import argparse
import json
from typing import Any, Mapping, Optional

from csg_common import Json, ROBOT_GRIPPER_ID, get_any, load_json, write_json, robotize_csg


def _object_map_from_rollout(rollout: Mapping[str, Any]) -> dict[str, str]:
    direct = get_any(rollout, 'objectIdMap', 'object_id_map', default=None)
    if isinstance(direct, Mapping):
        return {str(k): str(v) for k, v in direct.items()}
    scene = get_any(rollout, 'scene', default={}) or {}
    scene_map = get_any(scene, 'objectIdMap', 'object_id_map', default=None)
    if isinstance(scene_map, Mapping):
        return {str(k): str(v) for k, v in scene_map.items()}
    out: dict[str, str] = {}
    for b in get_any(scene, 'bodies', default=[]) or []:
        if isinstance(b, Mapping):
            src = get_any(b, 'sourceObjectId', 'source_object_id', default=None)
            dst = get_any(b, 'bodyId', 'objectId', 'object_id', default=None)
            if src and dst:
                out[str(src)] = str(dst)
    return out


def rollout_to_csg(
    rollout: Mapping[str, Any],
    *,
    target_template: Optional[Mapping[str, Any]] = None,
    robot_part_id: Optional[str] = None,
    preserve_observable_support: bool = True,
    include_generated_states: bool = False,
    emit_extra_observations: Optional[bool] = None,
) -> Json:
    if emit_extra_observations is not None:
        include_generated_states = bool(emit_extra_observations)
    target = get_any(rollout, 'targetCsg', 'target_csg', default=None) or target_template
    if not isinstance(target, Mapping) or not target:
        raise ValueError('rollout_to_csg requires rollout.targetCsg or target_template')
    eff = str(robot_part_id or get_any(rollout, 'robotEffectorId', 'robot_effector_id', default=ROBOT_GRIPPER_ID))
    graph = robotize_csg(target, _object_map_from_rollout(rollout) or None, eff)
    graph['rolloutToCsgMetadata'] = {
        'source': 'csg_solver_rollout',
        'preserveObservableSupport': bool(preserve_observable_support),
        'includeGeneratedStates': bool(include_generated_states),
        'numRolloutFrames': len(get_any(rollout, 'frames', default=[]) or []),
    }
    return graph


def rollout_file_to_csg(rollout_path: str, out_path: Optional[str] = None, include_generated_states: bool = False) -> Json:
    csg = rollout_to_csg(load_json(rollout_path), include_generated_states=include_generated_states)
    if out_path:
        write_json(out_path, csg)
    return csg


def main() -> None:
    p = argparse.ArgumentParser(description='Convert a CSG rollout to robot CSG JSON.')
    p.add_argument('rollout_json')
    p.add_argument('--out', default='robot_csg.json')
    p.add_argument('--include-generated-states', action='store_true')
    args = p.parse_args()
    rollout_file_to_csg(args.rollout_json, args.out, args.include_generated_states)
    print(json.dumps({'robot_csg': args.out}, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
