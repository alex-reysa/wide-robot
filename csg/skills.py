#!/usr/bin/env python3
"""Infer candidate skill skeletons from a compiled scene's constraints.

Reads the *scene* (solver input: bodies + planner constraints), not the raw
observation graph, so skill routing depends only on the TaskSpec. Routing by
goal structure follows the roadmap's skill-routing table (Phase 5).
"""
from __future__ import annotations

import argparse
import json
from typing import Any, Dict, List, Mapping, Optional

from .common import Json, as_list, enum_name, get_any, load_json, norm_label, object_id, write_json


def _bodies(scene: Mapping[str, Any]) -> Dict[str, Json]:
    return {str(get_any(b, "objectId", "bodyId", default="")): b for b in as_list(get_any(scene, "bodies", default=[]))}


def _object_parts(scene: Mapping[str, Any], oid: str) -> set:
    """Part kinds of a compiled body. ``compile_scene`` emits parts at the body
    top level; reading only ``geometry.parts`` left insert routing dead (V0.2
    audit). Geometry-nested parts are kept as a fallback for raw graphs."""
    b = _bodies(scene).get(oid, {})
    parts = as_list(get_any(b, "parts", default=[]))
    if not parts:
        geom = get_any(b, "geometry", default={}) or {}
        parts = as_list(get_any(geom, "parts", default=[]))
    return {enum_name(get_any(p, "kind", "part_kind", default="")) for p in parts}


# Contact modes that forbid grasping: a goal whose subject is constrained to
# one of these must be achieved by pushing, not pick-and-place.
PUSH_CONTACT_MODES = {"TOUCHING_LIKELY", "SLIDING_LIKELY"}


def _push_constrained_objects(scene: Mapping[str, Any]) -> set:
    """Object ids whose effector contact manner is constrained to a non-grasp
    mode (CONTACT_MODE_CONSTRAINT with TOUCHING/SLIDING_LIKELY)."""
    out: set = set()
    for c in as_list(get_any(scene, "constraints", default=[])):
        if enum_name(get_any(c, "kind", default="")) != "CONTACT_MODE_CONSTRAINT":
            continue
        cc = get_any(c, "contact", default={}) or {}
        mode = enum_name(get_any(cc, "mode", default=get_any(c, "mode", default="")))
        if mode not in PUSH_CONTACT_MODES:
            continue
        for ent in (get_any(cc, "a", default=None), get_any(cc, "b", default=None)):
            if isinstance(ent, Mapping) and enum_name(get_any(ent, "kind", default="")) == "OBJECT_ENTITY":
                out.add(str(get_any(ent, "id", default="")))
    out.discard("")
    return out


def _relation_goals(scene: Mapping[str, Any]) -> List[Json]:
    out: List[Json] = []
    seen = set()
    for c in as_list(get_any(scene, "constraints", default=[])):
        rel = get_any(c, "relation", default=None)
        if isinstance(rel, Mapping):
            subj = str(get_any(rel, "subjectObjectId", "subject_object_id", default=""))
            obj = str(get_any(rel, "objectObjectId", "object_object_id", default=""))
            r = enum_name(get_any(rel, "desiredRelation", "desired_relation", default="NEAR"))
            key = (subj, obj, r)
            if subj and key not in seen:
                seen.add(key)
                out.append({"subjectObjectId": subj, "objectObjectId": obj, "relation": r,
                            "hard": bool(get_any(c, "hard", default=False))})
    return out


def _articulation_goals(scene: Mapping[str, Any]) -> List[Json]:
    out: List[Json] = []
    for c in as_list(get_any(scene, "constraints", default=[])):
        art = get_any(c, "articulation", default=None)
        if isinstance(art, Mapping):
            out.append({
                "articulatedObjectId": str(get_any(art, "articulatedObjectId", "articulated_object_id", default="")),
                "jointKind": enum_name(get_any(art, "jointKind", "joint_kind", default="UNKNOWN_JOINT")),
                "targetJointValue": get_any(art, "targetJointValue", "target_joint_value", default=0.0),
                "valueKind": enum_name(get_any(art, "valueKind", "value_kind", default="OPEN_FRACTION_0_TO_1")),
            })
    return out


def _classify(scene: Mapping[str, Any], goal: Mapping[str, Any]) -> str:
    rel = enum_name(get_any(goal, "relation", default="NEAR"))
    subj = str(get_any(goal, "subjectObjectId", default=""))
    # A non-grasp contact constraint on the subject overrides relation routing:
    # the demonstrated manner (touch/slide) is part of the task.
    if subj and subj in _push_constrained_objects(scene):
        return "push"
    parts = _object_parts(scene, str(get_any(goal, "objectObjectId", default="")))
    if rel in {"INSIDE", "CONTAINS"}:
        return "insert" if {"OPENING", "RIM"} & parts else "pick_place"
    if rel in {"ON_TOP_OF", "SUPPORTED_BY", "ABOVE_3D"}:
        return "place_on"
    if rel in {"NEAR", "ALIGNED_WITH"}:
        return "pick_place"
    return "push"


def _steps_for(skill: str, subj: str, target: str, rel: str) -> List[Json]:
    if skill in {"pick_place", "insert", "place_on"}:
        return [
            {"op": "move_effector_to_pregrasp", "objectId": subj},
            {"op": "close_gripper_until_contact", "objectId": subj, "contactMode": "GRASP_LIKELY"},
            {"op": "lift_object", "objectId": subj},
            {"op": "move_object_to_relation_target", "objectId": subj, "targetObjectId": target, "relation": rel},
            {"op": "open_gripper_release", "objectId": subj},
        ]
    if skill == "push":
        return [
            {"op": "move_effector_to_push_pose", "objectId": subj},
            {"op": "maintain_touching_contact", "objectId": subj, "contactMode": "TOUCHING_LIKELY"},
            {"op": "push_until_relation", "objectId": subj, "targetObjectId": target, "relation": rel},
        ]
    if skill == "open":
        return [
            {"op": "move_effector_to_handle", "objectId": subj},
            {"op": "maintain_touching_contact", "objectId": subj, "contactMode": "TOUCHING_LIKELY"},
            {"op": "move_along_articulation_axis", "objectId": subj},
        ]
    return [{"op": "preserve_observable_trace"}]


def generate_skill_skeletons(scene: Mapping[str, Any]) -> List[Json]:
    programs: List[Json] = []
    for i, g in enumerate(_relation_goals(scene)):
        subj = str(get_any(g, "subjectObjectId", default=""))
        target = str(get_any(g, "objectObjectId", default=""))
        rel = enum_name(get_any(g, "relation", default="NEAR"))
        skill = _classify(scene, g)
        programs.append({
            "programId": f"candidate_{i:03d}_{skill}_{norm_label(subj)}_{rel.lower()}",
            "skillType": skill,
            "manipulatedObjectId": subj,
            "targetObjectId": target,
            "relationGoal": rel,
            "score": 1.0 + (0.2 if bool(get_any(g, "hard", default=False)) else 0.0),
            "steps": _steps_for(skill, subj, target, rel),
        })
    base = len(programs)
    for j, g in enumerate(_articulation_goals(scene)):
        oid = str(get_any(g, "articulatedObjectId", default=""))
        programs.append({
            "programId": f"candidate_{base + j:03d}_open_{norm_label(oid)}",
            "skillType": "open",
            "manipulatedObjectId": oid,
            "targetObjectId": oid,
            "articulationGoal": dict(g),
            "score": 1.1,
            "steps": _steps_for("open", oid, oid, "ARTICULATION_GOAL"),
        })
    if not programs:
        objs = [str(get_any(b, "objectId", default="")) for b in as_list(get_any(scene, "bodies", default=[]))]
        programs.append({"programId": "candidate_000_noop", "skillType": "noop",
                         "manipulatedObjectId": objs[0] if objs else "", "score": 0.0,
                         "steps": _steps_for("noop", "", "", "")})
    programs.sort(key=lambda p: -float(get_any(p, "score", default=0.0) or 0.0))
    return programs


def choose_primary_program(programs: List[Json]) -> Json:
    if not programs:
        return {"programId": "candidate_000_noop", "skillType": "noop", "score": 0.0, "steps": [{"op": "preserve_observable_trace"}]}
    return max(programs, key=lambda p: (float(get_any(p, "score", default=0.0) or 0.0), str(get_any(p, "programId", default=""))))


def main() -> None:
    p = argparse.ArgumentParser(description="Infer candidate skill skeletons from a compiled scene.")
    p.add_argument("scene_json")
    p.add_argument("--out", default="skill_skeletons.json")
    args = p.parse_args()
    programs = generate_skill_skeletons(load_json(args.scene_json))
    write_json(args.out, {"schemaVersion": "csg.skill_skeletons.v0", "skeletons": programs})
    print(json.dumps({"out": args.out, "num_skeletons": len(programs)}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
