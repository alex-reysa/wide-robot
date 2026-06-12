#!/usr/bin/env python3
"""Independent rollout extractor: rollout frames -> robot CSG.

This is the anti-leakage heart of the loop. It derives the robot's observation
graph ONLY from:
  * ``rollout["frames"]``  (effector + object poses, gripper state, articulation)
  * ``rollout["sceneBodies"]``  (object identity + geometry the sim instantiated)

It NEVER reads ``targetCsg`` (the rollout no longer carries one) nor any planner
view. Relations and contacts are decided by the shared ``predicates`` registry,
so the words it emits are drawn from the same grammar a future perception
compiler would use. All evidence is tagged ``SIM_STATE_EXTRACTION``.

The emitted graph is an ObservationGraph: objects, object states, relations,
contacts, events, evidence. No plannerView, no task spec.
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Any, Dict, List, Mapping, Optional, Tuple

from . import predicates as P
from .common import Json, ROBOT_GRIPPER_ID, as_list, enum_name, get_any, load_json, pose_xyz, s_to_ns, write_json

Vec3 = Tuple[float, float, float]

MOTION_EPS_M = 0.005
MIN_PERSIST_FRAMES = 2
EVIDENCE_ID = "ev_sim_state_extraction"


def _bodies(rollout: Mapping[str, Any]) -> Dict[str, Json]:
    out: Dict[str, Json] = {}
    for b in as_list(get_any(rollout, "sceneBodies", "scene_bodies", default=[])):
        oid = str(get_any(b, "objectId", "bodyId", default=""))
        if oid:
            out[oid] = dict(b)
    return out


def _size(body: Mapping[str, Any]) -> Vec3:
    s = list(get_any(body, "sizeM", "size_m", default=[0.04, 0.04, 0.04])) + [0.04, 0.04, 0.04]
    return (float(s[0]), float(s[1]), float(s[2]))


def _box(center: Vec3, size: Vec3) -> P.Box:
    return P.box_from(center, size)


def _frame_objects(frame: Mapping[str, Any]) -> Dict[str, Vec3]:
    out: Dict[str, Vec3] = {}
    poses = get_any(frame, "objectPoses", "object_poses", default={}) or {}
    if isinstance(poses, Mapping):
        for oid, pose in poses.items():
            out[str(oid)] = pose_xyz(pose)
    return out


def _figure_ground(frames: List[Mapping[str, Any]], bodies: Dict[str, Json]) -> List[Tuple[str, str]]:
    """Pick (figure, ground) ordered pairs to report relations for: each object
    that moved, paired with the object it ends structurally related to (else its
    nearest neighbour). Keeps relation facts directional and minimal, matching
    how a target is authored (mover -> reference)."""
    if not frames:
        return []
    first = _frame_objects(frames[0])
    last = _frame_objects(frames[-1])
    oids = [o for o in bodies if o in last]
    moved = [o for o in oids if o in first and math.dist(first[o], last[o]) > MOTION_EPS_M]
    pairs: List[Tuple[str, str]] = []
    for fo in moved:
        fbox = _box(last[fo], _size(bodies[fo]))
        ground = None
        for go in oids:
            if go == fo:
                continue
            gbox = _box(last[go], _size(bodies[go]))
            if P.primary_topo_relation(fbox, gbox) is not None:
                ground = go
                break
        if ground is None:
            others = [o for o in oids if o != fo]
            if others:
                ground = min(others, key=lambda o: math.dist(last[fo], last[o]))
        if ground:
            pairs.append((fo, ground))
    return pairs


def _relation_state(subj: str, obj: str, rel: str, t: float, rid: str) -> Json:
    return {
        "relationId": rid, "timeNs": s_to_ns(t),
        "subjectObjectId": subj, "objectObjectId": obj, "relation": rel,
        "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
    }


def _primary_relation(a: Vec3, asz: Vec3, b: Vec3, bsz: Vec3) -> str:
    ab, bb = _box(a, asz), _box(b, bsz)
    topo = P.primary_topo_relation(ab, bb)
    if topo:
        return topo
    return "NEAR" if P.is_near(ab, bb) else "FAR_FROM"


def extract_robot_csg(rollout: Mapping[str, Any], graph_id: Optional[str] = None) -> Json:
    bodies = _bodies(rollout)
    frames = [f for f in as_list(get_any(rollout, "frames", default=[])) if isinstance(f, Mapping)]
    eff_id = str(get_any(rollout, "robotEffectorId", "robot_effector_id", default=ROBOT_GRIPPER_ID))

    objects: List[Json] = []
    for b in bodies.values():
        # Neutral metadata only: the rollout's sanitized bodies carry no
        # target-authored text (labels, geometry notes, part labels), and the
        # extractor must not re-introduce any (audit A4).
        objects.append({
            "objectId": str(get_any(b, "objectId", "bodyId", default="")),
            "categoryLabel": "object",
            "categoryConfidence": 1.0,
            "physicalKind": get_any(b, "physicalKind", "physical_kind", default="UNKNOWN_OBJECT_KIND"),
            "evidenceIds": [EVIDENCE_ID],
        })

    object_states: List[Json] = []
    relations: List[Json] = []
    contacts: List[Json] = []
    events: List[Json] = []

    times = [float(get_any(f, "timeS", "time_s", default=i)) for i, f in enumerate(frames)]

    # ---- object states (pose per frame) + articulation ----------------------
    for i, f in enumerate(frames):
        objs = _frame_objects(f)
        art = get_any(f, "articulation", default={}) or {}
        for oid, xyz in objs.items():
            st: Json = {
                "stateId": f"st_{oid}_{i}", "objectId": oid, "timeNs": s_to_ns(times[i]),
                "pose3D": {"frameId": "world", "positionM": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
                           "orientationWxyz": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, "confidence": 1.0},
                "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
            }
            if isinstance(art, Mapping) and oid in art:
                body = bodies.get(oid, {})
                a0 = get_any(body, "articulation", default={}) or {}
                st["articulation"] = {
                    "articulatedObjectId": oid,
                    "jointKind": enum_name(get_any(a0, "jointKind", "joint_kind", default="PRISMATIC")),
                    "jointValue": float(art[oid]),
                    "valueKind": enum_name(get_any(a0, "valueKind", "value_kind", default="EXTENSION_M")),
                    "confidence": 1.0,
                }
            object_states.append(st)

    # ---- relations at first / last frame for figure-ground pairs -------------
    pairs = _figure_ground(frames, bodies)
    if frames:
        for endpoint, idx in (("first", 0), ("last", len(frames) - 1)):
            objs = _frame_objects(frames[idx])
            for (fo, go) in pairs:
                if fo in objs and go in objs:
                    rel = _primary_relation(objs[fo], _size(bodies[fo]), objs[go], _size(bodies[go]))
                    relations.append(_relation_state(fo, go, rel, times[idx], f"rel_{fo}_{go}_{endpoint}"))

    # Objects the gripper actually interacted with: those that moved, OR whose
    # articulation changed (handle grasp on a body whose pose is static). A
    # static object the closed gripper merely passes near (e.g. the tray) is NOT
    # interacted with.
    moved_objects = set()
    articulated_changed = set()
    if frames:
        first_objs = _frame_objects(frames[0])
        last_objs = _frame_objects(frames[-1])
        for oid in bodies:
            if oid in first_objs and oid in last_objs and math.dist(first_objs[oid], last_objs[oid]) > MOTION_EPS_M:
                moved_objects.add(oid)
        for oid in bodies:
            vals = [float(get_any(f, "articulation", default={}).get(oid)) for f in frames if isinstance(get_any(f, "articulation", default={}), Mapping) and oid in (get_any(f, "articulation", default={}) or {})]
            if len(vals) >= 2 and abs(vals[-1] - vals[0]) > 1e-6:
                articulated_changed.add(oid)
    interacted_objects = moved_objects | articulated_changed

    # ---- contact intervals -> contacts + CONTACT_BEGIN / co-motion / release -
    # Two contact manners, decided per interacted object:
    #   * grasp: gripper closed AND effector within grasp reach;
    #   * push (non-grasp): effector at the object surface (touching gap) with
    #     gripper open, accepted only when the object actually co-moves with
    #     the effector (>= PredConfig.co_motion_corr) — a static brush past an
    #     object is not a manipulation contact.
    # Grasp takes precedence: pick-place approach/release phases pass through
    # touching poses with the gripper open and must not emit spurious pushes.
    for oid in bodies:
        if oid not in interacted_objects:
            continue
        grasped: List[int] = []
        touched: List[int] = []
        for i, f in enumerate(frames):
            objs = _frame_objects(f)
            if oid not in objs:
                continue
            closed = bool(get_any(f, "gripperClosed", "gripper_closed", default=False))
            eff = pose_xyz(get_any(f, "effectorPose", "gripperPose", default=None))
            box = _box(objs[oid], _size(bodies[oid]))
            if closed and P.effector_reaches(eff, box):
                grasped.append(i)
            elif P.effector_touches(eff, box):
                touched.append(i)

        if grasped:
            start, end = grasped[0], grasped[-1]
            mode, rel_motion = "GRASP_LIKELY", "STICKING_LIKELY"
        elif touched:
            start, end = touched[0], touched[-1]
            mode, rel_motion = "TOUCHING_LIKELY", "SLIDING_LIKELY"
        else:
            continue
        t_start, t_end = times[start], times[end]

        # Object motion over the contact interval (co-motion correlation).
        eff_traj = [pose_xyz(get_any(frames[i], "effectorPose", default=None)) for i in range(start, end + 1)]
        obj_traj = [_frame_objects(frames[i]).get(oid, (0, 0, 0)) for i in range(start, end + 1)]
        comotion = P.co_motion_correlation(eff_traj, obj_traj)
        moved = math.dist(obj_traj[0], obj_traj[-1]) > MOTION_EPS_M if obj_traj else False
        scnb = moved  # object started/stopped moving within the contact boundary

        if not grasped and (comotion < P.DEFAULT.co_motion_corr or not moved):
            continue  # incidental touch, not a manipulation contact

        contacts.append({
            "contactId": f"contact_{eff_id}_{oid}",
            "a": {"kind": "ROBOT_PART_ENTITY", "id": eff_id},
            "b": {"kind": "OBJECT_ENTITY", "id": oid},
            "timeSpan": {"startTimeNs": s_to_ns(t_start), "endTimeNs": s_to_ns(t_end)},
            "mode": mode,
            "relativeMotion": rel_motion,
            "contactEvidence": {"motionCorrelation": comotion, "stateChangeNearContactBoundary": scnb},
            "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
        })

        events.append({
            "eventId": f"ev_contact_begin_{oid}", "eventKind": "CONTACT_BEGIN",
            "timeSpan": {"startTimeNs": s_to_ns(t_start), "endTimeNs": s_to_ns(t_start)},
            "involvedObjectIds": [oid], "involvedAgentPartIds": [eff_id], "contactIds": [f"contact_{eff_id}_{oid}"],
            "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
        })
        if grasped:
            # Only a grasp has a release; a push contact just ends.
            release_t = times[min(end + 1, len(times) - 1)]
            events.append({
                "eventId": f"ev_release_{oid}", "eventKind": "RELEASE_INFERRED",
                "timeSpan": {"startTimeNs": s_to_ns(release_t), "endTimeNs": s_to_ns(release_t)},
                "involvedObjectIds": [oid], "involvedAgentPartIds": [eff_id],
                "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
            })

        # Stash for co-motion event after we know containment time.
        bodies[oid]["_graspInterval"] = (start, end, comotion, moved)  # type: ignore[index]

    # ---- structural relation changes (containment / support / generic) ------
    def structural_runs(fo: str, go: str) -> List[Tuple[int, str]]:
        seq: List[Optional[str]] = []
        for f in frames:
            objs = _frame_objects(f)
            if fo in objs and go in objs:
                seq.append(P.primary_topo_relation(_box(objs[fo], _size(bodies[fo])), _box(objs[go], _size(bodies[go]))))
            else:
                seq.append(None)
        runs: List[Tuple[int, str]] = []
        i = 0
        while i < len(seq):
            if seq[i] is None:
                i += 1
                continue
            j = i
            while j < len(seq) and seq[j] == seq[i]:
                j += 1
            run_len = j - i
            if run_len >= MIN_PERSIST_FRAMES or j == len(seq):
                runs.append((i, seq[i]))  # type: ignore[arg-type]
            i = j
        return runs

    for (fo, go) in pairs:
        runs = structural_runs(fo, go)
        # Collapse consecutive duplicate structural relations.
        deduped: List[Tuple[int, str]] = []
        for fi, rel in runs:
            if not deduped or deduped[-1][1] != rel:
                deduped.append((fi, rel))
        prev_rel = "NEAR"
        for (fi, rel) in deduped:
            t = times[fi]
            kind = {"INSIDE": "CONTAINMENT_CHANGE", "ON_TOP_OF": "SUPPORT_CHANGE"}.get(rel, "RELATION_CHANGE")
            events.append({
                "eventId": f"ev_{kind.lower()}_{fo}_{go}_{fi}", "eventKind": kind,
                "timeSpan": {"startTimeNs": s_to_ns(t), "endTimeNs": s_to_ns(t)},
                "involvedObjectIds": [fo, go],
                "observedDeltas": [{"objectId": fo, "confidence": 1.0,
                                    "relationTransition": {"subjectObjectId": fo, "objectObjectId": go,
                                                           "fromRelation": prev_rel, "toRelation": rel}}],
                "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
            })
            prev_rel = rel
        # Co-motion event: from grasp start to just before the first structural change.
        gi = bodies.get(fo, {}).get("_graspInterval")  # type: ignore[union-attr]
        if gi and gi[3]:  # object moved while grasped
            start, end, comotion, _ = gi
            first_struct_fi = deduped[0][0] if deduped else end
            # Co-motion starts strictly AFTER contact is established (so
            # CONTACT_BEGIN precedes it) and ends before the structural change.
            co_start_idx = min(start + 1, len(times) - 1)
            co_end_idx = max(co_start_idx, min(first_struct_fi - 1, end))
            if co_end_idx > co_start_idx:
                events.append({
                    "eventId": f"ev_co_motion_{fo}", "eventKind": "HAND_OBJECT_CO_MOTION",
                    "timeSpan": {"startTimeNs": s_to_ns(times[co_start_idx]), "endTimeNs": s_to_ns(times[co_end_idx])},
                    "involvedObjectIds": [fo], "involvedAgentPartIds": [eff_id],
                    "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
                })

    # ---- articulation change events -----------------------------------------
    for oid, body in bodies.items():
        vals = []
        for i, f in enumerate(frames):
            art = get_any(f, "articulation", default={}) or {}
            if isinstance(art, Mapping) and oid in art:
                vals.append((times[i], float(art[oid])))
        if len(vals) >= 2 and abs(vals[-1][1] - vals[0][1]) > 1e-6:
            a0 = get_any(body, "articulation", default={}) or {}
            jk = enum_name(get_any(a0, "jointKind", "joint_kind", default="PRISMATIC"))
            vk = enum_name(get_any(a0, "valueKind", "value_kind", default="EXTENSION_M"))
            v0, vlast = vals[0][1], vals[-1][1]
            # Span only the interval where the value actually changes (last frame
            # still at the start value -> first frame at the final value), so the
            # event does not engulf the preceding CONTACT_BEGIN.
            change_start = max((t for t, v in vals if abs(v - v0) <= 1e-6), default=vals[0][0])
            change_end = min((t for t, v in vals if abs(v - vlast) <= 1e-6), default=vals[-1][0])
            if change_end < change_start:
                change_start, change_end = vals[0][0], vals[-1][0]
            events.append({
                "eventId": f"ev_articulation_change_{oid}", "eventKind": "ARTICULATION_CHANGE",
                "timeSpan": {"startTimeNs": s_to_ns(change_start), "endTimeNs": s_to_ns(change_end)},
                "involvedObjectIds": [oid],
                "observedDeltas": [{"objectId": oid, "confidence": 1.0, "articulationTransition": {
                    "articulatedObjectId": oid,
                    "fromState": {"articulatedObjectId": oid, "jointKind": jk, "jointValue": vals[0][1], "valueKind": vk},
                    "toState": {"articulatedObjectId": oid, "jointKind": jk, "jointValue": vals[-1][1], "valueKind": vk}}}],
                "confidence": 1.0, "evidenceIds": [EVIDENCE_ID],
            })

    # Clean stash keys.
    for b in bodies.values():
        b.pop("_graspInterval", None)

    return {
        "schemaVersion": "csg.v0",
        "graphId": graph_id or f"robot_{get_any(rollout, 'backend', default='sim')}_rollout",
        "objects": objects,
        "agentParts": [{"agentPartId": eff_id, "agentKind": "ROBOT", "partKind": "ROBOT_GRIPPER", "label": eff_id}],
        "objectStates": object_states,
        "relations": relations,
        "contacts": contacts,
        "events": events,
        "evidence": [{
            "evidenceId": EVIDENCE_ID, "estimator": "SIM_STATE_EXTRACTION",
            "modelName": "csg.rollout_extract", "modelVersion": P.PREDICATES_VERSION,
        }],
        "extractionMetadata": {
            "source": "rollout_frames", "predicatesVersion": P.PREDICATES_VERSION,
            "numFrames": len(frames), "readTargetCsg": False,
        },
    }


# Back-compat alias.
def rollout_to_csg(rollout: Mapping[str, Any], **_kw: Any) -> Json:
    return extract_robot_csg(rollout)


def main() -> None:
    p = argparse.ArgumentParser(description="Extract a robot CSG from a rollout (frames only, no leakage).")
    p.add_argument("rollout_json")
    p.add_argument("--out", default="robot_csg.json")
    args = p.parse_args()
    csg = extract_robot_csg(load_json(args.rollout_json))
    write_json(args.out, csg)
    print(json.dumps({"robot_csg": args.out, "objects": len(csg["objects"]), "events": len(csg["events"])}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
