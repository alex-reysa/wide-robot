#!/usr/bin/env python3
"""Canonical form of a Causal Skill Graph.

``canonical_form(raw, role)`` turns a raw CSG JSON into a deterministic,
probe-ready ``CanonGraph`` that the matcher compares. This is where the audit's
"honest distance-0 is unsatisfiable" problems are fixed *once*, before any
distance is computed:

  * TaskSpec separation: for ``role="rollout"`` the planner view / target copy /
    solver metadata are stripped (and flagged for leakage tests). A rollout is
    an ObservationGraph; only a target carries a TaskSpec.
  * Converse-relation normalization: CONTAINS/SUPPORTED_BY/PARTIALLY_OCCLUDED_BY
    are rewritten to INSIDE/ON_TOP_OF/OCCLUDES so phrasing does not change the
    task class.
  * Temporal edges are *recomputed* from event time spans on both sides, so an
    honest converter that emits no temporal_edges is not penalized.
  * Confidence is a *mask* (drop facts below threshold), never a weight.
  * Deterministic ordering of every derived structure.
  * Planner goals are preserved (target only) as predicates to evaluate against
    the rollout's terminal state, instead of being string-compared planner view
    to planner view.

Camera-frame relations (LEFT_OF_IMAGE/...) and pixel-space contact evidence are
viewpoint-dependent and excluded from quotient facts by default.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Tuple

from .common import (
    Json,
    as_list,
    category_label,
    confidence,
    enum_name,
    geometry_kind,
    get_any,
    norm_label,
    ns_to_s,
    object_id,
    object_size_m,
    pose_quat,
    pose_xyz,
    span_end_s,
    span_mid_s,
    span_start_s,
)

# Relations that depend on the camera frame; excluded from the quotient.
IMAGE_FRAME_RELATIONS = {"LEFT_OF_IMAGE", "RIGHT_OF_IMAGE", "ABOVE_IMAGE", "BELOW_IMAGE"}

# Topological relations preserved as hard task structure.
TOPO_RELATIONS = {"INSIDE", "ON_TOP_OF", "OCCLUDES", "ALIGNED_WITH"}

EFFECTOR_PART_KINDS = {"LEFT_HAND", "RIGHT_HAND", "ROBOT_GRIPPER", "ROBOT_TOOL", "LEFT_ARM", "RIGHT_ARM"}

TASKSPEC_KEYS = ("plannerView", "planner_view", "targetCsg", "target_csg", "solverMetadata", "solver_metadata")


@dataclass(frozen=True)
class CanonConfig:
    relation_conf_threshold: float = 0.50
    contact_conf_threshold: float = 0.50
    event_conf_threshold: float = 0.50
    object_conf_threshold: float = 0.30
    pose_conf_threshold: float = 0.30
    planner_conf_threshold: float = 0.30
    # Two events are "ordered" only if disjoint by at least this margin (s);
    # keeps near-simultaneous events unordered and robust to time warps.
    order_margin_s: float = 1e-9
    drop_image_frame_relations: bool = True


@dataclass
class ObjectProfile:
    object_id: str
    category: str
    physical_kind: str
    geometry_kind: str
    geometry_source: str
    parts: Tuple[str, ...]
    attributes: Tuple[Tuple[str, str], ...]
    mobility: str  # may be UNKNOWN_MOBILITY for rollouts

    def hard_signature(self) -> Tuple[Any, ...]:
        """Signature that must match for two objects to be mappable. Only the
        physical kind: geometry kind, parts, mobility and visual attributes
        are estimator-dependent (a video target reports MASK_ONLY where sim
        extraction would report ORIENTED_BOX), so gating on them would make
        honest cross-domain matches impossible. They remain soft diagnostics
        (``MatchResult.diagnostics['object_profile_soft_mismatches']``)."""
        return (self.physical_kind,)


@dataclass
class RelationFact:
    t: float
    subject: str
    object: str
    relation: str


@dataclass
class ContactFact:
    t_start: float
    t_end: float
    a: str  # object id, or "EFFECTOR" / "SCENE"
    b: str
    mode: str
    relative_motion: str
    state_change_near_boundary: bool


@dataclass
class ArticulationFact:
    t: float
    object_id: str
    joint_kind: str
    value_kind: str
    value: Optional[float]


@dataclass
class EventRec:
    t_start: float
    t_end: float
    kind: str
    objects: Tuple[str, ...]
    has_agent: bool
    # Pairs of independently-normalized endpoint facts:
    # ((subj_from, obj_from, rel_from), (subj_to, obj_to, rel_to)).
    relation_transitions: Tuple[Tuple[Tuple[str, str, str], Tuple[str, str, str]], ...]
    articulation_transitions: Tuple[Tuple[str, str, str], ...]  # oid, jk, vk
    pose_delta_objects: Tuple[str, ...]


@dataclass
class PlannerGoal:
    kind: str
    hard: bool
    subject: str = ""
    object: str = ""
    relation: str = ""
    articulation_object: str = ""
    joint_kind: str = ""
    value_kind: str = ""
    target_value: Optional[float] = None


@dataclass
class CanonGraph:
    role: str
    objects: Dict[str, ObjectProfile]
    relevant_objects: List[str]
    relation_facts: List[RelationFact]
    contact_facts: List[ContactFact]
    articulation_facts: List[ArticulationFact]
    events: List[EventRec]
    planner_goals: List[PlannerGoal]
    pose_series: Dict[str, List[Tuple[float, Tuple[float, float, float], Tuple[float, float, float, float], Tuple[float, float, float]]]]
    observed_channels: Dict[str, bool]
    # Entity pairs (sorted, effector-abstracted) whose contact manner the target
    # explicitly constrains (CONTACT_MODE_CONSTRAINT goal or CONTACT_REQUIRED
    # permission). Non-empty promotes the otherwise-soft contact-word probe to
    # hard, restricted to exactly these pairs. Always empty for rollouts.
    contact_word_pairs: FrozenSet[Tuple[str, str]] = frozenset()
    leakage: Dict[str, Any] = field(default_factory=dict)

    @property
    def requires_contact_word(self) -> bool:
        return bool(self.contact_word_pairs)


# -----------------------------------------------------------------------------
# Converse-relation normalization
# -----------------------------------------------------------------------------


def normalize_relation(subject: str, obj: str, relation: str) -> Tuple[str, str, str]:
    """Rewrite converse relations to a single canonical direction."""
    rel = enum_name(relation)
    if rel == "CONTAINS":
        return obj, subject, "INSIDE"
    if rel == "SUPPORTED_BY":
        return subject, obj, "ON_TOP_OF"
    if rel == "PARTIALLY_OCCLUDED_BY":
        return obj, subject, "OCCLUDES"
    return subject, obj, rel


def _normalize_transition(subj: str, obj: str, from_rel: str, to_rel: str) -> Tuple[Tuple[str, str, str], Tuple[str, str, str]]:
    """Normalize a relation transition into two independently-canonicalized
    endpoint facts ((s, o, from_rel), (s, o, to_rel)). Each endpoint gets its
    own orientation from ``normalize_relation``: a converse *from* label
    (e.g. CONTAINS -> NEAR, "tray no longer contains cube") must not inherit
    the orientation chosen for the *to* label — coupling them inverted the
    from-fact's subject/object (V0.1 audit finding A5)."""
    return normalize_relation(subj, obj, from_rel), normalize_relation(subj, obj, to_rel)


# -----------------------------------------------------------------------------
# Entity normalization (effector abstraction)
# -----------------------------------------------------------------------------


def _agent_effector_map(raw: Json) -> Dict[str, str]:
    m: Dict[str, str] = {}
    for ap in as_list(get_any(raw, "agent_parts", "agentParts", default=[])):
        aid = str(get_any(ap, "agent_part_id", "agentPartId", default=""))
        if not aid:
            continue
        kind = enum_name(get_any(ap, "part_kind", "partKind", default="UNKNOWN_AGENT_PART"))
        m[aid] = "EFFECTOR" if kind in EFFECTOR_PART_KINDS else kind
    return m


def _entity_id(entity: Mapping[str, Any], agent_map: Mapping[str, str], object_ids: set) -> str:
    kind = enum_name(get_any(entity, "kind", default="UNKNOWN_ENTITY"))
    eid = str(get_any(entity, "id", default=""))
    if kind == "OBJECT_ENTITY" or eid in object_ids:
        return eid
    if kind in {"HUMAN_PART_ENTITY", "ROBOT_PART_ENTITY"}:
        return agent_map.get(eid, "EFFECTOR")
    if kind == "SCENE_REGION_ENTITY":
        return "SCENE"
    if eid in agent_map:
        return agent_map[eid]
    return eid or "EFFECTOR"


# -----------------------------------------------------------------------------
# Object profiles
# -----------------------------------------------------------------------------


def _object_profiles(raw: Json) -> Dict[str, ObjectProfile]:
    planner_mobility: Dict[str, str] = {}
    pv = get_any(raw, "planner_view", "plannerView", default={}) or {}
    for b in as_list(get_any(pv, "bodies", default=[])):
        oid = str(get_any(b, "object_id", "objectId", default=""))
        if oid:
            planner_mobility[oid] = enum_name(get_any(b, "mobility", default="UNKNOWN_MOBILITY"))

    out: Dict[str, ObjectProfile] = {}
    for obj in as_list(get_any(raw, "objects", default=[])):
        oid = object_id(obj)
        if not oid:
            continue
        geom = get_any(obj, "geometry", default={}) or {}
        parts = sorted({enum_name(get_any(p, "kind", "part_kind", default="UNKNOWN_PART")) for p in as_list(get_any(obj, "parts", default=[]))})
        attrs = sorted({(norm_label(get_any(a, "name", default="")), norm_label(get_any(a, "value", default=""))) for a in as_list(get_any(obj, "visual_attributes", "visualAttributes", default=[]))})
        out[oid] = ObjectProfile(
            object_id=oid,
            category=norm_label(category_label(obj)),
            physical_kind=enum_name(get_any(obj, "physical_kind", "physicalKind", default="UNKNOWN_OBJECT_KIND")),
            geometry_kind=geometry_kind(geom),
            geometry_source=enum_name(get_any(geom, "source", default="UNKNOWN_GEOMETRY_SOURCE")),
            parts=tuple(parts),
            attributes=tuple(attrs),
            mobility=planner_mobility.get(oid, "UNKNOWN_MOBILITY"),
        )
    return out


# -----------------------------------------------------------------------------
# Main entry
# -----------------------------------------------------------------------------


def canonical_form(raw: Json, role: str = "target", cfg: Optional[CanonConfig] = None) -> CanonGraph:
    cfg = cfg or CanonConfig()
    role = role.lower()
    objects = _object_profiles(raw)
    object_ids = set(objects)
    agent_map = _agent_effector_map(raw)
    relevant: set = set()

    leakage: Dict[str, Any] = {
        "had_planner_view": bool(get_any(raw, "planner_view", "plannerView", default=None)),
        "had_target_csg": bool(get_any(raw, "targetCsg", "target_csg", default=None)),
        "had_solver_metadata": bool(get_any(raw, "solverMetadata", "solver_metadata", default=None)),
        "evidence_estimators": sorted({enum_name(get_any(e, "estimator", default="")) for e in as_list(get_any(raw, "evidence", default=[]))} - {"UNKNOWN", ""}),
    }
    treat_taskspec = role == "target"

    # ---- relation states -----------------------------------------------------
    relation_facts: List[RelationFact] = []
    for r in as_list(get_any(raw, "relations", default=[])):
        if confidence(r) < cfg.relation_conf_threshold:
            continue
        subj = str(get_any(r, "subject_object_id", "subjectObjectId", default=""))
        obj = str(get_any(r, "object_object_id", "objectObjectId", default=""))
        rel = enum_name(get_any(r, "relation", default="UNKNOWN_REL"))
        if not subj or not obj:
            continue
        if cfg.drop_image_frame_relations and rel in IMAGE_FRAME_RELATIONS:
            continue
        s, o, rk = normalize_relation(subj, obj, rel)
        t = ns_to_s(get_any(r, "time_ns", "timeNs", default=0))
        relation_facts.append(RelationFact(t=t, subject=s, object=o, relation=rk))
        relevant.update([s, o])

    # ---- contacts ------------------------------------------------------------
    contact_facts: List[ContactFact] = []
    for c in as_list(get_any(raw, "contacts", default=[])):
        if confidence(c) < cfg.contact_conf_threshold:
            continue
        a = _entity_id(get_any(c, "a", default={}) or {}, agent_map, object_ids)
        b = _entity_id(get_any(c, "b", default={}) or {}, agent_map, object_ids)
        mode = enum_name(get_any(c, "mode", default="UNKNOWN_CONTACT"))
        rel_motion = enum_name(get_any(c, "relative_motion", "relativeMotion", default="UNKNOWN_RELATIVE_MOTION"))
        ce = get_any(c, "contact_evidence", "contactEvidence", default={}) or {}
        scnb = bool(get_any(ce, "state_change_near_contact_boundary", "stateChangeNearContactBoundary", default=False))
        contact_facts.append(ContactFact(span_start_s(c), span_end_s(c), a, b, mode, rel_motion, scnb))
        for ent in (a, b):
            if ent in object_ids:
                relevant.add(ent)

    # ---- articulation states (from object_states) ----------------------------
    articulation_facts: List[ArticulationFact] = []
    for st in as_list(get_any(raw, "object_states", "objectStates", default=[])):
        art = get_any(st, "articulation", default=None)
        if not isinstance(art, Mapping):
            continue
        oid = str(get_any(st, "object_id", "objectId", default="") or get_any(art, "articulated_object_id", "articulatedObjectId", default=""))
        if not oid:
            continue
        val = get_any(art, "joint_value", "jointValue", default=None)
        articulation_facts.append(ArticulationFact(
            t=ns_to_s(get_any(st, "time_ns", "timeNs", default=0)),
            object_id=oid,
            joint_kind=enum_name(get_any(art, "joint_kind", "jointKind", default="UNKNOWN_JOINT")),
            value_kind=enum_name(get_any(art, "value_kind", "valueKind", default="UNKNOWN_VALUE")),
            value=None if val is None else float(val),
        ))
        relevant.add(oid)

    # ---- events --------------------------------------------------------------
    events: List[EventRec] = []
    for e in as_list(get_any(raw, "events", default=[])):
        if confidence(e) < cfg.event_conf_threshold:
            continue
        kind = enum_name(get_any(e, "event_kind", "eventKind", default="UNKNOWN_EVENT"))
        objs = tuple(str(x) for x in as_list(get_any(e, "involved_object_ids", "involvedObjectIds", default=[])) if str(x))
        agents = as_list(get_any(e, "involved_agent_part_ids", "involvedAgentPartIds", default=[]))
        rtrans: List[Tuple[Tuple[str, str, str], Tuple[str, str, str]]] = []
        atrans: List[Tuple[str, str, str]] = []
        pose_delta_objs: List[str] = []
        for d in as_list(get_any(e, "observed_deltas", "observedDeltas", default=[])):
            if confidence(d, default=confidence(e)) < cfg.event_conf_threshold:
                continue
            rt = get_any(d, "relation_transition", "relationTransition", default=None)
            if isinstance(rt, Mapping):
                subj = str(get_any(rt, "subject_object_id", "subjectObjectId", default=""))
                obj = str(get_any(rt, "object_object_id", "objectObjectId", default=""))
                if subj and obj:
                    f_fact, t_fact = _normalize_transition(subj, obj, get_any(rt, "from_relation", "fromRelation", default="UNKNOWN_REL"), get_any(rt, "to_relation", "toRelation", default="UNKNOWN_REL"))
                    if not (cfg.drop_image_frame_relations and (f_fact[2] in IMAGE_FRAME_RELATIONS or t_fact[2] in IMAGE_FRAME_RELATIONS)):
                        rtrans.append((f_fact, t_fact))
                        relevant.update([f_fact[0], f_fact[1], t_fact[0], t_fact[1]])
            at = get_any(d, "articulation_transition", "articulationTransition", default=None)
            if isinstance(at, Mapping):
                to_state = get_any(at, "to_state", "toState", default={}) or {}
                from_state = get_any(at, "from_state", "fromState", default={}) or {}
                oid = str(get_any(at, "articulated_object_id", "articulatedObjectId", default="") or get_any(to_state, "articulated_object_id", "articulatedObjectId", default=""))
                if oid:
                    jk = enum_name(get_any(to_state, "joint_kind", "jointKind", default="UNKNOWN_JOINT"))
                    vk = enum_name(get_any(to_state, "value_kind", "valueKind", default="UNKNOWN_VALUE"))
                    atrans.append((oid, jk, vk))
                    relevant.add(oid)
                    for s_ in (from_state, to_state):
                        v = get_any(s_, "joint_value", "jointValue", default=None)
                        if v is not None:
                            articulation_facts.append(ArticulationFact(span_mid_s(e), oid, jk, vk, float(v)))
            doid = str(get_any(d, "object_id", "objectId", default=""))
            pd = get_any(d, "pose_delta_3d", "poseDelta3d", "poseDelta3D", default=None)
            if doid and isinstance(pd, Mapping):
                pose_delta_objs.append(doid)
                relevant.add(doid)
        events.append(EventRec(
            t_start=span_start_s(e),
            t_end=span_end_s(e),
            kind=kind,
            objects=tuple(o for o in objs if o in object_ids),
            has_agent=bool(agents),
            relation_transitions=tuple(rtrans),
            articulation_transitions=tuple(atrans),
            pose_delta_objects=tuple(pose_delta_objs),
        ))
        relevant.update(o for o in objs if o in object_ids)

    # ---- planner goals (target role only) ------------------------------------
    planner_goals: List[PlannerGoal] = []
    contact_word_pairs: set = set()

    def _constrained_pair(a_ent: Any, b_ent: Any) -> Optional[Tuple[str, str]]:
        if not isinstance(a_ent, Mapping) or not isinstance(b_ent, Mapping):
            return None
        a = _entity_id(a_ent, agent_map, object_ids)
        b = _entity_id(b_ent, agent_map, object_ids)
        if not a or not b:
            return None
        return tuple(sorted((a, b)))  # type: ignore[return-value]

    if treat_taskspec:
        pv = get_any(raw, "planner_view", "plannerView", default={}) or {}
        for stage in as_list(get_any(pv, "stages", default=[])):
            if confidence(stage, default=1.0) < cfg.planner_conf_threshold:
                continue
            for grp in ("preconditions", "path_constraints", "pathConstraints", "goal_constraints", "goalConstraints"):
                for c in as_list(get_any(stage, grp, default=[])):
                    if enum_name(get_any(c, "kind", default="")) == "CONTACT_MODE_CONSTRAINT":
                        cc = get_any(c, "contact", default={}) or {}
                        pair = _constrained_pair(get_any(cc, "a", default=None), get_any(cc, "b", default=None))
                        if pair:
                            contact_word_pairs.add(pair)
            for cp in as_list(get_any(stage, "contact_permissions", "contactPermissions", default=[])):
                if enum_name(get_any(cp, "permission", default="")) == "CONTACT_REQUIRED":
                    pair = _constrained_pair(get_any(cp, "a", default=None), get_any(cp, "b", default=None))
                    if pair:
                        contact_word_pairs.add(pair)
            for c in as_list(get_any(stage, "goal_constraints", "goalConstraints", default=[])):
                if confidence(c, default=1.0) < cfg.planner_conf_threshold:
                    continue
                hard = bool(get_any(c, "hard", default=False))
                kind = enum_name(get_any(c, "kind", default="UNKNOWN_CONSTRAINT"))
                rel = get_any(c, "relation", default=None)
                if isinstance(rel, Mapping):
                    subj = str(get_any(rel, "subject_object_id", "subjectObjectId", default=""))
                    obj = str(get_any(rel, "object_object_id", "objectObjectId", default=""))
                    desired = enum_name(get_any(rel, "desired_relation", "desiredRelation", default="UNKNOWN_REL"))
                    s, o, rk = normalize_relation(subj, obj, desired)
                    if s and o:
                        planner_goals.append(PlannerGoal(kind=kind, hard=hard, subject=s, object=o, relation=rk))
                        relevant.update([s, o])
                    continue
                art = get_any(c, "articulation", default=None)
                if isinstance(art, Mapping):
                    oid = str(get_any(art, "articulated_object_id", "articulatedObjectId", default=""))
                    tv = get_any(art, "target_joint_value", "targetJointValue", default=None)
                    if oid:
                        planner_goals.append(PlannerGoal(
                            kind=kind, hard=hard, articulation_object=oid,
                            joint_kind=enum_name(get_any(art, "joint_kind", "jointKind", default="UNKNOWN_JOINT")),
                            value_kind=enum_name(get_any(art, "value_kind", "valueKind", default="UNKNOWN_VALUE")),
                            target_value=None if tv is None else float(tv),
                        ))
                        relevant.add(oid)

    # ---- pose series ---------------------------------------------------------
    pose_series: Dict[str, List[Tuple[float, Tuple[float, float, float], Tuple[float, float, float, float], Tuple[float, float, float]]]] = {}
    sizes = {oid: object_size_m(obj) for obj in as_list(get_any(raw, "objects", default=[])) if (oid := object_id(obj))}
    observed_pose = False
    for st in as_list(get_any(raw, "object_states", "objectStates", default=[])):
        oid = str(get_any(st, "object_id", "objectId", default=""))
        pose = get_any(st, "pose_3d", "pose3d", "pose3D", default=None)
        if not oid or not isinstance(pose, Mapping):
            continue
        if confidence(pose, default=confidence(st)) < cfg.pose_conf_threshold:
            continue
        observed_pose = True
        t = ns_to_s(get_any(st, "time_ns", "timeNs", default=0))
        pose_series.setdefault(oid, []).append((t, pose_xyz(pose), pose_quat(pose), sizes.get(oid, (0.04, 0.04, 0.04))))
        relevant.add(oid)
    for oid in pose_series:
        pose_series[oid].sort(key=lambda x: x[0])

    relevant = {oid for oid in relevant if oid in object_ids}
    if not relevant:
        relevant = set(object_ids)

    observed_channels = {
        "relations": bool(relation_facts),
        "contacts": bool(contact_facts),
        "events": bool(events),
        "articulations": bool(articulation_facts),
        "poses": observed_pose,
    }

    # Deterministic ordering everywhere.
    relation_facts.sort(key=lambda f: (f.t, f.subject, f.object, f.relation))
    contact_facts.sort(key=lambda f: (f.t_start, f.t_end, f.a, f.b, f.mode))
    articulation_facts.sort(key=lambda f: (f.t, f.object_id, f.value_kind))
    events.sort(key=lambda e: (e.t_start, e.t_end, e.kind, e.objects))

    return CanonGraph(
        role=role,
        objects=objects,
        relevant_objects=sorted(relevant),
        relation_facts=relation_facts,
        contact_facts=contact_facts,
        articulation_facts=articulation_facts,
        events=events,
        planner_goals=planner_goals,
        pose_series=pose_series,
        observed_channels=observed_channels,
        contact_word_pairs=frozenset(contact_word_pairs),
        leakage=leakage,
    )
