#!/usr/bin/env python3
"""Structural-equivalence proof for the per-source ``object_inside_container`` target cards.

The four worlds are NOT verified against one identical target file — each pilot ships its
own card (different ``graphId``, object labels, geometry provenance, captions). The honest
cross-source claim is therefore *"the same SEMANTIC task graph, instantiated per source"*,
not *"one shared target file"*. This module proves it: it reduces each card to the
**verifier-enforced semantic core** (all hard goal constraints, the authored relation
endpoints, the containment event, contacts, and stage structure — plus the object roles, with
``physicalKind`` matcher-enforced and ``mobility`` carried as a *stricter-than-verifier*
discriminant) — discarding every authoring-only field the matcher never reads (``graphId``,
``taskCaption``, ``categoryLabel``, geometry sizes/sources, confidences, ``timeNs`` magnitudes,
and all object/relation/event ids, which are canonicalised to roles) — and shows the cores are
byte-identical across sources per tier.

The signature is deliberately STRICTER than the matcher (it reads everything the matcher gates
on, plus mobility, which the matcher treats only as a soft diagnostic): a stricter signature can
only ever produce a FALSE NON-equivalence, never a false equivalence. As belt-and-suspenders,
``build_target_equivalence`` also asserts each audited card stays within the simplifying
assumptions the signature fully models (single stage, single hard relation goal, no contacts, no
articulation goal) — so the proof fails loudly the moment a future card outgrows it, rather than
silently laundering a real difference.

Nothing here imports ``csg`` — it is a pure structural normaliser over the target JSON, so it
cannot accidentally launder a real semantic difference through the verifier.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

REPO = Path(__file__).resolve().parents[2]

# Per-tier source cards. terminal_only + relation_event exist for all three external
# pilots; placed_from_outside (the FAR-start sibling) exists only where a far-start was
# actually captured (rlbench far-start drawers, real-camera far-start placements).
PILOT_TARGET_DIRS = {
    "rlbench": REPO / "pilots" / "rlbench" / "targets",
    "real_camera": REPO / "pilots" / "real_camera" / "targets",
    "rh20t": REPO / "pilots" / "rh20t" / "targets",
}
TIERS = ("terminal_only", "relation_event", "placed_from_outside")
TIER_FILE = {t: f"object_inside_container_{t}.json" for t in TIERS}

# The internal-sim world is judged against its native gold pick-place target, a SUPERSET
# of the relation tiers (it adds contact / carry / temporal-order probes). We show its
# INSIDE goal core matches the shared core — it strengthens, never weakens, the claim.
MUJOCO_GOLD_TARGET = REPO / "gold_tests" / "put_cube_in_tray" / "target.json"


def _goal_constraint(target: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """The first OBJECT_RELATION_GOAL constraint in the plannerView (the enforced goal)."""
    for stage in (target.get("plannerView", {}) or {}).get("stages", []) or []:
        for gc in stage.get("goalConstraints", []) or []:
            if gc.get("kind") == "OBJECT_RELATION_GOAL":
                return gc
    return None


def _role_map(target: Mapping[str, Any]) -> Dict[str, str]:
    """Canonicalise object ids → roles. The goal's subject/object define __SUBJECT__/__OBJECT__;
    any other object ids get stable ordinal roles (there are only ever two here)."""
    gc = _goal_constraint(target)
    roles: Dict[str, str] = {}
    if gc and isinstance(gc.get("relation"), Mapping):
        rel = gc["relation"]
        if rel.get("subjectObjectId"):
            roles[str(rel["subjectObjectId"])] = "__SUBJECT__"
        if rel.get("objectObjectId"):
            roles[str(rel["objectObjectId"])] = "__OBJECT__"
    n = 0
    for obj in target.get("objects", []) or []:
        oid = str(obj.get("objectId"))
        if oid not in roles:
            roles[oid] = f"__OBJ{n}__"
            n += 1
    return roles


def _all_goal_constraints(target: Mapping[str, Any]):
    """Every goalConstraint across every stage, with the stage index, in document order."""
    out = []
    for si, stage in enumerate((target.get("plannerView", {}) or {}).get("stages", []) or []):
        for gc in stage.get("goalConstraints", []) or []:
            out.append((si, gc))
    return out


def canonical_signature(target: Mapping[str, Any]) -> Dict[str, Any]:
    """Reduce a target card to its verifier-enforced semantic core, with all ids → roles
    and every authoring-only field stripped. Two cards with the same signature enforce the
    same task semantics regardless of labels/geometry/provenance.

    Reads everything the frozen matcher gates on — ALL hard goal constraints (relation +
    articulation) across ALL stages, the authored relation endpoints, the containment event(s),
    contacts (+ whether any carries a contact word), and the stage count — so it cannot
    over-strip a real difference. ``mobility`` is also retained, as a stricter-than-verifier
    discriminant (the matcher's ``hard_signature`` gates on ``physicalKind`` only, treating
    mobility as a soft diagnostic); including it can only cause a false non-equivalence, never a
    false equivalence."""
    roles = _role_map(target)

    def role(oid: Any) -> str:
        return roles.get(str(oid), f"__UNMAPPED:{oid}__")

    # Object roles: physicalKind (matcher-enforced) + mobility (stricter-than-verifier discriminant).
    mobility = {str(b.get("objectId")): b.get("mobility")
                for b in (target.get("plannerView", {}) or {}).get("bodies", []) or []}
    objects = {}
    for obj in target.get("objects", []) or []:
        oid = str(obj.get("objectId"))
        objects[role(oid)] = {"physicalKind": obj.get("physicalKind"), "mobility": mobility.get(oid)}

    # ALL goal constraints (not just the first), normalised + sorted; relation AND articulation.
    goals = []
    for si, gc in _all_goal_constraints(target):
        rel = gc.get("relation", {}) or {}
        art = gc.get("articulation", {}) or gc.get("articulationGoal", {}) or {}
        goals.append({
            "stage": si,
            "kind": gc.get("kind"),
            "hard": bool(gc.get("hard")),
            "subject": role(rel.get("subjectObjectId")) if rel.get("subjectObjectId") else None,
            "object": role(rel.get("objectObjectId")) if rel.get("objectObjectId") else None,
            "desiredRelation": rel.get("desiredRelation"),
            "articulation": {k: art.get(k) for k in sorted(art)} if art else None,
        })
    goals.sort(key=lambda g: json.dumps(g, sort_keys=True))

    # The primary relation goal (kept for goal_core + tier_strengthening narrative).
    gc0 = _goal_constraint(target) or {}
    grel = gc0.get("relation", {}) or {}
    goal = {
        "kind": gc0.get("kind"),
        "hard": bool(gc0.get("hard")),
        "subject": role(grel.get("subjectObjectId")),
        "object": role(grel.get("objectObjectId")),
        "desiredRelation": grel.get("desiredRelation"),
    }

    # Authored relation endpoints, ordered by time (magnitudes discarded → ordinal rank).
    rels = sorted((target.get("relations", []) or []), key=lambda r: int(r.get("timeNs", 0)))
    relations = [{
        "order": i,
        "subject": role(r.get("subjectObjectId")),
        "object": role(r.get("objectObjectId")),
        "relation": r.get("relation"),
    } for i, r in enumerate(rels)]

    # Containment events: only the enforced shape (kind + relation transition), not ids/spans/conf.
    events = []
    for ev in target.get("events", []) or []:
        transitions = []
        for d in ev.get("observedDeltas", []) or []:
            rt = d.get("relationTransition") or {}
            transitions.append({
                "subject": role(rt.get("subjectObjectId")),
                "object": role(rt.get("objectObjectId")),
                "from": rt.get("fromRelation"),
                "to": rt.get("toRelation"),
            })
        events.append({"eventKind": ev.get("eventKind"),
                       "transitions": sorted(transitions, key=lambda t: json.dumps(t, sort_keys=True))})
    events.sort(key=lambda e: json.dumps(e, sort_keys=True))

    # Contacts (the gold pick-place superset has them; the relation tiers do not) + contact-word flag.
    contacts = []
    requires_contact_word = False
    for c in target.get("contacts", []) or []:
        pair = sorted(role(o) for o in (c.get("objectIds") or
                      [c.get("subjectObjectId"), c.get("objectObjectId")]) if o)
        word = c.get("contactWord") or c.get("word")
        if word:
            requires_contact_word = True
        contacts.append({"pair": pair, "hasWord": bool(word)})
    contacts.sort(key=lambda c: json.dumps(c, sort_keys=True))

    return {
        "objects": objects,
        "goal": goal,
        "goals": goals,
        "stageCount": len((target.get("plannerView", {}) or {}).get("stages", []) or []),
        "contacts": contacts,
        "requiresContactWord": requires_contact_word,
        "relations": relations,
        "events": events,
    }


def _sig_key(sig: Mapping[str, Any]) -> str:
    return json.dumps(sig, sort_keys=True)


def goal_core(sig: Mapping[str, Any]) -> Dict[str, Any]:
    """Just the INSIDE goal + object roles/mobilities — the part EVERY tier and even the
    MuJoCo gold superset share. Used to tie the internal-sim world into the equivalence."""
    return {"objects": sig["objects"], "goal": sig["goal"]}


def load_target(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def tier_equivalence(tier: str) -> Dict[str, Any]:
    """Compare the canonical signatures of every source that ships this tier."""
    fname = TIER_FILE[tier]
    sigs: Dict[str, Dict[str, Any]] = {}
    for src, d in PILOT_TARGET_DIRS.items():
        p = d / fname
        if p.exists():
            sigs[src] = canonical_signature(load_target(p))
    keys = {src: _sig_key(s) for src, s in sigs.items()}
    distinct = sorted(set(keys.values()))
    all_identical = len(distinct) <= 1
    # Pairwise field-level diffs (empty when identical) for honest failure reporting.
    diffs: List[Dict[str, Any]] = []
    srcs = sorted(sigs)
    for i in range(len(srcs)):
        for j in range(i + 1, len(srcs)):
            a, b = srcs[i], srcs[j]
            if keys[a] != keys[b]:
                diffs.append({"a": a, "b": b,
                              "aSig": sigs[a], "bSig": sigs[b]})
    return {
        "tier": tier,
        "sources": srcs,
        "allIdentical": all_identical,
        "sharedSignature": sigs[srcs[0]] if (all_identical and srcs) else None,
        "distinctSignatureCount": len(distinct),
        "pairwiseDiffs": diffs,
    }


def mujoco_goal_core_matches() -> Dict[str, Any]:
    """The MuJoCo gold pick-place target is a SUPERSET (adds contact/carry/order probes),
    so we don't claim full-signature equivalence — only that its INSIDE goal core matches
    the shared relation-tier core. MuJoCo is judged at a STRONGER tier, not a weaker one."""
    gold_sig = canonical_signature(load_target(MUJOCO_GOLD_TARGET))
    # reference core = the terminal_only shared core (any external source; they're identical)
    ref = canonical_signature(load_target(PILOT_TARGET_DIRS["real_camera"] / TIER_FILE["terminal_only"]))
    gold_core, ref_core = goal_core(gold_sig), goal_core(ref)
    return {
        "mujocoGoldGraphId": load_target(MUJOCO_GOLD_TARGET).get("graphId"),
        "goalCoreMatchesSharedCore": _sig_key(gold_core) == _sig_key(ref_core),
        "goldGoalCore": gold_core,
        "sharedGoalCore": ref_core,
        "note": "The MuJoCo internal-sim world is judged against its native gold pick-place "
                "target, a SUPERSET of the relation tiers (it additionally enforces contact / "
                "object-carrier / temporal-order probes). Only its INSIDE goal core is claimed "
                "equivalent to the shared core — the internal world is judged at a STRONGER tier.",
    }


def tier_strengthening() -> Dict[str, Any]:
    """Show the tiers are nested: terminal_only's core is contained in relation_event's
    (relation_event adds the authored endpoints + the containment event), and
    placed_from_outside differs from relation_event ONLY in the initial relation
    (NEAR vs FAR_FROM) — the single bit that distinguishes near-start from far-start put-ins."""
    rc = PILOT_TARGET_DIRS["real_camera"]
    term = canonical_signature(load_target(rc / TIER_FILE["terminal_only"]))
    rele = canonical_signature(load_target(rc / TIER_FILE["relation_event"]))
    plac = canonical_signature(load_target(rc / TIER_FILE["placed_from_outside"]))
    return {
        "terminalOnlyHasNoRelationsOrEvents": term["relations"] == [] and term["events"] == [],
        "relationEventAddsEndpointsAndEvent": len(rele["relations"]) == 2 and len(rele["events"]) == 1,
        "terminalGoalUnchangedAcrossTiers": _sig_key(term["goal"]) == _sig_key(rele["goal"]) == _sig_key(plac["goal"]),
        # placed_from_outside and relation_event are identical in objects, goal, AND the
        # containment event (both author a NEAR→INSIDE delta) — they differ ONLY in the initial
        # authored relation endpoint r0 (NEAR for relation_event, FAR_FROM for placed), the single
        # fact initial_state reads to tell a near-start put-in from a far-start one.
        "placedDiffersFromRelationEventOnlyInStartEndpoint": (
            _sig_key({k: v for k, v in rele.items() if k != "relations"})
            == _sig_key({k: v for k, v in plac.items() if k != "relations"})
            and [r["relation"] for r in rele["relations"]] == ["NEAR", "INSIDE"]
            and [r["relation"] for r in plac["relations"]] == ["FAR_FROM", "INSIDE"]
        ),
    }


def _assert_within_simplifying_assumptions(target: Mapping[str, Any], label: str) -> Dict[str, Any]:
    """The signature fully models single-stage / single-hard-relation-goal / contact-free /
    articulation-free cards (all object_inside_container tiers are such). Fail LOUDLY if a card
    outgrows that — better a broken proof than a silently laundered equivalence."""
    stages = (target.get("plannerView", {}) or {}).get("stages", []) or []
    goals = _all_goal_constraints(target)
    hard_goals = [gc for _si, gc in goals if gc.get("hard")]
    hard_rel_goals = [gc for gc in hard_goals if gc.get("kind") == "OBJECT_RELATION_GOAL"]
    hard_art_goals = [gc for gc in hard_goals if gc.get("kind") != "OBJECT_RELATION_GOAL"]
    contacts = target.get("contacts", []) or []
    scope = {
        "stageCount": len(stages),
        "nHardGoals": len(hard_goals),
        "nHardRelationGoals": len(hard_rel_goals),
        "nHardArticulationGoals": len(hard_art_goals),
        "nContacts": len(contacts),
    }
    violations = []
    if len(stages) > 1:
        violations.append(f"{len(stages)} stages (signature models single-stage)")
    if len(hard_rel_goals) != 1:
        violations.append(f"{len(hard_rel_goals)} hard relation goals (signature models exactly one)")
    if hard_art_goals:
        violations.append(f"{len(hard_art_goals)} hard articulation goals (not modelled)")
    if contacts:
        violations.append(f"{len(contacts)} contacts (not modelled)")
    if violations:
        raise AssertionError(
            f"target card {label!r} outgrew the equivalence signature's simplifying assumptions: "
            f"{'; '.join(violations)}. Extend canonical_signature() to fully model these before "
            f"trusting the equivalence proof on this card.")
    return scope


def build_target_equivalence() -> Dict[str, Any]:
    # Guard: every audited external card must stay within the assumptions the signature fully
    # models, or the proof self-disables (raises) rather than risk a false equivalence.
    scopes = {}
    for src, d in PILOT_TARGET_DIRS.items():
        for tier in TIERS:
            p = d / TIER_FILE[tier]
            if p.exists():
                scopes[f"{src}/{tier}"] = _assert_within_simplifying_assumptions(load_target(p), f"{src}/{tier}")

    tiers = {t: tier_equivalence(t) for t in TIERS}
    all_tiers_identical = all(v["allIdentical"] for v in tiers.values())
    return {
        "kind": "structural equivalence of the per-source object_inside_container target cards",
        "claim": "Each world is verified against its OWN target card (different graphId, object "
                 "labels, geometry provenance) — but every card reduces to the SAME verifier-enforced "
                 "semantic core per tier. This proves 'same semantic task, instantiated per source', "
                 "NOT 'one identical target file'.",
        "strippedAuthoringFields": ["graphId", "taskCaption", "categoryLabel", "categoryConfidence",
                                    "geometry(source+sizeM)", "confidence(s)", "timeNs/timeSpan magnitudes",
                                    "object/relation/event/constraint/stage ids (canonicalised to roles)",
                                    "pilotMetadata"],
        "signatureStricterThanVerifierNote": "The signature reads everything the matcher hard-gates on "
            "(all hard goal constraints, relation endpoints, events, contacts, stage count) PLUS object "
            "mobility, which the matcher's hard_signature treats only as a soft diagnostic (it gates on "
            "physicalKind). A stricter signature can only yield a false NON-equivalence, never a false "
            "equivalence — the safe direction.",
        "simplifyingAssumptionScopes": scopes,
        "perTier": tiers,
        "allExternalTiersIdentical": all_tiers_identical,
        "tierStrengthening": tier_strengthening(),
        "internalSim": mujoco_goal_core_matches(),
    }


if __name__ == "__main__":
    print(json.dumps(build_target_equivalence(), indent=2))
