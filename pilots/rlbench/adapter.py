#!/usr/bin/env python3
"""Map an RLBench demonstration into a leakage-clean ``csg.rollout.v0`` artifact.

The csg verifier loop is frozen and dependency-free; this adapter lives *outside*
the ``csg`` package and consumes it exactly as a third party would. Its single job
is to produce the one thing the frozen extractor is allowed to read — a
``csg.rollout.v0`` rollout (see ``csg/rollout_schema.md``) — from an external
demonstration trace, **without leaking any target-authored information**. The
scientific question of the pilot is precisely whether an external trace (one csg's
own solver did not produce) survives the same hard-probe matcher + leakage gate.

Two layers, deliberately split so the csg-side contract is real and testable today
while the RLBench-side parsing (which needs RLBench + PyRep + recorded demos) is a
documented stub:

  * :func:`assemble_rollout` — REAL, unit-tested. Given already-neutralised inputs
    (sanitised bodies + per-frame effector/object/gripper state) it assembles a
    schema-valid ``csg.rollout.v0`` and enforces the leakage contract on the way
    out (whitelisted body fields, neutral ids, no forbidden keys, physics reported
    ``null`` because csg cannot re-check another simulator's physics).

  * :func:`rlbench_demo_to_rollout` — STUB. Parses an RLBench ``Demo`` (a list of
    ``Observation``) into the neutral inputs :func:`assemble_rollout` expects. It
    raises :class:`NotImplementedError` with the exact field mapping until wired,
    so no one mistakes the scaffold for a working ingest. It imports ``rlbench``
    lazily, so this module imports fine on a machine with no RLBench installed.

Leakage rule of thumb (``csg/rollout_schema.md`` §Versioning): the default answer
to "can the rollout carry X?" is **no** unless a simulator with no access to the
demonstration's authoring could have produced X. RLBench object *category labels*,
task names, waypoint/goal annotations, and ground-truth target poses are authoring
— they must be dropped here, never carried into the rollout.
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence

# The frozen verifier is imported as-is — the adapter adapts to it, never the
# reverse. ``sanitize_bodies_for_rollout`` applies the exact ``ROLLOUT_BODY_FIELDS``
# whitelist the solver uses; ``LEAKAGE_FORBIDDEN_KEYS`` is the same set the
# benchmark's ``leakage_report`` fails a case on.
from csg.common import Json
from csg.to_sim import ROLLOUT_BODY_FIELDS, sanitize_bodies_for_rollout
from csg.benchmark import LEAKAGE_FORBIDDEN_KEYS

ROLLOUT_SCHEMA_VERSION = "csg.rollout.v0"
EXTERNAL_BACKEND = "rlbench_external"

# Unobservable causal variables an external kinematic demo cannot honestly ground
# (mirrors the symbolic backend's contract in ``csg/rollout_schema.md`` §diagnostics).
_HIDDEN_VARIABLES_NOT_USED = ["force", "torque", "mass", "friction", "stable_grasp_quality"]

# Frame keys the extractor consumes (``csg/rollout_schema.md`` §frames). ``phase`` is
# solver provenance and ignored; an external adapter may set it to ``"external"``.
_REQUIRED_FRAME_KEYS = ("timeS", "effectorPose", "gripperClosed", "objectPoses")


class ExternalTraceLeakage(ValueError):
    """An external rollout carries target-authored information it must not.

    Raised at assembly time so a buggy or cheating adapter cannot mint a rollout
    that smuggles target identity past the frozen verifier. The verifier's own
    ``leakage_report`` is the second line of defence (it runs on the extracted
    robot CSG); this is the first, at the rollout door.
    """


def assert_rollout_leakage_clean(rollout: Mapping[str, Any]) -> None:
    """Reject a rollout that carries forbidden keys or non-whitelisted body fields.

    The external-trace threat model is stricter than the solver's: with csg's own
    solver the rollout is produced under the schema discipline; an external adapter
    could (by bug or design) inject ``targetCsg`` / ``plannerView`` / ``solverMetadata``
    or a body field that encodes the target's identity (``categoryLabel``,
    ``sourceObjectId``, part labels). Both are caught here.
    """
    forbidden = [k for k in LEAKAGE_FORBIDDEN_KEYS if k in rollout]
    if forbidden:
        raise ExternalTraceLeakage(f"rollout carries forbidden target-authored keys: {forbidden}")
    bodies = rollout.get("sceneBodies", []) or []
    for body in bodies:
        if not isinstance(body, Mapping):
            raise ExternalTraceLeakage(f"sceneBodies entry is not an object: {body!r}")
        extra = sorted(set(body) - set(ROLLOUT_BODY_FIELDS))
        if extra:
            raise ExternalTraceLeakage(
                f"sceneBody {body.get('objectId') or body.get('bodyId')!r} carries non-whitelisted "
                f"fields {extra} (allowed: {sorted(ROLLOUT_BODY_FIELDS)})")
        obj_id = str(body.get("objectId") or body.get("bodyId") or "")
        if obj_id and not obj_id.startswith("body_"):
            raise ExternalTraceLeakage(
                f"sceneBody id {obj_id!r} is not a neutral body_NNN id (target identity must not leak)")


def assemble_rollout(
    *,
    bodies: Sequence[Mapping[str, Any]],
    frames: Sequence[Mapping[str, Any]],
    robot_effector_id: str = "robot_gripper",
    object_id_map: Optional[Mapping[str, str]] = None,
    backend: str = EXTERNAL_BACKEND,
    physical_validity_reason: Optional[str] = None,
    extra_diagnostics: Optional[Mapping[str, Any]] = None,
) -> Json:
    """Assemble a leakage-clean ``csg.rollout.v0`` from neutral inputs.

    ``bodies`` are sanitised through :func:`csg.to_sim.sanitize_bodies_for_rollout`
    (so only the whitelisted, simulator-honest fields survive) and ``frames`` must
    already be neutral state (see :data:`_REQUIRED_FRAME_KEYS`). The result reports
    ``physicalValidity: null`` — csg cannot re-check another engine's physics, so by
    contract (``csg/validity.md``) the external trace is *physics-unverified*, never
    claimed valid. The assembled rollout is passed through
    :func:`assert_rollout_leakage_clean` before return.
    """
    sanitized = sanitize_bodies_for_rollout(list(bodies))
    clean_frames: List[Json] = []
    for i, frame in enumerate(frames):
        missing = [k for k in _REQUIRED_FRAME_KEYS if k not in frame]
        if missing:
            raise ValueError(f"frame {i} missing required keys {missing}")
        clean_frames.append(dict(frame))

    diagnostics: Json = {
        "selectedProgramId": "external_demo",
        "skill": "external",
        "planProduced": True,
        "numFrames": len(clean_frames),
        "hiddenVariablesNotUsed": list(_HIDDEN_VARIABLES_NOT_USED),
        "physicalValidity": None,
        "physicalValidityReason": physical_validity_reason or (
            "external trace: csg cannot re-check another simulator's physics; "
            "physics-unverified by contract (csg/validity.md)"),
    }
    if extra_diagnostics:
        # Diagnostics are honest-by-contract; callers may add neutral notes but not
        # a physicalValidity:true claim, which an external kinematic trace cannot earn.
        for k, v in extra_diagnostics.items():
            if k == "physicalValidity" and v is True:
                raise ExternalTraceLeakage("an external trace may not claim physicalValidity:true")
            diagnostics[k] = v

    rollout: Json = {
        "schemaVersion": ROLLOUT_SCHEMA_VERSION,
        "backend": backend,
        "robotEffectorId": robot_effector_id,
        "objectIdMap": dict(object_id_map or {}),
        "sceneBodies": sanitized,
        "skillProgram": {"programId": "external_demo", "source": "rlbench", "steps": []},
        "frames": clean_frames,
        "success": True,
        "failures": [],
        "diagnostics": diagnostics,
    }
    assert_rollout_leakage_clean(rollout)
    return rollout


# RLBench → neutral-input field mapping the stub must implement. Documented here so
# the contract is reviewable before any RLBench dependency is installed. Keys are
# the neutral inputs :func:`assemble_rollout` consumes; values describe the RLBench
# source and the leakage rule applied.
RLBENCH_FIELD_MAPPING = {
    "bodies[].physicalKind": "infer RIGID / ARTICULATED from the task's object shape handles; "
                             "NEVER from RLBench category names (those are authoring → leakage)",
    "bodies[].sizeM": "axis-aligned bounding box of each task object's shape (CoppeliaSim handle extents)",
    "bodies[].mobility": "MOVABLE / STATIC / ARTICULATED from the object's joint/dynamic flags, not labels",
    "bodies[].objectId": "assign neutral body_000, body_001, … in a fixed scan order; drop RLBench names",
    "frames[].effectorPose": "Observation.gripper_pose (xyz + quaternion) → effectorPose{positionM, orientationWxyz}",
    "frames[].gripperClosed": "Observation.gripper_open < threshold → gripperClosed=True",
    "frames[].objectPoses": "per-object pose from Observation low-dim state, keyed by the neutral body_NNN id",
    "frames[].articulation": "joint value for the articulated body (e.g. drawer prismatic joint) if any",
    "frames[].timeS": "frame index / control rate (RLBench demos are fixed-step)",
    "DROP (leakage)": "task name, waypoint/goal annotations, category labels, ground-truth target poses, "
                      "any RLBench Task object — none may enter the rollout",
}


def rlbench_demo_to_rollout(demo: Any, *, task: str, control_rate_hz: float = 20.0) -> Json:
    """Convert an RLBench ``Demo`` (list of ``Observation``) into ``csg.rollout.v0``.

    STUB — not yet wired. Wiring it requires RLBench + PyRep + recorded demos (see
    ``docs/rlbench_external_trace_pilot.md``). Implement the mapping in
    :data:`RLBENCH_FIELD_MAPPING`, then hand the neutral bodies + frames to
    :func:`assemble_rollout`, which enforces the leakage contract. The RLBench import
    is intentionally lazy (inside the function) so this module loads without RLBench.
    """
    raise NotImplementedError(
        "rlbench_demo_to_rollout is a pilot stub. Implement the RLBENCH_FIELD_MAPPING "
        "(parse RLBench Observations → neutral bodies + frames) and call assemble_rollout(). "
        "Until then, exercise the verifier seam with pilots/rlbench/fixtures/*.rollout.json. "
        f"(requested task={task!r}, control_rate_hz={control_rate_hz})"
    )
