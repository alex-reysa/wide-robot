#!/usr/bin/env python3
"""Source-agnostic assembly of a leakage-clean ``csg.rollout.v0`` from external traces.

This module is the **rollout door**: the single place an external evidence source (an
RLBench demo, a real-camera episode, a future robot log) hands neutralised state to the
frozen csg verifier. It is deliberately source-agnostic — it imports only ``csg.*`` and
knows nothing about RLBench, cameras, or any particular task. Each pilot (``pilots/rlbench``,
``pilots/real_camera``) builds its own neutral bodies + frames and calls
:func:`assemble_rollout`; the source-specific parsing lives in the pilot, never here.

It was extracted verbatim from ``pilots/rlbench/adapter.py`` so a second source (real
camera) can reuse the exact same leakage contract that the RLBench pilot proved. The
RLBench adapter now re-exports these names for back-compat (so existing imports such as
``from pilots.rlbench.adapter import assert_rollout_leakage_clean`` keep working unchanged).

Two pieces:

  * :func:`assemble_rollout` — given already-neutralised inputs (sanitised bodies +
    per-frame effector/object/gripper state) it assembles a schema-valid
    ``csg.rollout.v0`` and enforces the leakage contract on the way out (whitelisted body
    fields, neutral ids, no forbidden keys, physics reported ``null`` because csg cannot
    re-check another engine's physics).

  * :func:`assert_rollout_leakage_clean` — the hardened gate that rejects a rollout
    carrying target-authored information anywhere the frozen extractor/matcher can read.

Leakage rule of thumb (``csg/rollout_schema.md`` §Versioning): the default answer to "can
the rollout carry X?" is **no** unless a simulator with no access to the demonstration's
authoring could have produced X.
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence

# The frozen verifier is imported as-is — the adapter adapts to it, never the
# reverse. ``sanitize_bodies_for_rollout`` applies the exact ``ROLLOUT_BODY_FIELDS``
# whitelist the solver uses; ``LEAKAGE_FORBIDDEN_KEYS`` is the same set the
# benchmark's ``leakage_report`` fails a case on.
from csg.common import Json, as_list, get_any
from csg.to_sim import ROLLOUT_BODY_FIELDS, sanitize_bodies_for_rollout
from csg.benchmark import LEAKAGE_FORBIDDEN_KEYS

ROLLOUT_SCHEMA_VERSION = "csg.rollout.v0"
# Historical default backend (a published RLBench fixture value asserted in
# tests/test_rlbench_pilot.py). New pilots pass their own backend explicitly
# (e.g. the camera pilot passes backend="real_camera_external").
EXTERNAL_BACKEND = "rlbench_external"

# Neutral robot-side object ids (``body_000``, ``body_001``, …). The whole leakage
# contract turns on this: a rollout may carry only neutral ids, never a target
# identity. Any id the extractor or matcher could read — body ids, objectIdMap
# keys/values, nested ``articulatedObjectId``, per-frame ``objectPoses`` /
# ``articulation`` keys — must match this prefix.
NEUTRAL_ID_PREFIX = "body_"


def _is_neutral_id(value: Any) -> bool:
    return isinstance(value, str) and value.startswith(NEUTRAL_ID_PREFIX)


# Unobservable causal variables an external kinematic demo cannot honestly ground
# (mirrors the symbolic backend's contract in ``csg/rollout_schema.md`` §diagnostics).
_HIDDEN_VARIABLES_NOT_USED = ["force", "torque", "mass", "friction", "stable_grasp_quality"]

# Frame keys the extractor consumes (``csg/rollout_schema.md`` §frames). ``phase`` is
# solver provenance and ignored; an external adapter may set it to ``"external"``.
_REQUIRED_FRAME_KEYS = ("timeS", "effectorPose", "gripperClosed", "objectPoses")

_IDENTITY_WXYZ = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}


class ExternalTraceLeakage(ValueError):
    """An external rollout carries target-authored information it must not.

    Raised at assembly time so a buggy or cheating adapter cannot mint a rollout
    that smuggles target identity past the frozen verifier. The verifier's own
    ``leakage_report`` is the second line of defence (it runs on the extracted
    robot CSG); this is the first, at the rollout door.
    """


def assert_rollout_leakage_clean(rollout: Mapping[str, Any]) -> None:
    """Reject a rollout that carries forbidden keys, non-whitelisted body fields, or
    non-neutral object ids anywhere the extractor/matcher could read them.

    The external-trace threat model is stricter than the solver's: with csg's own
    solver the rollout is produced under the schema discipline; an external adapter
    could (by bug or design) inject ``targetCsg`` / ``plannerView`` / ``solverMetadata``
    or a body field that encodes the target's identity (``categoryLabel``,
    ``sourceObjectId``, part labels). Beyond top-level keys and the body whitelist,
    target identity can also hide in *ids*: ``objectIdMap`` keys (target id → robot
    id is solver bookkeeping with no honest external analogue), the nested
    ``sceneBodies[].articulation.articulatedObjectId``, and the ``objectPoses`` /
    ``articulation`` keys on every frame. All of these must be neutral ``body_NNN``
    ids; any source/target name is rejected here, at the rollout door.

    Every field is read through :func:`csg.common.get_any` / :func:`as_list` — the SAME
    accessors the frozen extractor uses — so the gate cannot drift from the reader: a
    snake_case spelling (``scene_bodies``, ``object_poses``) the extractor accepts is
    gated identically, never silently bypassed. Identity-carrying fields that are
    present but malformed (a list / string where a mapping is required) are rejected,
    not skipped — the gate is strictly fail-closed.

    Not gated on purpose: ``robotEffectorId`` and ``backend`` are not *object* identity
    — the matcher abstracts the effector to a single ``EFFECTOR`` role (csg/canon.py)
    and ``backend`` only names the graph, so neither can enter the object bijection or
    ``relevant_objects`` (which csg/canon.py structurally restricts to ids present in
    ``sceneBodies``). The adapter sets both to neutral constants regardless.
    """
    forbidden = [k for k in LEAKAGE_FORBIDDEN_KEYS if k in rollout]
    if forbidden:
        raise ExternalTraceLeakage(f"rollout carries forbidden target-authored keys: {forbidden}")

    bodies = as_list(get_any(rollout, "sceneBodies", "scene_bodies", default=[]))
    for body in bodies:
        if not isinstance(body, Mapping):
            raise ExternalTraceLeakage(f"sceneBodies entry is not an object: {body!r}")
        extra = sorted(set(body) - set(ROLLOUT_BODY_FIELDS))
        if extra:
            raise ExternalTraceLeakage(
                f"sceneBody {get_any(body, 'objectId', 'bodyId', default=None)!r} carries non-whitelisted "
                f"fields {extra} (allowed: {sorted(ROLLOUT_BODY_FIELDS)})")
        obj_id = str(get_any(body, "objectId", "bodyId", default="") or "")
        if obj_id and not _is_neutral_id(obj_id):
            raise ExternalTraceLeakage(
                f"sceneBody id {obj_id!r} is not a neutral body_NNN id (target identity must not leak)")
        # Nested articulation id: the body whitelist lets ``articulation`` through as
        # a block, but its ``articulatedObjectId`` can still smuggle a target/source
        # name (e.g. ``h_drawer``, ``drawer_joint_top``). It must be a neutral id.
        art = get_any(body, "articulation", default=None)
        if art is not None:
            if not isinstance(art, Mapping):
                raise ExternalTraceLeakage(
                    f"sceneBody {obj_id!r} articulation must be an object, got {art!r}")
            aid = get_any(art, "articulatedObjectId", default=None)
            if aid is not None and not _is_neutral_id(aid):
                raise ExternalTraceLeakage(
                    f"sceneBody articulation.articulatedObjectId {aid!r} is not a neutral body_NNN id "
                    f"(target identity must not leak via nested articulation ids)")

    # objectIdMap: target id → neutral id is csg-solver bookkeeping the extractor
    # ignores. An external trace has no legitimate target id to map (the adapter emits
    # it empty); any non-neutral key OR value is a target/source identity leaking in.
    obj_map = get_any(rollout, "objectIdMap", "object_id_map", default={})
    if obj_map is not None:
        if not isinstance(obj_map, Mapping):
            raise ExternalTraceLeakage(f"objectIdMap must be an object, got {obj_map!r}")
        bad_ids = sorted(
            {str(k) for k in obj_map if not _is_neutral_id(k)}
            | {str(v) for v in obj_map.values() if not _is_neutral_id(v)}
        )
        if bad_ids:
            raise ExternalTraceLeakage(
                f"objectIdMap carries non-neutral ids {bad_ids}: an external trace must not map "
                f"target/source identities (emit an empty objectIdMap)")

    # Per-frame object-id keys: poses and articulation must be keyed by neutral ids.
    # Read each block through the extractor's aliases (objectPoses/object_poses) and
    # reject a present-but-malformed (non-mapping) block.
    for i, frame in enumerate(as_list(get_any(rollout, "frames", default=[]))):
        if not isinstance(frame, Mapping):
            continue
        for label, aliases in (("objectPoses", ("objectPoses", "object_poses")), ("articulation", ("articulation",))):
            block = get_any(frame, *aliases, default=None)
            if block is None:
                continue
            if not isinstance(block, Mapping):
                raise ExternalTraceLeakage(f"frame {i} {label} must be an object, got {block!r}")
            bad_keys = sorted(str(k) for k in block if not _is_neutral_id(k))
            if bad_keys:
                raise ExternalTraceLeakage(
                    f"frame {i} {label} is keyed by non-neutral ids {bad_keys} "
                    f"(per-frame object poses/articulation must use neutral body_NNN ids)")


def assemble_rollout(
    *,
    bodies: Sequence[Mapping[str, Any]],
    frames: Sequence[Mapping[str, Any]],
    robot_effector_id: str = "robot_gripper",
    object_id_map: Optional[Mapping[str, str]] = None,
    backend: str = EXTERNAL_BACKEND,
    physical_validity_reason: Optional[str] = None,
    extra_diagnostics: Optional[Mapping[str, Any]] = None,
    skill_source: str = "rlbench",
) -> Json:
    """Assemble a leakage-clean ``csg.rollout.v0`` from neutral inputs.

    ``bodies`` are sanitised through :func:`csg.to_sim.sanitize_bodies_for_rollout`
    (so only the whitelisted, simulator-honest fields survive) and ``frames`` must
    already be neutral state (see :data:`_REQUIRED_FRAME_KEYS`). The result reports
    ``physicalValidity: null`` — csg cannot re-check another engine's physics, so by
    contract (``csg/validity.md``) the external trace is *physics-unverified*, never
    claimed valid. The assembled rollout is passed through
    :func:`assert_rollout_leakage_clean` before return.

    ``backend`` defaults to the historical ``"rlbench_external"`` (a published fixture
    value); a new source passes its own (e.g. ``"real_camera_external"``).
    ``skill_source`` names the provenance written into ``skillProgram.source`` and
    defaults to ``"rlbench"`` so existing fixtures stay byte-identical; the camera path
    passes ``"real_camera"`` for honest provenance. Neither is leakage-gated (see
    :func:`assert_rollout_leakage_clean`).
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
        "skillProgram": {"programId": "external_demo", "source": skill_source, "steps": []},
        "frames": clean_frames,
        "success": True,
        "failures": [],
        "diagnostics": diagnostics,
    }
    assert_rollout_leakage_clean(rollout)
    return rollout


def _xyzw_to_wxyz(quat: Sequence[float]) -> Json:
    """RLBench/CoppeliaSim and most CV libraries report quaternions as (x, y, z, w);
    csg uses WXYZ. Source-agnostic: any pilot reading an XYZW quaternion uses this."""
    qx, qy, qz, qw = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    return {"w": qw, "x": qx, "y": qy, "z": qz}


# Public alias so a pilot can import a non-underscore name without reaching into
# pilots.rlbench.* or a private symbol.
xyzw_to_wxyz = _xyzw_to_wxyz


__all__ = [
    "ROLLOUT_SCHEMA_VERSION",
    "EXTERNAL_BACKEND",
    "NEUTRAL_ID_PREFIX",
    "ExternalTraceLeakage",
    "assert_rollout_leakage_clean",
    "assemble_rollout",
    "xyzw_to_wxyz",
]
