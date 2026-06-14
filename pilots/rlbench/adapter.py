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

  * :func:`rlbench_demo_to_rollout` — REAL for ``task="open_drawer"`` (the pilot's
    only scope). It consumes a recorded RLBench ``Demo`` (for the effector pose +
    gripper state) plus already-neutralised per-frame ``measurements`` (the drawer's
    body pose, size, and articulation value, extracted by
    :mod:`pilots.rlbench.record_open_drawer` using quarantined CoppeliaSim handles),
    converts the RLBench XYZW gripper quaternion to CSG WXYZ, maps
    ``gripper_open < 0.5`` to ``gripperClosed``, and hands neutral bodies + frames to
    :func:`assemble_rollout`. It touches **no** RLBench import (it reads plain
    attributes off the ``Observation`` objects), so this module imports fine on a
    machine with no RLBench installed and the converter is unit-testable with fakes.

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
from csg.common import Json, as_list, get_any, pose_xyz
from csg.to_sim import ROLLOUT_BODY_FIELDS, sanitize_bodies_for_rollout
from csg.benchmark import LEAKAGE_FORBIDDEN_KEYS

ROLLOUT_SCHEMA_VERSION = "csg.rollout.v0"
EXTERNAL_BACKEND = "rlbench_external"

# Neutral robot-side object ids (``body_000``, ``body_001``, …). The whole leakage
# contract turns on this: a rollout may carry only neutral ids, never a target /
# RLBench identity (``h_drawer``, ``drawer_frame``, ``drawer_joint_top``). Any id
# the extractor or matcher could read — body ids, objectIdMap keys/values, nested
# ``articulatedObjectId``, per-frame ``objectPoses`` / ``articulation`` keys — must
# match this prefix.
NEUTRAL_ID_PREFIX = "body_"


def _is_neutral_id(value: Any) -> bool:
    return isinstance(value, str) and value.startswith(NEUTRAL_ID_PREFIX)

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
    ids; any RLBench/target name is rejected here, at the rollout door.

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
        # a block, but its ``articulatedObjectId`` can still smuggle a target/RLBench
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
    # it empty); any non-neutral key OR value is a target/RLBench identity leaking in.
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
                f"target/RLBench identities (emit an empty objectIdMap)")

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


# RLBench → neutral-input field mapping this converter implements. Documented here so
# the contract is reviewable; keys are the neutral inputs :func:`assemble_rollout`
# consumes, values describe the RLBench source and the leakage rule applied. The
# *body* pose/size/articulation are not on the raw ``Observation``; they are read
# from quarantined CoppeliaSim handles by :mod:`pilots.rlbench.record_open_drawer`
# and handed to this converter as neutral ``measurements`` (see
# :data:`NEUTRAL_MEASUREMENT_FIELDS`), so no handle name ever reaches the rollout.
RLBENCH_FIELD_MAPPING = {
    "bodies[].physicalKind": "ARTICULATED_OBJECT for the open_drawer body; inferred from the joint "
                             "handle, NEVER from RLBench category names (those are authoring → leakage)",
    "bodies[].sizeM": "axis-aligned bounding box of the drawer body's shape (CoppeliaSim handle extents)",
    "bodies[].mobility": "ARTICULATED from the drawer's prismatic joint, not from any label",
    "bodies[].objectId": "neutral body_000 (single task body for open_drawer); RLBench names dropped",
    "frames[].effectorPose": "Observation.gripper_pose (xyz + XYZW quaternion) → effectorPose"
                             "{positionM, orientationWxyz(WXYZ)}",
    "frames[].gripperClosed": "Observation.gripper_open < 0.5 → gripperClosed=True",
    "frames[].objectPoses": "neutral measurement bodyPose, keyed by the neutral body_NNN id",
    "frames[].articulation": "neutral measurement articulationValue (drawer prismatic extension), "
                             "keyed by the neutral body_NNN id",
    "frames[].timeS": "measurement timeS, else frame index / control_rate_hz (RLBench demos are fixed-step)",
    "DROP (leakage)": "task name, variation name, waypoint/goal annotations, category labels, "
                      "ground-truth target poses, RLBench handle names, any RLBench Task object — "
                      "none may enter the rollout (they live in the recorder sidecar instead)",
}

# Tasks this converter supports. The pilot scope is open_drawer ONLY
# (docs/rlbench_external_trace_pilot.md); anything else raises rather than silently
# producing a structurally-wrong rollout.
SUPPORTED_TASKS = ("open_drawer",)

# The single neutral task body for open_drawer (the drawer). Articulation, not body
# motion, encodes the open — mirroring how the symbolic backend reports a static
# body whose joint value ramps (see csg/rollout_extract.py articulation handling).
_DRAWER_BODY_ID = NEUTRAL_ID_PREFIX + "000"

# The neutral per-frame measurement shape the recorder must emit (fail-closed): a
# measurement carrying ANY other key (an RLBench label, variation name, handle name,
# target id) is rejected, so the recorder cannot accidentally smuggle authoring into
# the converter. No object id appears here at all — the converter assigns body_000.
NEUTRAL_MEASUREMENT_FIELDS = frozenset(
    {"frameIndex", "timeS", "bodyPose", "articulationValue", "bodySizeM", "sizeApproximate"}
)

_IDENTITY_WXYZ = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}


def _xyzw_to_wxyz(quat: Sequence[float]) -> Json:
    """RLBench/CoppeliaSim report quaternions as (x, y, z, w); csg uses WXYZ."""
    qx, qy, qz, qw = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    return {"w": qw, "x": qx, "y": qy, "z": qz}


def _obs_field(obs: Any, name: str) -> Any:
    """Read a field off an RLBench ``Observation`` (attribute) or a plain mapping
    (test fake), without importing RLBench."""
    if isinstance(obs, Mapping):
        return obs.get(name)
    return getattr(obs, name, None)


def _validate_measurement(i: int, m: Any) -> Mapping[str, Any]:
    if not isinstance(m, Mapping):
        raise ValueError(f"measurement {i} is not a mapping: {m!r}")
    extra = sorted(set(m) - NEUTRAL_MEASUREMENT_FIELDS)
    if extra:
        raise ExternalTraceLeakage(
            f"measurement {i} carries non-neutral fields {extra} "
            f"(allowed: {sorted(NEUTRAL_MEASUREMENT_FIELDS)}); RLBench names/labels/ids must not leak")
    # bodySizeM, when given, must be a 3-vector — fail loudly (naming the frame) like
    # every other shape guard, rather than IndexError'ing downstream.
    size = m.get("bodySizeM")
    if size is not None and (not isinstance(size, Sequence) or isinstance(size, str) or len(size) < 3):
        raise ValueError(f"measurement {i} bodySizeM must be a 3-element [x, y, z], got {size!r}")
    return m


def rlbench_demo_to_rollout(
    demo: Any,
    *,
    task: str,
    control_rate_hz: float = 20.0,
    measurements: Optional[Sequence[Mapping[str, Any]]] = None,
) -> Json:
    """Convert a recorded RLBench ``open_drawer`` demo into a leakage-clean ``csg.rollout.v0``.

    ``demo`` is a sequence of RLBench ``Observation`` objects (only ``gripper_pose``
    and ``gripper_open`` are read, by plain attribute access — no RLBench import).
    ``measurements`` is the parallel sequence of *neutral* per-frame measurements
    (:data:`NEUTRAL_MEASUREMENT_FIELDS`) the recorder extracted from quarantined
    CoppeliaSim handles: the drawer body's pose, size, and prismatic articulation
    value. The two sequences must be the same length.

    Per frame this emits ``effectorPose`` (XYZW→WXYZ), ``gripperClosed``
    (``gripper_open < 0.5``), ``objectPoses[body_000]`` (the neutral body pose), and
    ``articulation[body_000]`` (the joint extension). The single body is the neutral
    ``body_000`` ARTICULATED_OBJECT. Everything is handed to :func:`assemble_rollout`,
    which sanitises and re-checks the leakage contract; ``objectIdMap`` is empty (an
    external trace has no target identity to map).

    Only ``task="open_drawer"`` is supported (the pilot scope); other tasks raise.
    """
    if task not in SUPPORTED_TASKS:
        raise NotImplementedError(
            f"rlbench_demo_to_rollout supports task in {SUPPORTED_TASKS}; got {task!r}. "
            "The pilot scope is open_drawer only (docs/rlbench_external_trace_pilot.md).")
    if control_rate_hz <= 0:
        raise ValueError(f"control_rate_hz must be positive, got {control_rate_hz!r}")
    if measurements is None:
        raise ValueError(
            "rlbench_demo_to_rollout requires neutral per-frame `measurements` (the drawer body's "
            "pose, size and articulation value). Produce them with "
            "pilots.rlbench.record_open_drawer (which reads quarantined CoppeliaSim handles); "
            "the raw RLBench Observation does not expose the neutral body state.")

    observations = list(demo)
    meas = [_validate_measurement(i, m) for i, m in enumerate(measurements)]
    if not observations:
        raise ValueError("empty demo: no observations to convert")
    if len(observations) != len(meas):
        raise ValueError(
            f"demo has {len(observations)} observations but {len(meas)} measurements were provided")

    # Static body record (the drawer): pose is static across the demo; the joint
    # value ramps. Size/sizeApproximate are read from the first measurement.
    m0 = meas[0]
    size = list(m0.get("bodySizeM") or [0.4, 0.3, 0.15])
    bodies = [{
        "objectId": _DRAWER_BODY_ID,
        "bodyId": _DRAWER_BODY_ID,
        "physicalKind": "ARTICULATED_OBJECT",
        "mobility": "ARTICULATED",
        "sizeM": [float(size[0]), float(size[1]), float(size[2])],
        "sizeApproximate": bool(m0.get("sizeApproximate", False)),
        "isContainer": False,
        "articulation": {
            "articulatedObjectId": _DRAWER_BODY_ID,  # neutral self-reference, never the RLBench joint name
            "jointKind": "PRISMATIC",
            "valueKind": "EXTENSION_M",
            "jointValue": float(m0.get("articulationValue", 0.0) or 0.0),
            "confidence": 1.0,
        },
    }]

    frames: List[Json] = []
    for i, (obs, m) in enumerate(zip(observations, meas)):
        gp = _obs_field(obs, "gripper_pose")
        if gp is None or len(gp) < 7:
            raise ValueError(f"observation {i} has no 7-dof gripper_pose (got {gp!r})")
        go = _obs_field(obs, "gripper_open")
        if go is None:
            raise ValueError(f"observation {i} has no gripper_open")
        px, py, pz = float(gp[0]), float(gp[1]), float(gp[2])
        body_pose = m.get("bodyPose") or {}
        bx, by, bz = pose_xyz(body_pose)
        b_orient = get_any(body_pose, "orientationWxyz", "orientation_wxyz", default=None) or dict(_IDENTITY_WXYZ)
        t = float(m.get("timeS", i / control_rate_hz))
        frames.append({
            "timeS": t,
            "phase": "external",
            "effectorPose": {
                "frameId": "world",
                "positionM": {"x": px, "y": py, "z": pz},
                "orientationWxyz": _xyzw_to_wxyz(gp[3:7]),
                "confidence": 1.0,
            },
            "gripperClosed": bool(float(go) < 0.5),
            "objectPoses": {_DRAWER_BODY_ID: {
                "frameId": "world",
                "positionM": {"x": bx, "y": by, "z": bz},
                "orientationWxyz": dict(b_orient),
                "confidence": 1.0,
            }},
            "articulation": {_DRAWER_BODY_ID: float(m.get("articulationValue", 0.0) or 0.0)},
        })

    return assemble_rollout(
        bodies=bodies,
        frames=frames,
        object_id_map={},  # external trace: no target identities to map
        physical_validity_reason=(
            "external RLBench open_drawer demo: csg cannot re-check CoppeliaSim physics; "
            "physics-unverified by contract (csg/validity.md)"),
        extra_diagnostics={"controlRateHz": float(control_rate_hz), "numObservations": len(observations)},
    )
