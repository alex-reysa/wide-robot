#!/usr/bin/env python3
"""Map an RLBench demonstration into a leakage-clean ``csg.rollout.v0`` artifact.

The csg verifier loop is frozen and dependency-free; this adapter lives *outside*
the ``csg`` package and consumes it exactly as a third party would. Its single job
is to produce the one thing the frozen extractor is allowed to read — a
``csg.rollout.v0`` rollout (see ``csg/rollout_schema.md``) — from an external
demonstration trace, **without leaking any target-authored information**. The
scientific question of the pilot is precisely whether an external trace (one csg's
own solver did not produce) survives the same hard-probe matcher + leakage gate.

The **source-agnostic** half of that job — :func:`assemble_rollout` and the leakage
gate :func:`assert_rollout_leakage_clean` (the "rollout door") — now lives in
:mod:`pilots.external_rollout` so a second source (real camera) reuses the exact same
contract. It is re-exported here so existing imports
(``from pilots.rlbench.adapter import assemble_rollout, assert_rollout_leakage_clean,
ExternalTraceLeakage, _xyzw_to_wxyz, …``) keep working unchanged.

The RLBench-**specific** half stays here: :func:`rlbench_demo_to_rollout` is REAL for
``task="open_drawer"`` (the pilot's only scope). It consumes a recorded RLBench ``Demo``
(for the effector pose + gripper state) plus already-neutralised per-frame
``measurements`` (the drawer's body pose, size, and articulation value, extracted by
:mod:`pilots.rlbench.record_open_drawer` using quarantined CoppeliaSim handles), converts
the RLBench XYZW gripper quaternion to CSG WXYZ, maps ``gripper_open < 0.5`` to
``gripperClosed``, and hands neutral bodies + frames to :func:`assemble_rollout`. It
touches **no** RLBench import (it reads plain attributes off the ``Observation`` objects),
so this module imports fine on a machine with no RLBench installed and the converter is
unit-testable with fakes.

Leakage rule of thumb (``csg/rollout_schema.md`` §Versioning): the default answer to
"can the rollout carry X?" is **no** unless a simulator with no access to the
demonstration's authoring could have produced X. RLBench object *category labels*,
task names, waypoint/goal annotations, and ground-truth target poses are authoring
— they must be dropped here, never carried into the rollout.
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence

from csg.common import Json, get_any, pose_xyz

# Source-agnostic rollout door + leakage gate, re-exported for back-compat. Every
# symbol below is referenced by name from tests/ or sibling pilot modules; the
# ``# noqa: F401`` keeps a linter from stripping the "unused" re-exports.
from pilots.external_rollout import (  # noqa: F401
    ExternalTraceLeakage,
    assert_rollout_leakage_clean,
    assemble_rollout,
    EXTERNAL_BACKEND,
    ROLLOUT_SCHEMA_VERSION,
    NEUTRAL_ID_PREFIX,
    _is_neutral_id,
    _HIDDEN_VARIABLES_NOT_USED,
    _REQUIRED_FRAME_KEYS,
    _xyzw_to_wxyz,
    _IDENTITY_WXYZ,
)


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
