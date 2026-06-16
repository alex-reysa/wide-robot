#!/usr/bin/env python3
"""Map an RLBench ``PutItemInDrawer`` demonstration into a leakage-clean ``csg.rollout.v0``.

This is the **second** RLBench converter (the first, :mod:`pilots.rlbench.adapter`, handles
the single-body articulated ``open_drawer``). ``PutItemInDrawer`` is a *two-body container*
task — a movable item placed into the selected drawer — so it is the external-simulation
leg of the ``object_inside_container`` flagship task already proven on MuJoCo (internal
sim), Sony/iPhone (real camera) and RH20T (real-robot video). The converter is kept
separate from the OpenDrawer adapter so that frozen-ish path is untouched.

Like the OpenDrawer adapter it imports **no** RLBench (it reads plain attributes off the
``Observation`` objects for the effector pose/gripper state), so it loads and is fully
unit-testable on a machine with no RLBench installed. The RLBench/CoppeliaSim identities
(item object, drawer/container handle, the ``success_<variation>`` proximity sensor,
variation/task names) live ONLY in :mod:`pilots.rlbench.record_put_item_in_drawer`; what
crosses into this converter is a list of neutral ``measurements`` (:data:`NEUTRAL_MEASUREMENT_FIELDS`):
the item's per-frame pose/size and the container volume's pose/size (the container volume
is bound from the success proximity sensor's pose + bounding box — never its boolean).

Neutral body assignment (mirrors RH20T's ``object_inside_container`` rollout door):
``body_000`` = item (MOVABLE, ``isContainer:false``); ``body_001`` = container volume
(STATIC, ``isContainer:true``). ``isContainer`` is diagnostic only — the frozen extractor
(``csg/rollout_extract.py``) is purely geometric — but it is set honestly. The static
container is clamped to its median world position across frames so annotation jitter
cannot promote it to a moving "figure" in the extractor's figure-ground pairing (the same
guard ``pilots/rh20t/tracks_to_rollout.py`` applies).

Leakage rule of thumb (``csg/rollout_schema.md`` §Versioning): the default answer to "can
the rollout carry X?" is **no** unless a simulator with no access to the demonstration's
authoring could have produced X. RLBench category labels, task/variation names, success
flags, waypoint/goal annotations, and handle names are authoring — dropped here, never
carried into the rollout (they live in the recorder sidecar instead).
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence

from csg.common import Json, get_any, pose_xyz

# Source-agnostic rollout door + shared helpers (the same ones the OpenDrawer adapter and
# the RH20T/real-camera doors use). Imported from the shared module, never re-implemented.
from pilots.external_rollout import (
    ExternalTraceLeakage,
    assemble_rollout,
    NEUTRAL_ID_PREFIX,
    _IDENTITY_WXYZ,
    _xyzw_to_wxyz,
)

# Tasks this converter supports. ``put_item_in_drawer`` ONLY (the Phase 2F-4 scope);
# anything else raises rather than silently producing a structurally-wrong rollout.
SUPPORTED_TASKS = ("put_item_in_drawer",)

# Neutral robot-side body ids: item moves, container is static.
_ITEM_BODY_ID = NEUTRAL_ID_PREFIX + "000"
_CONTAINER_BODY_ID = NEUTRAL_ID_PREFIX + "001"

# The neutral per-frame measurement shape the recorder must emit (fail-closed): a
# measurement carrying ANY other key (an RLBench label, variation name, handle name,
# success flag, target id) is rejected, so the recorder cannot accidentally smuggle
# authoring into the converter. No object id appears here — the converter assigns
# body_000 (item) / body_001 (container volume).
NEUTRAL_MEASUREMENT_FIELDS = frozenset(
    {"frameIndex", "timeS", "itemPose", "itemSizeM", "containerPose", "containerSizeM", "sizeApproximate"}
)

# RLBench → neutral-input field mapping this converter implements (reviewable contract).
RLBENCH_FIELD_MAPPING = {
    "bodies[body_000].objectId": "neutral body_000 (the movable item); RLBench item name dropped",
    "bodies[body_000].sizeM": "item axis-aligned bounding box (CoppeliaSim shape extents)",
    "bodies[body_001].objectId": "neutral body_001 (the container volume); drawer handle dropped",
    "bodies[body_001].sizeM": "container volume extents from the success proximity sensor bounding box",
    "bodies[body_001].isContainer": "true (diagnostic; the frozen extractor is purely geometric)",
    "frames[].effectorPose": "Observation.gripper_pose (xyz + XYZW quaternion) → effectorPose{positionM, orientationWxyz(WXYZ)}",
    "frames[].gripperClosed": "Observation.gripper_open < 0.5 → gripperClosed=True",
    "frames[].objectPoses[body_000]": "neutral measurement itemPose",
    "frames[].objectPoses[body_001]": "neutral measurement containerPose (median-clamped; container is static)",
    "frames[].timeS": "measurement timeS, else frame index / control_rate_hz",
    "DROP (leakage)": "task/variation names, success_<variation> boolean, waypoint/goal annotations, "
                      "category labels, ground-truth target poses, RLBench handle names, any RLBench "
                      "Task object — none may enter the rollout (they live in the recorder sidecar)",
}


def _obs_field(obs: Any, name: str) -> Any:
    """Read a field off an RLBench ``Observation`` (attribute) or a plain mapping
    (test fake), without importing RLBench."""
    if isinstance(obs, Mapping):
        return obs.get(name)
    return getattr(obs, name, None)


def _validate_size(i: int, key: str, size: Any) -> None:
    if not isinstance(size, Sequence) or isinstance(size, str) or len(size) < 3:
        raise ValueError(f"measurement {i} {key} must be a 3-element [x, y, z], got {size!r}")


def _validate_measurement(i: int, m: Any) -> Mapping[str, Any]:
    if not isinstance(m, Mapping):
        raise ValueError(f"measurement {i} is not a mapping: {m!r}")
    extra = sorted(set(m) - NEUTRAL_MEASUREMENT_FIELDS)
    if extra:
        raise ExternalTraceLeakage(
            f"measurement {i} carries non-neutral fields {extra} "
            f"(allowed: {sorted(NEUTRAL_MEASUREMENT_FIELDS)}); RLBench names/labels/success flags must not leak")
    for key in ("itemPose", "containerPose"):
        if not isinstance(m.get(key), Mapping):
            raise ValueError(f"measurement {i} {key} must be a pose object with positionM, got {m.get(key)!r}")
    for key in ("itemSizeM", "containerSizeM"):
        size = m.get(key)
        if size is not None:
            _validate_size(i, key, size)
    return m


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    return ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def _body(body_id: str, size: Sequence[float], *, mobility: str, is_container: bool,
          size_approximate: bool) -> Json:
    s = list(size)
    return {
        "objectId": body_id,
        "bodyId": body_id,
        "physicalKind": "RIGID_OBJECT",
        "mobility": mobility,
        "sizeM": [float(s[0]), float(s[1]), float(s[2])],
        "sizeApproximate": bool(size_approximate),
        "isContainer": is_container,
    }


def _pose(xyz: Sequence[float], confidence: float = 1.0) -> Json:
    return {
        "frameId": "world",
        "positionM": {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])},
        "orientationWxyz": dict(_IDENTITY_WXYZ),
        "confidence": float(confidence),
    }


def put_item_in_drawer_demo_to_rollout(
    demo: Any,
    *,
    measurements: Optional[Sequence[Mapping[str, Any]]] = None,
    control_rate_hz: float = 20.0,
    task: str = "put_item_in_drawer",
) -> Json:
    """Convert a recorded RLBench ``PutItemInDrawer`` demo into a leakage-clean ``csg.rollout.v0``.

    ``demo`` is a sequence of RLBench ``Observation`` objects (only ``gripper_pose`` and
    ``gripper_open`` are read, by plain attribute access — no RLBench import).
    ``measurements`` is the parallel sequence of *neutral* per-frame measurements
    (:data:`NEUTRAL_MEASUREMENT_FIELDS`) the recorder extracted from quarantined
    CoppeliaSim handles: the item's pose/size and the container volume's pose/size.
    The two sequences must be the same length.

    Per frame this emits ``effectorPose`` (XYZW→WXYZ), ``gripperClosed``
    (``gripper_open < 0.5``), ``objectPoses[body_000]`` (item) and ``objectPoses[body_001]``
    (container volume, clamped to its median world position). The frozen extractor then
    derives the (item, container) relation timeline + ``CONTAINMENT_CHANGE`` event;
    ``objectIdMap`` is empty (an external trace has no target identity to map).

    Only ``task="put_item_in_drawer"`` is supported (the Phase 2F-4 scope); other tasks raise.
    """
    if task not in SUPPORTED_TASKS:
        raise NotImplementedError(
            f"put_item_in_drawer_demo_to_rollout supports task in {SUPPORTED_TASKS}; got {task!r}. "
            "The Phase 2F-4 scope is put_item_in_drawer only (docs/rlbench_external_trace_pilot.md).")
    if control_rate_hz <= 0:
        raise ValueError(f"control_rate_hz must be positive, got {control_rate_hz!r}")
    if measurements is None:
        raise ValueError(
            "put_item_in_drawer_demo_to_rollout requires neutral per-frame `measurements` (the item's "
            "pose/size and the container volume's pose/size). Produce them with "
            "pilots.rlbench.record_put_item_in_drawer (which reads quarantined CoppeliaSim handles); "
            "the raw RLBench Observation does not expose the neutral body state.")

    observations = list(demo)
    meas = [_validate_measurement(i, m) for i, m in enumerate(measurements)]
    if not observations:
        raise ValueError("empty demo: no observations to convert")
    if len(observations) != len(meas):
        raise ValueError(
            f"demo has {len(observations)} observations but {len(meas)} measurements were provided")

    m0 = meas[0]
    item_size = list(m0.get("itemSizeM") or [0.04, 0.04, 0.04])
    container_size = list(m0.get("containerSizeM") or [0.24, 0.18, 0.03])
    size_approx = bool(m0.get("sizeApproximate", True))
    bodies = [
        _body(_ITEM_BODY_ID, item_size, mobility="MOVABLE", is_container=False, size_approximate=size_approx),
        _body(_CONTAINER_BODY_ID, container_size, mobility="STATIC", is_container=True, size_approximate=size_approx),
    ]

    # Median-clamp the static container so annotation/sensor jitter cannot push it past the
    # extractor's MOTION_EPS and turn it into a moving "figure" (mirrors RH20T's static_clamp).
    container_xyzs = [pose_xyz(m["containerPose"]) for m in meas]
    container_clamp = [_median([xyz[k] for xyz in container_xyzs]) for k in range(3)]
    max_container_excursion = max(
        (sum((xyz[k] - container_clamp[k]) ** 2 for k in range(3)) ** 0.5 for xyz in container_xyzs),
        default=0.0,
    )

    frames: List[Json] = []
    for i, (obs, m) in enumerate(zip(observations, meas)):
        gp = _obs_field(obs, "gripper_pose")
        if gp is None or len(gp) < 7:
            raise ValueError(f"observation {i} has no 7-dof gripper_pose (got {gp!r})")
        go = _obs_field(obs, "gripper_open")
        if go is None:
            raise ValueError(f"observation {i} has no gripper_open")
        px, py, pz = float(gp[0]), float(gp[1]), float(gp[2])
        item_xyz = pose_xyz(m["itemPose"])
        item_conf = float(get_any(m["itemPose"], "confidence", default=1.0))
        container_conf = float(get_any(m["containerPose"], "confidence", default=1.0))
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
            "objectPoses": {
                _ITEM_BODY_ID: _pose(item_xyz, item_conf),
                _CONTAINER_BODY_ID: _pose(container_clamp, container_conf),
            },
            "articulation": {},
        })

    return assemble_rollout(
        bodies=bodies,
        frames=frames,
        object_id_map={},  # external trace: no target identities to map
        physical_validity_reason=(
            # Deliberately task-name-free: the rollout names neither the RLBench task, the
            # variation, nor any handle anywhere (even in honest diagnostics prose), so it is
            # fully source-blind — see tests/test_rlbench_object_inside_container.py quarantine.
            "external RLBench simulation trace: csg cannot re-check CoppeliaSim physics; "
            "physics-unverified by contract (csg/validity.md)"),
        extra_diagnostics={
            "controlRateHz": float(control_rate_hz),
            "numObservations": len(observations),
            "neutralBodyCount": len(bodies),
            "maxStaticBodyMotionM": max_container_excursion,
            "staticBodyClampApplied": [_CONTAINER_BODY_ID],
            "source": "rlbench",
        },
    )
