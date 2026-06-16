#!/usr/bin/env python3
"""Record RLBench ``PutItemInDrawer`` demos and emit leakage-clean ``csg.rollout.v0`` traces.

The live half of the Phase 2F-4 pilot — the only place RLBench / PyRep / CoppeliaSim are
touched (imported **lazily**, inside functions, so this module imports fine with none of
them installed; ``rlbench_available()`` reports whether a live record is possible). It is
the two-body sibling of :mod:`pilots.rlbench.record_open_drawer`.

**Handle quarantine — the load-bearing discipline.** Every RLBench / CoppeliaSim identity
(the item shape, the ``drawer_frame`` + ``drawer_joint_<variation>``, the
``success_<variation>`` proximity sensor, the variation names, the task name) lives ONLY
in this module. They are used to *read neutral numbers* — the item's per-frame pose + size
and the container volume's pose + bounding box — and are never written into the rollout.
What crosses into :func:`pilots.rlbench.adapter_object_inside_container.put_item_in_drawer_demo_to_rollout`
is a list of neutral ``measurements`` (:data:`~pilots.rlbench.adapter_object_inside_container.NEUTRAL_MEASUREMENT_FIELDS`,
keys only: ``frameIndex``/``timeS``/``itemPose``/``itemSizeM``/``containerPose``/
``containerSizeM``/``sizeApproximate``). The converter re-validates that shape and rejects
any extra key, and :func:`assert_rollout_leakage_clean` re-checks the assembled rollout — so
a handle name leaking through is caught twice. Variation/demo/task metadata is written to a
separate **sidecar** (``*.summary.json``), never into the rollout the extractor reads. The
success proximity sensor's *boolean* is NEVER emitted — only its pose + bounding box, as the
container volume the item must end inside.

**Handle/layout names below are the best-guess CoppeliaSim names for the installed RLBench
``PutItemInDrawer`` and MUST be confirmed at capture time** (against the task's
``.ttm``/``__init__``); the resolvers fail loudly rather than guess if a handle or
low-dim-state layout differs.

Live record (out of band — needs CoppeliaSim v4.1.0 + PyRep + RLBench per the RLBench
README, https://github.com/stepjam/RLBench):

    python -m pilots.rlbench.record_put_item_in_drawer \
        --variations bottom,middle,top --demos-per-variation 3 \
        --out-dir pilots/rlbench/_out --verify

Outputs (under ``--out-dir``, gitignored): one
``put_item_in_drawer_<variation>_demo<NN>.rollout.json`` per demo plus a matching
``*.summary.json`` sidecar, and a top-level ``run_summary.json``.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import write_json

from .adapter_object_inside_container import (
    NEUTRAL_MEASUREMENT_FIELDS,
    put_item_in_drawer_demo_to_rollout,
)

# ---------------------------------------------------------------------------
# Quarantined RLBench / CoppeliaSim identities — NEVER written into a rollout.
# ---------------------------------------------------------------------------
TASK_NAME = "put_item_in_drawer"  # RLBench task name; the gold containment task is put_cube_in_tray
CONTROL_RATE_HZ = 20.0
_POSE_MATCH_EPS = 1e-3  # tolerance for locating the item's pose triple in task_low_dim_state
# RLBench places the item with a top-down drop: it is geometrically INSIDE the drawer only on
# the FINAL demo frame (the release instant), so the terminal containment transition coincides
# with the last timestamp and the frozen extractor's relation timeline can end on the NEAR
# transition-source rather than INSIDE. We therefore step the live sim a few frames after the
# demo to record the item GENUINELY AT REST inside the drawer (real physics, verified static in
# the probe) so the achieved INSIDE relation persists past the transition and registers as the
# terminal relation. These are honest captured frames, not fabricated padding.
_SETTLE_FRAMES = 6

# The three drawer variations and the CoppeliaSim handles each one drives. The item shape
# is shared; the container volume is the variation's success proximity sensor.
# CONFIRM these names against the installed RLBench PutItemInDrawer task before a live run.
_QUARANTINED_HANDLES: Dict[str, Dict[str, str]] = {
    "bottom": {"item": "item", "drawer_frame": "drawer_frame",
               "joint": "drawer_joint_bottom", "success_sensor": "success_bottom"},
    "middle": {"item": "item", "drawer_frame": "drawer_frame",
               "joint": "drawer_joint_middle", "success_sensor": "success_middle"},
    "top": {"item": "item", "drawer_frame": "drawer_frame",
            "joint": "drawer_joint_top", "success_sensor": "success_top"},
}
DEFAULT_VARIATIONS = ("bottom", "middle", "top")


def rlbench_available() -> bool:
    """True iff RLBench + PyRep import (a live CoppeliaSim is still required to record)."""
    try:  # pragma: no cover - exercised only where RLBench is installed
        import rlbench  # noqa: F401
        import pyrep  # noqa: F401
    except Exception:
        return False
    return True


# ---------------------------------------------------------------------------
# Neutralisation: RLBench scene state -> neutral measurements (no handle names out).
# ---------------------------------------------------------------------------
def _neutral_size(obj: Any) -> List[float]:  # pragma: no cover - needs RLBench
    """Axis-aligned full extents (meters) from a live shape/sensor's bounding box."""
    x0, x1, y0, y1, z0, z1 = (float(v) for v in obj.get_bounding_box())
    return [abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)]


def _container_volume(sensor: Any) -> Dict[str, Any]:  # pragma: no cover - needs RLBench
    """Container volume = the success sensor's POSE (origin) as centre + its bounding-box extents.

    The RLBench success ProximitySensor sits in the drawer; its pose origin anchors the drawer
    region and its extents (~0.3 x 0.3 x 0.083) approximate the drawer opening + depth. We do NOT
    re-centre on the bbox's asymmetric AABB centre: the sensor is a *passage* detector (it trips as
    the item descends through it) and the item then settles on the drawer floor slightly BELOW the
    bbox; anchoring the box at the origin (extending to origin - extent/2) keeps the drawer floor
    inside the volume, so the at-rest item is INSIDE across all three drawer heights. Validated:
    9/9 terminal INSIDE; the AABB-centre alternative wrongly excludes the middle drawer's at-rest
    item (floor raised ~3 cm above the rest). Returns the pose + extents for body_001."""
    px, py, pz = (float(v) for v in sensor.get_position())
    return {
        "frameId": "world",
        "positionM": {"x": px, "y": py, "z": pz},
        "orientationWxyz": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "confidence": 1.0,
    }


def _task_low_dim_state(obs: Any) -> List[Any]:
    """Observation low-dim state as a list; RLBench supplies this as a NumPy array."""
    state = getattr(obs, "task_low_dim_state", None)
    if state is None:
        return []
    return list(state)


def _resolve_item_pose_offset(item_obj: Any, demo: Sequence[Any]) -> int:  # pragma: no cover - needs RLBench
    """Index where the item's (x, y, z) appears as a contiguous triple in
    ``Observation.task_low_dim_state``.

    The neutral, post-hoc per-frame item position source for an RLBench demo is each
    ``Observation.task_low_dim_state``. This maps the quarantined item handle to its slot
    by matching the live item position (read after recording, so it is the item's final
    resting position inside the drawer — a distinguishable point) against contiguous
    triples in the LAST frame's state. A tie (several offsets within ``_POSE_MATCH_EPS``)
    is raised rather than guessed; confirm the PutItemInDrawer low-dim layout for the
    installed RLBench version if resolution fails.
    """
    if not demo:
        raise RuntimeError("empty demo: cannot resolve the item pose slot")
    state = _task_low_dim_state(demo[-1])
    if not state:
        raise RuntimeError(
            "Observation.task_low_dim_state is empty; cannot recover the item pose without it "
            "(check the installed RLBench PutItemInDrawer task config).")
    tx, ty, tz = (float(v) for v in item_obj.get_position())
    matches = [
        i for i in range(len(state) - 2)
        if abs(float(state[i]) - tx) <= _POSE_MATCH_EPS
        and abs(float(state[i + 1]) - ty) <= _POSE_MATCH_EPS
        and abs(float(state[i + 2]) - tz) <= _POSE_MATCH_EPS
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"could not uniquely resolve the item pose triple in task_low_dim_state "
            f"(item position {(tx, ty, tz)}; {len(matches)} candidate offsets within "
            f"{_POSE_MATCH_EPS}). Confirm the PutItemInDrawer low-dim layout for the installed "
            "RLBench version rather than guessing.")
    return matches[0]


def _item_measurement(frame_index: int, xyz, item_size, container_pose,
                      container_size) -> Dict[str, Any]:  # pragma: no cover - needs RLBench
    ix, iy, iz = (float(v) for v in xyz)
    return {
        "frameIndex": frame_index,
        "timeS": frame_index / CONTROL_RATE_HZ,
        "itemPose": {"frameId": "world", "positionM": {"x": ix, "y": iy, "z": iz},
                     "orientationWxyz": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, "confidence": 1.0},
        "itemSizeM": list(item_size),
        "containerPose": container_pose,
        "containerSizeM": list(container_size),
        "sizeApproximate": False,
    }


def _assert_measurements_neutral(measurements: Sequence[Mapping[str, Any]]) -> None:  # pragma: no cover - needs RLBench
    # Defence in depth: a measurement must carry only neutral keys (the converter re-checks,
    # but failing here names the recorder as the leak source).
    for i, m in enumerate(measurements):
        extra = sorted(set(m) - set(NEUTRAL_MEASUREMENT_FIELDS))
        if extra:
            raise RuntimeError(f"recorder built a non-neutral measurement {i}: extra keys {extra}")


def _demo_to_measurements(demo: Sequence[Any], *, item_obj: Any, item_size,
                          container_pose, container_size) -> List[Dict[str, Any]]:  # pragma: no cover - needs RLBench
    """Build neutral per-frame measurements for the recorded demo frames.

    The item moves per frame (its position read from each ``Observation.task_low_dim_state``);
    sizes + the container volume are precomputed once by the caller (the drawer interior is
    static within a demo). No handle name, and no success boolean, appears in the output.
    """
    item_offset = _resolve_item_pose_offset(item_obj, demo)
    measurements = [
        _item_measurement(i, [_task_low_dim_state(obs)[item_offset + k] for k in range(3)],
                          item_size, container_pose, container_size)
        for i, obs in enumerate(demo)
    ]
    _assert_measurements_neutral(measurements)
    return measurements


# ---------------------------------------------------------------------------
# Live recording (lazy RLBench imports).
# ---------------------------------------------------------------------------
def _make_task_env(env: Any, variation: str) -> Any:  # pragma: no cover - needs RLBench
    """Open an RLBench task env for ``PutItemInDrawer`` at the given variation index."""
    from rlbench.tasks import PutItemInDrawer

    task_env = env.get_task(PutItemInDrawer)
    var_index = list(DEFAULT_VARIATIONS).index(variation)
    var_index = var_index % max(1, task_env.variation_count())
    task_env.set_variation(var_index)
    return task_env


def record_variation(variation: str, *, amount: int = 1, headless: bool = True,
                     env: Optional[Any] = None) -> List[Dict[str, Any]]:  # pragma: no cover - needs RLBench
    """Record ``amount`` ``PutItemInDrawer`` demos for one variation and return, per demo,
    ``{"demo": Demo, "measurements": [...]}`` — RLBench objects + neutral measurements."""
    if variation not in _QUARANTINED_HANDLES:
        raise ValueError(f"unknown PutItemInDrawer variation {variation!r}; expected one of {DEFAULT_VARIATIONS}")

    from pyrep.objects.proximity_sensor import ProximitySensor
    from pyrep.objects.shape import Shape

    own_env = env is None
    if own_env:
        from rlbench.action_modes.action_mode import MoveArmThenGripper
        from rlbench.action_modes.arm_action_modes import JointVelocity
        from rlbench.action_modes.gripper_action_modes import Discrete
        from rlbench.environment import Environment
        from rlbench.observation_config import ObservationConfig

        obs_config = ObservationConfig()
        obs_config.set_all(True)
        env = Environment(
            action_mode=MoveArmThenGripper(arm_action_mode=JointVelocity(), gripper_action_mode=Discrete()),
            obs_config=obs_config,
            headless=headless,
        )
        env.launch()

    handles = _QUARANTINED_HANDLES[variation]
    out: List[Dict[str, Any]] = []
    try:
        task_env = _make_task_env(env, variation)
        pr = getattr(env, "_pyrep", None)
        for _ in range(amount):
            demo = list(task_env.get_demos(1, live_demos=True)[0])
            item_obj = Shape(handles["item"])
            success_sensor_obj = ProximitySensor(handles["success_sensor"])
            item_size = _neutral_size(item_obj)
            container_pose = _container_volume(success_sensor_obj)
            container_size = _neutral_size(success_sensor_obj)
            measurements = _demo_to_measurements(
                demo, item_obj=item_obj, item_size=item_size,
                container_pose=container_pose, container_size=container_size)
            obs_list: List[Any] = list(demo)
            # Honest settle tail: the item rests inside the drawer after release; the demo ends
            # the instant the success sensor trips (1 INSIDE frame). Step the live sim to capture
            # genuine at-rest frames so the achieved INSIDE relation persists to the terminal.
            last_gp = [float(v) for v in getattr(demo[-1], "gripper_pose")]
            if pr is not None:
                for _s in range(_SETTLE_FRAMES):
                    pr.step()
                    xyz = [float(v) for v in item_obj.get_position()]
                    measurements.append(_item_measurement(
                        len(measurements), xyz, item_size, container_pose, container_size))
                    obs_list.append({"gripper_pose": last_gp, "gripper_open": 1.0})
            _assert_measurements_neutral(measurements)
            out.append({"demo": obs_list, "measurements": measurements})
    finally:
        if own_env:
            env.shutdown()
    return out


# ---------------------------------------------------------------------------
# Orchestration: record -> convert -> write rollout + sidecar (+ optional verify).
# ---------------------------------------------------------------------------
def _sidecar(variation: str, demo_index: int, rollout: Mapping[str, Any], *,
             verification: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Run-summary metadata for a demo. Variation/task/handle names live HERE, not in the
    rollout the extractor reads (docs/rlbench_external_trace_pilot.md)."""
    sidecar = {
        "schemaVersion": "csg.rlbench_pilot_summary.v0",
        "task": TASK_NAME,
        "variation": variation,
        "demoIndex": demo_index,
        "controlRateHz": CONTROL_RATE_HZ,
        "numFrames": len(rollout.get("frames", []) or []),
        "quarantinedHandles": _QUARANTINED_HANDLES[variation],
        "note": ("RLBench/CoppeliaSim identities (item/drawer/success-sensor handles, variation, task) "
                 "are recorded here for provenance and are deliberately absent from the rollout. The "
                 "success sensor's boolean is never emitted — only its pose + bounding box, as the "
                 "container volume."),
    }
    if verification is not None:
        sidecar["verification"] = dict(verification)
    return sidecar


def build_rollout(record: Mapping[str, Any]) -> Mapping[str, Any]:
    """Convert one ``record_variation`` record (demo + measurements) to a rollout."""
    return put_item_in_drawer_demo_to_rollout(
        record["demo"], task=TASK_NAME, control_rate_hz=CONTROL_RATE_HZ,
        measurements=record["measurements"],
    )


def write_outputs(out_dir: str | Path, variation: str, demo_index: int,
                  rollout: Mapping[str, Any], *, verify: bool = False,
                  gold_dir: str | Path = "gold_tests",
                  target_dir: str | Path = "pilots/rlbench/targets") -> Dict[str, str]:
    """Write one demo's rollout + sidecar; optionally verify the pilot targets + gold
    confusion into the sidecar.

    Unlike OpenDrawer (whose gold task IS open_drawer), PutItemInDrawer's gold containment
    task is ``put_cube_in_tray``; the pilot's own acceptance targets are the two
    ``object_inside_container_*`` diagnostics. ``--verify`` records the relation-event and
    terminal-only verdicts plus the 1×N gold confusion (expected_case=put_cube_in_tray).
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stem = f"{TASK_NAME}_{variation}_demo{demo_index:02d}"
    rollout_path = out / f"{stem}.rollout.json"
    sidecar_path = out / f"{stem}.summary.json"

    verification: Optional[Dict[str, Any]] = None
    if verify:
        from csg.common import load_json
        from .run_external import external_confusion_report, load_gold_targets, verify_external_rollout

        tdir = Path(target_dir)
        rel = verify_external_rollout(
            load_json(tdir / "object_inside_container_relation_event.json"), rollout,
            case_name="rlbench_object_inside_container_relation_event")
        term = verify_external_rollout(
            load_json(tdir / "object_inside_container_terminal_only.json"), rollout,
            case_name="rlbench_object_inside_container_terminal_only")
        confusion = external_confusion_report(
            rollout, load_gold_targets(gold_dir), expected_case="put_cube_in_tray")
        verification = {
            "relationEvent": {"passed": rel["passed"], "hardMismatches": rel["hardMismatches"]},
            "terminalOnly": {"passed": term["passed"], "hardMismatches": term["hardMismatches"]},
            "leakageClean": rel["leakageClean"],
            "physicalValidity": rel["physicalValidity"],
            "confusion": confusion,
        }

    write_json(rollout_path, rollout)
    write_json(sidecar_path, _sidecar(variation, demo_index, rollout, verification=verification))
    return {"rollout": str(rollout_path), "summary": str(sidecar_path)}


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover - needs RLBench
    parser = argparse.ArgumentParser(
        description="Record RLBench PutItemInDrawer demos and emit leakage-clean csg.rollout.v0 traces.")
    parser.add_argument("--variations", default=",".join(DEFAULT_VARIATIONS),
                        help="comma-separated PutItemInDrawer variations (bottom,middle,top)")
    parser.add_argument("--demos-per-variation", type=int, default=3)
    parser.add_argument("--out-dir", default="pilots/rlbench/_out")
    parser.add_argument("--gold-dir", default="gold_tests")
    parser.add_argument("--target-dir", default="pilots/rlbench/targets")
    parser.add_argument("--headless", dest="headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--verify", action="store_true",
                        help="run the frozen verifier (pilot targets + gold confusion) on each rollout, into the sidecar")
    args = parser.parse_args(argv)

    if not rlbench_available():
        raise SystemExit(
            "RLBench/PyRep not importable. Install CoppeliaSim v4.1.0 + PyRep + RLBench out of band "
            "(see docs/rlbench_external_trace_pilot.md); this recorder needs a live CoppeliaSim.")

    variations = [v.strip() for v in str(args.variations).split(",") if v.strip()]
    written: List[Dict[str, Any]] = []
    for variation in variations:
        records = record_variation(variation, amount=args.demos_per_variation, headless=args.headless)
        for demo_index, record in enumerate(records):
            rollout = build_rollout(record)
            paths = write_outputs(args.out_dir, variation, demo_index, rollout,
                                  verify=args.verify, gold_dir=args.gold_dir, target_dir=args.target_dir)
            written.append({"variation": variation, "demoIndex": demo_index, **paths})
            print(f"wrote {paths['rollout']}")

    run_summary = Path(args.out_dir) / "run_summary.json"
    write_json(run_summary, {
        "schemaVersion": "csg.rlbench_pilot_run.v0",
        "task": TASK_NAME,
        "variations": variations,
        "demosPerVariation": args.demos_per_variation,
        "verified": bool(args.verify),
        "outputs": written,
    })
    print(f"run summary written to {run_summary}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
