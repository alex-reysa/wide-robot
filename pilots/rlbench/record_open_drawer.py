#!/usr/bin/env python3
"""Record RLBench ``OpenDrawer`` demos and emit leakage-clean ``csg.rollout.v0`` traces.

This is the live half of the pilot — the only place RLBench / PyRep / CoppeliaSim are
touched. They are imported **lazily** (inside functions), so this module imports fine
on a machine with none of them installed; ``rlbench_available()`` reports whether a
live record is possible.

**Handle quarantine — the load-bearing discipline.** Every RLBench / CoppeliaSim
identity (``drawer_frame``, ``drawer_joint_bottom|middle|top``, ``waypoint_anchor_*``,
the variation names ``bottom``/``middle``/``top``, the task name) lives ONLY in this
module. They are used to *read neutral numbers* — the drawer body's pose, its bounding
box, and its prismatic joint extension per frame — and are never written into the
rollout. What crosses into :func:`pilots.rlbench.adapter.rlbench_demo_to_rollout` is a
list of neutral ``measurements`` (:data:`~pilots.rlbench.adapter.NEUTRAL_MEASUREMENT_FIELDS`,
keys only: ``frameIndex``/``timeS``/``bodyPose``/``articulationValue``/``bodySizeM``/
``sizeApproximate``). The converter re-validates that shape and rejects any extra key,
and :func:`assert_rollout_leakage_clean` re-checks the assembled rollout — so a handle
name leaking through is caught twice. Variation / demo / task metadata is written to a
separate **sidecar** (``*.summary.json``), never into the rollout the extractor reads.

Live record (out of band — needs CoppeliaSim v4.1.0 + PyRep + RLBench per the RLBench
README, https://github.com/stepjam/RLBench):

    python -m pilots.rlbench.record_open_drawer \
        --variations bottom,middle,top --demos-per-variation 1 \
        --out-dir pilots/rlbench/_out --verify

Outputs (under ``--out-dir``, gitignored): one ``open_drawer_<variation>_demo<NN>.rollout.json``
per demo plus a matching ``*.summary.json`` sidecar, and a top-level ``run_summary.json``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import write_json

from .adapter import NEUTRAL_MEASUREMENT_FIELDS, rlbench_demo_to_rollout

# ---------------------------------------------------------------------------
# Quarantined RLBench / CoppeliaSim identities — NEVER written into a rollout.
# ---------------------------------------------------------------------------
TASK_NAME = "open_drawer"  # csg gold task AND RLBench task name; stays out of the rollout
CONTROL_RATE_HZ = 20.0
_JOINT_MATCH_EPS = 1e-4  # tolerance for matching the live joint extension to its low-dim slot

# The three official OpenDrawer variations and the CoppeliaSim handles each one drives.
# (rlbench/tasks/open_drawer.py: a shared ``drawer_frame`` with three prismatic joints.)
_QUARANTINED_HANDLES: Dict[str, Dict[str, str]] = {
    "bottom": {"joint": "drawer_joint_bottom", "frame": "drawer_frame", "anchor": "waypoint_anchor_bottom"},
    "middle": {"joint": "drawer_joint_middle", "frame": "drawer_frame", "anchor": "waypoint_anchor_middle"},
    "top": {"joint": "drawer_joint_top", "frame": "drawer_frame", "anchor": "waypoint_anchor_top"},
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
def _neutral_body_pose(frame_obj: Any) -> Dict[str, Any]:
    """Drawer body pose as a neutral CSG pose. ``frame_obj`` is a live PyRep object
    (the quarantined ``drawer_frame`` shape); only its numeric pose escapes."""
    x, y, z, qx, qy, qz, qw = (float(v) for v in frame_obj.get_pose())
    return {
        "frameId": "world",
        "positionM": {"x": x, "y": y, "z": z},
        "orientationWxyz": {"w": qw, "x": qx, "y": qy, "z": qz},
        "confidence": 1.0,
    }


def _neutral_body_size(frame_obj: Any) -> List[float]:
    """Axis-aligned full extents (meters) from the quarantined shape's bounding box."""
    x0, x1, y0, y1, z0, z1 = (float(v) for v in frame_obj.get_bounding_box())
    return [abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)]


def _task_low_dim_state(obs: Any) -> List[Any]:
    """Observation low-dim state as a list; RLBench supplies this as a NumPy array."""
    state = getattr(obs, "task_low_dim_state", None)
    if state is None:
        return []
    return list(state)


def _resolve_joint_index(joint_obj: Any, demo: Sequence[Any]) -> int:
    """Index of this variation's drawer joint inside ``Observation.task_low_dim_state``.

    The neutral, post-hoc per-frame articulation source for an RLBench demo is each
    ``Observation.task_low_dim_state`` (the task's low-dim state vector). For OpenDrawer
    that vector contains the active drawer joint's position; this maps the quarantined
    joint handle to its slot by matching the live joint extension against the recorded
    state.

    Resolved against the LAST frame (drawer OPEN), not frame 0: at frame 0 the drawer
    is closed and the joint reads ~0, indistinguishable from the many static zeros in
    the low-dim state (identity-quaternion components, the other closed joints), so a
    nearest-value match there silently picks the wrong slot and the articulation never
    ramps. The live joint handle is read after recording, so its position is the open
    extension; matching that distinguishable non-zero value at the open frame is
    unambiguous, and a tie (several slots within ``_JOINT_MATCH_EPS``) is raised rather
    than guessed. Confirm against the installed RLBench's ``OpenDrawer`` if the layout
    differs.
    """
    if not demo:
        raise RuntimeError("empty demo: cannot resolve the drawer joint slot")
    state = _task_low_dim_state(demo[-1])
    if not state:
        raise RuntimeError(
            "Observation.task_low_dim_state is empty; cannot recover the drawer joint "
            "extension without it (check the installed RLBench OpenDrawer task config).")
    target = float(joint_obj.get_joint_position())
    matches = [i for i in range(len(state)) if abs(float(state[i]) - target) <= _JOINT_MATCH_EPS]
    if len(matches) != 1:
        raise RuntimeError(
            f"could not uniquely resolve the drawer joint slot in task_low_dim_state "
            f"(open extension {target:.5f}; {len(matches)} of {len(state)} slots within "
            f"{_JOINT_MATCH_EPS}). Confirm the OpenDrawer low-dim layout for the installed "
            "RLBench version rather than guessing a slot.")
    return matches[0]


def _demo_to_measurements(demo: Sequence[Any], *, frame_obj: Any, joint_obj: Any) -> List[Dict[str, Any]]:
    """Build neutral per-frame measurements from a recorded demo + live handles.

    The drawer *frame* is static within a demo (only the drawer slides on its joint),
    so its pose + bounding box are read once from the quarantined shape; per-frame the
    only dynamic value is the prismatic joint extension, taken from each
    ``Observation.task_low_dim_state``. No handle name appears in the output.
    """
    body_pose = _neutral_body_pose(frame_obj)
    body_size = _neutral_body_size(frame_obj)
    joint_index = _resolve_joint_index(joint_obj, demo)
    measurements: List[Dict[str, Any]] = []
    for i, obs in enumerate(demo):
        state = _task_low_dim_state(obs)
        articulation = float(state[joint_index]) if joint_index < len(state) else 0.0
        measurements.append({
            "frameIndex": i,
            "timeS": i / CONTROL_RATE_HZ,
            "bodyPose": body_pose,
            "articulationValue": articulation,
            "bodySizeM": list(body_size),
            "sizeApproximate": False,
        })
    # Defence in depth: a measurement must carry only neutral keys (the converter
    # re-checks, but failing here names the recorder as the leak source).
    for i, m in enumerate(measurements):
        extra = sorted(set(m) - set(NEUTRAL_MEASUREMENT_FIELDS))
        if extra:
            raise RuntimeError(f"recorder built a non-neutral measurement {i}: extra keys {extra}")
    return measurements


# ---------------------------------------------------------------------------
# Live recording (lazy RLBench imports).
# ---------------------------------------------------------------------------
def _make_task_env(env: Any, variation: str) -> Any:  # pragma: no cover - needs RLBench
    """Open an RLBench task env for ``OpenDrawer`` at the given variation index."""
    from rlbench.tasks import OpenDrawer

    task_env = env.get_task(OpenDrawer)
    var_index = list(DEFAULT_VARIATIONS).index(variation)
    var_index = var_index % max(1, task_env.variation_count())
    task_env.set_variation(var_index)
    return task_env


def record_variation(variation: str, *, amount: int = 1, headless: bool = True,
                     env: Optional[Any] = None) -> List[Dict[str, Any]]:  # pragma: no cover - needs RLBench
    """Record ``amount`` ``OpenDrawer`` demos for one variation and return, per demo,
    ``{"demo": Demo, "measurements": [...]}`` — RLBench objects + neutral measurements.

    The Demo (``Observation`` list) supplies the effector pose + gripper state the
    converter reads by attribute; the neutral measurements supply the drawer body
    pose/size/articulation, read from quarantined handles.
    """
    if variation not in _QUARANTINED_HANDLES:
        raise ValueError(f"unknown OpenDrawer variation {variation!r}; expected one of {DEFAULT_VARIATIONS}")

    from pyrep.objects.joint import Joint
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
        for _ in range(amount):
            # One demo at a time so the static drawer-frame handle is read at this
            # demo's (randomised) configuration.
            demo = task_env.get_demos(1, live_demos=True)[0]
            frame_obj = Shape(handles["frame"])
            joint_obj = Joint(handles["joint"])
            measurements = _demo_to_measurements(list(demo), frame_obj=frame_obj, joint_obj=joint_obj)
            out.append({"demo": list(demo), "measurements": measurements})
    finally:
        if own_env:
            env.shutdown()
    return out


# ---------------------------------------------------------------------------
# Orchestration: record -> convert -> write rollout + sidecar (+ optional verify).
# ---------------------------------------------------------------------------
def _sidecar(variation: str, demo_index: int, rollout: Mapping[str, Any], *,
             verification: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Run-summary metadata for a demo. Variation/task/handle names live HERE, not in
    the rollout the extractor reads (docs/rlbench_external_trace_pilot.md)."""
    sidecar = {
        "schemaVersion": "csg.rlbench_pilot_summary.v0",
        "task": TASK_NAME,
        "variation": variation,
        "demoIndex": demo_index,
        "controlRateHz": CONTROL_RATE_HZ,
        "numFrames": len(rollout.get("frames", []) or []),
        "quarantinedHandles": _QUARANTINED_HANDLES[variation],
        "note": ("RLBench/CoppeliaSim identities (handles, variation, task) are recorded here "
                 "for provenance and are deliberately absent from the rollout."),
    }
    if verification is not None:
        sidecar["verification"] = dict(verification)
    return sidecar


def build_rollout(record: Mapping[str, Any]) -> Mapping[str, Any]:
    """Convert one ``record_variation`` record (demo + measurements) to a rollout."""
    return rlbench_demo_to_rollout(
        record["demo"], task=TASK_NAME, control_rate_hz=CONTROL_RATE_HZ,
        measurements=record["measurements"],
    )


def write_outputs(out_dir: str | Path, variation: str, demo_index: int,
                  rollout: Mapping[str, Any], *, verify: bool = False,
                  gold_dir: str | Path = "gold_tests") -> Dict[str, str]:
    """Write one demo's rollout + sidecar; optionally verify + confusion into the sidecar."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stem = f"{TASK_NAME}_{variation}_demo{demo_index:02d}"
    rollout_path = out / f"{stem}.rollout.json"
    sidecar_path = out / f"{stem}.summary.json"

    verification: Optional[Dict[str, Any]] = None
    if verify:
        # Imported here to keep the recorder importable even if the verifier seam
        # changes; both live in pilots/rlbench and consume only frozen csg.
        from csg.common import load_json
        from .run_external import external_confusion_report, load_gold_targets, verify_external_rollout

        target = load_json(Path(gold_dir) / TASK_NAME / "target.json")
        case = verify_external_rollout(target, rollout, case_name=TASK_NAME)
        confusion = external_confusion_report(rollout, load_gold_targets(gold_dir), expected_case=TASK_NAME)
        verification = {
            "passed": case["passed"],
            "matcherPassed": case["matcherPassed"],
            "leakageClean": case["leakageClean"],
            "physicalValidity": case["physicalValidity"],
            "hardMismatches": case["hardMismatches"],
            "confusion": confusion,
        }

    write_json(rollout_path, rollout)
    write_json(sidecar_path, _sidecar(variation, demo_index, rollout, verification=verification))
    return {"rollout": str(rollout_path), "summary": str(sidecar_path)}


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover - needs RLBench
    parser = argparse.ArgumentParser(
        description="Record RLBench OpenDrawer demos and emit leakage-clean csg.rollout.v0 traces.")
    parser.add_argument("--variations", default=",".join(DEFAULT_VARIATIONS),
                        help="comma-separated OpenDrawer variations (bottom,middle,top)")
    parser.add_argument("--demos-per-variation", type=int, default=1)
    parser.add_argument("--out-dir", default="pilots/rlbench/_out")
    parser.add_argument("--gold-dir", default="gold_tests")
    parser.add_argument("--headless", dest="headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--verify", action="store_true",
                        help="run the frozen verifier + confusion on each rollout, into the sidecar")
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
                                  verify=args.verify, gold_dir=args.gold_dir)
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
