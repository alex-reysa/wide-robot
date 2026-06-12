#!/usr/bin/env python3
"""Physical-validity checks over a :class:`SimTrace` (``csg/validity.md`` §33).

Pure stdlib + the shared predicate grammar — **no** ``mujoco`` import — so the
six checks are unit-testable on synthetic traces and the verdict uses the same
geometry the extractor and matcher use.

The result is a :class:`ValidityReport`: a single ``physicalValidity`` boolean
(AND over the *applicable* checks), a one-line ``reason`` naming the first
failure, and a per-check breakdown that becomes the sidecar
``validity_report.json``. A check that cannot apply to a given task (e.g.
articulation limits on a pure pick-place) is reported ``applicable: false`` and
never counts against the verdict — honesty over a false green.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from ... import predicates as P
from ...common import dist3, quat_angle
from .trace import SimTrace, Vec3


@dataclass(frozen=True)
class ValidityConfig:
    penetration_tol_m: float = 0.005          # check 1 (matches tests/test_validity.py)
    max_step_translation_m: float = 0.05      # check 2, per recorded frame
    max_step_rotation_rad: float = 0.5        # check 2
    settle_window_s: float = 1.0              # check 3
    settle_vel_eps_m_s: float = 0.05          # check 3
    settle_drop_eps_m: float = 0.02           # check 3
    support_normal_z_min: float = 0.4         # check 3 (upward contact normal)
    grip_width_tol_m: float = 0.006           # check 4
    force_tol_frac: float = 0.05              # check 4
    ik_residual_tol_m: float = 0.005          # check 5 (informational)
    joint_limit_margin_rad: float = 1e-3      # check 5 / 6


DEFAULT = ValidityConfig()


@dataclass
class ValidityReport:
    passed: bool
    reason: str
    checks: Dict[str, dict] = field(default_factory=dict)

    def to_json(self) -> dict:
        return {"physicalValidity": self.passed, "reason": self.reason, "checks": self.checks}


def _check(passed: bool, applicable: bool, detail: str) -> dict:
    return {"passed": bool(passed), "applicable": bool(applicable), "detail": detail}


def _box(center: Vec3, size: Vec3) -> P.Box:
    return P.box_from(center, size)


# -----------------------------------------------------------------------------
# Individual checks
# -----------------------------------------------------------------------------


def check_non_penetration(trace: SimTrace, cfg: ValidityConfig) -> dict:
    worst = 0.0
    worst_detail = ""
    for s in trace.steps:
        for c in s.contacts:
            if c.depth_m > worst:
                worst = c.depth_m
                worst_detail = f"{c.body_a}~{c.body_b} depth {c.depth_m*1000:.1f}mm at t={s.time_s:.2f}s"
    passed = worst <= cfg.penetration_tol_m
    detail = (f"max interpenetration {worst*1000:.1f}mm <= {cfg.penetration_tol_m*1000:.0f}mm"
              if passed else f"interpenetration {worst*1000:.1f}mm > {cfg.penetration_tol_m*1000:.0f}mm ({worst_detail})")
    return _check(passed, True, detail)


def check_pose_continuity(trace: SimTrace, cfg: ValidityConfig) -> dict:
    if len(trace.steps) < 2:
        return _check(True, False, "fewer than two frames")
    worst_t = 0.0
    worst_r = 0.0
    bad = ""
    for a, b in zip(trace.steps, trace.steps[1:]):
        for oid, (pos, quat) in b.object_poses.items():
            if oid not in a.object_poses:
                continue
            pa, qa = a.object_poses[oid]
            dt = dist3(pa, pos)
            dr = quat_angle(qa, quat)
            if dt > worst_t:
                worst_t = dt
            if dr > worst_r:
                worst_r = dr
            if dt > cfg.max_step_translation_m and not bad:
                bad = f"{oid} jumped {dt*1000:.0f}mm between t={a.time_s:.2f} and {b.time_s:.2f}"
            elif dr > cfg.max_step_rotation_rad and not bad:
                bad = f"{oid} rotated {dr:.2f}rad between t={a.time_s:.2f} and {b.time_s:.2f}"
    passed = worst_t <= cfg.max_step_translation_m and worst_r <= cfg.max_step_rotation_rad
    detail = (f"max step {worst_t*1000:.0f}mm / {worst_r:.2f}rad within limits"
              if passed else f"teleport-like step: {bad}")
    return _check(passed, True, detail)


def _window_end_index(trace: SimTrace, start: int, cfg: ValidityConfig) -> int:
    t0 = trace.steps[start].time_s
    end = start
    for i in range(start, len(trace.steps)):
        if trace.steps[i].time_s <= t0 + cfg.settle_window_s:
            end = i
        else:
            break
    return end


def check_quasi_static_support(trace: SimTrace, cfg: ValidityConfig) -> dict:
    fig = trace.figure_id
    if not trace.release_indices or not fig:
        return _check(True, False, "no release / no manipulated object")
    for r in trace.release_indices:
        if r >= len(trace.steps) or fig not in trace.steps[r].object_poses:
            continue
        end = _window_end_index(trace, r, cfg)
        rpos = trace.steps[r].object_poses[fig][0]
        epos = trace.steps[end].object_poses[fig][0]
        # (a) speed at window end
        if end > 0 and fig in trace.steps[end - 1].object_poses:
            dt = max(1e-6, trace.steps[end].time_s - trace.steps[end - 1].time_s)
            speed = dist3(trace.steps[end - 1].object_poses[fig][0], epos) / dt
        else:
            speed = 0.0
        if speed > cfg.settle_vel_eps_m_s:
            return _check(False, True, f"released {fig} still moving {speed*1000:.0f}mm/s at settle end")
        # (b) net drop over the settle window
        drop = rpos[2] - epos[2]
        if drop > cfg.settle_drop_eps_m:
            return _check(False, True, f"released {fig} fell {drop*1000:.0f}mm after release")
        # (c) a supporting (upward-normal) contact exists at window end
        supported = any(c.involves(fig) and c.normal_z >= cfg.support_normal_z_min
                        for c in trace.steps[end].contacts)
        if not supported:
            return _check(False, True, f"released {fig} has no supporting contact at settle end")
        # (d) terminal relation does not degrade over the settle window
        gnd = trace.ground_id
        if gnd and fig in trace.body_sizes and gnd in trace.body_sizes:
            gr = trace.steps[r].object_poses.get(gnd, (None,))[0]
            ge = trace.steps[end].object_poses.get(gnd, (None,))[0]
            if gr is not None and ge is not None:
                rel_r = P.primary_topo_relation(_box(rpos, trace.body_sizes[fig]), _box(gr, trace.body_sizes[gnd]))
                rel_e = P.primary_topo_relation(_box(epos, trace.body_sizes[fig]), _box(ge, trace.body_sizes[gnd]))
                if rel_r is not None and rel_e != rel_r:
                    return _check(False, True, f"{fig} lost {rel_r} (now {rel_e}) during settle")
    return _check(True, True, "released object(s) rest supported in terminal relation")


def check_gripper_feasibility(trace: SimTrace, cfg: ValidityConfig) -> dict:
    if not trace.grasped_object or trace.grasp_interval is None \
            or trace.object_min_width_m is None or trace.object_max_width_m is None:
        return _check(True, False, "no grasp in this rollout")
    lo, hi = trace.grasp_interval
    lo = max(0, lo)
    hi = min(len(trace.steps) - 1, hi)
    lo_w = trace.object_min_width_m - cfg.grip_width_tol_m
    hi_w = trace.object_max_width_m + cfg.grip_width_tol_m
    bilateral = False
    max_force = 0.0
    for i in range(lo, hi + 1):
        s = trace.steps[i]
        if not (lo_w <= s.gripper_aperture_m <= hi_w):
            return _check(False, True,
                          f"aperture {s.gripper_aperture_m*1000:.0f}mm outside object width "
                          f"[{trace.object_min_width_m*1000:.0f},{trace.object_max_width_m*1000:.0f}]mm")
        if len(s.finger_contacts) >= 2:
            bilateral = True
        max_force = max(max_force, abs(s.gripper_force))
    if not bilateral:
        return _check(False, True, "no bilateral finger contact on grasped object")
    if trace.gripper_force_limit_n and max_force > trace.gripper_force_limit_n * (1 + cfg.force_tol_frac):
        return _check(False, True, f"grip force {max_force:.1f}N exceeds limit {trace.gripper_force_limit_n:.1f}N")
    return _check(True, True, "aperture spans object, bilateral contact, force within limit")


def check_workspace_reachability(trace: SimTrace, cfg: ValidityConfig) -> dict:
    if trace.ik_failures:
        return _check(False, True, f"IK failed at: {', '.join(trace.ik_failures[:3])}")
    if not any(s.joint_values for s in trace.steps):
        return _check(True, False, "no joint state recorded")
    m = cfg.joint_limit_margin_rad
    for s in trace.steps:
        for j, v in s.joint_values.items():
            lim = s.joint_limits.get(j)
            if lim is None:
                continue
            if v < lim[0] - m or v > lim[1] + m:
                return _check(False, True, f"joint {j}={v:.3f} outside limit [{lim[0]:.3f},{lim[1]:.3f}] at t={s.time_s:.2f}")
    return _check(True, True, "all effector poses reachable, joints within limits, no IK failures")


def check_articulation_limits(trace: SimTrace, cfg: ValidityConfig) -> dict:
    if not trace.articulation_limits:
        return _check(True, False, "no articulated joints in this task")
    m = cfg.joint_limit_margin_rad
    for s in trace.steps:
        for j, v in s.articulation.items():
            lim = trace.articulation_limits.get(j)
            if lim is None:
                continue
            if v < lim[0] - m or v > lim[1] + m:
                return _check(False, True, f"articulation {j}={v:.3f} outside [{lim[0]:.3f},{lim[1]:.3f}]")
    return _check(True, True, "articulated joints within range")


CHECKS = [
    ("non_penetration", check_non_penetration),
    ("pose_continuity", check_pose_continuity),
    ("quasi_static_support_at_release", check_quasi_static_support),
    ("gripper_feasibility", check_gripper_feasibility),
    ("workspace_reachability", check_workspace_reachability),
    ("articulation_limits", check_articulation_limits),
]


def check_validity(trace: SimTrace, cfg: Optional[ValidityConfig] = None) -> ValidityReport:
    """Run all six checks; AND the applicable ones into a single verdict."""
    cfg = cfg or DEFAULT
    results: Dict[str, dict] = {}
    first_failure = ""
    for name, fn in CHECKS:
        res = fn(trace, cfg)
        results[name] = res
        if res["applicable"] and not res["passed"] and not first_failure:
            first_failure = f"{name}: {res['detail']}"
    passed = all(r["passed"] for r in results.values() if r["applicable"])
    reason = first_failure if not passed else "all applicable physical-validity checks passed"
    return ValidityReport(passed=passed, reason=reason, checks=results)
