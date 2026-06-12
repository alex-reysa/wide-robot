#!/usr/bin/env python3
"""Differential-IK helpers for the scripted pick-place controller.

Imports ``mujoco`` (and numpy) — only loaded when the MuJoCo backend actually
runs. ``solve_ik`` is damped-least-squares 6-DoF IK on a *scratch* MjData copy
(no physics, no contacts) used to pre-plan a feasible joint target per Cartesian
waypoint; the runner then drives the real arm with position actuators toward
those joint targets under physics, so contacts are genuine. The gripper points
straight down (identity site orientation) for a top-down grasp.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np
import mujoco

# Identity quaternion: the grasp site's frame == world, so the fingers (which
# extend along palm -z) point at the table.
DOWN_QUAT = np.array([1.0, 0.0, 0.0, 0.0])


def arm_dof_indices(model, joint_names: Sequence[str]) -> List[int]:
    return [int(model.jnt_dofadr[model.joint(j).id]) for j in joint_names]


def clamp_joints(model, qpos, joint_names: Sequence[str], limits) -> None:
    for j in joint_names:
        adr = model.jnt_qposadr[model.joint(j).id]
        lo, hi = limits[j]
        qpos[adr] = min(hi, max(lo, qpos[adr]))


def solve_ik(model, data, site_id: int, target_pos, joint_names: Sequence[str], limits,
             target_quat=DOWN_QUAT, iters: int = 400, lam: float = 0.15,
             pos_tol: float = 1e-4, rot_tol: float = 1e-3) -> Tuple[np.ndarray, float]:
    """Kinematic DLS IK toward (target_pos, target_quat). Mutates ``data.qpos``;
    callers pass a scratch MjData. Returns (qpos_copy, final_position_residual)."""
    dofs = arm_dof_indices(model, joint_names)
    target_pos = np.asarray(target_pos, dtype=float)
    mujoco.mj_forward(model, data)
    perr = np.zeros(3)
    for _ in range(iters):
        perr = target_pos - data.site_xpos[site_id]
        cq = np.zeros(4); mujoco.mju_mat2Quat(cq, data.site_xmat[site_id])
        cc = np.zeros(4); mujoco.mju_negQuat(cc, cq)
        qe = np.zeros(4); mujoco.mju_mulQuat(qe, target_quat, cc)
        rerr = np.zeros(3); mujoco.mju_quat2Vel(rerr, qe, 1.0)
        err = np.concatenate([perr, rerr])
        jp = np.zeros((3, model.nv)); jr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jp, jr, site_id)
        J = np.vstack([jp, jr])[:, dofs]
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), err)
        full = np.zeros(model.nv)
        for k, d in enumerate(dofs):
            full[d] = dq[k]
        mujoco.mj_integratePos(model, data.qpos, full, 1.0)
        clamp_joints(model, data.qpos, joint_names, limits)
        mujoco.mj_forward(model, data)
        if np.linalg.norm(perr) < pos_tol and np.linalg.norm(err[3:]) < rot_tol:
            break
    return data.qpos.copy(), float(np.linalg.norm(perr))


def joint_targets(model, qpos_solution, joint_names: Sequence[str]) -> np.ndarray:
    """Extract arm joint angles (in joint order) from a full qpos vector."""
    return np.array([qpos_solution[model.jnt_qposadr[model.joint(j).id]] for j in joint_names])
