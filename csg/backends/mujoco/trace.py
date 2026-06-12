#!/usr/bin/env python3
"""Pure-data seam between the MuJoCo simulation and the validity checker.

``runner.py`` (which imports ``mujoco``) populates a :class:`SimTrace`;
``validity.py`` (which does NOT import ``mujoco``) consumes it. Keeping this
module stdlib-only means the validity checks are unit-testable on synthetic
traces without a simulator installed, and the whole ``csg`` test suite keeps
running without the optional ``mujoco`` dependency.

Nothing here ever reaches the rollout. The rollout carries only the whitelisted
``csg.rollout.v0`` fields (``csg/rollout_schema.md``); these structures are the
physics-side detail used to decide ``physicalValidity`` and to build a sidecar
``validity_report.json``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (w, x, y, z)


@dataclass(frozen=True)
class ContactRecord:
    """One MuJoCo contact, reduced to what the validity checks need.

    ``depth_m`` is positive penetration depth (``-contact.dist``; MuJoCo reports
    ``dist < 0`` when two geoms overlap). ``normal_z`` is the world-z component
    of the contact normal, used to tell a supporting (upward) contact from a
    side contact when checking quasi-static support at release.
    """

    body_a: str
    body_b: str
    depth_m: float
    normal_z: float

    def involves(self, body: str) -> bool:
        return body in (self.body_a, self.body_b)


@dataclass
class SimStep:
    """A single recorded simulation frame (recorded at the rollout frame rate,
    not the physics rate)."""

    time_s: float
    effector_xyz: Vec3
    effector_quat: Quat
    gripper_aperture_m: float
    gripper_closed_cmd: bool
    object_poses: Dict[str, Tuple[Vec3, Quat]] = field(default_factory=dict)
    joint_values: Dict[str, float] = field(default_factory=dict)
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    contacts: List[ContactRecord] = field(default_factory=list)
    articulation: Dict[str, float] = field(default_factory=dict)
    gripper_force: float = 0.0
    # Bodies the gripper fingers touched this step (for gripper feasibility).
    finger_contacts: Tuple[str, ...] = ()


@dataclass
class SimTrace:
    """Everything the validity checker needs about a single rollout.

    ``release_indices`` are the recorded-frame indices at which the controller
    opened the gripper to release a grasped object. ``figure_id`` /
    ``ground_id`` name the manipulated object and the surface/container it should
    end related to (used by the quasi-static-support check to confirm the
    terminal relation survives the settle window). ``body_sizes`` lets that
    check recompute predicate relations with the shared grammar.
    """

    steps: List[SimStep] = field(default_factory=list)
    frame_dt_s: float = 0.1
    release_indices: List[int] = field(default_factory=list)
    ik_failures: List[str] = field(default_factory=list)
    grasped_object: Optional[str] = None
    grasp_interval: Optional[Tuple[int, int]] = None  # [start, end] recorded-frame indices
    object_min_width_m: Optional[float] = None
    object_max_width_m: Optional[float] = None
    figure_id: Optional[str] = None
    ground_id: Optional[str] = None
    body_sizes: Dict[str, Vec3] = field(default_factory=dict)
    static_bodies: Tuple[str, ...] = ()
    gripper_force_limit_n: float = 0.0
    articulation_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
