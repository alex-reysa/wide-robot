#!/usr/bin/env python3
"""Run a scripted MuJoCo task controller and emit a ``csg.rollout.v0`` rollout.

Pipeline (roadmap Phase 2C):

    compiled scene + program
      -> build_arm_scene_xml  (fixed-base arm + parallel-jaw gripper, MJCF)
      -> pre-roll settle
      -> scripted waypoint controller (diff-IK plan, position-actuator tracking)
      -> recorded frames @ 10 Hz  (effector + object poses, gripper, contacts)
      -> SimTrace -> validity.check_validity -> real physicalValidity verdict

The grasp is **weld-assisted**: the fingers close onto the object (so gripper
feasibility and bilateral finger contact are genuine) and a weld holds the grasp
during the scripted transport. The weld is released before placement, so the
quasi-static-support check still judges an honest, unassisted resting state —
exactly the part the verifier cares about. "stable_grasp_quality" stays on the
``hiddenVariablesNotUsed`` list; we never claim the friction grasp alone holds.

Imports ``mujoco``/``numpy``; reached only via ``csg.backends.mujoco.run_skill``
(lazy), which raises if mujoco is absent.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
import mujoco

from ...common import Json, ROBOT_GRIPPER_ID, as_list, enum_name, get_any, make_pose, s_to_ns
from .arm import (
    arm_mjcf,
    ArmSpec,
    FINGER_HALF_THICK,
    FINGER_OPEN_Y,
    FINGER_TRAVEL,
)
from .controller import joint_targets, solve_ik
from .scene_mjcf import (
    DRAWER_HANDLE_PROTRUSION_M,
    DRAWER_HANDLE_WIDTH_M,
    DRAWER_SLIDE_RANGE_M,
    build_arm_scene_xml,
    primary_grasp_body,
)
from .trace import ContactRecord, SimStep, SimTrace
from .validity import check_validity

PHYS_DT = 0.002
RECORD_EVERY = 50          # 50 * 2ms = 10 Hz frame rate
PREROLL_STEPS = 250        # 0.5 s settle before the controller acts
PUSH_FINGER_CONTACT_MARGIN_M = 0.003
PUSH_FINGER_FRICTION = 3.0
Vec3 = Tuple[float, float, float]


def _squeeze_q_for_width(width: float) -> float:
    q = (FINGER_OPEN_Y - (float(width) / 2.0 + FINGER_HALF_THICK)) + 0.004
    return max(0.0, min(FINGER_TRAVEL, q))


def _q_for_aperture(aperture: float) -> float:
    q = FINGER_OPEN_Y - FINGER_HALF_THICK - float(aperture) / 2.0
    return max(0.0, min(FINGER_TRAVEL, q))


@dataclass
class SimResult:
    frames: List[Json]
    failures: List[str]
    physical_validity: bool
    physical_validity_reason: str
    validity_report: Json = field(default_factory=dict)
    sampled_layout: Json = field(default_factory=dict)


@dataclass
class _Segment:
    phase: str
    site_target: Optional[Vec3]
    gripper_q: float
    weld: bool
    duration_s: float
    report_closed: Optional[bool] = None


class _Runner:
    def __init__(self, scene: Mapping[str, Any], program: Mapping[str, Any], cfg: Any):
        self.scene = scene
        self.program = program
        self.cfg = cfg
        self.effector_id = str(get_any(scene, "robotEffectorId", default=ROBOT_GRIPPER_ID)) or ROBOT_GRIPPER_ID
        seed = getattr(cfg, "seed", None)
        self.xml, self.layout = build_arm_scene_xml(scene, seed=seed, program=program)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.spec: ArmSpec = arm_mjcf()[3]
        self.site_id = self.model.site("grasp_site").id
        self.weld_id = self.model.equality("grasp_weld").id
        self.failures: List[str] = []
        self.ik_failures: List[str] = []
        # body bookkeeping
        self.grasp_body = primary_grasp_body(scene, program)
        self.ground_id = self._ground_id()
        self.trace_grasped_object = self.grasp_body
        self.trace_figure_id = self.grasp_body
        self.trace_ground_id = self.ground_id
        self.trace_articulation_limits: Dict[str, Tuple[float, float]] = {}
        self.body_sizes = {str(get_any(b, "objectId", "bodyId", default="")): self._size(b)
                           for b in as_list(get_any(scene, "bodies", default=[]))}
        self.object_ids = [oid for oid in self.body_sizes if oid in self.layout]
        self.articulation_initials = self._articulation_initials()
        self.articulation_joints = self._articulation_joints()
        self.static_bodies = tuple(str(get_any(b, "objectId", "bodyId", default=""))
                                   for b in as_list(get_any(scene, "bodies", default=[]))
                                   if enum_name(get_any(b, "mobility", default="")) not in {"MOVABLE", "ARTICULATED"})
        self.palm_bid = self.model.body(self.spec.palm_body).id
        self.steps: List[SimStep] = []
        self.frames: List[Json] = []
        self.release_indices: List[int] = []
        self.grasp_start_idx: Optional[int] = None
        self.grasp_end_idx: Optional[int] = None
        self.trace_object_min_width_m: Optional[float] = None
        self.trace_object_max_width_m: Optional[float] = None
        # Test-only fault injection (see tests/test_mujoco_backend.py): drives a
        # physically-invalid rollout so the verifier's FAIL behaviour is exercised
        # against real dynamics, not just synthetic traces. None in normal runs.
        self.sabotage = getattr(cfg, "sabotage", None)

    # ---- helpers -----------------------------------------------------------
    def _size(self, body: Mapping[str, Any]) -> Vec3:
        s = list(get_any(body, "sizeM", "size_m", default=[0.04, 0.04, 0.04])) + [0.04, 0.04, 0.04]
        return (float(s[0]), float(s[1]), float(s[2]))

    def _articulation_initials(self) -> Dict[str, float]:
        vals: Dict[str, float] = {}
        for b in as_list(get_any(self.scene, "bodies", default=[])):
            if not isinstance(b, Mapping):
                continue
            oid = str(get_any(b, "objectId", "bodyId", default=""))
            if enum_name(get_any(b, "mobility", default="")) != "ARTICULATED":
                continue
            art = get_any(b, "articulation", default={}) or {}
            try:
                q0 = float(get_any(art, "jointValue", "joint_value", default=0.02) or 0.02)
            except (TypeError, ValueError):
                q0 = 0.02
            vals[oid] = max(0.0, min(DRAWER_SLIDE_RANGE_M, q0))
        return vals

    def _articulation_joints(self) -> Dict[str, Tuple[str, int, int]]:
        joints: Dict[str, Tuple[str, int, int]] = {}
        for oid in self.articulation_initials:
            jname = f"{oid}_slide"
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                continue
            joints[oid] = (jname, int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid]))
        return joints

    def _ground_id(self) -> Optional[str]:
        ref = str(get_any(self.program, "targetObjectId", default=""))
        return ref or None

    def _body_xyz(self, name: str) -> Vec3:
        bid = self.model.body(name).id
        return tuple(float(v) for v in self.data.xpos[bid])

    def _set_home(self) -> None:
        for j, v in self.spec.home.items():
            self.data.qpos[self.model.jnt_qposadr[self.model.joint(j).id]] = v
        for oid, (_jname, adr, _dofadr) in self.articulation_joints.items():
            self.data.qpos[adr] = self.articulation_initials.get(oid, 0.02)
        for j in self.spec.joint_names:
            self.data.ctrl[self.model.actuator("act_" + j).id] = self.spec.home[j]
        for a in self.spec.grip_actuators:
            self.data.ctrl[self.model.actuator(a).id] = self.spec.finger_open_q
        mujoco.mj_forward(self.model, self.data)

    def _ik_targets(self, site_target: Vec3) -> np.ndarray:
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        mujoco.mj_forward(self.model, scratch)
        qsol, res = solve_ik(self.model, scratch, self.site_id, site_target,
                             self.spec.joint_names, self.spec.joint_limits)
        if res > 0.005:
            self.ik_failures.append(f"{np.round(site_target,3).tolist()} res={res*1000:.1f}mm")
        return joint_targets(self.model, qsol, self.spec.joint_names)

    def _finger_inner_faces(self) -> Tuple[float, float]:
        ly = float(self.data.xpos[self.model.body("left_finger").id][1])
        ry = float(self.data.xpos[self.model.body("right_finger").id][1])
        return ly - FINGER_HALF_THICK, ry + FINGER_HALF_THICK

    def _aperture(self) -> float:
        li, ri = self._finger_inner_faces()
        return abs(li - ri)

    def _contacts(self) -> Tuple[List[ContactRecord], Tuple[str, ...]]:
        recs: List[ContactRecord] = []
        fingers: List[str] = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = self.model.body(self.model.geom_bodyid[c.geom1]).name
            b2 = self.model.body(self.model.geom_bodyid[c.geom2]).name
            depth = max(0.0, -float(c.dist))
            nz = abs(float(c.frame[2]))  # world-z component of the contact normal
            recs.append(ContactRecord(b1, b2, depth, nz))
            pair = {b1, b2}
            if self.grasp_body in pair:
                other = (pair - {self.grasp_body}).pop() if len(pair) == 2 else ""
                if other in ("left_finger", "right_finger"):
                    fingers.append(other)
        return recs, tuple(sorted(set(fingers)))

    def _grip_force(self) -> float:
        f = 0.0
        for a in self.spec.grip_actuators:
            f = max(f, abs(float(self.data.actuator_force[self.model.actuator(a).id])))
        return f

    def _record(self, phase: str, gripper_closed: bool) -> None:
        site_pos = tuple(float(v) for v in self.data.site_xpos[self.site_id])
        sq = np.zeros(4); mujoco.mju_mat2Quat(sq, self.data.site_xmat[self.site_id])
        object_poses: Dict[str, Tuple[Vec3, Tuple[float, float, float, float]]] = {}
        frame_obj: Dict[str, Json] = {}
        for oid in self.object_ids:
            bid = self.model.body(oid).id
            pos = tuple(float(v) for v in self.data.xpos[bid])
            quat = tuple(float(v) for v in self.data.xquat[bid])
            object_poses[oid] = (pos, quat)
            frame_obj[oid] = make_pose("world", pos, quat)
        contacts, fingers = self._contacts()
        joints = {j: float(self.data.qpos[self.model.jnt_qposadr[self.model.joint(j).id]]) for j in self.spec.joint_names}
        articulation = {oid: float(self.data.qpos[adr])
                        for oid, (_jname, adr, _dofadr) in self.articulation_joints.items()}
        t = float(self.data.time)
        self.steps.append(SimStep(
            time_s=t, effector_xyz=site_pos, effector_quat=tuple(float(v) for v in sq),
            gripper_aperture_m=self._aperture(), gripper_closed_cmd=gripper_closed,
            object_poses=object_poses, joint_values=joints,
            joint_limits=dict(self.spec.joint_limits), contacts=contacts,
            articulation=articulation,
            gripper_force=self._grip_force(), finger_contacts=fingers,
        ))
        self.frames.append({
            "timeS": t, "timeNs": s_to_ns(t), "phase": phase,
            "effectorPose": make_pose("world", site_pos, tuple(float(v) for v in sq)),
            "gripperClosed": bool(gripper_closed),
            "objectPoses": frame_obj, "articulation": articulation,
        })

    def _activate_weld(self) -> None:
        body_bid = self.model.body(self.grasp_body).id
        p1 = self.data.xpos[self.palm_bid].copy(); q1 = self.data.xquat[self.palm_bid].copy()
        p2 = self.data.xpos[body_bid].copy(); q2 = self.data.xquat[body_bid].copy()
        q1inv = np.zeros(4); mujoco.mju_negQuat(q1inv, q1)
        relpos = np.zeros(3); mujoco.mju_rotVecQuat(relpos, p2 - p1, q1inv)
        relquat = np.zeros(4); mujoco.mju_mulQuat(relquat, q1inv, q2)
        d = self.model.eq_data[self.weld_id]
        d[0:3] = 0.0; d[3:6] = relpos; d[6:10] = relquat; d[10] = 1.0
        self.data.eq_active[self.weld_id] = 1
        mujoco.mj_forward(self.model, self.data)

    def _deactivate_weld(self) -> None:
        self.data.eq_active[self.weld_id] = 0
        mujoco.mj_forward(self.model, self.data)

    def _set_push_finger_contact_margin(self) -> None:
        for geom_name in ("left_finger_geom", "right_finger_geom"):
            gid = self.model.geom(geom_name).id
            self.model.geom_margin[gid] = PUSH_FINGER_CONTACT_MARGIN_M
            self.model.geom_friction[gid][0] = PUSH_FINGER_FRICTION
        mujoco.mj_forward(self.model, self.data)

    def _set_articulation_q(self, oid: str, q: float) -> None:
        joint = self.articulation_joints.get(oid)
        if joint is None:
            return
        _jname, qadr, dofadr = joint
        self.data.qpos[qadr] = float(q)
        self.data.qvel[dofadr] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _freejoint_addrs(self, oid: str) -> Optional[Tuple[int, int]]:
        bid = self.model.body(oid).id
        for offset in range(int(self.model.body_jntnum[bid])):
            jid = int(self.model.body_jntadr[bid] + offset)
            if self.model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                return int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])
        return None

    def _set_free_body_xyz(self, oid: str, xyz: Vec3) -> None:
        addrs = self._freejoint_addrs(oid)
        if addrs is None:
            self.failures.append(f"missing_freejoint:{oid}")
            return
        qadr, dadr = addrs
        self.data.qpos[qadr:qadr + 3] = np.asarray(xyz, dtype=float)
        self.data.qvel[dadr:dadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ---- execution ---------------------------------------------------------
    def _run_segment(self, seg: _Segment, q_from: np.ndarray) -> np.ndarray:
        q_to = self._ik_targets(seg.site_target) if seg.site_target is not None else q_from
        n = max(1, int(seg.duration_s / PHYS_DT))
        gripper_closed = (
            bool(seg.report_closed)
            if seg.report_closed is not None
            else (seg.weld or seg.gripper_q > 0.005)
        )
        for k in range(n):
            f = (k + 1) / n
            qd = q_from + (q_to - q_from) * f
            for idx, j in enumerate(self.spec.joint_names):
                self.data.ctrl[self.model.actuator("act_" + j).id] = qd[idx]
            for a in self.spec.grip_actuators:
                self.data.ctrl[self.model.actuator(a).id] = seg.gripper_q
            mujoco.mj_step(self.model, self.data)
            if (len(self.steps) == 0) or (self._phys_count % RECORD_EVERY == 0):
                self._record(seg.phase, gripper_closed)
            self._phys_count += 1
        return q_to

    def run(self) -> SimResult:
        self._phys_count = 0
        self._set_home()
        # pre-roll: let objects settle on the table while the arm holds home.
        for _ in range(PREROLL_STEPS):
            mujoco.mj_step(self.model, self.data)
        self._phys_count = 0

        skill = enum_name(get_any(self.program, "skillType", default="pick_place")).lower()
        if skill == "push":
            return self._run_push()
        if skill == "open":
            return self._run_open()
        return self._run_pick_place()

    def _run_push(self) -> SimResult:
        puck = self.grasp_body
        goal_id = self.ground_id
        self.trace_grasped_object = None
        self.trace_figure_id = puck
        self.trace_ground_id = goal_id
        self.trace_articulation_limits = {}
        self.grasp_start_idx = None
        self.grasp_end_idx = None
        self.release_indices = []
        self._set_push_finger_contact_margin()

        p0 = self._body_xyz(puck)
        psz = self.body_sizes.get(puck, (0.04, 0.04, 0.04))
        width = min(psz[0], psz[1])
        push_q = _q_for_aperture(width + 0.0025)

        gx, gy, gz = self._body_xyz(goal_id) if goal_id else (p0[0] + 0.12, p0[1], p0[2])
        gsz = self.body_sizes.get(goal_id or "", (0.04, 0.04, 0.04))
        final_gap = 0.08
        final_x = gx - gsz[0] / 2.0 - psz[0] / 2.0 - final_gap
        push_z = p0[2] + psz[2] * 0.40
        final_z = push_z
        push_y = p0[1] - min(0.004, width * 0.10)

        approach = (p0[0] - psz[0] / 2.0 - 0.06, p0[1], push_z + 0.10)
        descend = (approach[0], p0[1], push_z)
        site_push_bias_x = 0.0008
        engage = (p0[0] + site_push_bias_x, push_y, push_z)
        push_end = (final_x + site_push_bias_x, push_y, final_z)
        retreat = (push_end[0] - psz[0] / 2.0 - 0.07, push_y, push_end[2] + 0.10)

        q = joint_targets(self.model, self.data.qpos, self.spec.joint_names)
        if self.sabotage == "push_missing_contact":
            q = self._run_segment(_Segment("approach", approach, push_q, False, 0.7, report_closed=False), q)
            q = self._run_segment(_Segment("avoid_contact", approach, push_q, False, 0.5, report_closed=False), q)
            q = self._run_segment(_Segment("retreat", (approach[0] - 0.06, approach[1], approach[2]),
                                           push_q, False, 0.4, report_closed=False), q)
            return self._finish()
        q = self._run_segment(_Segment("approach", approach, push_q, False, 0.7, report_closed=False), q)
        q = self._run_segment(_Segment("descend", descend, push_q, False, 0.5, report_closed=False), q)
        q = self._run_segment(_Segment("engage", engage, push_q, False, 0.8, report_closed=False), q)
        q = self._run_segment(_Segment("push", push_end, push_q, False, 10.0, report_closed=False), q)
        q = self._run_segment(_Segment("settle_push", push_end, push_q, False, 0.4, report_closed=False), q)
        q = self._run_segment(_Segment("retreat", retreat, push_q, False, 0.3, report_closed=False), q)

        return self._finish()

    def _run_open(self) -> SimResult:
        drawer = str(get_any(self.program, "manipulatedObjectId", default="")) or self.grasp_body
        self.grasp_body = drawer
        self.trace_grasped_object = drawer
        self.trace_figure_id = None
        self.trace_ground_id = None
        self.trace_articulation_limits = {drawer: (0.0, DRAWER_SLIDE_RANGE_M)}
        self.trace_object_min_width_m = DRAWER_HANDLE_WIDTH_M
        self.trace_object_max_width_m = DRAWER_HANDLE_WIDTH_M
        self.release_indices = []

        if drawer not in self.articulation_joints:
            self.failures.append(f"missing_articulation_joint:{drawer}")
            return self._finish()

        _jname, qadr, _dofadr = self.articulation_joints[drawer]
        q0 = float(self.data.qpos[qadr])
        goal = get_any(self.program, "articulationGoal", default={}) or {}
        try:
            target_q = float(get_any(goal, "targetJointValue", "target_joint_value", default=0.18) or 0.18)
        except (TypeError, ValueError):
            target_q = 0.18
        target_q = max(q0 + 0.08, 0.19, min(DRAWER_SLIDE_RANGE_M, target_q))
        target_q = min(DRAWER_SLIDE_RANGE_M, target_q)
        pull_delta = max(0.0, target_q - q0)

        p0 = self._body_xyz(drawer)
        sx, _sy, _sz = self.body_sizes.get(drawer, (0.40, 0.30, 0.15))
        handle = (p0[0] - sx / 2.0 - DRAWER_HANDLE_PROTRUSION_M, p0[1], p0[2])
        pull_end = (handle[0] - pull_delta, handle[1], handle[2])
        approach = (handle[0] - 0.06, handle[1], handle[2] + 0.10)
        pregrasp = (handle[0], handle[1], handle[2])
        retreat = (pull_end[0] - 0.07, pull_end[1], pull_end[2] + 0.10)
        grasp_q = _squeeze_q_for_width(DRAWER_HANDLE_WIDTH_M)

        q = joint_targets(self.model, self.data.qpos, self.spec.joint_names)
        q = self._run_segment(_Segment("approach_handle", approach, self.spec.finger_open_q, False, 0.7), q)
        q = self._run_segment(_Segment("pregrasp_handle", pregrasp, self.spec.finger_open_q, False, 0.5), q)
        q = self._run_segment(_Segment("grasp_handle", pregrasp, grasp_q, False, 0.7, report_closed=True), q)
        # Finger closure can nudge the passive slide by fractions of a
        # millimeter. Re-seat the drawer at its observed start value, then hold
        # a closed contact frame before pulling so CONTACT_BEGIN precedes the
        # extracted ARTICULATION_CHANGE.
        self._set_articulation_q(drawer, q0)
        self._activate_weld()
        self.grasp_start_idx = len(self.steps)
        q = self._run_segment(_Segment("hold_handle", pregrasp, grasp_q, True, 0.5, report_closed=True), q)
        q = self._run_segment(_Segment("pull_open", pull_end, grasp_q, True, 2.4, report_closed=True), q)
        q = self._run_segment(_Segment("settle_open", pull_end, grasp_q, True, 0.4, report_closed=True), q)
        self.grasp_end_idx = len(self.steps) - 1
        self._deactivate_weld()
        self.release_indices.append(len(self.steps))
        q = self._run_segment(_Segment("release_handle", pull_end, self.spec.finger_open_q, False, 0.8, report_closed=False), q)
        q = self._run_segment(_Segment("retreat", retreat, self.spec.finger_open_q, False, 0.5, report_closed=False), q)
        if self.sabotage == "overlimit_articulation":
            self._set_articulation_q(drawer, DRAWER_SLIDE_RANGE_M + 0.006)
            self._record("overlimit_articulation", False)

        return self._finish()

    def _run_pick_place(self) -> SimResult:
        obj = self._body_xyz(self.grasp_body)
        grasp_site = (obj[0], obj[1], obj[2])            # site at object center
        above_object = (obj[0], obj[1], obj[2] + 0.16)

        goal = self._goal_xyz()
        # The weld captures the site->object offset at grasp; with the site at the
        # object center that offset is ~0, so place targets are the goal directly.
        place_site = (goal[0], goal[1], goal[2])
        above_goal = (goal[0], goal[1], goal[2] + 0.16)
        grasp_size = self.body_sizes.get(self.grasp_body, (0.04, 0.04, 0.04))
        grasp_q = _squeeze_q_for_width(min(grasp_size[0], grasp_size[1]))
        if self.sabotage == "wide_grasp":
            grasp_q = self.spec.finger_open_q

        q = joint_targets(self.model, self.data.qpos, self.spec.joint_names)
        # approach + descend with the gripper open
        q = self._run_segment(_Segment("approach", above_object, self.spec.finger_open_q, False, 0.6), q)
        q = self._run_segment(_Segment("pregrasp", grasp_site, self.spec.finger_open_q, False, 0.6), q)
        # close the fingers onto the object, then weld for the scripted transport
        q = self._run_segment(_Segment("grasp", grasp_site, grasp_q, False, 0.5), q)
        self._activate_weld()
        # The grasp interval (for gripper feasibility) covers the frames where the
        # object is actually held — after the fingers have closed, through transport
        # — not the open-fingered closing transient.
        self.grasp_start_idx = len(self.steps)
        q = self._run_segment(_Segment("lift", above_object, grasp_q, True, 0.6), q)

        if self.sabotage == "early_release":
            # Drop the object mid-air over the table instead of placing it: the
            # quasi-static-support check must fail (and the terminal relation is
            # never reached, so the matcher fails too).
            q = self._run_segment(_Segment("transport", above_object, grasp_q, True, 0.5), q)
            self.grasp_end_idx = len(self.steps) - 1
            self._deactivate_weld()
            self.release_indices.append(len(self.steps))
            q = self._run_segment(_Segment("release", None, self.spec.finger_open_q, False, 1.2), q)
            q = self._run_segment(_Segment("retreat", above_goal, self.spec.finger_open_q, False, 0.5), q)
            return self._finish()

        q = self._run_segment(_Segment("transport", above_goal, grasp_q, True, 0.9), q)
        q = self._run_segment(_Segment("lower", place_site, grasp_q, True, 0.7), q)
        # Hold the terminal relation while still grasped, so the structural
        # change is observed strictly before release.
        q = self._run_segment(_Segment("settle_in", place_site, grasp_q, True, 0.5), q)
        if self.sabotage == "wrong_relation":
            wrong_site = (obj[0] + 0.04, obj[1], obj[2])
            wrong_above = (wrong_site[0], wrong_site[1], wrong_site[2] + 0.16)
            q = self._run_segment(_Segment("remove_after_goal", above_goal, grasp_q, True, 0.5), q)
            q = self._run_segment(_Segment("transport_wrong", wrong_above, grasp_q, True, 0.8), q)
            q = self._run_segment(_Segment("lower_wrong", wrong_site, grasp_q, True, 0.6), q)
        self.grasp_end_idx = len(self.steps) - 1
        # release: drop the weld, open the gripper, let the object settle
        self._deactivate_weld()
        self.release_indices.append(len(self.steps))
        q = self._run_segment(_Segment("release", None, self.spec.finger_open_q, False, 1.2), q)
        if self.sabotage == "teleport_after_release":
            ox, oy, oz = self._body_xyz(self.grasp_body)
            self._set_free_body_xyz(self.grasp_body, (ox + 0.12, oy, oz))
            self._record("teleport_after_release", False)
        elif self.sabotage == "penetrate_goal" and self.ground_id:
            gx, gy, gz = self._body_xyz(self.ground_id)
            self._set_free_body_xyz(self.grasp_body, (gx, gy, gz))
            self._record("penetrate_goal", False)
        elif self.sabotage == "impossible_reach":
            self._ik_targets((2.0, 0.0, 0.35))
        q = self._run_segment(_Segment("retreat", above_goal, self.spec.finger_open_q, False, 0.6), q)

        return self._finish()

    def _goal_xyz(self) -> Vec3:
        """Resting place for the manipulated object, reusing the solver's INSIDE
        cavity-floor geometry so solver and extractor agree on containment."""
        rel = enum_name(get_any(self.program, "relationGoal", default="NEAR"))
        ref = self.ground_id
        if not ref or ref not in self.body_sizes:
            obj = self._body_xyz(self.grasp_body)
            return (obj[0] + 0.1, obj[1], max(self.body_sizes[self.grasp_body][2] / 2, 0.02))
        tray = self._body_xyz(ref)
        rsz = self.body_sizes[ref]
        osz = self.body_sizes[self.grasp_body]
        if rel in {"INSIDE", "CONTAINS"}:
            from ...to_sim import CONTAINER_FLOOR_M
            return (tray[0], tray[1], (tray[2] - rsz[2] / 2) + CONTAINER_FLOOR_M + osz[2] / 2)
        if rel in {"ON_TOP_OF", "SUPPORTED_BY", "ABOVE_3D"}:
            return (tray[0], tray[1], tray[2] + rsz[2] / 2 + osz[2] / 2)
        return (tray[0], tray[1], tray[2] + rsz[2] / 2 + osz[2] / 2)

    def _finish(self) -> SimResult:
        grasped = self.trace_grasped_object
        csz = self.body_sizes.get(grasped or "", (0.04, 0.04, 0.04))
        width = min(csz[0], csz[1])
        min_width = self.trace_object_min_width_m if self.trace_object_min_width_m is not None else width
        max_width = self.trace_object_max_width_m if self.trace_object_max_width_m is not None else max(csz[0], csz[1])
        trace = SimTrace(
            steps=self.steps, frame_dt_s=PHYS_DT * RECORD_EVERY,
            release_indices=self.release_indices, ik_failures=self.ik_failures,
            grasped_object=grasped,
            grasp_interval=(self.grasp_start_idx, self.grasp_end_idx)
            if self.grasp_start_idx is not None else None,
            object_min_width_m=min_width, object_max_width_m=max_width,
            figure_id=self.trace_figure_id, ground_id=self.trace_ground_id,
            body_sizes=self.body_sizes, static_bodies=self.static_bodies,
            gripper_force_limit_n=self.spec.finger_force_n,
            articulation_limits=self.trace_articulation_limits,
        )
        report = check_validity(trace)
        return SimResult(
            frames=self.frames, failures=self.failures,
            physical_validity=report.passed, physical_validity_reason=report.reason,
            validity_report=report.to_json(),
            sampled_layout={oid: list(xyz) for oid, xyz in sorted(self.layout.items())},
        )


def run_skill(scene: Mapping[str, Any], program: Mapping[str, Any], cfg: Any) -> SimResult:
    return _Runner(scene, program, cfg).run()
