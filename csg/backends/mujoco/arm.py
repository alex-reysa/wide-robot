#!/usr/bin/env python3
"""A hand-written fixed-base 6-DoF arm + parallel-jaw gripper as MJCF.

No external assets (no meshes, no menagerie download): the arm is built from
capsules and boxes so the scene is deterministic and fully inspectable. The
verifier never sees the arm — it only reads the grasp-site pose and object
poses from the rollout — but a *real* arm with real joint limits is what makes
the workspace-reachability validity check (``csg/validity.md`` #5) meaningful.

This module is stdlib-only (string templating); ``mujoco`` is imported only by
``controller.py`` / ``runner.py``. ``arm_mjcf`` returns the worldbody fragment,
the actuator/equality sections, and an :class:`ArmSpec` describing joint names,
limits, the grasp site, and finger bodies so the controller and validity check
can address them by name.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# Link geometry (meters). Sized so the reachable envelope covers the tabletop
# task zone (x in ~[0.25, 0.55], down to the table) with margin; reachability of
# the corners is unit-tested in tests/test_mujoco_backend.py.
SHOULDER_H = 0.55
UPPER_LEN = 0.30
FORE_LEN = 0.30
WRIST_LEN = 0.07
PALM_LEN = 0.05
FINGER_LEN = 0.06
FINGER_HALF_THICK = 0.006
FINGER_HALF_DEPTH = 0.012
# Finger slide range: q=0 fully open, q=FINGER_TRAVEL fully closed.
FINGER_OPEN_Y = 0.055   # finger center y at q=0 (open)
FINGER_TRAVEL = 0.045   # max inward travel
GRIPPER_FORCE_N = 25.0

JOINTS: Tuple[Tuple[str, str, Tuple[float, float]], ...] = (
    # name,            axis, range (radians)
    ("shoulder_pan",  "0 0 1", (-2.9, 2.9)),
    ("shoulder_lift", "0 1 0", (-2.9, 2.9)),
    ("elbow",         "0 1 0", (-2.9, 2.9)),
    ("wrist_pitch",   "0 1 0", (-2.9, 2.9)),
    ("wrist_roll",    "0 0 1", (-2.9, 2.9)),
    ("wrist_yaw",     "0 1 0", (-2.9, 2.9)),
)

# A home configuration that holds the gripper above the table center pointing
# straight down (grasp site ~ (0.40, 0, 0.28), identity orientation). Found by
# kinematic IK; the arm spawns clear of the table so physics is stable and the
# differential IK starts in a good basin. See scene_mjcf.CONTAINER_CENTER_XY.
HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.565,
    "elbow": 1.849,
    "wrist_pitch": -0.795,
    "wrist_roll": 0.0,
    "wrist_yaw": -0.489,
}


@dataclass
class ArmSpec:
    joint_names: List[str]
    joint_limits: Dict[str, Tuple[float, float]]
    home: Dict[str, float]
    grasp_site: str = "grasp_site"
    finger_joints: Tuple[str, str] = ("finger_left", "finger_right")
    finger_bodies: Tuple[str, str] = ("left_finger", "right_finger")
    grip_actuators: Tuple[str, str] = ("act_finger_left", "act_finger_right")
    arm_actuators: List[str] = field(default_factory=list)
    palm_body: str = "palm"
    weld_name: str = "grasp_weld"
    base_xy: Tuple[float, float] = (0.0, 0.0)
    finger_force_n: float = GRIPPER_FORCE_N
    finger_open_q: float = 0.0
    finger_closed_q: float = FINGER_TRAVEL


def _finger_body(name: str, sign: int) -> str:
    """One parallel jaw: a slide joint along y (q=0 open, +q closes toward the
    centerline) carrying a thin vertical plate that straddles the object."""
    y0 = sign * FINGER_OPEN_Y
    axis = f"0 {-sign} 0"  # +q moves the jaw toward y=0
    return (
        f"<body name='{name}' pos='0 {y0:.4f} {-PALM_LEN:.4f}' gravcomp='1'>"
        f"<joint name='{'finger_left' if sign > 0 else 'finger_right'}' type='slide' axis='{axis}' "
        f"range='0 {FINGER_TRAVEL:.4f}' damping='5' armature='0.01'/>"
        f"<geom name='{name}_geom' type='box' "
        f"size='{FINGER_HALF_DEPTH:.4f} {FINGER_HALF_THICK:.4f} {FINGER_LEN/2:.4f}' "
        f"pos='0 0 {-FINGER_LEN/2:.4f}' condim='4' friction='2 0.05 0.0002' "
        f"solref='0.01 1' rgba='0.2 0.2 0.25 1'/>"
        f"</body>"
    )


def arm_mjcf(base_xy: Tuple[float, float] = (0.0, 0.0)) -> Tuple[str, str, str, ArmSpec]:
    """Return (worldbody_fragment, actuator_section, equality_section, spec)."""
    bx, by = base_xy
    jn = [j[0] for j in JOINTS]

    # Serial chain. Each link is a capsule along +x; child bodies sit at the
    # parent link's tip. Wrist carries the palm; palm carries two jaws + the
    # grasp site at the fingertip midline. ``gravcomp='1'`` gravity-compensates
    # the arm (as a real arm's controller does), so the position actuators hold
    # configuration without droop and the sim is numerically stable; the cube
    # keeps gravity (gravcomp default 0). ``armature`` adds reflected rotor
    # inertia for stiff-controller stability.
    # The arm's structural links never need collision — the verifier only cares
    # about gripper↔object and object↔surface contacts. Disabling self/world
    # collision on the links (contype/conaffinity 0) removes spurious
    # self-contacts at the home pose that would otherwise destabilize the sim.
    A = "armature='0.1'"
    NC = "contype='0' conaffinity='0'"
    chain = (
        f"<body name='shoulder' pos='0 0 {SHOULDER_H:.4f}' gravcomp='1'>"
        f"<joint name='shoulder_pan' type='hinge' axis='0 0 1' range='-2.9 2.9' damping='8' {A}/>"
        f"<geom type='capsule' fromto='0 0 0 0 0 -0.06' size='0.05' {NC} rgba='0.3 0.3 0.35 1'/>"
        f"<body name='upper_arm' pos='0 0 0' gravcomp='1'>"
        f"<joint name='shoulder_lift' type='hinge' axis='0 1 0' range='-2.9 2.9' damping='8' {A}/>"
        f"<geom type='capsule' fromto='0 0 0 {UPPER_LEN:.4f} 0 0' size='0.035' {NC} rgba='0.5 0.5 0.55 1'/>"
        f"<body name='forearm' pos='{UPPER_LEN:.4f} 0 0' gravcomp='1'>"
        f"<joint name='elbow' type='hinge' axis='0 1 0' range='-2.9 2.9' damping='6' {A}/>"
        f"<geom type='capsule' fromto='0 0 0 {FORE_LEN:.4f} 0 0' size='0.03' {NC} rgba='0.5 0.5 0.55 1'/>"
        f"<body name='wrist' pos='{FORE_LEN:.4f} 0 0' gravcomp='1'>"
        f"<joint name='wrist_pitch' type='hinge' axis='0 1 0' range='-2.9 2.9' damping='4' {A}/>"
        f"<geom type='capsule' fromto='0 0 0 {WRIST_LEN:.4f} 0 0' size='0.025' {NC} rgba='0.4 0.4 0.45 1'/>"
        f"<body name='wrist2' pos='{WRIST_LEN:.4f} 0 0' gravcomp='1'>"
        f"<joint name='wrist_roll' type='hinge' axis='0 0 1' range='-2.9 2.9' damping='3' {A}/>"
        f"<joint name='wrist_yaw' type='hinge' axis='0 1 0' range='-2.9 2.9' damping='3' {A}/>"
        f"<geom type='box' size='0.03 0.04 0.02' {NC} rgba='0.4 0.4 0.45 1'/>"
        f"<body name='palm' pos='0 0 0' gravcomp='1'>"
        f"<geom type='box' size='0.03 {FINGER_OPEN_Y+0.01:.4f} 0.015' pos='0 0 {-PALM_LEN/2:.4f}' {NC} rgba='0.3 0.3 0.35 1'/>"
        f"<site name='grasp_site' pos='0 0 {-(PALM_LEN + FINGER_LEN):.4f}' size='0.006' rgba='1 0 0 1'/>"
        f"{_finger_body('left_finger', +1)}"
        f"{_finger_body('right_finger', -1)}"
        f"</body></body></body></body></body></body>"
    )

    pedestal = (
        f"<body name='arm_base' pos='{bx:.4f} {by:.4f} 0'>"
        f"<geom type='cylinder' fromto='0 0 0 0 0 {SHOULDER_H:.4f}' size='0.05' {NC} rgba='0.2 0.2 0.2 1'/>"
        f"{chain}"
        f"</body>"
    )

    arm_acts = [f"act_{n}" for n in jn]
    actuator = "".join(
        f"<position name='act_{n}' joint='{n}' kp='800' kv='40' ctrlrange='-3.0 3.0'/>"
        for n in jn
    ) + (
        f"<position name='act_finger_left' joint='finger_left' kp='400' "
        f"forcerange='-{GRIPPER_FORCE_N:.1f} {GRIPPER_FORCE_N:.1f}' ctrlrange='0 {FINGER_TRAVEL:.4f}'/>"
        f"<position name='act_finger_right' joint='finger_right' kp='400' "
        f"forcerange='-{GRIPPER_FORCE_N:.1f} {GRIPPER_FORCE_N:.1f}' ctrlrange='0 {FINGER_TRAVEL:.4f}'/>"
    )

    equality = "<weld name='grasp_weld' body1='palm' body2='__GRASP_BODY__' active='false' solref='0.01 1'/>"

    spec = ArmSpec(
        joint_names=jn,
        joint_limits={j[0]: j[2] for j in JOINTS},
        home=dict(HOME),
        arm_actuators=arm_acts,
        base_xy=base_xy,
    )
    return pedestal, actuator, equality, spec
