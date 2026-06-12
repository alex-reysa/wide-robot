#!/usr/bin/env python3
"""Compile a (solver-side) scene into a full MJCF document with the arm.

Stdlib-only string assembly — ``mujoco`` is not imported here, so the builder is
unit-testable for structure and the produced XML is what ``runner.py`` loads.

Two jobs beyond gluing the arm to the objects:

* **Shared cavity geometry.** Container bodies reuse ``csg.to_sim._body_geoms``
  (floor slab + four walls, audit A8) so the MJCF tray matches what the compiler
  and the goal-pose admissibility test assume. The existing
  ``scene_to_mujoco_xml`` (pinned by ``tests/test_validity.py``) is left
  untouched; this is the richer, arm-aware builder.

* **Initial-pose deconfliction.** The compiler's *invented* initial poses
  (``initialPoseApproximate``) can interpenetrate — for ``put_cube_in_tray`` the
  cube spawns inside the tray wall, which the symbolic backend never notices but
  MuJoCo fails at frame 0. We re-seat bodies on the table and place movable
  objects NEAR (but clear of) the static container, preserving the observable
  NEAR→INSIDE transition without a spawn penetration.
"""
from __future__ import annotations

import random
from typing import Any, Dict, List, Mapping, Optional, Tuple

from ...common import as_list, enum_name, get_any
from ...to_sim import _body_geoms
from .arm import arm_mjcf

Vec3 = Tuple[float, float, float]

# Where the (first) static container sits in the world; chosen to be inside the
# arm's reachable envelope (see arm.py geometry).
CONTAINER_CENTER_XY = (0.46, 0.0)
SPAWN_GAP_M = 0.05           # cube↔tray gap at spawn: NEAR (<0.10) but clear
PLACE_ON_SPAWN_GAP_M = 0.13  # block↔cube gap at spawn: FAR (>0.10) but reachable
PUSH_SPAWN_GAP_M = 0.13      # puck↔goal gap at spawn: FAR (>0.10) but reachable
MOVER_ROW_PITCH_M = 0.10    # y spacing if several movers share the table
OPEN_HANDLE_X_M = 0.40       # closed drawer handle x, in the arm's reachable envelope
DRAWER_HANDLE_PROTRUSION_M = 0.035
DRAWER_HANDLE_WIDTH_M = 0.065
DRAWER_HANDLE_HEIGHT_M = 0.030
DRAWER_HANDLE_DEPTH_M = 0.024
DRAWER_SLIDE_RANGE_M = 0.22
RANDOM_GLOBAL_Y_JITTER_M = 0.002
PUSH_RANDOM_GLOBAL_X_JITTER_M = 0.002


def _mobility(body: Mapping[str, Any]) -> str:
    return enum_name(get_any(body, "mobility", default="UNKNOWN_MOBILITY"))


def _is_movable(body: Mapping[str, Any]) -> bool:
    return _mobility(body) == "MOVABLE"


def _is_articulated(body: Mapping[str, Any]) -> bool:
    return _mobility(body) == "ARTICULATED"


def _is_manipulable(body: Mapping[str, Any]) -> bool:
    return _mobility(body) in {"MOVABLE", "ARTICULATED"}


def _size(body: Mapping[str, Any]) -> Vec3:
    s = list(get_any(body, "sizeM", "size_m", default=[0.04, 0.04, 0.04])) + [0.04, 0.04, 0.04]
    return (float(s[0]), float(s[1]), float(s[2]))


def _program_skill(program: Optional[Mapping[str, Any]]) -> str:
    if program is None:
        return ""
    return enum_name(get_any(program, "skillType", default="")).lower()


def _apply_pose_overrides(layout: Dict[str, Vec3], pose_overrides: Optional[Dict[str, Vec3]]) -> Dict[str, Vec3]:
    out = dict(layout)
    for oid, xyz in (pose_overrides or {}).items():
        out[oid] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return out


def _seeded_layout(layout: Dict[str, Vec3],
                   bodies: List[Mapping[str, Any]],
                   seed: int,
                   program: Optional[Mapping[str, Any]]) -> Dict[str, Vec3]:
    """Conservative seeded pose variation around the deconflicted layout.

    Non-push skills use a shared y translation. Push uses a shared x
    translation instead so seeded rollouts sample distinct reachable workspace
    locations while preserving the tight non-grasp contact line.
    """
    rng = random.Random(int(seed))
    skill = _program_skill(program)
    if skill == "push":
        dx = rng.uniform(-PUSH_RANDOM_GLOBAL_X_JITTER_M, PUSH_RANDOM_GLOBAL_X_JITTER_M)
        out: Dict[str, Vec3] = {}
        for b in sorted(bodies, key=lambda body: str(get_any(body, "objectId", "bodyId", default=""))):
            oid = str(get_any(b, "objectId", "bodyId", default=""))
            if oid not in layout:
                continue
            x, y, z = layout[oid]
            out[oid] = (round(x + dx, 6), float(y), float(z))
        return out
    dy = rng.uniform(-RANDOM_GLOBAL_Y_JITTER_M, RANDOM_GLOBAL_Y_JITTER_M)
    out: Dict[str, Vec3] = {}
    for b in sorted(bodies, key=lambda body: str(get_any(body, "objectId", "bodyId", default=""))):
        oid = str(get_any(b, "objectId", "bodyId", default=""))
        if oid not in layout:
            continue
        x, y, z = layout[oid]
        out[oid] = (float(x), round(y + dy, 6), float(z))
    return out


def deconflict_layout(bodies: List[Mapping[str, Any]],
                      pose_overrides: Optional[Dict[str, Vec3]] = None,
                      program: Optional[Mapping[str, Any]] = None) -> Dict[str, Vec3]:
    """World-frame centers per body: every body rests on the table (z = sz/2);
    movable objects spawn near the primary static body except place_on, which
    must begin FAR_FROM the support block."""
    overrides = pose_overrides or {}
    layout: Dict[str, Vec3] = {}
    statics = [b for b in bodies if not _is_movable(b)]
    movers = [b for b in bodies if _is_movable(b)]
    skill = _program_skill(program)
    if skill == "push":
        spawn_gap = PUSH_SPAWN_GAP_M
    elif skill == "place_on":
        spawn_gap = PLACE_ON_SPAWN_GAP_M
    else:
        spawn_gap = SPAWN_GAP_M
    target_id = str(get_any(program or {}, "targetObjectId", default=""))
    manipulated_id = str(get_any(program or {}, "manipulatedObjectId", default=""))
    anchor_x, anchor_y = CONTAINER_CENTER_XY

    if skill == "open":
        ordered = list(bodies)
        for i, b in enumerate(ordered):
            oid = str(get_any(b, "objectId", "bodyId", default=""))
            sx, _sy, sz = _size(b)
            if oid == manipulated_id and _is_articulated(b):
                # Place the robot-facing handle near x=0.40; the drawer center
                # is deeper in +x, and the slide joint opens along -x.
                layout[oid] = (
                    OPEN_HANDLE_X_M + sx / 2.0 + DRAWER_HANDLE_PROTRUSION_M,
                    anchor_y,
                    sz / 2.0,
                )
            else:
                layout[oid] = (
                    anchor_x,
                    anchor_y + (i + 1) * MOVER_ROW_PITCH_M,
                    sz / 2.0,
                )
        for oid, xyz in overrides.items():
            layout[oid] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        return layout

    # Anchor: the program target if static, otherwise the first static body.
    if statics:
        anchor_idx = next((
            i for i, b in enumerate(statics)
            if str(get_any(b, "objectId", "bodyId", default="")) == target_id
        ), 0)
        sb = statics[anchor_idx]
        ssz = _size(sb)
        sid = str(get_any(sb, "objectId", "bodyId", default=""))
        layout[sid] = (anchor_x, anchor_y, ssz[2] / 2)
        other_statics = [b for i, b in enumerate(statics) if i != anchor_idx]
        for i, sb2 in enumerate(other_statics, start=1):
            s2 = _size(sb2)
            sid2 = str(get_any(sb2, "objectId", "bodyId", default=""))
            layout[sid2] = (anchor_x, anchor_y + 0.35 * i, s2[2] / 2)
        anchor_half_x = ssz[0] / 2
    else:
        anchor_half_x = 0.12

    manip_idx = next((
        i for i, b in enumerate(movers)
        if str(get_any(b, "objectId", "bodyId", default="")) == manipulated_id
    ), None)
    if manip_idx is None:
        ordered_movers = movers
    else:
        ordered_movers = [movers[manip_idx]] + [b for i, b in enumerate(movers) if i != manip_idx]

    # Movers: line them up to the -x side of the anchor, clear of contact.
    for i, mb in enumerate(ordered_movers):
        msz = _size(mb)
        mid = str(get_any(mb, "objectId", "bodyId", default=""))
        x = anchor_x - anchor_half_x - spawn_gap - msz[0] / 2
        if manip_idx is None:
            y = anchor_y + (i - (len(ordered_movers) - 1) / 2.0) * MOVER_ROW_PITCH_M
        elif i == 0:
            y = anchor_y
        else:
            row = (i + 1) // 2
            sign = 1 if i % 2 else -1
            y = anchor_y + sign * row * MOVER_ROW_PITCH_M
        layout[mid] = (x, y, msz[2] / 2)

    # Explicit overrides win (used by milestone-7 randomized rollouts).
    for oid, xyz in overrides.items():
        layout[oid] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return layout


def _object_body_xml(body: Mapping[str, Any], center: Vec3) -> str:
    oid = str(get_any(body, "objectId", "bodyId", default="body"))
    sx, sy, sz = _size(body)
    if _is_articulated(body):
        return _articulated_body_xml(body, center)
    geoms = _body_geoms(body, sx, sy, sz)  # shared cavity math (audit A8)
    x, y, z = center
    freejoint = "<freejoint/>" if _is_movable(body) else ""
    return (f"<body name='{oid}' pos='{x:.6f} {y:.6f} {z:.6f}'>"
            f"{freejoint}{''.join(geoms)}</body>")


def _articulation_initial_value(body: Mapping[str, Any]) -> float:
    art = get_any(body, "articulation", default={}) or {}
    try:
        q0 = float(get_any(art, "jointValue", "joint_value", default=0.02) or 0.02)
    except (TypeError, ValueError):
        q0 = 0.02
    return max(0.0, min(DRAWER_SLIDE_RANGE_M, q0))


def _articulated_body_xml(body: Mapping[str, Any], center: Vec3) -> str:
    oid = str(get_any(body, "objectId", "bodyId", default="body"))
    sx, sy, sz = _size(body)
    x, y, z = center
    q0 = _articulation_initial_value(body)
    handle_x = -(sx / 2.0 + DRAWER_HANDLE_PROTRUSION_M)
    handle_y = min(DRAWER_HANDLE_WIDTH_M / 2.0, max(0.018, sy / 2.0 - 0.02))
    handle_z = min(DRAWER_HANDLE_HEIGHT_M / 2.0, max(0.012, sz / 2.0 - 0.01))
    handle_depth = DRAWER_HANDLE_DEPTH_M / 2.0
    shell_t = min(0.012, sx / 10.0, sy / 10.0, sz / 4.0)
    shell_z = shell_t / 2.0
    rail_z = sz / 2.0 + shell_t / 2.0
    rail_y = sy / 2.0 + shell_t / 2.0
    # The child body is offset by q0 so setting the slide qpos to q0 leaves the
    # drawer's observable center at ``center``. Increasing q moves along -x.
    child_x = q0
    shell = (
        f"<geom name='{oid}_cabinet_back' type='box' size='{shell_t/2:.6f} {sy/2:.6f} {sz/2:.6f}' "
        f"pos='{sx/2:.6f} 0 0' contype='0' conaffinity='0' rgba='0.55 0.55 0.50 1'/>"
        f"<geom name='{oid}_cabinet_top' type='box' size='{sx/2:.6f} {sy/2 + shell_t:.6f} {shell_t/2:.6f}' "
        f"pos='0 0 {rail_z:.6f}' contype='0' conaffinity='0' rgba='0.55 0.55 0.50 1'/>"
        f"<geom name='{oid}_cabinet_left' type='box' size='{sx/2:.6f} {shell_t/2:.6f} {sz/2:.6f}' "
        f"pos='0 {rail_y:.6f} 0' contype='0' conaffinity='0' rgba='0.55 0.55 0.50 1'/>"
        f"<geom name='{oid}_cabinet_right' type='box' size='{sx/2:.6f} {shell_t/2:.6f} {sz/2:.6f}' "
        f"pos='0 {-rail_y:.6f} 0' contype='0' conaffinity='0' rgba='0.55 0.55 0.50 1'/>"
    )
    drawer = (
        f"<joint name='{oid}_slide' type='slide' axis='-1 0 0' "
        f"range='0 {DRAWER_SLIDE_RANGE_M:.6f}' damping='8' frictionloss='20.0' armature='0.02'/>"
        f"<geom name='{oid}_box' type='box' size='{sx/2:.6f} {sy/2:.6f} {sz/2:.6f}' "
        f"density='80' rgba='0.35 0.38 0.42 1'/>"
        f"<geom name='{oid}_handle' type='box' "
        f"size='{handle_depth:.6f} {handle_y:.6f} {handle_z:.6f}' "
        f"pos='{handle_x:.6f} 0 0' density='200' condim='4' friction='2 0.05 0.0002' "
        f"rgba='0.12 0.12 0.13 1'/>"
    )
    return (
        f"<body name='{oid}_cabinet' pos='{x:.6f} {y:.6f} {z:.6f}'>"
        f"{shell}"
        f"<body name='{oid}' pos='{child_x:.6f} 0 0'>{drawer}</body>"
        f"</body>"
    )


def primary_grasp_body(scene: Mapping[str, Any], program: Optional[Mapping[str, Any]] = None) -> str:
    """The movable body the gripper welds to during transport: the program's
    manipulated object, else the first movable body."""
    if program is not None:
        mid = str(get_any(program, "manipulatedObjectId", default=""))
        if mid:
            return mid
    for b in as_list(get_any(scene, "bodies", default=[])):
        if _is_manipulable(b):
            return str(get_any(b, "objectId", "bodyId", default=""))
    return ""


def build_arm_scene_xml(scene: Mapping[str, Any], *, seed: Optional[int] = None,
                        pose_overrides: Optional[Dict[str, Vec3]] = None,
                        program: Optional[Mapping[str, Any]] = None) -> Tuple[str, Dict[str, Vec3]]:
    """Return (mjcf_xml, layout). ``layout`` is the world center per body so the
    runner can read where each object started."""
    bodies = [b for b in as_list(get_any(scene, "bodies", default=[])) if isinstance(b, Mapping)]
    layout = deconflict_layout(bodies, None, program)
    if seed is not None:
        layout = _seeded_layout(layout, bodies, int(seed), program)
    layout = _apply_pose_overrides(layout, pose_overrides)
    grasp_body = primary_grasp_body(scene, program)

    pedestal, actuator, equality, _spec = arm_mjcf()
    equality = equality.replace("__GRASP_BODY__", grasp_body or "body_000")

    obj_xml = "".join(_object_body_xml(b, layout[str(get_any(b, "objectId", "bodyId", default=""))])
                      for b in bodies)

    xml = (
        "<mujoco model='csg_arm_scene'>"
        "<compiler angle='radian'/>"
        "<option timestep='0.002' integrator='implicitfast' cone='elliptic' impratio='10'/>"
        "<default>"
        "<geom condim='4' friction='1 0.05 0.001' solref='0.01 1' solimp='0.95 0.99 0.001'/>"
        "</default>"
        "<worldbody>"
        "<light pos='0 0 2' dir='0 0 -1'/>"
        "<geom name='table' type='plane' size='3 3 0.05' pos='0 0 0' "
        "condim='3' friction='1 0.05 0.001' rgba='0.8 0.8 0.78 1'/>"
        f"{obj_xml}"
        f"{pedestal}"
        "</worldbody>"
        f"<actuator>{actuator}</actuator>"
        f"<equality>{equality}</equality>"
        "</mujoco>"
    )
    return xml, layout
