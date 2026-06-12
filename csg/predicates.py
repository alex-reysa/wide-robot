#!/usr/bin/env python3
"""Versioned, executable geometric semantics of RelationKind / ContactMode.

This module is the *normative grammar* shared by:
  * ``rollout_extract`` (sim state trace  -> robot CSG facts), and later
  * the perception compiler (roadmap Phase 3: video -> target CSG facts).

If both producers import the same predicate definitions, the matcher compares
words drawn from one grammar instead of two dialects. Every fact emitted via
these predicates should record ``PREDICATES_VERSION`` in its evidence so a CSG
can be re-checked against the exact semantics that produced it.

Geometry model (V0): each object is an axis-aligned box centered at its pose
position with full-extent ``size``. Yaw is currently ignored for footprint
tests; this is a documented V0 limitation (see ``csg/validity.md``). All
thresholds live in ``PredConfig`` so the semantics are inspectable in one place.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

PREDICATES_VERSION = "csg.predicates.v0"

Vec3 = Tuple[float, float, float]

# Relation kinds this registry can decide (subject -> object).
TOPO_RELATIONS = ("INSIDE", "ON_TOP_OF", "ALIGNED_WITH")
PROXIMITY_RELATIONS = ("NEAR", "FAR_FROM", "ABOVE_3D", "BELOW_3D", "TOUCHING_LIKELY")


@dataclass(frozen=True)
class PredConfig:
    # NEAR holds when the gap between the two boxes is below this (meters).
    near_gap_m: float = 0.10
    # TOUCHING_LIKELY when the gap is below this (meters).
    touching_gap_m: float = 0.012
    # Vertical tolerance for "resting on" tests (meters).
    on_top_eps_m: float = 0.012
    # XY footprint must overlap by at least this fraction of the smaller box.
    min_xy_overlap_frac: float = 0.30
    # INSIDE: subject center must sit at/below the container rim plus this slack.
    inside_rim_slack_m: float = 0.010
    # INSIDE: subject xy-center must lie within the container footprint shrunk
    # by this margin (meters) so a cube resting *on the rim* is not "inside".
    inside_footprint_margin_m: float = 0.005
    # ABOVE/BELOW_3D vertical separation threshold (meters).
    vertical_sep_m: float = 0.015
    # ALIGNED_WITH orientation tolerance (radians).
    aligned_angle_rad: float = 0.20
    # GRASP: effector within this distance of object center (meters).
    grasp_reach_m: float = 0.06
    # Co-motion correlation threshold over a contact interval.
    co_motion_corr: float = 0.6


DEFAULT = PredConfig()


@dataclass(frozen=True)
class Box:
    center: Vec3
    half: Vec3  # half extents
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    @property
    def top(self) -> float:
        return self.center[2] + self.half[2]

    @property
    def bottom(self) -> float:
        return self.center[2] - self.half[2]

    def xy_min(self) -> Tuple[float, float]:
        return (self.center[0] - self.half[0], self.center[1] - self.half[1])

    def xy_max(self) -> Tuple[float, float]:
        return (self.center[0] + self.half[0], self.center[1] + self.half[1])


def box_from(center: Vec3, size: Vec3, quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)) -> Box:
    return Box(center=tuple(float(c) for c in center), half=tuple(float(s) / 2.0 for s in size), quat=quat)  # type: ignore[arg-type]


# -----------------------------------------------------------------------------
# Geometry primitives
# -----------------------------------------------------------------------------


def _interval_gap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
    """Signed gap between two 1D intervals; negative means overlap."""
    if a_max < b_min:
        return b_min - a_max
    if b_max < a_min:
        return a_min - b_max
    return -min(a_max, b_max) + max(a_min, b_min)  # <= 0 overlap depth


def box_gap(a: Box, b: Box) -> float:
    """Axis-aligned gap between boxes; 0 or negative means touching/overlap."""
    gaps = []
    for ax in range(3):
        a_min = a.center[ax] - a.half[ax]
        a_max = a.center[ax] + a.half[ax]
        b_min = b.center[ax] - b.half[ax]
        b_max = b.center[ax] + b.half[ax]
        gaps.append(_interval_gap(a_min, a_max, b_min, b_max))
    pos = [g for g in gaps if g > 0]
    if not pos:
        return max(gaps)  # most-overlapping (least negative) axis ~ penetration
    # Euclidean gap across separated axes.
    return (sum(g * g for g in pos)) ** 0.5


def xy_overlap_frac(a: Box, b: Box) -> float:
    ax0, ay0 = a.xy_min()
    ax1, ay1 = a.xy_max()
    bx0, by0 = b.xy_min()
    bx1, by1 = b.xy_max()
    ox = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    oy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ox * oy
    area_a = max(1e-9, (ax1 - ax0) * (ay1 - ay0))
    area_b = max(1e-9, (bx1 - bx0) * (by1 - by0))
    return inter / min(area_a, area_b)


def _quat_angle(qa, qb) -> float:
    import math

    dot = max(-1.0, min(1.0, abs(sum(qa[i] * qb[i] for i in range(4)))))
    return 2.0 * math.acos(dot)


# -----------------------------------------------------------------------------
# Relation predicates (subject a, reference b)
# -----------------------------------------------------------------------------


def is_inside(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    """a is inside container b: horizontally within b's (shrunk) footprint and
    vertically at/below b's rim. Disambiguated from ON_TOP_OF by the rim test."""
    m = cfg.inside_footprint_margin_m
    bx0, by0 = b.xy_min()
    bx1, by1 = b.xy_max()
    cx, cy, _ = a.center
    within_footprint = (bx0 + m) <= cx <= (bx1 - m) and (by0 + m) <= cy <= (by1 - m)
    if not within_footprint:
        return False
    center_below_rim = a.center[2] <= b.top + cfg.inside_rim_slack_m
    not_below_floor = a.bottom >= b.bottom - cfg.on_top_eps_m
    return center_below_rim and not_below_floor


def is_on_top_of(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    """a rests on top of b: a's bottom near b's top, with xy overlap, and a is
    NOT contained (so a cube sunk into a tray is INSIDE, not ON_TOP_OF)."""
    if is_inside(a, b, cfg):
        return False
    resting = abs(a.bottom - b.top) <= cfg.on_top_eps_m
    return resting and xy_overlap_frac(a, b) >= cfg.min_xy_overlap_frac


def is_above_3d(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    return (a.center[2] - b.center[2]) > cfg.vertical_sep_m and xy_overlap_frac(a, b) > 0.0


def is_below_3d(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    return is_above_3d(b, a, cfg)


def is_near(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    return box_gap(a, b) <= cfg.near_gap_m


def is_touching(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    return box_gap(a, b) <= cfg.touching_gap_m


def is_aligned_with(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> bool:
    return _quat_angle(a.quat, b.quat) <= cfg.aligned_angle_rad


def relations_between(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> List[str]:
    """All directed relations (a -> b) that hold, in canonical priority order.

    Topological relations are mutually exclusive where it matters: INSIDE wins
    over ON_TOP_OF (the rim test inside ``is_on_top_of`` enforces this)."""
    out: List[str] = []
    if is_inside(a, b, cfg):
        out.append("INSIDE")
    elif is_on_top_of(a, b, cfg):
        out.append("ON_TOP_OF")
    if is_touching(a, b, cfg):
        out.append("TOUCHING_LIKELY")
    if is_near(a, b, cfg):
        out.append("NEAR")
    else:
        out.append("FAR_FROM")
    if is_above_3d(a, b, cfg):
        out.append("ABOVE_3D")
    elif is_below_3d(a, b, cfg):
        out.append("BELOW_3D")
    if is_aligned_with(a, b, cfg):
        out.append("ALIGNED_WITH")
    return out


def primary_topo_relation(a: Box, b: Box, cfg: PredConfig = DEFAULT) -> Optional[str]:
    """The single strongest topological relation a -> b, or None."""
    if is_inside(a, b, cfg):
        return "INSIDE"
    if is_on_top_of(a, b, cfg):
        return "ON_TOP_OF"
    return None


# -----------------------------------------------------------------------------
# Contact predicates (for rollout extraction)
# -----------------------------------------------------------------------------


def point_to_box_distance(p: Vec3, box: Box) -> float:
    """Euclidean distance from a point to an axis-aligned box (0 if inside)."""
    d2 = 0.0
    for ax in range(3):
        over = abs(p[ax] - box.center[ax]) - box.half[ax]
        if over > 0:
            d2 += over * over
    return d2 ** 0.5


def effector_reaches(effector_xyz: Vec3, obj_box: Box, cfg: PredConfig = DEFAULT) -> bool:
    """Effector is near the object's *surface* (handles/edges of large objects
    are far from the center, so center distance is wrong here)."""
    return point_to_box_distance(effector_xyz, obj_box) <= cfg.grasp_reach_m


def effector_touches(effector_xyz: Vec3, obj_box: Box, cfg: PredConfig = DEFAULT) -> bool:
    """Effector point at the object's surface within the touching gap. The
    non-grasp analogue of ``effector_reaches``: used to detect pushing contact,
    so it is deliberately tighter than the grasp reach."""
    return point_to_box_distance(effector_xyz, obj_box) <= cfg.touching_gap_m


def co_motion_correlation(eff_traj: Sequence[Vec3], obj_traj: Sequence[Vec3]) -> float:
    """Cosine similarity of step-to-step displacement of effector vs object over
    a window. 1.0 == perfectly co-moving, 0 == uncorrelated."""
    if len(eff_traj) < 2 or len(obj_traj) < 2:
        return 0.0
    n = min(len(eff_traj), len(obj_traj))
    num = 0.0
    de = 0.0
    do = 0.0
    for i in range(1, n):
        ev = tuple(eff_traj[i][k] - eff_traj[i - 1][k] for k in range(3))
        ov = tuple(obj_traj[i][k] - obj_traj[i - 1][k] for k in range(3))
        num += sum(ev[k] * ov[k] for k in range(3))
        de += sum(ev[k] * ev[k] for k in range(3))
        do += sum(ov[k] * ov[k] for k in range(3))
    if de < 1e-12 or do < 1e-12:
        # No motion on one side: co-motion undefined -> treat as static hold (1.0)
        # only if neither moved; otherwise 0.
        return 1.0 if (de < 1e-12 and do < 1e-12) else 0.0
    return max(0.0, num / (de ** 0.5 * do ** 0.5))


def grasp_likely(
    gripper_closed: bool,
    effector_xyz: Vec3,
    obj_box: Box,
    co_motion: float,
    cfg: PredConfig = DEFAULT,
) -> bool:
    return bool(gripper_closed) and effector_reaches(effector_xyz, obj_box, cfg) and co_motion >= cfg.co_motion_corr
