#!/usr/bin/env python3
"""Naive ``object_inside_container`` success predicates — the *baselines* the
counterexample experiment puts next to the structured wide-robot verifier.

The point of this module is NOT to be a bad strawman. Each predicate is a
reasonable thing a practitioner might write to score "did the cube end up in the
tray?" from object poses. They climb a ladder of strictness:

    B1  terminal cube CENTER inside the tray's outer footprint        (2D, no height)
    B2  terminal cube FOOTPRINT overlaps the tray footprint >= frac   (2D, no height)
    B3  terminal cube FOOTPRINT fully inside the shrunk inner region  (2D, margin)
    B4  B3 AND the cube STARTED outside the footprint                 (2D + initial)
    B5  terminal 3D containment: csg.is_inside on the LAST frame      (3D, rim-aware)
    B6  B4 AND the evidence-quality gate passes                       (2D + initial + evidence)

**B6 is the engineered steelman** — B4 plus the verifier's *own* fail-closed
evidence gate (``pilots.real_camera.verify_episode.assess_evidence_quality``)
bolted on. On this dataset B6 ties the structured verifier's *aggregate scoreboard*
(both 27/38 success-cert, both 0/40 false-PASS) — but it is NOT clip-for-clip
identical: B6 and the verifier disagree on **4 successes that cancel out** (B6's
B3 full-footprint containment is stricter than the verifier's center-based
``is_inside``, so B6 rejects 2 the verifier certifies; conversely B6's raw-center
read certifies 2 obstruction successes the verifier hard-FAILs as false-negatives).
See ``b6_vs_structured.json`` for the clip-level diff. That tie-with-disagreement
is the honest conclusion: a sufficiently engineered task-specific predicate CAN
approximate this one task, by reimplementing (or, as here, importing) the very
components wide-robot bundles. The point wide-robot makes is that those
components — initial-state, transition, evidence-confidence, leakage — are made
**explicit, auditable, reusable, and leakage-clean** as a task *graph* (data),
instead of accreted as bespoke per-task predicate code. The evidence gate is
**target-blind and separable**; B6 is the executable proof.

**B5 is the strongest single-frame terminal predicate possible** — it is the
verifier's OWN ``csg.predicates.is_inside`` (shrunk footprint *and* rim-height
test) applied to the terminal frame, with no transition and no evidence gate. It
exists to defeat the obvious objection that B1's failure on the rim is just a
2D-vs-3D deficit: B5 *does* reject the rim, but it still cannot see the things
that make a put-in a put-in. So the gap that REMAINS after B5 is the honest,
irreducible "structured residue":
  * **rim** — closed by B5 (a dimensionality lesson: 2D center -> 3D containment);
  * **born-inside** — B5 still PASSES (ends inside), only the initial-state /
    transition check rejects it;
  * **occlusion** — B5 still PASSES (first+last frame look fine), only the
    fail-closed evidence gate rejects it.

What every Bn shares — and why they answer a *weaker question than the task* —
is that they read at most the first and last frame and never check (a) whether a
real outside->inside *transition* occurred vs. the cube being born inside, or
(b) whether the evidence was good enough to certify anything at all
(mid-trajectory occlusion). B1/B2 additionally ignore the container *rim height*;
B5 fixes that but the first two gaps remain. The wide-robot verifier checks all
of it. See ``README.md``.

This module is geometry-PURE for B1..B5: it imports ``csg.predicates`` for the
*same* box geometry the verifier uses (so the comparison is apples-to-apples, not
a rigged frame). B6 additionally imports the verifier's *target-blind* evidence
gate (``assess_evidence_quality``) — that is the whole point of B6 (the gate is a
separable, bolt-on-able component). Nothing here needs OpenCV or raw video; every
predicate is computed from a committed ``real_camera.tracks.v0`` episode. ``csg/``
is only READ, never modified.
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence, Tuple

from csg.predicates import DEFAULT, box_from, is_inside, xy_overlap_frac
from pilots.real_camera.verify_episode import assess_evidence_quality

BASELINE_PREDICATES_VERSION = "baseline_counterexamples.predicates.v0"

Vec3 = Tuple[float, float, float]

# B2 calls an overlap "success" at or above this fraction of the cube footprint.
# 0.5 = "more than half the cube sits over the tray opening" — a lenient but not
# unreasonable bar a practitioner might pick.
DEFAULT_B2_MIN_OVERLAP_FRAC = 0.5

# Evidence-quality thresholds B6 gates on. These are the SAME relaxed 30fps
# thresholds the dataset ingest (scripts/ingest_recordings.py) and the experiment
# build pass to the structured verifier, so B6's evidence gate and the verifier's
# evidence gate see identical inputs — making B6's parity with the verifier an
# apples-to-apples result, not an artifact of mismatched thresholds. A test pins
# this equal to the build script's REAL_VIDEO_THRESHOLDS.
EVIDENCE_THRESHOLDS = {"max_consecutive_missing": 30, "max_dropout_frac": 0.35}


# ---------------------------------------------------------------------------
# Geometry helpers (reuse csg.predicates' Box so the footprint math is shared).
# ---------------------------------------------------------------------------


def _xy_in_rect(cx: float, cy: float, x0: float, y0: float, x1: float, y1: float) -> bool:
    return x0 <= cx <= x1 and y0 <= cy <= y1


def _outer_footprint_bounds(tray_center: Vec3, tray_size: Sequence[float]) -> Tuple[float, float, float, float]:
    """(x0, y0, x1, y1) of the tray's full XY footprint (no margin)."""
    b = box_from(tray_center, tuple(tray_size))
    x0, y0 = b.xy_min()
    x1, y1 = b.xy_max()
    return x0, y0, x1, y1


def _inner_footprint_bounds(tray_center: Vec3, tray_size: Sequence[float], margin: float) -> Tuple[float, float, float, float]:
    """(x0, y0, x1, y1) of the tray footprint shrunk by ``margin`` per side —
    the SAME inner region ``csg.predicates.is_inside`` tests the cube center
    against (``inside_footprint_margin_m``)."""
    x0, y0, x1, y1 = _outer_footprint_bounds(tray_center, tray_size)
    return x0 + margin, y0 + margin, x1 - margin, y1 - margin


# ---------------------------------------------------------------------------
# The baseline ladder. Each takes plain centers/sizes so it is trivially testable.
# ---------------------------------------------------------------------------


def b1_terminal_center_in_footprint(cube_center: Vec3, tray_center: Vec3,
                                    tray_size: Sequence[float]) -> bool:
    """B1 — the single-condition terminal predicate: is the cube's terminal
    (x, y) center within the tray's outer footprint rectangle? No height, no
    margin. This is the headline baseline: a plausible "did it land in the
    tray?" check that ignores the rim."""
    x0, y0, x1, y1 = _outer_footprint_bounds(tray_center, tray_size)
    return _xy_in_rect(float(cube_center[0]), float(cube_center[1]), x0, y0, x1, y1)


def b2_footprint_overlap(cube_center: Vec3, cube_size: Sequence[float],
                         tray_center: Vec3, tray_size: Sequence[float],
                         *, min_frac: float = DEFAULT_B2_MIN_OVERLAP_FRAC) -> bool:
    """B2 — terminal cube XY footprint overlaps the tray footprint by at least
    ``min_frac`` of the (smaller) cube footprint. Still 2D, still rim-blind."""
    cube = box_from(cube_center, tuple(cube_size))
    tray = box_from(tray_center, tuple(tray_size))
    return xy_overlap_frac(cube, tray) >= float(min_frac)


def b3_full_inner_containment(cube_center: Vec3, cube_size: Sequence[float],
                              tray_center: Vec3, tray_size: Sequence[float],
                              *, margin: float = DEFAULT.inside_footprint_margin_m) -> bool:
    """B3 — the cube's WHOLE terminal XY footprint lies inside the shrunk inner
    region (not just its center). A cube on the rim pokes its footprint past the
    inner rectangle, so this is the first baseline that rejects the rim case —
    but it is still height-blind and transition-blind."""
    cube = box_from(cube_center, tuple(cube_size))
    cx0, cy0 = cube.xy_min()
    cx1, cy1 = cube.xy_max()
    ix0, iy0, ix1, iy1 = _inner_footprint_bounds(tray_center, tray_size, margin)
    return cx0 >= ix0 and cy0 >= iy0 and cx1 <= ix1 and cy1 <= iy1


def b4_full_containment_started_outside(cube_first_center: Vec3, cube_last_center: Vec3,
                                        cube_size: Sequence[float],
                                        tray_center: Vec3, tray_size: Sequence[float],
                                        *, margin: float = DEFAULT.inside_footprint_margin_m) -> bool:
    """B4 — the strongest naive predicate: B3 on the terminal frame AND the cube
    STARTED outside the tray footprint (its initial center is not within the
    outer footprint). This is a practitioner's honest attempt to also reject
    "born inside" — and it works for that. What it still cannot see is evidence
    quality: a clip with a 50-frame mid-trajectory occlusion looks identical at
    its first and last frame, so B4 certifies it anyway."""
    started_outside = not b1_terminal_center_in_footprint(cube_first_center, tray_center, tray_size)
    fully_in = b3_full_inner_containment(cube_last_center, cube_size, tray_center, tray_size, margin=margin)
    return bool(started_outside and fully_in)


def b5_terminal_3d_containment(cube_center: Vec3, cube_size: Sequence[float],
                               tray_center: Vec3, tray_size: Sequence[float]) -> bool:
    """B5 — the strongest single-frame terminal predicate: the verifier's OWN
    ``csg.predicates.is_inside`` (shrunk footprint + rim-height test) on the
    terminal frame. No transition, no evidence gate. It closes the rim case
    (3D containment), so any gap that survives B5 (born-inside, occlusion) is a
    genuinely structural one, not a 2D-vs-3D artifact."""
    cube = box_from(cube_center, tuple(cube_size))
    tray = box_from(tray_center, tuple(tray_size))
    return bool(is_inside(cube, tray, DEFAULT))


def b6_b4_plus_evidence_gate(b4_passed: bool, evidence_ok: bool) -> bool:
    """B6 — the engineered steelman: B4 (contained + started-outside) AND the
    verifier's fail-closed evidence gate. ``evidence_ok`` is exactly
    ``assess_evidence_quality(tracks, EVIDENCE_THRESHOLDS)["ok"]`` — the SAME
    target-blind gate the structured verifier runs. So B6 is B4 with the verifier's
    own occlusion/dropout discipline bolted on. On the committed dataset this
    reproduces the structured verifier's verdict profile exactly (it stops
    certifying the occlusion successes B4 over-certifies), which is the honest
    point: the gate is a *separable* component, and a fully-engineered baseline
    converges on reimplementing what wide-robot already bundles."""
    return bool(b4_passed and evidence_ok)


# ---------------------------------------------------------------------------
# Tracks adapters: derive the SAME geometry the verifier sees, no cv2 / video.
# ---------------------------------------------------------------------------


def _median(values: Sequence[float]) -> float:
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def _pose_xyz(frame: Mapping[str, Any], role: str) -> Optional[Vec3]:
    poses = frame.get("poses") if isinstance(frame, Mapping) else None
    pose = poses.get(role) if isinstance(poses, Mapping) else None
    if not isinstance(pose, Mapping):
        return None
    pm = pose.get("positionM")
    if isinstance(pm, Mapping):
        try:
            return (float(pm["x"]), float(pm["y"]), float(pm["z"]))
        except (KeyError, TypeError, ValueError):
            return None
    if isinstance(pm, (list, tuple)) and len(pm) >= 3:
        return (float(pm[0]), float(pm[1]), float(pm[2]))
    return None


def _object_size(objects: Sequence[Mapping[str, Any]], role: str) -> Optional[List[float]]:
    for obj in objects:
        if str(obj.get("sourceRole")) == role:
            size = obj.get("sizeM")
            return [float(v) for v in size] if size is not None else None
    return None


def clip_geometry(tracks: Mapping[str, Any], *, mover_role: str = "cube",
                  container_role: str = "tray") -> Optional[dict]:
    """Extract the terminal/initial cube centers, the median (static) tray
    center, and both sizes from a ``real_camera.tracks.v0`` episode — mirroring
    how ``tracks_to_rollout`` clamps the static container to its median and how
    the extractor reads the first/last mover frame. Returns ``None`` if the
    mover or container is never visible (so a baseline cannot be computed)."""
    frames = tracks.get("frames") or []
    objects = tracks.get("objects") or []
    cube_size = _object_size(objects, mover_role) or [0.05, 0.05, 0.05]
    tray_size = _object_size(objects, container_role) or [0.18, 0.18, 0.07]

    tray_xyz = [c for c in (_pose_xyz(f, container_role) for f in frames) if c is not None]
    cube_visible = [c for c in (_pose_xyz(f, mover_role) for f in frames) if c is not None]
    if not tray_xyz or not cube_visible:
        return None
    tray_center = (
        _median([p[0] for p in tray_xyz]),
        _median([p[1] for p in tray_xyz]),
        _median([p[2] for p in tray_xyz]),
    )
    return {
        "cubeFirst": cube_visible[0],
        "cubeLast": cube_visible[-1],
        "trayCenter": tray_center,
        "cubeSize": cube_size,
        "traySize": tray_size,
        "numCubeFrames": len(cube_visible),
        "numFrames": len(frames),
    }


def evaluate_clip(tracks: Mapping[str, Any], *,
                  b2_min_frac: float = DEFAULT_B2_MIN_OVERLAP_FRAC,
                  margin: float = DEFAULT.inside_footprint_margin_m,
                  evidence_thresholds: Optional[Mapping[str, float]] = None) -> Optional[dict]:
    """Run the full B1..B6 ladder on one episode. Returns the booleans plus the
    geometry they were computed from, or ``None`` if geometry is unavailable.

    B6 gates B4 on the verifier's fail-closed evidence quality
    (``assess_evidence_quality``) using ``evidence_thresholds`` (default
    :data:`EVIDENCE_THRESHOLDS`, the dataset's relaxed 30fps thresholds — the same
    ones the structured verifier sees), so B6 vs. the verifier is apples-to-apples."""
    geom = clip_geometry(tracks)
    if geom is None:
        return None
    cube_first, cube_last = geom["cubeFirst"], geom["cubeLast"]
    cube_size, tray_center, tray_size = geom["cubeSize"], geom["trayCenter"], geom["traySize"]

    cube_box = box_from(cube_last, tuple(cube_size))
    tray_box = box_from(tray_center, tuple(tray_size))

    b4 = b4_full_containment_started_outside(
        cube_first, cube_last, cube_size, tray_center, tray_size, margin=margin)
    th = dict(EVIDENCE_THRESHOLDS if evidence_thresholds is None else evidence_thresholds)
    evidence = assess_evidence_quality(tracks, th)
    evidence_ok = bool(evidence["ok"])

    return {
        "B1_center_in_footprint": b1_terminal_center_in_footprint(cube_last, tray_center, tray_size),
        "B2_footprint_overlap": b2_footprint_overlap(cube_last, cube_size, tray_center, tray_size, min_frac=b2_min_frac),
        "B3_full_inner_containment": b3_full_inner_containment(cube_last, cube_size, tray_center, tray_size, margin=margin),
        "B4_full_containment_started_outside": b4,
        "B5_terminal_3d_containment": b5_terminal_3d_containment(cube_last, cube_size, tray_center, tray_size),
        "B6_contained_started_outside_evidence_gated": b6_b4_plus_evidence_gate(b4, evidence_ok),
        "overlapFrac": xy_overlap_frac(cube_box, tray_box),
        "evidenceOk": evidence_ok,
        "evidenceFailureClass": evidence["failureClass"],
        "geometry": geom,
        "params": {"b2_min_frac": float(b2_min_frac), "inner_margin_m": float(margin),
                   "evidenceThresholds": th},
        "predicatesVersion": BASELINE_PREDICATES_VERSION,
    }


# ---------------------------------------------------------------------------
# Independent (from-scratch) terminal-relation reimplementation.
#
# This deliberately does NOT import csg.predicates' logic — it re-derives the
# INSIDE / ON_TOP_OF / NEAR / FAR_FROM decision with its own axis-aligned box
# math, so comparing it to the verifier's extracted terminal relation is a real
# second-implementation cross-check (catches transcription/off-by-one bugs), not
# a snapshot of the same code. The threshold *values* mirror csg.predicates.DEFAULT
# (they are the shared spec, not the algorithm); ``test_*`` pins them equal so a
# future csg retune is caught rather than silently diverging.
# ---------------------------------------------------------------------------

_IND_MARGIN_M = 0.005       # inside_footprint_margin_m
_IND_RIM_SLACK_M = 0.010    # inside_rim_slack_m
_IND_ON_TOP_EPS_M = 0.012   # on_top_eps_m
_IND_NEAR_GAP_M = 0.10      # near_gap_m
_IND_MIN_XY_OVERLAP = 0.30  # min_xy_overlap_frac

INDEPENDENT_CONSTANTS = {
    "inside_footprint_margin_m": _IND_MARGIN_M,
    "inside_rim_slack_m": _IND_RIM_SLACK_M,
    "on_top_eps_m": _IND_ON_TOP_EPS_M,
    "near_gap_m": _IND_NEAR_GAP_M,
    "min_xy_overlap_frac": _IND_MIN_XY_OVERLAP,
}


def independent_terminal_relation(cube_center: Vec3, cube_size: Sequence[float],
                                  tray_center: Vec3, tray_size: Sequence[float]) -> str:
    """Re-derive the strongest cube->tray relation (INSIDE / ON_TOP_OF / NEAR /
    FAR_FROM) from scratch, without csg.predicates' logic. Used to independently
    corroborate the verifier's extracted terminal relation."""
    cx, cy, cz = (float(v) for v in cube_center)
    chx, chy, chz = (float(s) / 2.0 for s in cube_size)
    tx, ty, tz = (float(v) for v in tray_center)
    thx, thy, thz = (float(s) / 2.0 for s in tray_size)

    tray_top, tray_bot, cube_bot = tz + thz, tz - thz, cz - chz
    within = ((tx - thx + _IND_MARGIN_M) <= cx <= (tx + thx - _IND_MARGIN_M) and
              (ty - thy + _IND_MARGIN_M) <= cy <= (ty + thy - _IND_MARGIN_M))
    if within and cz <= tray_top + _IND_RIM_SLACK_M and cube_bot >= tray_bot - _IND_ON_TOP_EPS_M:
        return "INSIDE"

    ox = max(0.0, min(cx + chx, tx + thx) - max(cx - chx, tx - thx))
    oy = max(0.0, min(cy + chy, ty + thy) - max(cy - chy, ty - thy))
    frac = (ox * oy) / max(1e-9, min((2 * chx) * (2 * chy), (2 * thx) * (2 * thy)))
    if abs(cube_bot - tray_top) <= _IND_ON_TOP_EPS_M and frac >= _IND_MIN_XY_OVERLAP:
        return "ON_TOP_OF"

    halves = ((chx, chy, chz), (thx, thy, thz))
    gaps = []
    for ax, (cc, tt) in enumerate(zip((cx, cy, cz), (tx, ty, tz))):
        amn, amx = cc - halves[0][ax], cc + halves[0][ax]
        bmn, bmx = tt - halves[1][ax], tt + halves[1][ax]
        if amx < bmn:
            gaps.append(bmn - amx)
        elif bmx < amn:
            gaps.append(amn - bmx)
        else:
            gaps.append(-min(amx, bmx) + max(amn, bmn))
    pos = [g for g in gaps if g > 0]
    gap = (sum(g * g for g in pos)) ** 0.5 if pos else max(gaps)
    return "NEAR" if gap <= _IND_NEAR_GAP_M else "FAR_FROM"


# A machine- and human-readable description of the ladder, for the README/table.
LADDER = (
    {"key": "B1_center_in_footprint",
     "label": "B1 center-in-footprint",
     "question": "terminal cube center within the tray outer footprint (2D, no rim)"},
    {"key": "B2_footprint_overlap",
     "label": "B2 footprint-overlap",
     "question": f"terminal cube footprint overlaps tray footprint >= {DEFAULT_B2_MIN_OVERLAP_FRAC:g} (2D, no rim)"},
    {"key": "B3_full_inner_containment",
     "label": "B3 full-inner-containment",
     "question": "terminal cube footprint fully inside shrunk inner region (2D, margin)"},
    {"key": "B4_full_containment_started_outside",
     "label": "B4 contained+started-outside",
     "question": "B3 and cube started outside the footprint (2D + initial, no evidence gate)"},
    {"key": "B5_terminal_3d_containment",
     "label": "B5 terminal-3D-containment",
     "question": "csg.is_inside on the LAST frame: shrunk footprint AND rim height (3D, the maximal single-frame terminal predicate)"},
    {"key": "B6_contained_started_outside_evidence_gated",
     "label": "B6 contained+started-outside+evidence-gated",
     "question": "B4 AND the verifier's fail-closed evidence gate passes (the engineered steelman: matches the structured verifier's verdict profile on this dataset)"},
)
