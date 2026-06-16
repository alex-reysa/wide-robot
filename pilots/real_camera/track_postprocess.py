#!/usr/bin/env python3
"""Pure-Python ``real_camera.tracks.v0`` post-processing for raw-video ingestion (cv2-free).

Marker detection legitimately drops objects for stretches of a real clip (a hand passes over
the cube during the place motion; the cube hides the tray's inside floor tag once placed).
These transforms make such an episode judgeable by the FROZEN verifier WITHOUT weakening it:

  * :func:`trim_to_mover_span` — the episode is the interval the mover (cube) is observed;
    leading/trailing no-cube frames aren't part of the manipulation and break the endpose gate.
  * :func:`interpolate_mover_gaps` — short mover occlusions are linearly interpolated (the
    rollout minter requires every frame populated); LONG gaps are left for the gate to flag.
  * :func:`stabilize_static_objects` — a STATIC container is held at one fitted pose for the
    whole episode (it does not move), so occlusion/jitter of its tag can't drop or wobble it.

The MOVER is never fabricated beyond short interpolation, so a genuinely untracked cube still
fails the evidence-quality gate as UNCERTAIN. Nothing here imports ``csg``.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional


def _median(values: List[float]) -> float:
    s = sorted(values)
    return s[len(s) // 2]


def _mover_roles(tracks: Dict[str, Any]) -> List[str]:
    return [str(o["sourceRole"]) for o in tracks["objects"] if str(o.get("mobility")) != "STATIC"]


def _has_pose(frame: Dict[str, Any], role: str) -> bool:
    p = frame.get("poses", {}).get(role)
    return isinstance(p, dict) and isinstance(p.get("positionM"), dict)


def trim_to_mover_span(tracks: Dict[str, Any], min_frames: int = 3) -> Dict[str, Any]:
    """Trim the episode to the frame span over which the MOVER (cube) is observed.

    No-op if the cube is absent or already spans the whole clip. Mutates and returns ``tracks``."""
    roles = _mover_roles(tracks)
    if not roles:
        return tracks
    present = [i for i, f in enumerate(tracks["frames"]) if any(_has_pose(f, r) for r in roles)]
    if len(present) < min_frames:
        return tracks
    lo, hi = present[0], present[-1]
    if lo == 0 and hi == len(tracks["frames"]) - 1:
        return tracks
    tracks["frames"] = tracks["frames"][lo:hi + 1]
    return tracks


def interpolate_mover_gaps(tracks: Dict[str, Any], max_gap: int) -> Dict[str, Any]:
    """Fill SHORT mover-occlusion gaps (<= ``max_gap`` frames) by linear interpolation between
    bracketing observations; flag filled poses ``interpolated``. Longer gaps are left unfilled
    so the evidence-quality gate surfaces them as UNCERTAIN. Mutates and returns ``tracks``."""
    frames = tracks["frames"]
    for role in _mover_roles(tracks):
        present = [i for i, f in enumerate(frames) if _has_pose(f, role)]
        for a, b in zip(present, present[1:]):
            gap = b - a - 1
            if not (0 < gap <= max_gap):
                continue
            pa, pb = frames[a]["poses"][role]["positionM"], frames[b]["poses"][role]["positionM"]
            ori = frames[a]["poses"][role].get("orientationWxyz")
            for k in range(a + 1, b):
                t = (k - a) / (b - a)
                pose: Dict[str, Any] = {
                    "positionM": {ax: pa[ax] + t * (pb[ax] - pa[ax]) for ax in ("x", "y", "z")},
                    "confidence": 0.8, "interpolated": True}
                if ori:
                    pose["orientationWxyz"] = dict(ori)
                frames[k].setdefault("poses", {})[role] = pose
    return tracks


def stabilize_static_objects(tracks: Dict[str, Any],
                             overrides: Optional[Dict[str, Optional[List[float]]]] = None) -> Dict[str, Any]:
    """Hold each STATIC object at one fixed pose for the whole episode. ``overrides`` supplies a
    world position per role (e.g. a fitted tray center, more accurate than the noisy per-frame
    marker-offset pose); otherwise the robust per-axis median of observed frames is used. A
    static object never observed and without an override is left missing (fail-closed). The
    mover is untouched. Mutates and returns ``tracks``."""
    overrides = overrides or {}
    static_roles = [str(o["sourceRole"]) for o in tracks["objects"] if str(o.get("mobility")) == "STATIC"]
    for role in static_roles:
        seen = [f["poses"][role] for f in tracks["frames"] if _has_pose(f, role)]
        if overrides.get(role) is not None:
            x, y, z = overrides[role]
            fixed = {"x": float(x), "y": float(y), "z": float(z)}
        elif seen:
            fixed = {k: _median([p["positionM"][k] for p in seen]) for k in ("x", "y", "z")}
        else:
            continue
        ori = next((p.get("orientationWxyz") for p in seen if p.get("orientationWxyz")), None)
        for f in tracks["frames"]:
            pose: Dict[str, Any] = {"positionM": dict(fixed), "confidence": 1.0}
            if ori:
                pose["orientationWxyz"] = dict(ori)
            f.setdefault("poses", {})[role] = pose
    return tracks
