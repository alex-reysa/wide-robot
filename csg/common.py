#!/usr/bin/env python3
"""Shared helpers for the CSG package (single source of truth).

Consolidates the helpers that previously lived in both
``CSG_Solver_Harness/csg_common.py`` and the JSON-parsing preamble of
``CSG_Matcher/csg_matcher.py``. Every helper here is pure and side-effect free
except ``load_json`` / ``write_json``.

Naming policy: proto JSON may arrive as snake_case or camelCase; ``get_any``
accepts both. Enum strings are normalized to UPPER_SNAKE via ``enum_name``.
"""
from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

Json = Dict[str, Any]
Fact = Tuple[Any, ...]

ROBOT_GRIPPER_ID = "robot_gripper"


# -----------------------------------------------------------------------------
# Case / key access
# -----------------------------------------------------------------------------


def snake_to_camel(s: str) -> str:
    parts = str(s).split("_")
    return parts[0] + "".join(p[:1].upper() + p[1:] for p in parts[1:])


def camel_to_snake(s: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", str(s)).lower()


def get_any(d: Any, *names: str, default: Any = None) -> Any:
    """Return the first present value across snake/camel/lower-first spellings."""
    if not isinstance(d, Mapping):
        return default
    keys: List[str] = []
    for n in names:
        if not n:
            continue
        keys.extend([n, snake_to_camel(n), camel_to_snake(n), n[:1].lower() + n[1:]])
    for k in keys:
        if k in d:
            return d[k]
    return default


def as_list(x: Any) -> List[Any]:
    if x is None:
        return []
    return x if isinstance(x, list) else [x]


def enum_name(x: Any, default: str = "UNKNOWN") -> str:
    s = str(x if x is not None else default).strip() or default
    return re.sub(r"[^A-Za-z0-9_]+", "_", s).upper()


def norm_label(x: Any) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(x if x is not None else "unknown").lower()).strip("_") or "unknown"


def safe_id(x: Any, prefix: str = "id") -> str:
    s = re.sub(r"[^A-Za-z0-9_]+", "_", str(x or "")).strip("_") or prefix
    return f"{prefix}_{s}" if s[0].isdigit() else s


# -----------------------------------------------------------------------------
# IO
# -----------------------------------------------------------------------------


def load_json(path: str | Path) -> Json:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


read_json = load_json


def write_json(path: str | Path, obj: Any) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


save_json = write_json


def copy_json(obj: Any) -> Any:
    """Deep-copy a JSON-like value (single canonical definition)."""
    return copy.deepcopy(obj)


# -----------------------------------------------------------------------------
# Time
# -----------------------------------------------------------------------------


def ns_to_s(x: Any) -> float:
    try:
        return float(x) / 1e9
    except (TypeError, ValueError):
        return 0.0


def s_to_ns(x: float) -> str:
    return str(int(round(float(x) * 1e9)))


def time_span_json(a: float, b: Optional[float] = None) -> Json:
    if b is None:
        b = a
    return {"startTimeNs": s_to_ns(a), "endTimeNs": s_to_ns(b)}


make_timespan = time_span_json


def span_start_s(obj: Mapping[str, Any]) -> float:
    ts = get_any(obj, "timeSpan", "time_span", default={}) or {}
    a = ns_to_s(get_any(ts, "startTimeNs", "start_time_ns", default=0))
    if a != 0.0:
        return a
    return ns_to_s(get_any(obj, "timeNs", "time_ns", default=0))


def span_end_s(obj: Mapping[str, Any]) -> float:
    ts = get_any(obj, "timeSpan", "time_span", default={}) or {}
    b = ns_to_s(get_any(ts, "endTimeNs", "end_time_ns", default=0))
    a = ns_to_s(get_any(ts, "startTimeNs", "start_time_ns", default=0))
    if b != 0.0:
        return max(a, b)
    if a != 0.0:
        return a
    return ns_to_s(get_any(obj, "timeNs", "time_ns", default=0))


def span_mid_s(obj: Mapping[str, Any]) -> float:
    a, b = span_start_s(obj), span_end_s(obj)
    return 0.5 * (a + max(a, b))


time_mid = span_mid_s


# -----------------------------------------------------------------------------
# Confidence
# -----------------------------------------------------------------------------


def confidence(obj: Any, default: float = 1.0) -> float:
    try:
        if isinstance(obj, Mapping):
            return float(get_any(obj, "confidence", default=default))
    except (TypeError, ValueError):
        pass
    return default


# -----------------------------------------------------------------------------
# Geometry
# -----------------------------------------------------------------------------


def vec3(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Json:
    return {"x": float(x), "y": float(y), "z": float(z)}


def make_pose(
    frame_id: str = "world",
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    confidence_value: float = 1.0,
) -> Json:
    return {
        "frameId": frame_id,
        "positionM": vec3(*xyz),
        "orientationWxyz": {"w": float(quat[0]), "x": float(quat[1]), "y": float(quat[2]), "z": float(quat[3])},
        "confidence": float(confidence_value),
    }


def pose_xyz(pose: Any) -> Tuple[float, float, float]:
    p = get_any(pose, "positionM", "position_m", "position", default={}) if isinstance(pose, Mapping) else {}
    return (
        float(get_any(p, "x", default=0.0) or 0.0),
        float(get_any(p, "y", default=0.0) or 0.0),
        float(get_any(p, "z", default=0.0) or 0.0),
    )


def pose_quat(pose: Any) -> Tuple[float, float, float, float]:
    q = get_any(pose, "orientationWxyz", "orientation_wxyz", default={}) if isinstance(pose, Mapping) else {}
    qq = (
        float(get_any(q, "w", default=1.0) or 1.0),
        float(get_any(q, "x", default=0.0) or 0.0),
        float(get_any(q, "y", default=0.0) or 0.0),
        float(get_any(q, "z", default=0.0) or 0.0),
    )
    n = math.sqrt(sum(v * v for v in qq))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / n for v in qq)  # type: ignore[return-value]


def pose_with_xyz(pose: Any, xyz: Tuple[float, float, float]) -> Json:
    frame = str(get_any(pose, "frameId", "frame_id", default="world")) if isinstance(pose, Mapping) else "world"
    return make_pose(frame, xyz, pose_quat(pose), confidence(pose, 1.0))


def offset_pose(pose: Any, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Json:
    x, y, z = pose_xyz(pose)
    return pose_with_xyz(pose, (x + dx, y + dy, z + dz))


def dist3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def quat_angle(qa: Tuple[float, float, float, float], qb: Tuple[float, float, float, float]) -> float:
    dot = max(-1.0, min(1.0, abs(sum(qa[i] * qb[i] for i in range(4)))))
    return 2.0 * math.acos(dot)


# -----------------------------------------------------------------------------
# Object helpers
# -----------------------------------------------------------------------------


def object_id(obj: Mapping[str, Any]) -> str:
    return str(get_any(obj, "objectId", "object_id", default=""))


def category_label(obj: Mapping[str, Any]) -> str:
    return str(get_any(obj, "categoryLabel", "category_label", default="object"))


def geometry_kind(geom: Mapping[str, Any]) -> str:
    for key, name in [
        ("oriented_box", "ORIENTED_BOX"),
        ("orientedBox", "ORIENTED_BOX"),
        ("cylinder", "CYLINDER"),
        ("mesh", "MESH"),
        ("point_cloud", "POINT_CLOUD"),
        ("pointCloud", "POINT_CLOUD"),
        ("mask_only", "MASK_ONLY"),
        ("maskOnly", "MASK_ONLY"),
    ]:
        if isinstance(geom, Mapping) and key in geom and geom[key] not in (None, {}, []):
            return name
    return "UNKNOWN_GEOMETRY"


def object_size_m(obj: Mapping[str, Any]) -> Tuple[float, float, float]:
    """Best-available object size. When invented (no geometry), callers should
    mark provenance as MANUAL_APPROXIMATION; see ``size_is_approximate``."""
    geom = get_any(obj, "geometry", default={}) or {}
    ob = get_any(geom, "orientedBox", "oriented_box", default=None)
    if isinstance(ob, Mapping):
        s = get_any(ob, "sizeM", "size_m", default={}) or {}
        return (
            float(get_any(s, "x", default=0.04) or 0.04),
            float(get_any(s, "y", default=0.04) or 0.04),
            float(get_any(s, "z", default=0.04) or 0.04),
        )
    cyl = get_any(geom, "cylinder", default=None)
    if isinstance(cyl, Mapping):
        r = float(get_any(cyl, "radiusM", "radius_m", default=0.03) or 0.03)
        h = float(get_any(cyl, "heightM", "height_m", default=0.1) or 0.1)
        return (2 * r, 2 * r, h)
    if enum_name(get_any(obj, "physicalKind", "physical_kind", default="")) == "STATIC_SCENE_SURFACE":
        return (0.24, 0.18, 0.03)
    return (0.04, 0.04, 0.04)


def size_is_approximate(obj: Mapping[str, Any]) -> bool:
    """True when ``object_size_m`` had to invent the size (no metric geometry)."""
    geom = get_any(obj, "geometry", default={}) or {}
    return geometry_kind(geom) in {"UNKNOWN_GEOMETRY", "MASK_ONLY"}
