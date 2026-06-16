#!/usr/bin/env python3
"""Render *how the ingestion pipeline maps a real clip* onto a video frame.

This is a diagnostic/QA tool, NOT part of the verdict path: it draws, over a chosen frame of
a real Sony/iPhone clip, the things the Phase 3A pipeline computed for that clip —

  * detected AprilTag corners + ids (cyan)                 — what the detector saw
  * the virtual cube box + center (green)                  — the tracked MOVABLE object
  * the virtual tray outer footprint + center (red)        — the STATIC container box
  * the shrunk INSIDE footprint (yellow)                   — csg.predicates' inside test
  * the terminal relation (INSIDE/ON_TOP_OF/NEAR/FAR_FROM) — what the geometry decides
  * the committed verdict for this episode/camera (white)  — terminal/relation PASS/FAIL

so a human can SEE whether a FAIL (or partial-recall PASS) is the calibration being slightly
off vs. the cube genuinely outside. It never re-judges anything; it reads the *already
committed* tracks / per-clip calibration / verdicts and the raw video, and writes a JPEG.

Split, like the rest of the pilot, into a pure cv2-free core (frame selection, world-frame
footprint corners, world->pixel projection, the relation label) that is fully unit-tested,
and a thin lazy-cv2 render path (``render_overlay`` / ``main``) that needs OpenCV + the raw
mp4. ``csg/`` is only READ (``csg.predicates``), never modified.

Run with the bundled Python that has the camera extra installed::

    /Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \\
      -m pilots.real_camera.visualize_episode \\
      --episode oic_success_001 --camera sony_front --frame terminal --out output/overlays/
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Tuple

from csg.common import load_json
from csg.predicates import DEFAULT, box_from, is_near, primary_topo_relation

# Repo-relative defaults (this file lives at pilots/real_camera/visualize_episode.py).
_REPO = Path(__file__).resolve().parents[2]
_RECORDINGS_DIR = _REPO / "recordings"
_DATASET_ROOT = _REPO / "datasets" / "sony_object_inside_container_v0"
_DEFAULT_OUT = _REPO / "output" / "overlays"

CAMERAS = ("sony_front", "iphone_top")
Vec3 = Tuple[float, float, float]

# BGR colours (OpenCV order). Documented contract shared with the overlay legend.
COLOR_TAG = (255, 255, 0)        # cyan   — detected AprilTags
COLOR_TRAY = (0, 0, 255)         # red    — tray outer footprint
COLOR_INSIDE = (0, 255, 255)     # yellow — shrunk INSIDE footprint
COLOR_CUBE = (0, 255, 0)         # green  — cube box / center
COLOR_TEXT = (255, 255, 255)     # white  — labels


# ===========================================================================
# Pure geometry / selection core (NO cv2, NO numpy) — unit-tested.
# ===========================================================================


def _has_role(frame: Mapping[str, Any], role: str) -> bool:
    poses = frame.get("poses") if isinstance(frame, Mapping) else None
    pose = poses.get(role) if isinstance(poses, Mapping) else None
    return isinstance(pose, Mapping) and pose.get("positionM") is not None


def select_frame_index(frames: Sequence[Mapping[str, Any]], which: Any = "terminal",
                       role: str = "cube") -> int:
    """Resolve ``which`` to a concrete ``frameIndex``.

    ``"terminal"`` / ``"start"`` pick the last / first frame that actually carries a ``role``
    (cube) pose — the marker is often occluded in the leading/trailing frames, and the overlay
    is only meaningful where the tracked object exists. ``"middle"`` picks the median such
    frame. An int (or numeric string) selects that exact ``frameIndex`` (must be present)."""
    if isinstance(which, str):
        key = which.strip().lower()
    else:
        key = which
    cube_frames = [int(f["frameIndex"]) for f in frames if _has_role(f, role)]

    if key in ("terminal", "end", "last"):
        if not cube_frames:
            raise ValueError(f"no frame carries a {role!r} pose; cannot pick 'terminal'")
        return cube_frames[-1]
    if key in ("start", "first", "begin"):
        if not cube_frames:
            raise ValueError(f"no frame carries a {role!r} pose; cannot pick 'start'")
        return cube_frames[0]
    if key in ("middle", "mid"):
        if not cube_frames:
            raise ValueError(f"no frame carries a {role!r} pose; cannot pick 'middle'")
        return cube_frames[len(cube_frames) // 2]

    # Explicit integer frameIndex.
    try:
        target = int(key)
    except (TypeError, ValueError):
        raise ValueError(f"unrecognised --frame value: {which!r}")
    present = {int(f["frameIndex"]) for f in frames}
    if target not in present:
        raise KeyError(f"frameIndex {target} not in tracks (have {min(present)}..{max(present)})")
    return target


def frame_by_index(frames: Sequence[Mapping[str, Any]], frame_index: int) -> Mapping[str, Any]:
    """The frame dict whose ``frameIndex`` equals ``frame_index`` (KeyError if absent)."""
    for f in frames:
        if int(f["frameIndex"]) == int(frame_index):
            return f
    raise KeyError(f"frameIndex {frame_index} not found")


def pose_xyz(frame: Mapping[str, Any], role: str) -> Optional[Vec3]:
    """``(x, y, z)`` world position of ``role`` in ``frame`` (None if absent). Tolerates both
    the dict (``{"x":..}``) and list (``[x,y,z]``) ``positionM`` spellings."""
    poses = frame.get("poses") if isinstance(frame, Mapping) else None
    pose = poses.get(role) if isinstance(poses, Mapping) else None
    if not isinstance(pose, Mapping):
        return None
    pm = pose.get("positionM")
    if isinstance(pm, Mapping):
        return (float(pm["x"]), float(pm["y"]), float(pm["z"]))
    if isinstance(pm, (list, tuple)) and len(pm) >= 3:
        return (float(pm[0]), float(pm[1]), float(pm[2]))
    return None


def object_size(objects: Sequence[Mapping[str, Any]], role: str) -> Optional[List[float]]:
    """``sizeM`` of the object with ``sourceRole == role`` (None if not present)."""
    for obj in objects:
        if str(obj.get("sourceRole")) == role:
            size = obj.get("sizeM")
            return [float(v) for v in size] if size is not None else None
    return None


def _footprint_z(center: Vec3, size: Sequence[float], level: str) -> float:
    cz, hz = float(center[2]), float(size[2]) / 2.0
    if level == "rim":
        return cz + hz
    if level == "floor":
        return cz - hz
    return cz  # "center"


def _xy_rect(cx: float, cy: float, hx: float, hy: float, z: float) -> List[List[float]]:
    # Counter-clockwise so the polyline closes cleanly.
    return [[cx - hx, cy - hy, z], [cx + hx, cy - hy, z],
            [cx + hx, cy + hy, z], [cx - hx, cy + hy, z]]


def tray_footprint_corners(center: Vec3, size: Sequence[float], *, level: str = "rim") -> List[List[float]]:
    """Four world corners of the tray's axis-aligned footprint (full XY extent).

    ``level`` chooses the height at which to draw it: ``"rim"`` (top, the visible opening —
    default), ``"floor"`` (bottom), or ``"center"``. The footprint is axis-aligned in WORLD
    coordinates because that is exactly the box ``csg.predicates`` judges INSIDE against (yaw
    is a documented V0 simplification; the world frame is tray-aligned by the calibration)."""
    hx, hy = float(size[0]) / 2.0, float(size[1]) / 2.0
    return _xy_rect(float(center[0]), float(center[1]), hx, hy, _footprint_z(center, size, level))


def inside_footprint_corners(center: Vec3, size: Sequence[float], margin: float,
                             *, level: str = "rim") -> List[List[float]]:
    """The tray footprint shrunk by ``margin`` per side — the SAME box ``is_inside`` uses
    (pass ``csg.predicates.DEFAULT.inside_footprint_margin_m``). A cube center outside this
    rectangle is judged not-inside even if it overlaps the outer footprint (e.g. on the rim)."""
    hx, hy = float(size[0]) / 2.0 - float(margin), float(size[1]) / 2.0 - float(margin)
    return _xy_rect(float(center[0]), float(center[1]), hx, hy, _footprint_z(center, size, level))


def box_corners_3d(center: Vec3, size: Sequence[float]) -> List[List[float]]:
    """The 8 corners of an axis-aligned box (for a cube wireframe). Order is the bit pattern
    of (sx, sy, sz) signs so :data:`BOX_EDGES` indexes the 12 edges."""
    cx, cy, cz = (float(v) for v in center)
    hx, hy, hz = (float(s) / 2.0 for s in size)
    out: List[List[float]] = []
    for sz in (-1, 1):
        for sy in (-1, 1):
            for sx in (-1, 1):
                out.append([cx + sx * hx, cy + sy * hy, cz + sz * hz])
    return out


# Edges as index pairs into box_corners_3d's 8-corner output.
BOX_EDGES = ((0, 1), (1, 3), (3, 2), (2, 0),   # bottom face (z-)
             (4, 5), (5, 7), (7, 6), (6, 4),   # top face (z+)
             (0, 4), (1, 5), (2, 6), (3, 7))   # verticals


def invert_rigid_transform(M: Sequence[Sequence[float]]) -> List[List[float]]:
    """Inverse of a 4x4 rigid ``[R|t]`` (e.g. ``cameraToWorld`` -> ``worldToCamera``).

    For a rotation+translation the inverse is ``[R^T | -R^T t]`` — exact and cv2-free, so the
    projection core needs no numpy. (The calibration's ``cameraToWorld`` is rigid by
    construction in ``author_calibration``.)"""
    R = [[float(M[i][j]) for j in range(3)] for i in range(3)]
    t = [float(M[i][3]) for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    nt = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [[Rt[0][0], Rt[0][1], Rt[0][2], nt[0]],
            [Rt[1][0], Rt[1][1], Rt[1][2], nt[1]],
            [Rt[2][0], Rt[2][1], Rt[2][2], nt[2]],
            [0.0, 0.0, 0.0, 1.0]]


def project_world_point(p_world: Sequence[float], world_to_camera: Sequence[Sequence[float]],
                        camera_matrix: Sequence[Sequence[float]]) -> Optional[Tuple[float, float]]:
    """Project a world point to a pixel via a pinhole model (zero distortion).

    Returns ``(u, v)`` or ``None`` if the point is at/behind the camera (``z_cam <= 0``)."""
    x, y, z = float(p_world[0]), float(p_world[1]), float(p_world[2])
    M = world_to_camera
    pc = [M[i][0] * x + M[i][1] * y + M[i][2] * z + M[i][3] for i in range(3)]
    if pc[2] <= 1e-9:
        return None
    fx, fy = float(camera_matrix[0][0]), float(camera_matrix[1][1])
    cx, cy = float(camera_matrix[0][2]), float(camera_matrix[1][2])
    return (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy)


def primary_relation_label(cube_center: Vec3, tray_center: Vec3,
                           cube_size: Sequence[float], tray_size: Sequence[float]) -> str:
    """The single strongest cube->tray relation, using the SAME predicates as the verifier:
    ``INSIDE`` / ``ON_TOP_OF`` (topological), else ``NEAR`` / ``FAR_FROM`` (proximity)."""
    cube = box_from(cube_center, tuple(cube_size))
    tray = box_from(tray_center, tuple(tray_size))
    topo = primary_topo_relation(cube, tray)
    if topo is not None:
        return topo
    return "NEAR" if is_near(cube, tray) else "FAR_FROM"


# ===========================================================================
# Data lookups (pure) — resolve clip-specific artefacts from the committed dataset.
# ===========================================================================


def resolve_video_path(manifest: Mapping[str, Any], episode_id: str, camera: str,
                       recordings_dir: Path) -> Path:
    """Raw mp4 path for ``(episode_id, camera)`` from ``recordings/manifest.json``'s videos."""
    for v in manifest.get("videos", []):
        if str(v.get("episodeId")) == episode_id and str(v.get("camera")) == camera:
            return Path(recordings_dir) / str(v["relativePath"])
    raise KeyError(f"no video for episode={episode_id!r} camera={camera!r} in manifest")


def verdict_for(verdicts: Mapping[str, Any], episode_id: str, camera: str) -> Optional[Mapping[str, Any]]:
    """The ``verdicts_all.json`` row for ``(episode_id, camera)`` (None if absent)."""
    for row in verdicts.get("rows", []):
        if str(row.get("episodeId")) == episode_id and str(row.get("camera")) == camera:
            return row
    return None


def _clip_stem(episode_id: str, camera: str) -> str:
    return f"{episode_id}__{camera}"


def load_clip_artifacts(episode_id: str, camera: str, *, dataset_root: Path = _DATASET_ROOT,
                        recordings_dir: Path = _RECORDINGS_DIR) -> dict:
    """Load everything the overlay needs for one clip: raw-video path, tracks, per-clip
    calibration, and the verdict row (verdict may be None). Raises if the core artefacts are
    missing so the CLI can report a clear error instead of half-drawing."""
    stem = _clip_stem(episode_id, camera)
    manifest = load_json(recordings_dir / "manifest.json")
    video_path = resolve_video_path(manifest, episode_id, camera, recordings_dir)

    tracks_path = dataset_root / "tracks" / f"{stem}.tracks.json"
    calib_path = dataset_root / "calibration" / "perclip" / f"{stem}.calibration.json"
    if not tracks_path.exists():
        raise FileNotFoundError(f"tracks not found: {tracks_path}")
    if not calib_path.exists():
        raise FileNotFoundError(f"per-clip calibration not found: {calib_path}")
    tracks = load_json(tracks_path)
    calibration = load_json(calib_path)

    verdicts_path = dataset_root / "verdicts_all.json"
    verdict = None
    if verdicts_path.exists():
        verdict = verdict_for(load_json(verdicts_path), episode_id, camera)

    return {"videoPath": video_path, "tracks": tracks, "calibration": calibration,
            "verdict": verdict, "clipStem": stem}


def build_overlay_plan(tracks: Mapping[str, Any], calibration: Mapping[str, Any],
                       frame_index: int, *, margin: float = DEFAULT.inside_footprint_margin_m) -> dict:
    """Compute, for one frame, all the world-frame geometry to be projected/drawn (cv2-free).

    Returns a dict with the chosen frame's cube/tray centers + sizes, the outer & inside tray
    footprints, the cube wireframe corners, and the terminal relation label. Heights/sizes are
    pulled from the committed tracks (objects[].sizeM) and frame poses, so the overlay reflects
    EXACTLY what was ingested. Either object may be missing (occlusion) -> its entry is None."""
    frame = frame_by_index(tracks["frames"], frame_index)
    objects = tracks.get("objects", [])
    cube_size = object_size(objects, "cube") or [0.05, 0.05, 0.05]
    tray_size = object_size(objects, "tray") or [0.18, 0.18, 0.07]
    cube_center = pose_xyz(frame, "cube")
    tray_center = pose_xyz(frame, "tray")

    plan: dict = {
        "frameIndex": frame_index,
        "timeS": frame.get("timeS"),
        "cubeCenter": cube_center, "cubeSize": cube_size,
        "trayCenter": tray_center, "traySize": tray_size,
        "trayFootprint": None, "insideFootprint": None, "cubeCorners": None,
        "relation": None, "margin": margin,
    }
    if tray_center is not None:
        plan["trayFootprint"] = tray_footprint_corners(tray_center, tray_size)
        plan["insideFootprint"] = inside_footprint_corners(tray_center, tray_size, margin)
    if cube_center is not None:
        plan["cubeCorners"] = box_corners_3d(cube_center, cube_size)
    if cube_center is not None and tray_center is not None:
        plan["relation"] = primary_relation_label(cube_center, tray_center, cube_size, tray_size)
    return plan


def verdict_summary(verdict: Optional[Mapping[str, Any]]) -> List[str]:
    """Short human lines describing the committed verdict for the overlay text block."""
    if not verdict:
        return ["verdict: (none)"]
    term = verdict.get("actualTerminal")
    rel = verdict.get("actualRelation")
    cls = verdict.get("terminalClass")
    lines = [f"terminal={term}  relation={rel}",
             f"expected={verdict.get('expectedClass')}"]
    if cls:
        lines.append(f"failClass={cls}")
    return lines


# ===========================================================================
# Render path (lazy cv2 + numpy + raw video) — smoke-only, not in CI.
# ===========================================================================


def _read_frame_at(video_path: Path, frame_index: int):  # pragma: no cover - real cv2 path
    """Sequentially decode up to ``frame_index`` and return that BGR frame.

    Sequential read (not ``CAP_PROP_POS_FRAMES`` seeking) guarantees the returned image is the
    SAME frame ``video_to_tracks`` indexed — keyframe seeking can land elsewhere, which would
    misalign the overlay from the pose it is drawing."""
    import cv2
    cap = cv2.VideoCapture(str(video_path))
    try:
        frame = None
        for _ in range(frame_index + 1):
            ok, frame = cap.read()
            if not ok:
                raise ValueError(f"video ended before frame {frame_index}: {video_path}")
        return frame
    finally:
        cap.release()


def _draw_polygon(image, corners_uv, color, thickness=2):  # pragma: no cover - real cv2 path
    import cv2
    import numpy as np
    pts = np.array([[int(round(u)), int(round(v))] for (u, v) in corners_uv], dtype=np.int32)
    cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness, lineType=cv2.LINE_AA)


def _draw_text_block(image, lines, *, origin=(20, 40), scale=1.0):  # pragma: no cover
    import cv2
    x, y = origin
    step = int(38 * scale)
    for i, line in enumerate(lines):
        yy = y + i * step
        # Dark outline for legibility over any background, then the white glyphs.
        cv2.putText(image, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(image, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, scale, COLOR_TEXT, 2, cv2.LINE_AA)


def render_overlay(episode_id: str, camera: str, which: Any = "terminal", *,
                   out_dir: Path = _DEFAULT_OUT, dataset_root: Path = _DATASET_ROOT,
                   recordings_dir: Path = _RECORDINGS_DIR,
                   draw_tags: bool = True, draw_centers: bool = True,
                   draw_inside: bool = True, draw_tray: bool = True,
                   draw_cube: bool = True) -> Path:  # pragma: no cover - real cv2 path
    """Render the overlay JPEG for one clip/frame and return the output path."""
    import cv2

    art = load_clip_artifacts(episode_id, camera, dataset_root=dataset_root,
                              recordings_dir=recordings_dir)
    tracks, calibration, verdict = art["tracks"], art["calibration"], art["verdict"]
    frame_index = select_frame_index(tracks["frames"], which)
    plan = build_overlay_plan(tracks, calibration, frame_index)

    image = _read_frame_at(art["videoPath"], frame_index)
    h, w = image.shape[:2]
    # The verdict scale (~1.0 at 1080p) so 4K text isn't microscopic.
    scale = max(1.0, w / 1920.0)

    world_to_camera = invert_rigid_transform(calibration["cameraToWorld"])
    K = calibration["cameraMatrix"]

    def proj(p):
        return project_world_point(p, world_to_camera, K)

    def proj_poly(corners):
        uv = [proj(c) for c in corners]
        return [p for p in uv if p is not None] if all(p is not None for p in uv) else None

    # --- detected AprilTags (drawn from the SAME detector the pipeline uses) ---
    if draw_tags:
        from pilots.real_camera.marker_tracker import ArucoDetector
        for obs in ArucoDetector().detect(image):
            poly = [(float(x), float(y)) for x, y in obs.corners]
            _draw_polygon(image, poly, COLOR_TAG, thickness=max(2, int(2 * scale)))
            cx = int(round(sum(p[0] for p in poly) / 4))
            cy = int(round(sum(p[1] for p in poly) / 4))
            cv2.putText(image, f"id{obs.marker_id}", (cx + 6, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(image, f"id{obs.marker_id}", (cx + 6, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, COLOR_TAG, 2, cv2.LINE_AA)

    # --- tray outer footprint (red) + inside shrunk footprint (yellow) ---
    if draw_tray and plan["trayFootprint"]:
        poly = proj_poly(plan["trayFootprint"])
        if poly:
            _draw_polygon(image, poly, COLOR_TRAY, thickness=max(2, int(2 * scale)))
    if draw_inside and plan["insideFootprint"]:
        poly = proj_poly(plan["insideFootprint"])
        if poly:
            _draw_polygon(image, poly, COLOR_INSIDE, thickness=max(2, int(2 * scale)))

    # --- cube wireframe (green) ---
    if draw_cube and plan["cubeCorners"]:
        uv = [proj(c) for c in plan["cubeCorners"]]
        for a, b in BOX_EDGES:
            if uv[a] is not None and uv[b] is not None:
                cv2.line(image, (int(round(uv[a][0])), int(round(uv[a][1]))),
                         (int(round(uv[b][0])), int(round(uv[b][1]))),
                         COLOR_CUBE, max(2, int(2 * scale)), cv2.LINE_AA)

    # --- object centers ---
    if draw_centers:
        if plan["cubeCenter"] is not None:
            c = proj(plan["cubeCenter"])
            if c is not None:
                cv2.circle(image, (int(round(c[0])), int(round(c[1]))), max(5, int(6 * scale)),
                           COLOR_CUBE, -1, cv2.LINE_AA)
        if plan["trayCenter"] is not None:
            c = proj(plan["trayCenter"])
            if c is not None:
                cv2.circle(image, (int(round(c[0])), int(round(c[1]))), max(5, int(6 * scale)),
                           COLOR_TRAY, -1, cv2.LINE_AA)

    # --- text block: identity, frame, relation, verdict, legend ---
    rel = plan["relation"] or "n/a (object occluded)"
    lines = [f"{episode_id}  [{camera}]",
             f"frame {frame_index}  ({which})",
             f"relation: {rel}"]
    lines += verdict_summary(verdict)
    lines.append("tags=cyan tray=red inside=yellow cube=green")
    _draw_text_block(image, lines, origin=(int(20 * scale), int(45 * scale)), scale=0.8 * scale)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{art['clipStem']}__{which}.jpg"
    cv2.imwrite(str(out_path), image)
    return out_path


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover - real cv2 path
    parser = argparse.ArgumentParser(
        description="Overlay the virtual tray/cube/INSIDE-footprint + detected tags onto a "
                    "real clip frame (needs OpenCV + the raw mp4).")
    parser.add_argument("--episode", required=True, help="episodeId, e.g. oic_success_001")
    parser.add_argument("--camera", required=True, choices=CAMERAS)
    parser.add_argument("--frame", default="terminal",
                        help="terminal | start | middle | <int frameIndex> (default: terminal)")
    parser.add_argument("--out", default=str(_DEFAULT_OUT), help="output directory")
    parser.add_argument("--no-tags", action="store_true", help="do not draw detected AprilTags")
    parser.add_argument("--no-centers", action="store_true", help="do not draw object centers")
    parser.add_argument("--no-inside-footprint", action="store_true",
                        help="do not draw the shrunk INSIDE footprint")
    parser.add_argument("--no-tray-footprint", action="store_true",
                        help="do not draw the tray outer footprint")
    parser.add_argument("--no-cube", action="store_true", help="do not draw the cube wireframe")
    args = parser.parse_args(argv)

    frame_arg: Any = args.frame
    if frame_arg.strip().lstrip("-").isdigit():
        frame_arg = int(frame_arg)

    out_path = render_overlay(
        args.episode, args.camera, frame_arg, out_dir=Path(args.out),
        draw_tags=not args.no_tags, draw_centers=not args.no_centers,
        draw_inside=not args.no_inside_footprint, draw_tray=not args.no_tray_footprint,
        draw_cube=not args.no_cube)
    print(f"visualize_episode: wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
