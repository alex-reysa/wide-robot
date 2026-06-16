#!/usr/bin/env python3
"""Ingest recorded ``object_inside_container`` clips through the real-camera pilot and judge
them with the FROZEN verifier:

    raw video --> real_camera.tracks.v0 --> csg.rollout.v0 --> verify_episode_both
                                                               (PASS / FAIL / UNCERTAIN)

Per clip we author an approximate, per-clip calibration (intrinsics from the camera profile at
the clip's real resolution; world-up extrinsic recovered from the flat markers 0/1/7), build
tracks with the real ArUco + solvePnP path, mint a leakage-clean rollout, and run BOTH bundled
camera targets. Artifacts (calibration, tracks, rollout, verdicts) are written under the
dataset dir; a confusion matrix vs the recorded labels is printed. Needs OpenCV (``.[camera]``).

Nothing in ``csg/`` is touched — the verifier runs unchanged; only the pilot-side calibration
and (optionally) evidence-quality thresholds are ours to set.
"""
from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

from csg.common import load_json, write_json
from pilots.real_camera import author_calibration as ac
from pilots.real_camera.marker_tracker import ArucoDetector
from pilots.real_camera.track_postprocess import (
    interpolate_mover_gaps,
    stabilize_static_objects,
    trim_to_mover_span,
)
from pilots.real_camera.tracks_to_rollout import tracks_to_rollout
from pilots.real_camera.verify_episode import _TARGETS_DIR, verify_episode, verify_episode_both
from pilots.real_camera.video_to_tracks import (
    PnPPoseEstimator,
    build_tracks,
    iter_video_frames,
    sha256_file,
)

TARGET_TERMINAL = "object_inside_container_terminal_only"
TARGET_RELATION = "object_inside_container_relation_event"        # initial NEAR -> INSIDE
TARGET_PLACED = "object_inside_container_placed_from_outside"     # initial FAR  -> INSIDE
TARGETS = (TARGET_TERMINAL, TARGET_RELATION, TARGET_PLACED)
FPS = 30000.0 / 1001.0  # 29.97


def combined_transition(rel_status: str, placed_status: str) -> str:
    """Put-in transition = relation_event PASS OR placed_from_outside PASS. The two targets bracket
    a real put-in's start (NEAR vs FAR); a born-inside cube (initial INSIDE) FAILs both. UNCERTAIN
    only if neither PASSes and at least one is UNCERTAIN (the evidence gate fired)."""
    if rel_status == "PASS" or placed_status == "PASS":
        return "PASS"
    if "UNCERTAIN" in (rel_status, placed_status):
        return "UNCERTAIN"
    return "FAIL"

# Evidence-quality thresholds RELAXED for raw 30 fps video (the pilot defaults were tuned for
# short synthetic fixtures). At 30 fps a hand occluding the mover for ~0.5-1 s during the place
# motion is normal and must not invalidate the episode, so the consecutive-missing run and the
# dropout fraction are loosened; the confidence/endpose/static-jitter gates are unchanged so a
# truly untracked object still fails closed. Rationale is documented in the plan/PR.
REAL_VIDEO_THRESHOLDS = {"max_consecutive_missing": 30, "max_dropout_frac": 0.35}

# expectedClass -> (terminal_expected, relation_expected). PASS_OR_UNCERTAIN == either is fine.
ORACLE: Dict[str, tuple] = {
    "success": ("PASS", "PASS"),
    "near_not_inside": ("FAIL", "FAIL"),
    "left_on_rim": ("FAIL", "FAIL"),
    "dropped_or_left_outside": ("FAIL", "FAIL"),
    "born_inside": ("PASS", "FAIL"),
    "born_inside_with_hand_motion": ("PASS", "FAIL"),
    "inside_to_inside": ("PASS", "FAIL"),
    "inside_to_outside": ("FAIL", "FAIL"),
    "static_no_task": ("FAIL", "FAIL"),
    "success_hand_obstruction": ("PASS_OR_UNCERTAIN", "PASS_OR_UNCERTAIN"),
    "success_tag_obstruction": ("PASS", "PASS_OR_UNCERTAIN"),
}

SMOKE_EPISODES = [
    "oic_success_001",
    "oic_fail_near_not_inside_001",
    "oic_fail_on_rim_001",
    "oic_control_born_inside_001",             # static cube -> motion-based extractor can't judge
    "oic_control_born_inside_hand_motion_001",  # cube moved inside -> the valid born discriminator
]
SONY_MARKER6_REF = "oic_success_001"  # clean clip (cube outside) to fit the tray marker-6 offset


def verdict_fits(expected: str, actual: str) -> bool:
    if expected == "PASS_OR_UNCERTAIN":
        return actual in ("PASS", "UNCERTAIN")
    return actual == expected


def fx_scale_for(camera: str, args: argparse.Namespace) -> float:
    return float(args.fx_scale_sony if camera == "sony_front" else args.fx_scale_iphone)


def tray_center_override(cdiag: Dict[str, Any], world_off7: Optional[List[float]]) -> Optional[List[float]]:
    """The tray center to hold the STATIC tray at, in the clip's world frame. When the front
    wall (marker 6) is visible we trust the per-clip midpoint; otherwise (top view) we apply
    the reference world-frame marker7->center offset to this clip's marker 7."""
    det = cdiag.get("detectedMarkers") or []
    if 6 in det and cdiag.get("trayCenterWorld"):
        return cdiag["trayCenterWorld"]
    m7 = cdiag.get("marker7World")
    if m7 and world_off7:
        return [m7[i] + world_off7[i] for i in range(3)]
    return cdiag.get("trayCenterWorld")


def ingest_clip(video_path: Path, camera: str, episode_id: str, *, fx_scale: float,
                marker6_offset: Optional[List[float]], marker7_offset: Optional[List[float]],
                world_off7: Optional[List[float]], out_dir: Path, max_frames: int) -> Dict[str, Any]:
    """Author calibration, build tracks, mint rollout, verify both targets. Returns a record."""
    calib, cdiag = ac.calibration_for_clip(video_path, camera, fx_scale=fx_scale,
                                           marker6_offset_m=marker6_offset,
                                           marker7_offset_m=marker7_offset, max_frames=max_frames)
    stem = f"{episode_id}__{camera}"
    write_json(out_dir / "calibration" / "perclip" / f"{stem}.calibration.json", calib)

    tracks = build_tracks(
        iter_video_frames(video_path),
        detector=ArucoDetector(),
        estimator=PnPPoseEstimator(),
        calibration=calib,
        fps=FPS,
        episode_id=episode_id,
        video_sha256=sha256_file(video_path),
    )
    trim_to_mover_span(tracks)  # episode = the span over which the cube is observed
    interpolate_mover_gaps(tracks, max_gap=int(REAL_VIDEO_THRESHOLDS["max_consecutive_missing"]))
    tray_world = tray_center_override(cdiag, world_off7)
    stabilize_static_objects(tracks, overrides={"tray": tray_world})  # hold STATIC tray at its fitted center
    cdiag["trayCenterInjected"] = tray_world
    write_json(out_dir / "tracks" / f"{stem}.tracks.json", tracks)

    record = verify_episode_both(tracks=tracks, thresholds=REAL_VIDEO_THRESHOLDS)
    # placed_from_outside is evaluated alongside the canonical bundle (kept at the RLBench-parity
    # pair) so the combined put-in transition can be reported without changing verify_episode_both.
    placed_target = load_json(_TARGETS_DIR / f"{TARGET_PLACED}.json")
    record[TARGET_PLACED] = verify_episode(placed_target, tracks=tracks,
                                           thresholds=REAL_VIDEO_THRESHOLDS, case_name=TARGET_PLACED)
    try:  # best-effort rollout artifact (skipped when the UNCERTAIN gate rejects the evidence)
        write_json(out_dir / "rollouts" / f"{stem}.rollout.json", tracks_to_rollout(tracks))
    except Exception:
        pass

    per_target = {t: {"status": record[t]["status"],
                      "failureClass": record[t].get("cameraFailureClass") or record[t].get("failureClass"),
                      "hardMismatches": record[t].get("hardMismatches")} for t in TARGETS}
    metrics = record[TARGETS[0]].get("trackingMetrics") or {}
    return {"calibrationHash": calib["markerMapHash"], "calibDiag": cdiag,
            "perTarget": per_target, "numFrames": len(tracks["frames"]),
            "trackingMetrics": metrics}


def fit_sony_tray_offsets(manifest: dict, recordings_root: Path, max_frames: int) -> tuple:
    """Fit the tray geometry once from the clean Sony reference clip (which sees markers 6 AND
    7, cube outside). Returns ``(off6, off7, world_off7)``: the marker-frame offsets plus the
    WORLD-frame marker7->tray-center offset. Both cameras anchor world to table markers 0/1, so
    ``world_off7`` transfers to the top view (which can't see marker 6) without depending on a
    single tag's fragile in-plane yaw."""
    for v in manifest["videos"]:
        if v["episodeId"] == SONY_MARKER6_REF and v["camera"] == "sony_front":
            path = recordings_root / v["relativePath"]
            _, diag = ac.calibration_for_clip(path, "sony_front", max_frames=max_frames)
            off6, off7 = diag.get("marker6Offset"), diag.get("marker7Offset")
            center, m7 = diag.get("trayCenterWorld"), diag.get("marker7World")
            world_off7 = [round(center[i] - m7[i], 5) for i in range(3)] if (center and m7) else None
            print(f"[ref] fitted Sony tray from {v['episodeId']}: trayCenter={center} "
                  f"marker7World={m7} worldOff7={world_off7}")
            return off6, off7, world_off7
    return None, None, None


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Ingest recorded clips and judge with the frozen verifier.")
    parser.add_argument("--manifest", default="recordings/manifest.json")
    parser.add_argument("--out-dir", default="datasets/sony_object_inside_container_v0")
    parser.add_argument("--select", default="smoke", help="'smoke', 'all', or comma-separated episodeIds")
    parser.add_argument("--cameras", default="sony_front,iphone_top")
    parser.add_argument("--fx-scale-sony", type=float, default=1.0)
    parser.add_argument("--fx-scale-iphone", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=30)
    parser.add_argument("--verdicts-out", default=None, help="defaults to <out-dir>/verdicts_<select>.json")
    args = parser.parse_args(argv)

    manifest = json.loads(Path(args.manifest).read_text())
    recordings_root = Path(args.manifest).resolve().parent
    out_dir = Path(args.out_dir)
    cameras = set(args.cameras.split(","))

    if args.select == "smoke":
        want = set(SMOKE_EPISODES)
    elif args.select == "all":
        want = None
    else:
        want = set(args.select.split(","))

    clips = [v for v in manifest["videos"]
             if v["camera"] in cameras
             and v.get("expectedClass") in ORACLE
             and (want is None or v["episodeId"] in want)]
    clips.sort(key=lambda v: (v["episodeId"], v["camera"]))
    print(f"selected {len(clips)} clips ({args.select}); cameras={sorted(cameras)}")

    sony_off6, ref_off7, world_off7 = fit_sony_tray_offsets(manifest, recordings_root, args.max_frames)

    rows: List[Dict[str, Any]] = []
    for v in clips:
        cam = v["camera"]
        eid = v["episodeId"]
        path = recordings_root / v["relativePath"]
        exp = ORACLE[v["expectedClass"]]
        row: Dict[str, Any] = {"episodeId": eid, "camera": cam, "expectedClass": v["expectedClass"],
                               "expectedTerminal": exp[0], "expectedRelation": exp[1]}
        try:
            rec = ingest_clip(path, cam, eid, fx_scale=fx_scale_for(cam, args),
                              marker6_offset=(sony_off6 if cam == "sony_front" else None),
                              marker7_offset=ref_off7, world_off7=world_off7,
                              out_dir=out_dir, max_frames=args.max_frames)
            term = rec["perTarget"][TARGET_TERMINAL]
            rel = rec["perTarget"][TARGET_RELATION]
            placed = rec["perTarget"][TARGET_PLACED]
            trans = combined_transition(rel["status"], placed["status"])
            row.update({
                "actualTerminal": term["status"], "actualRelation": rel["status"],
                "actualPlaced": placed["status"], "actualTransition": trans,
                "terminalClass": term["failureClass"],
                "transitionClass": (rel["failureClass"] if rel["status"] != "PASS" else None)
                                   or placed["failureClass"],
                "termFits": verdict_fits(exp[0], term["status"]),
                "relFits": verdict_fits(exp[1], rel["status"]),       # relation_event alone (NEAR-start)
                "transFits": verdict_fits(exp[1], trans),             # combined put-in transition
                "numFrames": rec["numFrames"],
                "minConf": rec["trackingMetrics"].get("minPoseConfidence"),
                "dropout": rec["trackingMetrics"].get("dropoutFraction"),
                "detectedMarkers": rec["calibDiag"].get("detectedMarkers"),
                "normalSpreadDeg": rec["calibDiag"].get("extrinsic", {}).get("maxNormalSpreadDeg"),
                "cubeSpacingM": rec["calibDiag"].get("cubeMarkerSpacingM"),
            })
            tag = "OK " if (row["termFits"] and row["transFits"]) else "XX "
            print(f"{tag}{eid:34s} {cam:11s} exp[{exp[0]}/{exp[1]}] "
                  f"got[term={term['status']} trans={trans} (rel={rel['status']},placed={placed['status']})] "
                  f"markers={row['detectedMarkers']}")
        except Exception as exc:  # never let one bad clip kill the batch
            row.update({"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-800:]})
            print(f"ERR {eid:34s} {cam:11s} -> {type(exc).__name__}: {exc}")
        rows.append(row)

    # Confusion summary by expectedClass x camera.
    print("\n=== confusion summary (matches expected per target) ===")
    by_key: Dict[tuple, Dict[str, int]] = {}
    for r in rows:
        if "error" in r:
            continue
        k = (r["expectedClass"], r["camera"])
        d = by_key.setdefault(k, {"n": 0, "term": 0, "rel": 0, "trans": 0, "both": 0})
        d["n"] += 1
        d["term"] += int(r["termFits"])
        d["rel"] += int(r["relFits"])
        d["trans"] += int(r["transFits"])
        d["both"] += int(r["termFits"] and r["transFits"])
    for (cls, cam), d in sorted(by_key.items()):
        print(f"  {cls:28s} {cam:11s} n={d['n']:2d}  term={d['term']}/{d['n']}  "
              f"rel-only={d['rel']}/{d['n']}  transition={d['trans']}/{d['n']}  both={d['both']}/{d['n']}")
    n_err = sum(1 for r in rows if "error" in r)
    n_both = sum(1 for r in rows if r.get("termFits") and r.get("transFits"))
    print(f"\noverall: {n_both}/{len(rows)} clips match terminal AND transition; errors={n_err}")

    verdicts_out = Path(args.verdicts_out) if args.verdicts_out else out_dir / f"verdicts_{args.select}.json"
    write_json(verdicts_out, {"select": args.select, "cameras": sorted(cameras),
                              "fxScaleSony": args.fx_scale_sony, "fxScaleIphone": args.fx_scale_iphone,
                              "rows": rows})
    print(f"wrote {verdicts_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
