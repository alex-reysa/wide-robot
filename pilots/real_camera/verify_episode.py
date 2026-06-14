#!/usr/bin/env python3
"""Judge a real-camera ``object_inside_container`` episode: PASS / FAIL / UNCERTAIN.

This is the real-camera pilot's entrypoint. It layers a **fail-closed evidence-quality
gate** on top of the shared, frozen ``verify_external_rollout``:

    tracks --[evidence gate]--> rollout --[frozen verifier]--> PASS / FAIL
              |                                                 (+ useful failure class)
              +--> UNCERTAIN  (perception_failure / extractor_uncertainty)

The frozen verifier (``pilots.external_verify``) is RLBench-identical and only emits
PASS/FAIL; UNCERTAIN is decided HERE, *before* the verifier runs, so a low-confidence or
occluded episode can never sneak through as a fake PASS. ``physicalValidity`` stays
``null`` throughout (a camera trace is physics-unverified by contract). The gate never
inspects the target — it judges only tracking quality.

Failure-class taxonomy (the verifier's own ``classify_failure`` lumps near/rim/dropped/
born-inside into one bucket, so we surface the discriminating evidence too):
  * ``perception_failure`` — structurally unusable evidence (a terminal marker missing,
    an object never tracked, too few frames, malformed tracks). Can't build honest evidence.
  * ``extractor_uncertainty`` — evidence present but too uncertain to trust (high marker
    dropout, low confidence, an over-jittery static container).
  * On FAIL, ``cameraFailureClass`` ∈ {NEAR_NOT_INSIDE, LEFT_ON_RIM, DROPPED_OUTSIDE,
    BORN_INSIDE_NO_TRANSITION, RELATION_MISMATCH} derived from the robot's terminal
    relation + the hard-probe mismatches.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from csg.common import Json, load_json
from csg.matcher import MatcherConfig
from csg.rollout_extract import extract_robot_csg

from pilots.external_rollout import ExternalTraceLeakage
from pilots.external_verify import verify_external_rollout
from pilots.real_camera.tracks_to_rollout import (
    MIN_TRACK_FRAMES,
    TracksError,
    tracks_to_rollout,
    validate_tracks_v0,
)

# Provisional evidence-quality thresholds (roadmap §3A: "uncertain tracking is surfaced
# as uncertainty, not hidden"). These are conservative defaults to be RECALIBRATED on the
# first real Sony capture; a synthetic-fixture test pins the behaviour, not the values.
DEFAULT_THRESHOLDS: Dict[str, float] = {
    "min_confidence": 0.6,          # any per-frame marker confidence below this counts as "weak"
    "max_dropout_frac": 0.2,        # max fraction of frames an object may be missing/weak
    "max_consecutive_missing": 5,   # max run of consecutive frames an object may be missing/weak
    "min_endpose_confidence": 0.6,  # BOTH the first AND last frame's cube+tray must be at least this
    "max_static_excursion_m": 0.05, # a static container jittering more than this is untrustworthy
}

_BUNDLED_TARGETS = ("object_inside_container_terminal_only", "object_inside_container_relation_event")
_TARGETS_DIR = Path(__file__).resolve().parent / "targets"


def _as_float(value: Any, default: float = 0.0) -> float:
    """Best-effort float; returns ``default`` for missing/non-numeric input. Keeps the
    evidence-quality gate fail-closed (a non-numeric confidence reads as weak, not a crash)."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default

# Robot terminal relation -> human failure class (for FAIL episodes).
_TERMINAL_RELATION_CLASS = {
    "ON_TOP_OF": "LEFT_ON_RIM",
    "NEAR": "NEAR_NOT_INSIDE",
    "FAR_FROM": "DROPPED_OUTSIDE",
    None: "DROPPED_OUTSIDE",
}


def assess_evidence_quality(
    tracks: Mapping[str, Any],
    thresholds: Optional[Mapping[str, float]] = None,
) -> Json:
    """Fail-closed tracking-quality gate over a ``real_camera.tracks.v0`` episode.

    Returns ``{"ok", "status", "failureClass", "reasons", "metrics"}``. ``ok=True`` means
    the evidence is good enough to mint a rollout and trust the verifier's verdict.
    ``ok=False`` means UNCERTAIN: ``perception_failure`` (unusable) or
    ``extractor_uncertainty`` (too noisy). Never raises — a malformed episode is reported,
    not thrown.
    """
    th = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
    reasons: List[str] = []
    metrics: Json = {}

    # Structural sanity first (perception_failure if unusable).
    if not isinstance(tracks, Mapping) or not isinstance(tracks.get("objects"), list) \
            or not isinstance(tracks.get("frames"), list):
        return {"ok": False, "status": "UNCERTAIN", "failureClass": "perception_failure",
                "reasons": ["tracks are structurally malformed (missing objects/frames)"], "metrics": {}}
    objects = tracks["objects"]
    frames = tracks["frames"]
    roles = [str(o.get("sourceRole", f"obj{i}")) for i, o in enumerate(objects)]
    n = len(frames)
    metrics["numFrames"] = n
    if n < MIN_TRACK_FRAMES or not objects:
        return {"ok": False, "status": "UNCERTAIN", "failureClass": "perception_failure",
                "reasons": [f"too few frames ({n} < {MIN_TRACK_FRAMES}) or no tracked objects"],
                "metrics": metrics}

    min_conf = 1.0
    per_role_dropout: Dict[str, float] = {}
    per_role_max_run: Dict[str, int] = {}
    for role in roles:
        missing = 0
        run = max_run = 0
        for frame in frames:
            poses = frame.get("poses", {}) if isinstance(frame, Mapping) else {}
            pose = poses.get(role) if isinstance(poses, Mapping) else None
            conf = _as_float(pose.get("confidence", 0.0)) if isinstance(pose, Mapping) else 0.0
            weak = pose is None or not isinstance(pose.get("positionM"), Mapping) or conf < th["min_confidence"]
            if pose is not None and isinstance(pose, Mapping):
                min_conf = min(min_conf, conf)
            if weak:
                missing += 1
                run += 1
                max_run = max(max_run, run)
            else:
                run = 0
        per_role_dropout[role] = missing / n
        per_role_max_run[role] = max_run
    metrics["minPoseConfidence"] = min_conf
    metrics["dropoutFraction"] = per_role_dropout
    metrics["maxConsecutiveMissing"] = per_role_max_run

    # BOTH endpose frames must carry every object at high confidence: the relation-event
    # verdict hinges on the INITIAL relation (initial_state -> "started NEAR") as much as the
    # terminal one, and the frozen extractor rewrites pose confidence to 1.0, so a weak
    # endpose must be caught HERE, before the rollout is minted, or it would PASS untrusted.
    for which, idx in (("initial", 0), ("terminal", -1)):
        frame = frames[idx] if isinstance(frames[idx], Mapping) else {}
        poses = frame.get("poses", {}) if isinstance(frame, Mapping) else {}
        for role in roles:
            p = poses.get(role) if isinstance(poses, Mapping) else None
            if not isinstance(p, Mapping) or not isinstance(p.get("positionM"), Mapping):
                return {"ok": False, "status": "UNCERTAIN", "failureClass": "perception_failure",
                        "reasons": [f"{which} frame is missing object {role!r}; cannot judge the "
                                    f"{'first' if which == 'initial' else 'final'} relation"],
                        "metrics": metrics}
            if _as_float(p.get("confidence", 0.0)) < th["min_endpose_confidence"]:
                return {"ok": False, "status": "UNCERTAIN", "failureClass": "extractor_uncertainty",
                        "reasons": [f"{which} {role!r} confidence {p.get('confidence')} "
                                    f"< {th['min_endpose_confidence']}"],
                        "metrics": metrics}

    # An object never reliably tracked is unusable; otherwise high dropout/jitter is noisy.
    for role in roles:
        if per_role_dropout[role] >= 1.0:
            return {"ok": False, "status": "UNCERTAIN", "failureClass": "perception_failure",
                    "reasons": [f"object {role!r} never tracked above confidence {th['min_confidence']}"],
                    "metrics": metrics}
    for role in roles:
        if per_role_dropout[role] > th["max_dropout_frac"]:
            reasons.append(f"object {role!r} dropout {per_role_dropout[role]:.2f} > {th['max_dropout_frac']}")
        if per_role_max_run[role] > th["max_consecutive_missing"]:
            reasons.append(f"object {role!r} missing for {per_role_max_run[role]} consecutive frames "
                           f"> {th['max_consecutive_missing']}")

    # Static-container jitter: recompute the max excursion from the per-role median.
    for i, obj in enumerate(objects):
        if str(obj.get("mobility")) != "STATIC":
            continue
        role = roles[i]
        xs = [f["poses"][role]["positionM"] for f in frames
              if isinstance(f.get("poses", {}).get(role), Mapping)]
        if not xs:
            continue
        med = [_median([_as_float(p.get(k)) for p in xs]) for k in ("x", "y", "z")]
        exc = max((sum((_as_float(p.get(k)) - med[j]) ** 2 for j, k in enumerate(("x", "y", "z"))) ** 0.5 for p in xs),
                  default=0.0)
        metrics.setdefault("staticExcursionM", {})[role] = exc
        if exc > th["max_static_excursion_m"]:
            reasons.append(f"static container {role!r} jitters {exc:.3f} m > {th['max_static_excursion_m']}")

    if reasons:
        return {"ok": False, "status": "UNCERTAIN", "failureClass": "extractor_uncertainty",
                "reasons": reasons, "metrics": metrics}
    return {"ok": True, "status": "OK", "failureClass": None, "reasons": [], "metrics": metrics}


def _median(values):
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def _terminal_relation(rollout: Mapping[str, Any]) -> Optional[str]:
    """The robot's extracted TERMINAL relation for the figure-ground pair (the
    discriminating evidence the verifier's failure category drops)."""
    robot = extract_robot_csg(rollout)
    last = [r for r in robot.get("relations", []) if str(r.get("relationId", "")).endswith("_last")]
    return last[-1].get("relation") if last else None


def _camera_failure_class(rollout: Mapping[str, Any], hard_mismatches: List[str]) -> str:
    rel = _terminal_relation(rollout)
    if rel == "INSIDE":
        # ended inside but still FAILed -> it never started near (born-inside) or another
        # endpoint mismatch; initial_state is the canonical born-inside signal.
        return "BORN_INSIDE_NO_TRANSITION" if "initial_state" in hard_mismatches else "RELATION_MISMATCH"
    return _TERMINAL_RELATION_CLASS.get(rel, "RELATION_MISMATCH")


def verify_episode(
    target: Mapping[str, Any],
    *,
    tracks: Optional[Mapping[str, Any]] = None,
    rollout: Optional[Mapping[str, Any]] = None,
    thresholds: Optional[Mapping[str, float]] = None,
    matcher_cfg: Optional[MatcherConfig] = None,
    case_name: str = "real_camera_episode",
) -> Json:
    """Judge one episode against ``target`` → PASS / FAIL / UNCERTAIN.

    Pass ``tracks`` (a ``real_camera.tracks.v0`` episode) for the full pipeline including
    the fail-closed UNCERTAIN gate, or ``rollout`` (an already-minted ``csg.rollout.v0``)
    to verify directly (no UNCERTAIN gate — the evidence is already committed).
    """
    if (tracks is None) == (rollout is None):
        raise ValueError("verify_episode requires exactly one of tracks= or rollout=")

    if tracks is not None:
        quality = assess_evidence_quality(tracks, thresholds)
        if not quality["ok"]:
            return {
                "case": case_name, "status": "UNCERTAIN", "passed": False,
                "failureClass": quality["failureClass"], "uncertaintyReasons": quality["reasons"],
                "trackingMetrics": quality["metrics"], "physicalValidity": None,
                "traceSource": "real_camera_external",
            }
        try:
            rollout = tracks_to_rollout(tracks)
        except (TracksError, ExternalTraceLeakage) as exc:  # backstop -> UNCERTAIN, never a crash/PASS
            return {
                "case": case_name, "status": "UNCERTAIN", "passed": False,
                "failureClass": "leakage_violation" if isinstance(exc, ExternalTraceLeakage) else "perception_failure",
                "uncertaintyReasons": [str(exc)],
                "trackingMetrics": quality["metrics"], "physicalValidity": None,
                "traceSource": "real_camera_external",
            }
        quality_metrics = quality["metrics"]
    else:
        quality_metrics = None

    try:
        case = verify_external_rollout(target, rollout, matcher_cfg=matcher_cfg, case_name=case_name)
    except ExternalTraceLeakage as exc:  # a leaky pre-built rollout fails closed, never a PASS
        return {
            "case": case_name, "status": "UNCERTAIN", "passed": False,
            "failureClass": "leakage_violation", "uncertaintyReasons": [str(exc)],
            "trackingMetrics": quality_metrics, "physicalValidity": None,
            "traceSource": "real_camera_external",
        }
    if quality_metrics is not None:
        case["trackingMetrics"] = quality_metrics
    if not case["passed"]:
        case["cameraFailureClass"] = _camera_failure_class(rollout, case["hardMismatches"])
    return case


def verify_episode_both(
    *,
    tracks: Optional[Mapping[str, Any]] = None,
    rollout: Optional[Mapping[str, Any]] = None,
    thresholds: Optional[Mapping[str, float]] = None,
    targets_dir: Path = _TARGETS_DIR,
) -> Json:
    """Run an episode against BOTH bundled camera targets (terminal-only + relation-event)."""
    out: Json = {}
    for name in _BUNDLED_TARGETS:
        target = load_json(targets_dir / f"{name}.json")
        out[name] = verify_episode(target, tracks=tracks, rollout=rollout,
                                   thresholds=thresholds, case_name=name)
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Judge a real-camera object_inside_container episode (PASS/FAIL/UNCERTAIN).")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--tracks", help="input real_camera.tracks.v0 JSON (full pipeline + UNCERTAIN gate)")
    src.add_argument("--rollout", help="already-minted csg.rollout.v0 JSON (verify only, no UNCERTAIN gate)")
    parser.add_argument("--target", help="a single camera target JSON; default: run BOTH bundled targets")
    parser.add_argument("--json", action="store_true", help="print the full record")
    args = parser.parse_args(argv)

    tracks = load_json(Path(args.tracks)) if args.tracks else None
    rollout = load_json(Path(args.rollout)) if args.rollout else None

    if args.target:
        target = load_json(Path(args.target))
        record = verify_episode(target, tracks=tracks, rollout=rollout,
                                case_name=Path(args.target).stem)
        results = {Path(args.target).stem: record}
    else:
        results = verify_episode_both(tracks=tracks, rollout=rollout)

    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        for name, rec in results.items():
            extra = rec.get("cameraFailureClass") or (rec.get("failureClass") or "")
            print(f"{name}: {rec['status']}"
                  + (f" [{extra}]" if extra else "")
                  + (f" mismatches={rec.get('hardMismatches')}" if rec.get("hardMismatches") else ""))

    # Exit 0 only if no episode is FAIL/UNCERTAIN under any target (CI-friendly).
    ok = all(r["status"] == "PASS" for r in results.values())
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
