#!/usr/bin/env python3
"""Deterministic synthetic fixtures for the baseline-counterexamples experiment.

The 78 real Sony/iPhone clips *demonstrate the issue visually*, but their geometry
comes from physical marker calibration, so a skeptic can always ask "is that an
artifact of your tray corners?". These hand-authored ``real_camera.tracks.v0``
episodes remove that question entirely: every pose is a round number placed by
hand, the tray is at the canonical ``(0.30, 0, 0.015)`` with size
``[0.24, 0.18, 0.03]`` (so its footprint is ``x∈[0.18,0.42], y∈[-0.09,0.09]`` and
its rim top is ``z=0.03``), and the cube is a ``0.04 m`` box. There is **nothing to
calibrate** — the fixtures pin the *semantics* precisely, as a complement to the
real clips' visual proof.

Seven scenarios producing six distinct verifier outcomes (``fx_born_inside`` and
``fx_inside_to_inside`` are an intentional *invariance* pair — see below):

  fx_outside_to_inside_success   the positive control: a real near→inside placement
  fx_rim_partial                 center inside the footprint but balanced on the rim
  fx_born_inside                 starts AND ends inside (tiny jitter), never placed
  fx_inside_to_inside            moves clear across the tray but never leaves it
  fx_occluded_uncertain          genuine success, cube occluded past the consecutive-missing gate
  fx_wrong_object                the cube stays out; a decoy sits inside the tray
  fx_leaky_metadata              a rollout that smuggles source identity (leakage)

``fx_born_inside`` and ``fx_inside_to_inside`` differ ONLY in cube-motion magnitude
(a 2 cm jitter vs a 16 cm traverse) and the verifier treats them **identically** —
both ``BORN_INSIDE_NO_TRANSITION`` — because neither has an outside→inside
transition. They produce byte-identical result records; we keep both deliberately,
as a demonstration that the verdict is *invariant* to how far the object moves
inside the tray (it is the transition, not the motion, that matters), not as two
independent lessons.

For each tracks fixture we run the full B1..B6 ladder, the three wide-robot targets,
and an identity-blind "footprint occupancy" strawman; ``fx_leaky_metadata`` is a
pre-built ``csg.rollout.v0`` (the baselines have no notion of leakage, so it is a
verifier-only scenario). ``FIXTURE_EXPECTATIONS`` records the asserted semantics so
the test pins *intent*, not merely whatever the code happens to emit. ``csg/`` is
only READ. No OpenCV, no raw video.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from csg.common import load_json
from experiments.baseline_counterexamples.baseline_predicates import (
    EVIDENCE_THRESHOLDS,
    LADDER,
    b1_terminal_center_in_footprint,
    clip_geometry,
    evaluate_clip,
)
from pilots.real_camera.tracks_to_rollout import tracks_to_rollout
from pilots.real_camera.verify_episode import verify_episode, verify_episode_both

REPO = Path(__file__).resolve().parents[2]
FIX_DIR = Path(__file__).resolve().parent / "fixtures"
_TARGETS_DIR = REPO / "pilots" / "real_camera" / "targets"
TERMINAL_ONLY = "object_inside_container_terminal_only"
RELATION_EVENT = "object_inside_container_relation_event"
PLACED_FROM_OUTSIDE = "object_inside_container_placed_from_outside"

# Canonical, calibration-free scene geometry (matches the dataset's ep_* fixtures).
TRAY_CENTER = (0.30, 0.0, 0.015)
TRAY_SIZE = [0.24, 0.18, 0.03]          # footprint x∈[0.18,0.42] y∈[-0.09,0.09], rim top z=0.03
CUBE_SIZE = [0.04, 0.04, 0.04]          # half-extent 0.02
FPS = 30.0


# --------------------------------------------------------------------------- #
# tiny authoring helpers
# --------------------------------------------------------------------------- #


def _pose(x: float, y: float, z: float, conf: float = 0.99) -> Dict[str, Any]:
    return {"positionM": {"x": float(x), "y": float(y), "z": float(z)}, "confidence": float(conf)}


def _lerp(a: float, b: float, f: float) -> float:
    return a + (b - a) * f


def _cube_object(role: str = "cube", marker: int = 7, container: bool = False,
                 mobility: str = "MOVABLE", size=CUBE_SIZE) -> Dict[str, Any]:
    return {"sourceRole": role, "physicalKind": "RIGID_OBJECT", "mobility": mobility,
            "isContainer": container, "sizeM": list(size), "markerIds": [marker]}


def _tray_object() -> Dict[str, Any]:
    return {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
            "isContainer": True, "sizeM": list(TRAY_SIZE), "markerIds": [10]}


def _episode(episode_id: str, objects: List[Dict[str, Any]], frames: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "schemaVersion": "real_camera.tracks.v0",
        "episodeId": episode_id,
        "videoSha256": None,            # synthetic: no provenance, by contract
        "calibrationHash": None,
        "fps": FPS,
        "frameSize": [1280, 720],
        "objects": objects,
        "frames": frames,
        "fixtureNote": "hand-authored deterministic fixture; round-number geometry, nothing calibrated",
    }


def _moving_cube_episode(episode_id: str, cube_path, *, n: int = 8,
                         extra_objects: Optional[List[Dict[str, Any]]] = None,
                         extra_static: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """An episode with a static tray and a cube moving along ``cube_path(f)`` for
    ``f`` in [0,1]. ``extra_static`` (role -> (x,y,z)) adds a static decoy object."""
    objects = [_cube_object(), _tray_object()]
    if extra_objects:
        objects += extra_objects
    frames = []
    for i in range(n):
        f = i / (n - 1)
        cx, cy, cz = cube_path(f)
        poses = {"cube": _pose(cx, cy, cz), "tray": _pose(*TRAY_CENTER)}
        if extra_static:
            for role, (x, y, z) in extra_static.items():
                poses[role] = _pose(x, y, z)
        frames.append({"frameIndex": i, "timeS": round(i / FPS, 6), "poses": poses})
    return _episode(episode_id, objects, frames)


# --------------------------------------------------------------------------- #
# the seven fixtures
# --------------------------------------------------------------------------- #


def fx_outside_to_inside_success() -> Dict[str, Any]:
    """Positive control: cube starts just OUTSIDE the footprint but NEAR (gap 0.08 m
    < the 0.10 m near threshold) and ends resting INSIDE. Everything PASSes."""
    return _moving_cube_episode(
        "fx_outside_to_inside_success",
        lambda f: (_lerp(0.50, 0.30, f), 0.0, _lerp(0.04, 0.02, f)))


def fx_rim_partial() -> Dict[str, Any]:
    """Cube ends with its CENTER inside the outer footprint (x=0.41 < 0.42 → B1 PASS)
    but its footprint pokes past the inner region (→ B3 reject) and it sits on the
    rim (cube bottom 0.03 == tray top → ON_TOP_OF → B5 + verifier reject)."""
    return _moving_cube_episode(
        "fx_rim_partial",
        lambda f: (_lerp(0.50, 0.41, f), 0.0, _lerp(0.04, 0.05, f)))


def fx_born_inside() -> Dict[str, Any]:
    """Cube starts inside and ends inside with only a tiny jitter — never outside,
    never placed. Terminal predicates PASS; only the initial-state check rejects."""
    return _moving_cube_episode(
        "fx_born_inside",
        lambda f: (_lerp(0.29, 0.31, f), 0.0, 0.02))


def fx_inside_to_inside() -> Dict[str, Any]:
    """Cube moves clear across the tray (0.16 m) but never leaves the footprint —
    motion is present, an outside→inside transition is not."""
    return _moving_cube_episode(
        "fx_inside_to_inside",
        lambda f: (_lerp(0.22, 0.38, f), 0.0, 0.02))


def fx_occluded_uncertain() -> Dict[str, Any]:
    """A genuine near→inside success, but the cube marker is occluded for 32
    consecutive mid-trajectory frames (> the 30-frame gate). The episode is sized so
    that ONLY the consecutive-missing rule trips: 32 missing of 92 frames is a 0.348
    dropout fraction, just under the 0.35 dropout threshold — so this isolates the
    consecutive-missing gate specifically (not the dropout-fraction gate). First/last
    VISIBLE frames look perfect, so every single/two-frame baseline (B1..B5) certifies;
    the evidence gate (and B6, and the verifier) refuse. Tray is visible throughout."""
    n = 92
    miss_lo, miss_hi = 30, 62         # frames [30, 62) omit the cube: 32 consecutive; 32/92 = 0.348 < 0.35
    frames = []
    for i in range(n):
        f = i / (n - 1)
        poses: Dict[str, Any] = {"tray": _pose(*TRAY_CENTER)}
        if not (miss_lo <= i < miss_hi):
            poses["cube"] = _pose(_lerp(0.50, 0.30, f), 0.0, _lerp(0.04, 0.02, f))
        frames.append({"frameIndex": i, "timeS": round(i / FPS, 6), "poses": poses})
    return _episode("fx_occluded_uncertain", [_cube_object(), _tray_object()], frames)


def fx_wrong_object() -> Dict[str, Any]:
    """The designated cube approaches but stays OUTSIDE (x ends 0.50 > 0.42); a
    STATIC decoy sits inside the tray the whole time. An identity-blind "is anything
    in the footprint?" check false-PASSes; the cube-bound baselines and the verifier
    (which judges the moving cube vs the tray) correctly reject."""
    return _moving_cube_episode(
        "fx_wrong_object",
        lambda f: (_lerp(0.60, 0.50, f), 0.0, 0.04),
        extra_objects=[_cube_object(role="decoy", marker=8, mobility="STATIC")],
        extra_static={"decoy": (0.30, 0.0, 0.02)})


def _clean_success_rollout() -> Dict[str, Any]:
    """The minted, leakage-clean rollout for a near→inside success — the base the
    leaky fixture corrupts (so the ONLY difference is the smuggled identity)."""
    return tracks_to_rollout(fx_outside_to_inside_success())


def fx_leaky_metadata_rollout() -> Dict[str, Any]:
    """A pre-built ``csg.rollout.v0`` that would PASS — except it smuggles a source
    role name ('tray') as an ``objectIdMap`` key. The leakage gate refuses it before
    the matcher runs. The baselines have no notion of leakage at all."""
    leaky = copy.deepcopy(_clean_success_rollout())
    leaky["objectIdMap"] = {"tray": "body_001"}   # non-neutral key = source identity leaking in
    return leaky


# Ordered (so the table is stable). Last entry is the rollout-only leakage fixture.
TRACKS_FIXTURES = (
    ("fx_outside_to_inside_success", fx_outside_to_inside_success, "success"),
    ("fx_rim_partial", fx_rim_partial, "fail"),
    ("fx_born_inside", fx_born_inside, "fail"),
    ("fx_inside_to_inside", fx_inside_to_inside, "fail"),
    ("fx_occluded_uncertain", fx_occluded_uncertain, "success"),
    ("fx_wrong_object", fx_wrong_object, "fail"),
)


# The asserted SEMANTICS (intent), pinned by the test so a regression is loud.
# Each predicate value is True (certifies a success) / False (rejects); wr values are
# the expected verdict STATUS for that target.
FIXTURE_EXPECTATIONS: Dict[str, Dict[str, Any]] = {
    "fx_outside_to_inside_success": {
        "human": "success",
        "ladder": {"B1": True, "B2": True, "B3": True, "B4": True, "B5": True, "B6": True},
        "occupancyStrawman": True,
        "wr": {TERMINAL_ONLY: "PASS", RELATION_EVENT: "PASS"},
        "structuredCertifies": True,
        "lesson": "the verifier is not a fail-everything oracle: a real near→inside placement PASSes all.",
    },
    "fx_rim_partial": {
        "human": "fail",
        "ladder": {"B1": True, "B2": True, "B3": False, "B4": False, "B5": False, "B6": False},
        "occupancyStrawman": True,
        "wr": {TERMINAL_ONLY: "FAIL", RELATION_EVENT: "FAIL"},
        "wrClass": "LEFT_ON_RIM",
        "structuredCertifies": False,
        "lesson": "center-in-footprint (B1/B2) PASSes the rim; 3D containment (B3/B5/verifier) rejects it.",
    },
    "fx_born_inside": {
        "human": "fail",
        "ladder": {"B1": True, "B2": True, "B3": True, "B4": False, "B5": True, "B6": False},
        "occupancyStrawman": True,
        "wr": {TERMINAL_ONLY: "PASS", RELATION_EVENT: "FAIL"},
        "wrClass": "BORN_INSIDE_NO_TRANSITION",
        "structuredCertifies": False,
        "lesson": "ends inside, so terminal predicates + terminal_only PASS; only the initial-state check rejects.",
    },
    "fx_inside_to_inside": {
        "human": "fail",
        "ladder": {"B1": True, "B2": True, "B3": True, "B4": False, "B5": True, "B6": False},
        "occupancyStrawman": True,
        "wr": {TERMINAL_ONLY: "PASS", RELATION_EVENT: "FAIL"},
        "wrClass": "BORN_INSIDE_NO_TRANSITION",
        "structuredCertifies": False,
        "lesson": "motion is present but no outside→inside transition — still not a placement.",
    },
    "fx_occluded_uncertain": {
        "human": "success",
        "ladder": {"B1": True, "B2": True, "B3": True, "B4": True, "B5": True, "B6": False},
        "occupancyStrawman": True,
        "wr": {TERMINAL_ONLY: "UNCERTAIN", RELATION_EVENT: "UNCERTAIN"},
        "structuredCertifies": False,
        "lesson": "clean endpoints, 32-frame mid occlusion: B1–B5 certify; B6 + the verifier refuse (fail-closed).",
    },
    "fx_wrong_object": {
        "human": "fail",
        "ladder": {"B1": False, "B2": False, "B3": False, "B4": False, "B5": False, "B6": False},
        "occupancyStrawman": True,   # the identity-blind strawman is fooled by the decoy
        "wr": {TERMINAL_ONLY: "FAIL", RELATION_EVENT: "FAIL"},
        "structuredCertifies": False,
        "lesson": "a decoy in the tray fools an occupancy check; identity-bound predicates + the verifier reject.",
    },
}

_LADDER_KEY = {
    "B1": "B1_center_in_footprint",
    "B2": "B2_footprint_overlap",
    "B3": "B3_full_inner_containment",
    "B4": "B4_full_containment_started_outside",
    "B5": "B5_terminal_3d_containment",
    "B6": "B6_contained_started_outside_evidence_gated",
}


def occupancy_strawman(tracks: Mapping[str, Any]) -> bool:
    """Identity-BLIND baseline: does ANY non-tray tracked object end with its center
    inside the tray's outer footprint? This is the predicate ``fx_wrong_object``
    defeats — it certifies because the decoy is inside, even though the cube is not.
    Included to make the object-identity lesson executable (the real baselines bind
    the 'cube' role and so are NOT fooled)."""
    geom_objs = [str(o.get("sourceRole")) for o in tracks.get("objects", [])
                 if str(o.get("sourceRole")) != "tray"]
    frames = tracks.get("frames") or []
    for role in geom_objs:
        last = None
        for fr in frames:
            p = (fr.get("poses") or {}).get(role)
            if isinstance(p, Mapping) and isinstance(p.get("positionM"), Mapping):
                last = p["positionM"]
        if last is not None and b1_terminal_center_in_footprint(
                (last["x"], last["y"], last["z"]), TRAY_CENTER, TRAY_SIZE):
            return True
    return False


def _run_all_targets(tracks: Mapping[str, Any]) -> Dict[str, Any]:
    both = verify_episode_both(tracks=tracks, thresholds=EVIDENCE_THRESHOLDS)
    placed = load_json(_TARGETS_DIR / f"{PLACED_FROM_OUTSIDE}.json")
    both[PLACED_FROM_OUTSIDE] = verify_episode(
        placed, tracks=tracks, thresholds=EVIDENCE_THRESHOLDS, case_name=PLACED_FROM_OUTSIDE)
    return both


def evaluate_fixture_suite() -> Dict[str, Any]:
    """Run the ladder + three wide-robot targets + the occupancy strawman on every
    tracks fixture, plus the clean-vs-leaky rollout comparison. Returns a record dict
    keyed by fixture id; pure recompute, no committed file is read for the verdicts."""
    records: Dict[str, Any] = {}
    for fid, builder, human in TRACKS_FIXTURES:
        tracks = builder()
        baseline = evaluate_clip(tracks)
        targets = _run_all_targets(tracks)
        structured = (targets[RELATION_EVENT].get("status") == "PASS"
                      or targets[PLACED_FROM_OUTSIDE].get("status") == "PASS")
        records[fid] = {
            "human": human,
            "ladder": {short: bool(baseline[key]) for short, key in _LADDER_KEY.items()},
            "evidenceOk": bool(baseline["evidenceOk"]),
            "evidenceFailureClass": baseline["evidenceFailureClass"],
            "occupancyStrawman": occupancy_strawman(tracks),
            "wr": {name: targets[name].get("status") for name in
                   (TERMINAL_ONLY, RELATION_EVENT, PLACED_FROM_OUTSIDE)},
            "wrClass": {name: (targets[name].get("cameraFailureClass") or targets[name].get("failureClass"))
                        for name in (TERMINAL_ONLY, RELATION_EVENT, PLACED_FROM_OUTSIDE)},
            "structuredCertifies": bool(structured),
        }
    # leakage fixture: clean rollout (PASS) vs leaky rollout (refused).
    target = load_json(_TARGETS_DIR / f"{RELATION_EVENT}.json")
    clean_rec = verify_episode(target, rollout=_clean_success_rollout(), case_name="fx_leaky_clean")
    leaky_rec = verify_episode(target, rollout=fx_leaky_metadata_rollout(), case_name="fx_leaky_metadata")
    records["fx_leaky_metadata"] = {
        "human": "success (refused on leakage)",
        "kind": "rollout-only (baselines have no leakage notion)",
        "cleanStatus": clean_rec.get("status"),
        "leakyStatus": leaky_rec.get("status"),
        "leakyFailureClass": leaky_rec.get("failureClass"),
        "leakyReasons": leaky_rec.get("uncertaintyReasons"),
        "lesson": "the SAME evidence PASSes clean but is REFUSED when a source role name leaks into "
                  "objectIdMap — a leakage gate the baselines do not have.",
    }
    return records


def write_fixtures(out_dir: Path = FIX_DIR) -> List[str]:
    """Write the committed fixture artifacts: each tracks fixture, the leaky + clean
    rollouts, and ``fixture_results.json`` (recompute + asserted expectations)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written: List[str] = []
    for fid, builder, _human in TRACKS_FIXTURES:
        path = out_dir / f"{fid}.tracks.json"
        path.write_text(json.dumps(builder(), indent=2) + "\n")
        written.append(path.name)
    (out_dir / "fx_leaky_metadata.rollout.json").write_text(
        json.dumps(fx_leaky_metadata_rollout(), indent=2, sort_keys=True) + "\n")
    written.append("fx_leaky_metadata.rollout.json")

    results = evaluate_fixture_suite()
    payload = {
        "kind": "deterministic synthetic fixtures — hand-authored round-number geometry, nothing calibrated",
        "scene": {"trayCenter": list(TRAY_CENTER), "traySizeM": TRAY_SIZE, "cubeSizeM": CUBE_SIZE,
                  "footprintX": [0.18, 0.42], "footprintY": [-0.09, 0.09], "rimTopZ": 0.03,
                  "evidenceThresholds": EVIDENCE_THRESHOLDS},
        "distinctOutcomes": 6,
        "invarianceNote": "7 fixtures, 6 distinct verifier outcomes: fx_born_inside (2cm jitter) and "
                          "fx_inside_to_inside (16cm traverse) produce byte-identical records "
                          "(both BORN_INSIDE_NO_TRANSITION) — kept on purpose to show the verdict is invariant "
                          "to motion magnitude inside the tray (it is the transition, not the motion, that matters).",
        "results": results,
        "expectations": FIXTURE_EXPECTATIONS,
    }
    (out_dir / "fixture_results.json").write_text(json.dumps(payload, indent=2) + "\n")
    written.append("fixture_results.json")
    return written


if __name__ == "__main__":
    names = write_fixtures()
    print(json.dumps({"wrote": names, "dir": str(FIX_DIR)}, indent=2))
