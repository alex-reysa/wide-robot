#!/usr/bin/env python3
"""Judge an RH20T external-source episode against the RH20T pilot targets.

Entry point for the RH20T smoke test. It runs a track/rollout through the shared,
FROZEN ``verify_external_rollout`` and reports PASS / FAIL / UNCERTAIN:

    tracks --[tracks_to_rollout]--> rollout --[frozen verifier]--> PASS / FAIL
              |                                                      (+ useful failure class)
              +--> UNCERTAIN (source_evidence_invalid / leakage_violation)

The frozen verifier (``pilots.external_verify``) is RLBench/real-camera identical and only
emits PASS/FAIL; UNCERTAIN is decided HERE, fail-closed, when the source evidence cannot be
neutralised honestly (a structurally-broken ``rh20t.tracks.v0`` raises ``RH20TTracksError``)
or a pre-built rollout is leaky (``ExternalTraceLeakage``). ``physicalValidity`` stays
``null`` throughout — an RH20T kinematic trace is physics-unverified by contract.

Unlike the real-camera pilot, there is no automated tracking-quality gate: the RH20T
annotation is human-reviewed source evidence, so evidence quality is asserted by the
reviewer (recorded in ``annotation.review``), not re-derived from marker dropout here.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Optional

from csg.common import Json, load_json
from pilots.external_rollout import ExternalTraceLeakage
from pilots.external_verify import verify_external_rollout
from pilots.rh20t.tracks_to_rollout import RH20TTracksError, tracks_to_rollout

TARGETS_DIR = Path(__file__).resolve().parent / "targets"
BUNDLED_TARGETS = ("object_inside_container_terminal_only", "object_inside_container_relation_event")


def _uncertain(case_name: str, failure_class: str, reason: str) -> Json:
    return {
        "case": case_name,
        "status": "UNCERTAIN",
        "passed": False,
        "failureClass": failure_class,
        "uncertaintyReasons": [reason],
        "physicalValidity": None,
        "traceSource": "rh20t_external",
    }


def verify_episode(
    target: Mapping[str, object],
    *,
    tracks: Optional[Mapping[str, object]] = None,
    rollout: Optional[Mapping[str, object]] = None,
    case_name: str = "rh20t_episode",
) -> Json:
    """Judge one episode against ``target`` -> PASS / FAIL / UNCERTAIN.

    Pass exactly one of ``tracks`` (an ``rh20t.tracks.v0`` episode — runs the full
    neutralise-then-verify pipeline) or ``rollout`` (an already-minted ``csg.rollout.v0``
    to verify directly).
    """
    if (tracks is None) == (rollout is None):
        raise ValueError("verify_episode requires exactly one of tracks= or rollout=")
    if tracks is not None:
        try:
            rollout = tracks_to_rollout(tracks)
        except RH20TTracksError as exc:  # unusable source evidence -> UNCERTAIN, never PASS
            return _uncertain(case_name, "source_evidence_invalid", str(exc))
        except ExternalTraceLeakage as exc:  # a neutralisation bug leaking identity -> fail closed
            return _uncertain(case_name, "leakage_violation", str(exc))
    try:
        return verify_external_rollout(target, rollout, case_name=case_name)
    except ExternalTraceLeakage as exc:  # a leaky pre-built rollout fails closed, never a PASS
        return _uncertain(case_name, "leakage_violation", str(exc))


def verify_episode_both(
    *,
    tracks: Optional[Mapping[str, object]] = None,
    rollout: Optional[Mapping[str, object]] = None,
    targets_dir: Path = TARGETS_DIR,
) -> Json:
    """Run an episode against BOTH bundled RH20T targets (terminal-only + relation-event)."""
    out: Json = {}
    for name in BUNDLED_TARGETS:
        out[name] = verify_episode(load_json(targets_dir / f"{name}.json"),
                                   tracks=tracks, rollout=rollout, case_name=name)
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Verify an RH20T external-source episode (PASS/FAIL/UNCERTAIN).")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--tracks", help="input rh20t.tracks.v0 JSON (full pipeline)")
    src.add_argument("--rollout", help="already-minted csg.rollout.v0 JSON (verify only)")
    parser.add_argument("--target", help="a single RH20T target JSON; default: run BOTH bundled targets")
    parser.add_argument("--json", action="store_true", help="print the full record")
    args = parser.parse_args(argv)

    tracks = load_json(Path(args.tracks)) if args.tracks else None
    rollout = load_json(Path(args.rollout)) if args.rollout else None
    if args.target:
        name = Path(args.target).stem
        results = {name: verify_episode(load_json(Path(args.target)), tracks=tracks,
                                        rollout=rollout, case_name=name)}
    else:
        results = verify_episode_both(tracks=tracks, rollout=rollout)

    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        for name, rec in results.items():
            extra = rec.get("failureClass") or ""
            print(f"{name}: {rec['status']}"
                  + (f" [{extra}]" if extra else "")
                  + (f" mismatches={rec.get('hardMismatches')}" if rec.get("hardMismatches") else ""))

    return 0 if all(rec["status"] == "PASS" for rec in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
