#!/usr/bin/env python3
"""The four source legs of the cross-source ``object_inside_container`` report.

Each leg recomputes its verdicts LIVE from committed inputs through the frozen verifier
core and returns a uniform per-clip record so one master table can compare them:

  * MuJoCo  — internal sim: ``extract_robot_csg -> match -> leakage_report`` (the same path
    ``csg.benchmark.run_one`` takes), plus ``physicalValidity`` from the committed validity
    report. NOT ``verify_external_rollout`` — its external-trace door rejects the populated
    ``objectIdMap`` a legitimate internal rollout carries.
  * RLBench — external sim: ``verify_external_rollout`` over 9 committed live demos.
  * Sony    — real camera: ``pilots.real_camera.verify_episode`` (the fail-closed UNCERTAIN
    evidence gate + the frozen verifier) over the committed tracks.
  * RH20T   — real robot video: ``verify_external_rollout`` over the committed positive +
    derived-negative rollouts.

Every clip is judged at the same two comparison tiers the others use: ``terminal_only``
(did it END inside?) and the STRUCTURED tier (a real outside→inside put-in =
``relation_event`` near-start OR ``placed_from_outside`` far-start). ``csg`` is only READ.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg
from csg.benchmark import leakage_report
from pilots.external_verify import verify_external_rollout
from pilots.real_camera.verify_episode import verify_episode, verify_episode_both
# Canonical relaxed 30fps evidence-quality thresholds (a MODULE constant, the same object the
# Sony ingest + baseline experiment use — not a re-derived copy).
from experiments.baseline_counterexamples.baseline_predicates import EVIDENCE_THRESHOLDS

REPO = Path(__file__).resolve().parents[2]
TIERS = ("terminal_only", "relation_event", "placed_from_outside")


def _tier(name: str) -> str:
    return f"object_inside_container_{name}"


# The shared object_inside_container card set, used for the MuJoCo + Sony tier verdicts.
# All pilot cards are structurally identical per ``target_equivalence`` (different labels only);
# real_camera's instantiation uses h_cube/h_tray, matching MuJoCo's objectIdMap, so it is the
# natural choice for the internal leg.
RC_TARGETS = REPO / "pilots" / "real_camera" / "targets"
RLBENCH_TARGETS = REPO / "pilots" / "rlbench" / "targets"
RH20T_TARGETS = REPO / "pilots" / "rh20t" / "targets"

MUJOCO_FIXTURE = REPO / "experiments" / "cross_source_oic" / "mujoco_fixture"
GOLD_PCIT = REPO / "gold_tests" / "put_cube_in_tray"
SONY = REPO / "datasets" / "sony_object_inside_container_v0"
RH20T = REPO / "datasets" / "rh20t_object_inside_container_v0"
RLBENCH_FIXTURES = REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260616_put_item"


# --------------------------------------------------------------------------------------
# uniform per-clip record + aggregate
# --------------------------------------------------------------------------------------

def _clip_record(clip_id: str, kind: str, ground_truth: str, tier_v: Mapping[str, Mapping[str, Any]],
                 *, expected_class: Optional[str] = None) -> Dict[str, Any]:
    """One row in the master table. ``tier_v`` maps tier -> a verdict dict that always has
    ``status`` (PASS/FAIL/UNCERTAIN) and may have leakageClean / physicalValidity / hardMismatches /
    cameraFailureClass / failureClass (UNCERTAIN records omit the matcher fields)."""
    term = tier_v["terminal_only"]
    rele = tier_v.get("relation_event")
    plac = tier_v.get("placed_from_outside")
    structured = bool((rele and rele.get("status") == "PASS") or (plac and plac.get("status") == "PASS"))
    statuses = [v.get("status") for v in tier_v.values()]
    return {
        "clipId": clip_id,
        "kind": kind,
        "groundTruth": ground_truth,                       # what a correct verifier should say
        "expectedClass": expected_class,
        "terminal": term.get("status"),
        "relationEvent": rele.get("status") if rele else None,
        "placed": plac.get("status") if plac else None,
        "structuredCertifies": structured,
        "anyUncertain": "UNCERTAIN" in statuses,
        "leakageClean": term.get("leakageClean"),          # None ⇒ n/a (UNCERTAIN, no rollout minted)
        "physicalValidity": term.get("physicalValidity"),
        "terminalClass": term.get("cameraFailureClass") or term.get("failureClass"),
        "structuredClass": (rele.get("cameraFailureClass") or rele.get("failureClass")) if rele else None,
        "terminalHardMismatches": term.get("hardMismatches"),
    }


def _aggregate(clips: List[Dict[str, Any]]) -> Dict[str, Any]:
    succ = [c for c in clips if c["groundTruth"] == "PASS"]
    fail = [c for c in clips if c["groundTruth"] == "FAIL"]
    return {
        "nClips": len(clips),
        "nSuccess": len(succ),
        "nFailure": len(fail),
        "successTerminalPass": sum(1 for c in succ if c["terminal"] == "PASS"),
        "successStructuredCertify": sum(1 for c in succ if c["structuredCertifies"]),
        "successUncertain": sum(1 for c in succ if c["anyUncertain"] and not c["structuredCertifies"]),
        # the headline safety number: a NON-success that STRUCTURED-certifies is a false PASS.
        "failureStructuredFalsePass": sum(1 for c in fail if c["structuredCertifies"]),
        # born-inside passes the WEAK terminal target (that is exactly why the structured tier exists).
        "failureTerminalPass": sum(1 for c in fail if c["terminal"] == "PASS"),
        "failureCorrectlyRejected": sum(1 for c in fail if not c["structuredCertifies"]),
        "noLeakageViolation": all(c["leakageClean"] is not False for c in clips),
        "leakageCleanCount": sum(1 for c in clips if c["leakageClean"] is True),
    }


# --------------------------------------------------------------------------------------
# MuJoCo — internal sim
# --------------------------------------------------------------------------------------

def _mujoco_verdict(target: Mapping[str, Any], robot: Mapping[str, Any], validity: Any) -> Dict[str, Any]:
    """The internal-sim equivalent of one ``verify_external_rollout`` verdict, via the frozen
    core directly (no external-trace door). PASS iff the matcher passes, the robot CSG is
    leakage-clean, and physical validity is not False."""
    res = match(target, robot, MatcherConfig())
    leak = leakage_report(robot)
    passed = res.passed and leak["clean"] and validity is not False
    return {
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "matcherPassed": res.passed,
        "leakageClean": leak["clean"],
        "physicalValidity": validity,
        "vacuous": res.vacuous,
        "hardMismatches": [p for p in res.hard_probes if not res.probe_agreement[p]],
    }


def mujoco_leg() -> Dict[str, Any]:
    rollout = load_json(MUJOCO_FIXTURE / "put_cube_in_tray.rollout.json")
    validity = load_json(MUJOCO_FIXTURE / "put_cube_in_tray.validity_report.json")["physicalValidity"]
    robot = extract_robot_csg(rollout)

    tier_v = {t: _mujoco_verdict(load_json(RC_TARGETS / f"{_tier(t)}.json"), robot, validity) for t in TIERS}
    clip = _clip_record("put_cube_in_tray__mujoco_solver", "success", "PASS", tier_v, expected_class="success")

    # Native full pick-place gold target (a SUPERSET tier: + contact / carry / temporal-order).
    gold_target = load_json(GOLD_PCIT / "target.json")
    native_gold = _mujoco_verdict(gold_target, robot, validity)

    # Internal acceptance corpus: the committed put_cube_in_tray robot-CSG fixtures (1 success +
    # 4 sabotaged variants) matched against the gold target, with expected.json as ground truth.
    # This is the internal world's genuine FAIL evidence through the frozen matcher.
    expected = load_json(GOLD_PCIT / "expected.json")
    corpus: List[Dict[str, Any]] = []
    for fixture in sorted(expected):
        robot_fx = load_json(GOLD_PCIT / f"{fixture}.json")
        res = match(gold_target, robot_fx, MatcherConfig())
        want = bool(expected[fixture].get("passed"))
        corpus.append({
            "fixture": fixture,
            "expectPass": want,
            "matcherPassed": res.passed,
            "verdictMatchesExpected": res.passed == want,
            "expectMismatch": expected[fixture].get("expect_mismatch"),
            "hardMismatches": [p for p in res.hard_probes if not res.probe_agreement[p]],
        })

    agg = _aggregate([clip])
    agg["nativeGoldPass"] = native_gold["passed"]
    agg["physicalValidity"] = validity
    agg["acceptanceCorpus"] = {
        "nFixtures": len(corpus),
        "successPass": sum(1 for c in corpus if c["expectPass"] and c["matcherPassed"]),
        "sabotagesFailed": sum(1 for c in corpus if not c["expectPass"] and not c["matcherPassed"]),
        "allMatchExpected": all(c["verdictMatchesExpected"] for c in corpus),
    }
    return {
        "world": "MuJoCo internal sim",
        "worldKey": "mujoco",
        "source": "Genuine MuJoCo backend solver run of put_cube_in_tray (committed fixture; "
                  "experiments/cross_source_oic/mujoco_fixture/).",
        "verifierPath": "extract_robot_csg -> csg.matcher.match -> leakage_report (== csg.benchmark.run_one)",
        "physicalValidityMode": "physics re-checked at capture (true)",
        "tiersRun": list(TIERS),
        "clips": [clip],
        "nativeGold": {"graphId": gold_target.get("graphId"), **native_gold},
        "acceptanceCorpus": corpus,
        "aggregate": agg,
    }


# --------------------------------------------------------------------------------------
# RLBench — external sim
# --------------------------------------------------------------------------------------

def _external_tier_verdicts(rollout: Mapping[str, Any], targets_dir: Path, tiers) -> Dict[str, Dict[str, Any]]:
    return {t: verify_external_rollout(load_json(targets_dir / f"{_tier(t)}.json"), rollout, case_name=t)
            for t in tiers}


def rlbench_leg() -> Dict[str, Any]:
    clips: List[Dict[str, Any]] = []
    fixtures = sorted(RLBENCH_FIXTURES.glob("*.rollout.json"))
    for fx in fixtures:
        rollout = load_json(fx)
        tier_v = _external_tier_verdicts(rollout, RLBENCH_TARGETS, TIERS)
        variation = fx.stem.replace("put_item_in_drawer_", "").replace(".rollout", "")
        clips.append(_clip_record(fx.name.replace(".rollout.json", ""), "success", "PASS", tier_v,
                                  expected_class=f"success/{variation}"))
    agg = _aggregate(clips)
    agg["physicalValidity"] = None
    # In-data discrimination for this success-only world: each demo FAILs the WRONG-precondition
    # structured tier (near-start demos fail placed_from_outside; far-start demos fail relation_event).
    agg["relationEventPass"] = sum(1 for c in clips if c["relationEvent"] == "PASS")
    agg["placedFromOutsidePass"] = sum(1 for c in clips if c["placed"] == "PASS")
    agg["wrongTierRejections"] = sum(
        1 for c in clips if (c["relationEvent"] == "FAIL") != (c["placed"] == "FAIL"))
    return {
        "world": "RLBench external sim",
        "worldKey": "rlbench",
        "source": f"{len(fixtures)} live RLBench PutItemInDrawer demos recorded on CoppeliaSim "
                  "(pilots/rlbench/fixtures/live_runpod_20260616_put_item/).",
        "verifierPath": "verify_external_rollout (leakage door -> extract_robot_csg -> match -> leakage_report)",
        "physicalValidityMode": "external trace (null = physics-unverified)",
        "tiersRun": list(TIERS),
        "clips": clips,
        "aggregate": agg,
    }


# --------------------------------------------------------------------------------------
# Sony / iPhone — real camera
# --------------------------------------------------------------------------------------

def _parse_stem(path: Path):
    stem = path.name[: -len(".tracks.json")]
    if "__" not in stem:
        return None
    episode_id, camera = stem.rsplit("__", 1)
    return episode_id, camera, stem


def _sony_expected() -> Dict[tuple, str]:
    verdicts = load_json(SONY / "verdicts_all.json")
    return {(str(r.get("episodeId")), str(r.get("camera"))): str(r.get("expectedClass"))
            for r in verdicts.get("rows", [])}


def sony_leg() -> Dict[str, Any]:
    expected = _sony_expected()
    placed_target = load_json(RC_TARGETS / f"{_tier('placed_from_outside')}.json")
    clips: List[Dict[str, Any]] = []
    for tp in sorted((SONY / "tracks").glob("*.tracks.json")):
        parsed = _parse_stem(tp)
        if not parsed:
            continue
        episode_id, camera, stem = parsed
        exp = expected.get((episode_id, camera))
        if exp is None:
            continue
        tracks = load_json(tp)
        both = verify_episode_both(tracks=tracks, thresholds=EVIDENCE_THRESHOLDS)
        placed = verify_episode(placed_target, tracks=tracks, thresholds=EVIDENCE_THRESHOLDS,
                                case_name="placed_from_outside")
        tier_v = {
            "terminal_only": both["object_inside_container_terminal_only"],
            "relation_event": both["object_inside_container_relation_event"],
            "placed_from_outside": placed,
        }
        is_success = exp.startswith("success")
        clips.append(_clip_record(stem, "success" if is_success else "non_success",
                                  "PASS" if is_success else "FAIL", tier_v, expected_class=exp))
    agg = _aggregate(clips)
    agg["physicalValidity"] = None
    return {
        "world": "Sony/iPhone real camera",
        "worldKey": "sony",
        "source": f"{len(clips)} committed real-camera clips (Sony 45° + iPhone top), "
                  "object_inside_container_v0 (datasets/sony_object_inside_container_v0/).",
        "verifierPath": "pilots.real_camera.verify_episode (fail-closed UNCERTAIN gate -> verify_external_rollout)",
        "physicalValidityMode": "external trace (null = physics-unverified)",
        "tiersRun": list(TIERS),
        "evidenceThresholds": dict(EVIDENCE_THRESHOLDS),
        "clips": clips,
        "aggregate": agg,
    }


# --------------------------------------------------------------------------------------
# RH20T — real robot video
# --------------------------------------------------------------------------------------

RH20T_TIERS = ("terminal_only", "relation_event")  # no far-start sibling in this episode


def rh20t_leg() -> Dict[str, Any]:
    rollouts_dir = RH20T / "rollouts"
    cases = [
        ("task_0017_user_0010_scene_0005_cfg_0003", "real_positive", "PASS"),
        ("task_0017_user_0010_scene_0005_cfg_0003_negative", "derived_negative", "FAIL"),
    ]
    clips: List[Dict[str, Any]] = []
    for name, kind, ground_truth in cases:
        rollout = load_json(rollouts_dir / f"{name}.rollout.json")
        tier_v = _external_tier_verdicts(rollout, RH20T_TARGETS, RH20T_TIERS)
        clips.append(_clip_record(name, kind, ground_truth, tier_v, expected_class=kind))
    agg = _aggregate(clips)
    agg["physicalValidity"] = None
    return {
        "world": "RH20T real robot video",
        "worldKey": "rh20t",
        "source": "RH20T public dataset, task_0017 'put the pen into the pen holder' (1 real positive "
                  "+ 1 derived negative; datasets/rh20t_object_inside_container_v0/).",
        "verifierPath": "verify_external_rollout (leakage door -> extract_robot_csg -> match -> leakage_report)",
        "physicalValidityMode": "external trace (null = physics-unverified)",
        "tiersRun": list(RH20T_TIERS),
        "clips": clips,
        "aggregate": agg,
    }


def all_legs() -> List[Dict[str, Any]]:
    """Run all four legs (MuJoCo, RLBench, Sony, RH20T), in master-table order."""
    return [mujoco_leg(), rlbench_leg(), sony_leg(), rh20t_leg()]
