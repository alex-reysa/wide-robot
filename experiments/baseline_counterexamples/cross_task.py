#!/usr/bin/env python3
"""Cross-task example: one frozen verifier engine, the task supplied as data.

The whole baseline ladder (B1..B6) is bespoke *cube-in-tray* code: footprint
rectangles, containment margins, a rim-height test, a started-outside clause. None
of it means anything for a *different* task. To score "did the drawer open?" a
practitioner must throw the ladder away and write a brand-new predicate over a
different quantity (a prismatic joint extension, not an XY footprint).

wide-robot does not. The verifier engine — :func:`pilots.external_verify.
verify_external_rollout` and the frozen :mod:`csg.matcher` underneath it — is
**task-agnostic**: the task is a ``csg.v0`` *target graph* (data). The exact same
function object that judges ``object_inside_container`` judges ``open_drawer``; only
the target file changes. This module proves that, executably, against the committed
live RLBench drawer rollouts (``pilots/rlbench/fixtures/live_runpod_20260614_rerun/``,
9 demos) — no RLBench, no OpenCV, ``csg/`` read-only.

What is honestly shared vs. task-specific:
  * SHARED (reused verbatim across tasks): ``verify_external_rollout`` + ``csg.matcher``
    + ``csg.rollout_extract`` + the leakage contract. Identical function objects.
  * TASK-SPECIFIC (thin, separable wrappers): the real-camera evidence gate and the
    ``cameraFailureClass`` naming. These wrap the shared engine; they do not re-derive it.
The baseline ladder has NO shared core — every task is a rewrite.

Honest scope of "task = data": both demonstrated tasks live within the frozen matcher's
EXISTING probe vocabulary — containment exercises ``relation_transitions`` and
``object_carrier``; the drawer exercises ``articulation_transitions``. Swapping only the
target works *because* the needed probe families already exist. A genuinely novel task
that required a probe family the matcher does not have (a new physical relation type)
would extend the frozen engine, not just the data. So "one engine, task = data" is shown
*within* the current probe families (relation / articulation / contact / goal / event),
not as a claim about arbitrary task families. The open_drawer PASSes are also
physics-unverified (``physicalValidity`` is ``null`` on every demo, as for any external
kinematic trace) — an honest PASS on kinematics, never a physics-validated one.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg

from pilots.external_rollout import assert_rollout_leakage_clean
# The SAME engine function, imported by three different pilots. Identity is the point.
from pilots.external_verify import verify_external_rollout as _ENGINE_EXTERNAL
from pilots.rlbench.run_external import verify_external_rollout as _ENGINE_RLBENCH
from pilots.real_camera.verify_episode import verify_external_rollout as _ENGINE_REALCAM

REPO = Path(__file__).resolve().parents[2]
EXP_DIR = Path(__file__).resolve().parent
CROSS_DIR = EXP_DIR / "cross_task"

DRAWER_FIXTURES = REPO / "pilots" / "rlbench" / "fixtures" / "live_runpod_20260614_rerun"
OPEN_DRAWER_TARGET = REPO / "pilots" / "rlbench" / "targets" / "open_drawer_rlbench_articulation_event.json"
OIC_TARGET = REPO / "pilots" / "real_camera" / "targets" / "object_inside_container_relation_event.json"

# The articulation probes the open_drawer target enforces (must carry real support).
_ARTICULATION_PROBES = ("goal_satisfaction", "articulation_transitions", "event_presence")


def engine_identity() -> Dict[str, Any]:
    """Prove the verifier engine is ONE shared function object, not a per-task copy."""
    return {
        "fn": f"{_ENGINE_EXTERNAL.__module__}.{_ENGINE_EXTERNAL.__qualname__}",
        "realCameraImportIsSameObject": _ENGINE_REALCAM is _ENGINE_EXTERNAL,
        "rlbenchImportIsSameObject": _ENGINE_RLBENCH is _ENGINE_EXTERNAL,
        "note": "object_inside_container (real-camera) and open_drawer (RLBench) call the IDENTICAL "
                "verify_external_rollout; only the target graph differs.",
    }


def _drawer_rollouts() -> List[Path]:
    return sorted(DRAWER_FIXTURES.glob("*.rollout.json"))


def verify_open_drawer_demos() -> Dict[str, Any]:
    """Run the open_drawer articulation target on every committed live drawer rollout
    through the shared engine — assert PASS, leakage-clean, non-vacuous."""
    target = load_json(OPEN_DRAWER_TARGET)
    cfg = MatcherConfig()
    rows: List[Dict[str, Any]] = []
    for path in _drawer_rollouts():
        rollout = load_json(path)
        assert_rollout_leakage_clean(rollout)            # source identity must not leak
        case = _ENGINE_EXTERNAL(target, rollout, case_name="open_drawer_rlbench_articulation_event")
        res = match(target, extract_robot_csg(rollout), cfg)
        rows.append({
            "demo": path.name[: -len(".rollout.json")],
            "status": case["status"],
            "passed": bool(case["passed"]),
            "leakageClean": bool(case["leakageClean"]),
            "physicalValidity": case["physicalValidity"],
            "vacuous": bool(res.vacuous),
            "probeSupport": {p: res.probe_support.get(p, 0) for p in _ARTICULATION_PROBES},
        })
    return {
        "target": "open_drawer_rlbench_articulation_event",
        "nDemos": len(rows),
        "allPass": all(r["passed"] for r in rows),
        "allLeakageClean": all(r["leakageClean"] for r in rows),
        "allNonVacuous": all(not r["vacuous"] for r in rows),
        "allProbesSupported": all(all(r["probeSupport"][p] >= 1 for p in _ARTICULATION_PROBES) for r in rows),
        "allPhysicsUnverified": all(r["physicalValidity"] is None for r in rows),
        "physicsNote": "every PASS is physics-unverified (physicalValidity null) — an honest kinematic PASS, "
                       "not a physics-validated one, as for any external trace",
        "rows": rows,
    }


def target_defines_task() -> Dict[str, Any]:
    """The same engine, the same drawer rollout, two different targets: the open_drawer
    target PASSes; the object_inside_container target FAILs. The TARGET is the task."""
    drawer = load_json(sorted(_drawer_rollouts())[0])
    od = _ENGINE_EXTERNAL(load_json(OPEN_DRAWER_TARGET), drawer, case_name="od")
    oic = _ENGINE_EXTERNAL(load_json(OIC_TARGET), drawer, case_name="oic_on_drawer")
    return {
        "drawerRollout": sorted(p.name for p in _drawer_rollouts())[0],
        "open_drawer_target": {"status": od["status"], "passed": bool(od["passed"])},
        "object_inside_container_target": {"status": oic["status"], "passed": bool(oic["passed"]),
                                           "hardMismatches": oic.get("hardMismatches")},
        "note": "One engine call signature, two task graphs. Swapping the target re-tasks the verifier; "
                "no engine code changes.",
    }


def baseline_inapplicable_to_drawer() -> Dict[str, Any]:
    """Show the cube/tray baseline ladder cannot run on a drawer at all. The ladder
    consumes a real_camera.tracks.v0 with (cube, tray) roles and computes XY
    containment; a drawer rollout has an ARTICULATED body, no container, no mover-cube,
    and a different success quantity (prismatic joint extension)."""
    drawer = load_json(sorted(_drawer_rollouts())[0])
    body_kinds = sorted({str(b.get("physicalKind")) for b in drawer.get("sceneBodies", [])})
    has_container = any(bool(b.get("isContainer")) for b in drawer.get("sceneBodies", []))
    return {
        "ladderRequires": ["real_camera.tracks.v0 input", "a 'cube' (mover) role", "a 'tray' (container) role",
                           "XY footprint containment geometry"],
        "drawerRolloutHas": {"schemaVersion": drawer.get("schemaVersion"),
                             "bodyPhysicalKinds": body_kinds, "anyContainerBody": has_container,
                             "successQuantity": "prismatic joint EXTENSION (~0.234 m), not XY containment"},
        "ladderApplicable": False,
        "note": "There is no cube, no tray, no footprint — the B1..B6 predicates have no inputs on a drawer. "
                "Scoring open_drawer with a 'baseline' means writing a brand-new joint-extension predicate "
                "from scratch. The verifier reuses one engine and swaps the target graph.",
    }


def cross_task_report() -> Dict[str, Any]:
    return {
        "kind": "cross-task — one frozen engine, task supplied as a target graph (data)",
        "engineIdentity": engine_identity(),
        "openDrawerDemos": verify_open_drawer_demos(),
        "targetDefinesTask": target_defines_task(),
        "baselineInapplicable": baseline_inapplicable_to_drawer(),
        "thesis": "The structured verifier framework handles object_inside_container and open_drawer with the "
                  "IDENTICAL engine; only the declarative target changes. The baseline ladder is task-bound "
                  "cube/tray code with no shared core — every new task is a full rewrite.",
    }


def write_cross_task(out_dir: Path = CROSS_DIR) -> str:
    out_dir.mkdir(parents=True, exist_ok=True)
    report = cross_task_report()
    path = out_dir / "cross_task_report.json"
    path.write_text(json.dumps(report, indent=2) + "\n")
    return path.name


if __name__ == "__main__":
    print(json.dumps(cross_task_report(), indent=2))
