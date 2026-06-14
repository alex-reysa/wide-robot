# RLBench External-Trace Pilot

**Status:** converter implemented and tested; live record exercised once on Runpod;
a value-only diagnostic target now accepts the real traces. The verifier seam, the
hardened leakage gate, the `open_drawer` ingest converter, and the cross-task confusion
report are all in place and unit-tested with fake observations (`pilots/rlbench/`,
`tests/test_rlbench_pilot.py`, green with **no** RLBench installed). The live
CoppeliaSim/PyRep/RLBench stack records real `OpenDrawer` demos and emits leakage-clean
rollouts. Two committed results now stand on those same traces (see "Live evidence"):
**Result A** — the existing gold `open_drawer` target does **not** accept them
(`event_order`, `goal_satisfaction`); **Result B** — a deliberately narrow *value-only*
target (`pilots/rlbench/targets/open_drawer_rlbench_value_only.json`), which asserts only
the terminal drawer extension, **PASSes** them leakage-clean with `physicalValidity:
null`. Contact/event/temporal semantics are deliberately deferred to a later target.

**Scope: deliberately very narrow.** One RLBench task (`open_drawer`), a handful of
its demonstrations, fed through the **frozen** csg verifier. This is a feasibility
and discipline probe, not a benchmark expansion.

- Upstream: <https://github.com/stepjam/RLBench> · <https://sites.google.com/view/rlbench>

## The question this answers

csg's thesis is *verification discipline, not robot capability*: a target CSG is
solved, the rollout is independently re-extracted, and a PASS requires hard-probe
agreement **and** leakage cleanliness. Every trace so far was produced by csg's own
solver (symbolic or MuJoCo). The open question:

> Does the leakage-clean, hard-probe verification discipline survive a trace that
> **csg did not produce** — a demonstration from a different simulator (RLBench /
> CoppeliaSim) with its own objects, controller, and physics?

A clean PASS on a genuine external `open_drawer` demo, plus a clean **FAIL** when
that demo is matched against a *different* task's target (confusion), is evidence
the verifier is testing the demonstrated behavior, not csg-solver-specific
trajectory shape. A leak (the only way to PASS is to let target authoring through)
is an equally valuable negative result.

## Claim boundary (unchanged, and narrowed further for the pilot)

This pilot claims **nothing** about RLBench task success, robot capability, or
sim-to-real transfer. It claims only: *an external kinematic demonstration trace
can be (a) reduced to the `csg.rollout.v0` information-flow contract without leaking
target identity, and (b) evaluated by the frozen hard-probe matcher + leakage gate.*
Physics is **not** re-checked — csg cannot re-validate another engine's contacts, so
an external trace is `physicalValidity: null` (*physics-unverified*) by contract
(`csg/validity.md`), exactly like the symbolic backend. A PASS here is
"interface-valid, leakage-clean, physics-unverified", never "physically valid".

## Design — swap the trace source, freeze everything downstream

`csg.benchmark.run_one` is:

```
target → solve(target) → rollout → extract_robot_csg → match → leakage_report
```

The pilot replaces exactly one arrow — `solve` — with an external adapter, and runs
the **same** `extract_robot_csg → match → leakage_report` unchanged:

```
RLBench Demo → pilots.rlbench.adapter → csg.rollout.v0 → [FROZEN] extract → match → leakage
```

Nothing in `csg/` is modified or re-imported in an altered form; `pilots/` lives
*outside* the package and consumes the verifier like a third party. That separation
is the point — it forecloses "you adapted the verifier to fit RLBench".

### The components (already in the repo)

| Path | Role | State |
|---|---|---|
| `pilots/rlbench/adapter.py` · `assemble_rollout` | build a leakage-clean `csg.rollout.v0` from neutral bodies + frames | **real, tested** |
| `pilots/rlbench/adapter.py` · `assert_rollout_leakage_clean` | reject forbidden keys / non-whitelisted body fields / non-neutral ids — incl. `objectIdMap`, nested `articulation.articulatedObjectId`, and per-frame `objectPoses`/`articulation` keys | **real, tested** |
| `pilots/rlbench/adapter.py` · `rlbench_demo_to_rollout` | convert a recorded `open_drawer` `Demo` + neutral measurements → `csg.rollout.v0` (XYZW→WXYZ, `gripper_open<0.5`→closed) | **real** (open_drawer only), **tested with fakes** |
| `pilots/rlbench/record_open_drawer.py` | record live RLBench `OpenDrawer` demos (3 variations), quarantine handle names, emit rollout + sidecar | **real** (lazy imports); **live record needs CoppeliaSim/PyRep/RLBench** |
| `pilots/rlbench/run_external.py` · `verify_external_rollout` | run a rollout through the frozen verifier; same PASS criterion as `run_one` | **real, tested** |
| `pilots/rlbench/run_external.py` · `external_confusion_report` | 1×N cross-task confusion: one external rollout vs every gold target | **real, tested** |
| `pilots/rlbench/fixtures/synthetic_open_drawer.rollout.json` | committed external-shaped stand-in trace (leakage-clean: empty `objectIdMap`, neutral ids) | **real** (PASSes the verifier today) |
| `pilots/rlbench/fixtures/live_runpod_20260614/*.rollout.json` | the three committed **real** RLBench `OpenDrawer` demos (bottom/middle/top) + provenance sidecars — so Results A/B reproduce without Runpod | **real** (leakage-clean; FAIL gold, PASS value-only) |
| `pilots/rlbench/targets/open_drawer_rlbench_value_only.json` | value-only diagnostic target: keeps the drawer + hard `ARTICULATION_GOAL` (`targetJointValue 0.234`), drops contacts/events/temporal/object-states | **real** (PASSes the live demos; not a gold task) |
| `tests/test_rlbench_pilot.py` | seam + hardened-leakage + converter + confusion + live-evidence (A/B) tests, no RLBench needed | **green** (66 passed, 3 live-only skipped) |

Run the seam **and** the confusion today, with no RLBench installed:

```bash
python3 -m pilots.rlbench.run_external \
  --target gold_tests/open_drawer/target.json \
  --rollout pilots/rlbench/fixtures/synthetic_open_drawer.rollout.json \
  --confusion --json
python3 -m pytest tests/test_rlbench_pilot.py -q
```

## Current verification

Locally reproducible verification for the offline pilot boundary:

```bash
python3 -m pytest tests/test_rlbench_pilot.py -q
# 66 passed, 3 skipped

python3 -m pilots.rlbench.run_external \
  --target gold_tests/open_drawer/target.json \
  --rollout pilots/rlbench/fixtures/synthetic_open_drawer.rollout.json \
  --confusion
# external-verify status=PASS matcher=True leakageClean=True physicalValidity=None traceSource=rlbench_external
#   confusion[open_drawer] CLEAN: passes=['open_drawer']

# Result A — the gold target FAILs the real RLBench demos (leakage-clean negative):
python3 -m pilots.rlbench.run_external \
  --target gold_tests/open_drawer/target.json \
  --rollout pilots/rlbench/fixtures/live_runpod_20260614/open_drawer_bottom_demo00.rollout.json
# external-verify status=FAIL matcher=False leakageClean=True physicalValidity=None traceSource=rlbench_external
#   hard-probe mismatches: ['event_order', 'goal_satisfaction']

# Result B — the value-only diagnostic target PASSes them (repeat for middle/top):
python3 -m pilots.rlbench.run_external \
  --target pilots/rlbench/targets/open_drawer_rlbench_value_only.json \
  --rollout pilots/rlbench/fixtures/live_runpod_20260614/open_drawer_bottom_demo00.rollout.json
# external-verify status=PASS matcher=True leakageClean=True physicalValidity=None traceSource=rlbench_external

git diff --name-only -- csg
# no output: csg/ is byte-frozen for this pilot
```

## Live evidence (Runpod, 2026-06-14)

Runpod setup used an interruptible community Pod (`runpod/pytorch:2.1.0-py3.10-cuda11.8.0-devel-ubuntu22.04`,
RTX 4000 Ada, about `$0.20/hr`), CoppeliaSim Edu 4.1.0, PyRep 4.1.0.3, and
RLBench 1.2.0. The pod was terminated after artifacts were copied back.

Commands run on the pod:

```bash
PYTHONPATH=/workspace/wide-robot python3 -m pytest \
  tests/test_rlbench_pilot.py::test_recorder_accepts_numpy_low_dim_state_from_live_rlbench -q
# 1 passed

PYTHONPATH=/workspace/wide-robot xvfb-run -a python3 -m pilots.rlbench.record_open_drawer \
  --variations bottom,middle,top \
  --demos-per-variation 1 \
  --out-dir pilots/rlbench/_out/live_runpod_20260614 \
  --verify
```

The recorder wrote to `pilots/rlbench/_out/live_runpod_20260614/` (gitignored). To make
both results reproducible from a clean clone, the three rollouts and their provenance
sidecars are **promoted to committed fixtures** under
`pilots/rlbench/fixtures/live_runpod_20260614/` (~330 KB). The rollouts are
leakage-clean; the sidecars are provenance and *deliberately* carry the RLBench handle
names quarantined out of the rollout (they never reach the verifier).

### Result A — the existing gold target FAILs the real demos (leakage-clean negative)

| Variation | Frames | Terminal extension | Matcher | Leakage | `physicalValidity` | Hard mismatches |
|---|---:|---:|---|---|---|---|
| bottom | 100 | `0.2337 m` | FAIL | clean | `null` | `event_order`, `goal_satisfaction` |
| middle | 101 | `0.2338 m` | FAIL | clean | `null` | `event_order`, `goal_satisfaction` |
| top | 105 | `0.2348 m` | FAIL | clean | `null` | `event_order`, `goal_satisfaction` |

The live trace path is real and leakage-clean, but the gold target is not equivalent to
RLBench's actual `OpenDrawer` behavior. The real demos open to ≈`0.234 m`; the gold hard
articulation goal is `0.18 m` at the frozen `0.05 m` tolerance (so `goal_satisfaction`
fails by ≈`0.004 m` past the window), and the gold's human-style
`CONTACT_BEGIN → ARTICULATION_CHANGE` order is absent because the extracted articulation
event starts at the same frame the contact begins (so `event_order` cannot embed). This
is outcome (3): **unmappable against the *existing* target**, not against any target.

### Result B — a value-only diagnostic target PASSes the same demos

The one question Result B isolates: *can the frozen verifier accept a leakage-clean
RLBench trace once the target asks only whether the drawer reached RLBench's extension?*
`pilots/rlbench/targets/open_drawer_rlbench_value_only.json` keeps the gold drawer object
and the **hard** `ARTICULATION_GOAL` (retargeted to `0.234 m`) and drops `contacts`,
`events`, `temporalEdges`, and `objectStates`. Against the same committed rollouts:

| Variation | Matcher | Leakage | `physicalValidity` | Hard mismatches | Non-vacuous? |
|---|---|---|---|---|---|
| bottom / middle / top | PASS | clean | `null` | *(none)* | yes — `goal_satisfaction` support 1 |

With the events/contacts/object-states removed, `event_order`, `event_presence`, and
`articulation_transitions` fall to **support 0** (they assert nothing), while
`object_carrier` (drawer → `body_000`) and `goal_satisfaction` (terminal `0.234 ± 0.05 m`)
carry real support and agree — so the PASS rests on the demonstrated terminal value, not
on vacuity. The value-only target is *calibrated*, not loosened: it correctly **rejects**
the `0.18 m` synthetic fixture that the gold `0.18 m` target accepts.

**What Result B does NOT claim.** It asserts only that a drawer-shaped articulated body
reached the target extension — **not** that the agent contacted or caused it, nor any
event ordering. Contact, articulation-change-event, and temporal semantics are
deliberately deferred to a follow-on target (see the step plan). The value-only target is
a pilot **diagnostic**, intentionally *not* added to `gold_tests/`.

**Tolerance caveat (honesty).** `pilotMetadata.articulationToleranceM` (`0.03 m`) records
*intent* only. The frozen matcher enforces a single global `MatcherConfig.articulation_tol`
(`0.05 m`), not per-target tolerance; honoring a per-target value would need verifier/config
work and is out of scope. `csg.canon` ignores unknown top-level keys, so the metadata never
enters any probe. All three demos PASS under the enforced `0.05 m` and also fall inside the
tighter intended `0.03 m`.

## Leakage contract for external traces (the heart of the pilot)

`csg/rollout_schema.md` defines the information-flow contract: the rollout is the
**only** thing the extractor may read, and the default answer to "can the rollout
carry X?" is **no** unless a simulator with no access to the demonstration's
authoring could have produced X. For an external source the threat model is
stricter than for csg's own solver, so the adapter enforces, at assembly and again
at the verifier door:

- **No forbidden keys** — `targetCsg`, `plannerView`, `solverMetadata`, target
  observation graphs (the same set `csg.benchmark.leakage_report` fails on).
- **Neutral ids only, everywhere a reader can reach** — `body_000`, `body_001`, …;
  **never** RLBench names (`drawer_frame`, `drawer_joint_top`). The gate now checks
  body ids **and** `objectIdMap` keys/values (emit it empty for an external trace),
  the nested `sceneBodies[].articulation.articulatedObjectId`, and every frame's
  `objectPoses` / `articulation` keys — the extractor ignores some of these, but the
  contract refuses to let target identity ride along in any of them. Every field is
  read through the **same** `get_any` / `as_list` accessors the frozen extractor uses,
  so a snake_case spelling the extractor would accept (`scene_bodies`, `object_poses`)
  cannot slip past the gate, and a present-but-malformed carrier (a list/string where
  an object is required) is rejected, not skipped — the gate is strictly fail-closed.
- **Whitelisted body fields only** — `csg.to_sim.ROLLOUT_BODY_FIELDS`. RLBench
  category labels, part labels, and source ids are authoring and are dropped.
- **Neutral measurements only** — the recorder hands the converter measurements with
  keys restricted to `frameIndex / timeS / bodyPose / articulationValue / bodySizeM /
  sizeApproximate` (no object id, no label, no handle name); the converter rejects any
  extra key, so a leak names the recorder as its source.
- **No `physicalValidity: true`** — an external kinematic trace cannot earn it.

RLBench specifics to **drop** during ingest (these are authoring, not observation):
task name, waypoint/goal annotations, object category labels, ground-truth target
poses, and any RLBench `Task` object. See `RLBENCH_FIELD_MAPPING` in `adapter.py`.

## Task mapping — `open_drawer` first

| csg gold task | RLBench task | Why / notes |
|---|---|---|
| **`open_drawer`** (first) | `open_drawer` | Direct 1:1: articulated single-DoF motion, an existing target + probe set, the lowest-friction first external trace. |
| `put_cube_in_tray` / `insert_object` | `put_item_in_drawer`, `put_rubbish_in_bin` | container insertion; two-phase (transport + insert). |
| `push_object` | `slide_block_to_target` | non-grasp contact → exercises the V0.3 `TOUCHING_LIKELY` predicate on external traces. |

Only `open_drawer` is in scope for the pilot; the rest are candidates for a later,
explicitly-separate expansion.

## Success / failure criteria

A meaningful pilot result is **one** of:

1. **PASS, leakage-clean** — a genuine RLBench `open_drawer` demo matches the
   `open_drawer` target with hard-probe agreement and `leakageClean: true`, **and**
   the same demo FAILs against a different task's target (confusion holds). Positive
   evidence the discipline transfers to external traces.
2. **Can only PASS by leaking** — the demo matches only when some authoring field is
   carried through (the adapter's leakage guard or `leakage_report` trips). A clean
   negative result: it documents exactly what external-trace evidence is missing.
3. **Structurally cannot map** — RLBench observations cannot be reduced to
   `csg.rollout.v0` without inventing target-side facts (e.g. object segmentation is
   unavailable without labels). Also a result: it bounds where the contract applies.

Any of these is a publishable finding for the §"Research contribution after Phase 2E"
line in `roadmap.md`. The failure modes are as informative as the success.

## Step-by-step plan to actually run it

1. ✅ **Target** — reuse `gold_tests/open_drawer/target.json` (articulated single-DoF
   drawer at the needed abstraction; observable-CSG-only).
2. ✅ **Converter** — `rlbench_demo_to_rollout(demo, task="open_drawer", measurements=…)`
   is implemented and unit-tested with fakes (XYZW→WXYZ, `gripper_open<0.5`→closed, one
   neutral `body_000` articulated body whose joint value ramps). The double leakage
   guard rejects any authoring carried through.
3. ✅ **Recorder + confusion** — `record_open_drawer.py` records the three `OpenDrawer`
   variations and writes rollout + sidecar; `external_confusion_report` matches one
   rollout against every gold target. Both are tested label-free with fakes.
4. ✅ **Install RLBench out-of-band + record real demos** — completed once on Runpod
   with CoppeliaSim v4.1.0 + PyRep + RLBench. The live recorder writes to
   `pilots/rlbench/_out/` (gitignored).
5. ✅ **Run + confusion on real demos** — `--verify` ran `verify_external_rollout` +
   `external_confusion_report` into each sidecar.
6. ✅ **Value-only diagnostic target** — both the leakage-clean negative (Result A) and
   the value-only positive (Result B) are committed and reproducible from a clean clone:
   the three live rollouts are promoted to `pilots/rlbench/fixtures/live_runpod_20260614/`
   and `pilots/rlbench/targets/open_drawer_rlbench_value_only.json` PASSes them, all
   without changing `csg/`.
7. ⏳ **Follow-on: articulation-event target** — author
   `open_drawer_rlbench_articulation_event.json` adding a minimal `ARTICULATION_CHANGE`
   event (closed/near-zero → ≈`0.234 m`), still **without** `CONTACT_BEGIN`, overlap/
   handle-contact claims, or strict event order until the adapter has an honest RLBench
   evidence source for them.

## What remains

The offline ingest/verifier path, one live Runpod capture, and the value-only diagnostic
(Result B) are implemented and committed. The remaining pilot work is to extend the
demonstrated semantics honestly: add the articulation-event target (step 7) once there is
an evidence source for the change event, and only later contact/order semantics. Two
constraints stay fixed: `csg/` is unchanged, and no target asserts more than the rollout
can honestly evidence.

## Out of scope (explicitly)

- RLBench task-success metrics, learned policies, or any capability claim.
- Re-validating RLBench physics (out of contract — `physicalValidity` stays `null`).
- Adding RLBench to the base package or the released wheel/sdist (`pilots/` is not in
  `[tool.setuptools] packages`; the csg verifier stays frozen and dependency-free).
- Byte-reproducibility of RLBench demos across machines.

## Risks / watch-items

- **Silent leakage** — the biggest risk is an adapter that "passes" only because it
  leaked. The double guard (`assert_rollout_leakage_clean` + `leakage_report`) and the
  confusion check are the defense; treat a too-easy PASS as suspect.
- **Object correspondence** — RLBench gives many scene props; mapping only the
  task-relevant bodies to neutral ids without using labels is the crux. If it can't be
  done label-free, that is result (3) above, not a bug to paper over.
- **Heavy dependency** — CoppeliaSim is a large GUI sim; keep ingest headless and the
  recorded demos out of the critical path (the seam stays testable without it).
