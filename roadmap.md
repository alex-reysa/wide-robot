# Arm-Bounded Demonstration Compiler — Project Roadmap

> **Renamed 2026-06-10** (scope decision, third audit). This project was
> previously titled *"The Universal Demonstration Compiler."* That framing is
> retired: it is too broad, invites impossible expectations (arbitrary videos,
> arbitrary robots, autonomous execution), and is not provable by a small team
> with limited hardware access. The long-horizon vision survives as background
> reading in `thesis.md`, which is **not** the current scope.

The project claim, stated exactly:

```text
A leakage-clean compiler/verifier loop for fixed-base robotic-arm manipulation.
```

```text
human tabletop demonstration
→ observable target CSG (Causal Skill Graph)
→ robotic-arm solver attempt
→ simulated or recorded rollout
→ independently extracted rollout CSG
→ unchanged hard-probe verifier
→ pass / fail / failure diagnosis
```

The goal is **not** general robot intelligence. The goal is to prove that
human demonstrations can be compiled into an inspectable, embodiment-agnostic
task representation, and that robotic-arm rollouts can be judged against it
**without target leakage**. The robot does not imitate the human body; the
system maps **object-state transitions, relations, contacts, and event
order** — never human joints to robot joints.

```text
The project makes human-to-robot arm manipulation measurable, falsifiable,
and leakage-clean. That is the wedge.
```

New readers: start at `README.md` (repo map, how to run, handoff notes), then
this file, then `physical_quotient.md` §0 (verifier semantics).

---

## 1. Claims discipline

What may honestly be claimed at each stage. Do not let a demo, README, paper
draft, or conversation drift past the strongest allowed claim.

**Allowed now (V0.3, symbolic backend):**

```text
We built a leakage-clean compiler/verifier loop for fixed-base robotic-arm
manipulation. Given a target CSG for a tabletop task, the system generates a
rollout, independently extracts a rollout CSG, and evaluates task-level
equivalence through frozen hard probes under no-target-leakage constraints —
with physical validity honestly reported as unverified.
```

**Allowed now (Phase 2C gold-task MuJoCo coverage):**

```text
The same object-centric target CSG can be evaluated across symbolic and
physically simulated robotic-arm rollouts for all five V0 gold tasks, with a
real physical-validity verdict on the MuJoCo rollouts.
```

**Allowed after DK1 recordings (Phase 4):**

```text
The same target CSG can be evaluated across symbolic, simulated, and real
recorded robotic-arm rollouts.
```

**Allowed after the perception compiler (Phase 3):**

```text
Early evidence that observable human tabletop demonstrations can be compiled
into target CSGs for simple rigid-object manipulation tasks.
```

**Forbidden claims (at any stage of this roadmap):**

```text
We solved robot learning from video.
We built a general one-shot robot learner.
The robot understands arbitrary human videos.
The system works for any robot.
The system is a full DK1 autonomy stack.
The CSG distance proves physical success by itself.
```

---

## 2. Scope: why robotic arms only

Fixed-base arm manipulation is the correct experimental boundary, not a
compromise. It removes locomotion, balance, navigation, mobile-base
uncertainty, and humanoid control — and keeps every hard part that matters:
object state, contact, grasping, placing, pushing, insertion, articulation,
physical validity, failure diagnosis.

The research question:

```text
Can an object-centric causal skill graph serve as an embodiment-independent
intermediate representation for fixed-base robotic-arm manipulation?
```

### V0 task boundary

In scope:

```text
single fixed-base arm · parallel-jaw gripper · fixed tabletop · known camera
known workspace · rigid objects · short-horizon tasks · one primary
manipulated object · simple relations · controlled lighting · low clutter
clear final state
```

V0 task list (all five exist as gold fixtures today):

```text
1. put_cube_in_tray      NEAR → INSIDE
2. place_on_top          NEAR → ON_TOP_OF
3. push_object           planar pose delta, manner-constrained contact
4. open_drawer           articulation value change (sim)
5. insert_object         NEAR → INSIDE via rim/opening parts
```

Explicitly **out of scope** for V0 (future work, not part of the first
proof): cloth, liquids, deformables, dexterous in-hand manipulation, tool
use, multi-object clutter, bimanual tasks, humanoids, mobile robots, natural
internet videos, world-model verification, VLA training from scratch, RL
from matcher distance.

---

## 3. Architecture

```text
human tabletop demo / manual target
        ↓
observable target CSG          (schema: Causal_Skill_Graph_V0.md)
        ↓
CSG task classifier            (csg/skills.py)
        ↓
arm skill solver               (csg/solver.py; MuJoCo in Phase 2C)
        ↓
sim or DK1 execution / teleop replay
        ↓
rollout traces                 (csg.rollout.v0 — csg/rollout_schema.md)
        ↓
independent rollout-to-CSG extractor   (csg/rollout_extract.py)
        ↓
robot_csg.json
        ↓
CSG matcher                    (csg/matcher.py, frozen hard probes)
        ↓
hard-probe report + leakage report + physical-validity report
```

The CSG is the hourglass waist. Above it: perception, tracking, contact
likelihood, event segmentation (robot-agnostic). Below it: motion planning,
gripper commands, calibration, safety (robot-specific). The CSG itself stays
robot-agnostic; the DK1 adapter (Phase 5) is robot-specific.

**The non-negotiable rule** (enforced by `tests/test_leakage.py` and the
benchmark leakage gate):

```text
The robot CSG must be generated from rollout traces only.
It must never read or copy the target CSG.
```

The rollout artifact is the information-flow boundary; its whitelist is
specified in `csg/rollout_schema.md`. Anything not honestly reportable by a
simulator with no access to the demonstration does not go in the rollout.

---

## 4. Acceptance rule (frozen)

```text
A solver pass requires:
  1. every HARD probe agrees           (match.passed — the probe-agreement
                                        vector, with the vacuity gate)
  2. the robot CSG is leakage-clean    (no TaskSpec, sim-extraction provenance)
  3. physical validity is true, or explicitly reported "not checked"
                                        (symbolic backend: always None,
                                        labeled "physics-unverified")
```

The scalar matcher distance survives **only** as: diagnostic signal,
curriculum signal, regression metric, failure clustering, soft score. It is
never the acceptance condition. (History: the original "distance == 0" KPI
was retired in the V0.1 audit — the honest-zero set was empty while a
target-copying cheat passed trivially. See §9.)

Hard probes, the hard/soft split, vacuity, subsumption-preorder semantics,
and the leakage gate are defined in `physical_quotient.md` §0 and implemented
in `csg/`. Cross-task separation is audited by the benchmark confusion matrix
(`python3 -m csg.benchmark gold_tests --confusion`); the only permitted
off-diagonal PASS is the documented `insert_object ~ put_cube_in_tray`
quotient equivalence (`KNOWN_EQUIVALENT_TASKS`, `tests/test_confusion.py`).

---

## 5. Phase plan

| Phase | Title | Status |
| --- | --- | --- |
| **1** | Lock the problem | ✅ **DONE** |
| **2** | No-hardware proof | 🟡 **2A/2B/2D done; 2C covers all five V0 gold tasks in MuJoCo with real validity verdicts; 2E shipped as the public v0.3.x sim-only benchmark release (randomized reports, invalid fixtures, failure taxonomy, baseline comparison, release hygiene, hardened `csg.verify_release`). One item open: MuJoCo physics is self-attested on the laptop-cut tags (`evidence.complete=false`/exit 1) until a CI-attested release lands.** |
| **3** | Cheap perception (video → target CSG) | ⬜ pending |
| **4** | DK1 data campaign | ⬜ pending (hardware-gated, 24 h access) |
| **5** | DK1 control adapter | ⬜ pending |
| **6** | Optional autonomy | ⬜ explicitly last |

### Phase 1 — Lock the problem ✅

Scope (fixed-base arm, rigid tabletop tasks), representation (observable CSG
only), and acceptance rule (§4) are locked. Done across the V0.1–V0.3 audits.

### Phase 2 — No-hardware proof

Prove: *the same target CSG can be solved by a (simulated) robotic arm, then
independently re-extracted from rollout traces, then verified without
leakage.* The goal is disciplined state/contact/relation extraction, not
photorealism.

| Sub | Deliverable | Status |
| --- | --- | --- |
| **2A** | Frozen verifier: gold tests, leakage tests, adversarial/metamorphic/separation tests, benchmark CLI, confusion matrix | ✅ **DONE** (V0.1–V0.3; 78 tests, 5/5 gold, clean confusion diagonal) |
| **2B** | Symbolic harness (Level-0 backend: plumbing proof, physics-unverified by contract) | ✅ **DONE** |
| **2C** | **MuJoCo arm backend** — MJCF scene from `to_sim`, arm + parallel-jaw gripper, scripted controller per skill, real `physicalValidity` verdict per `csg/validity.md` | 🟡 **all five V0 gold tasks covered** (`csg/backends/mujoco/`): hand-written 6-DoF arm + parallel-jaw gripper, scripted controllers, `csg.rollout.v0` frames, matcher PASS, leakage clean, `physicalValidity: true` in gated tests/benchmark. Seeded 30-rollout/task benchmark command now passes with sampled layouts for every V0 task, including x-shifted push starts. |
| **2D** | Task fixtures with failure variants (success / wrong relation / wrong order / missing contact / extra step / leakage attempt) | ✅ symbolic set done for all 5 tasks; ✅ frozen MuJoCo invalid fixtures cover all six physical-validity checks plus semantic verifier failures for push contact missing, wrong relation, and wrong event order |
| **2E** | **Credible sim-only benchmark package** — randomized MuJoCo trials, invalid-physics fixtures, failure taxonomy reports, baseline solver comparisons, reproducible release hygiene | ✅ **shipped** as the public sim-only benchmark release (`v0.3.0` → `v0.3.2`): randomized reports, invalid fixtures, failure taxonomy, source provenance, release audit/rehearsal, MIT metadata, and symbolic/no-op/MuJoCo comparison are regenerated from a clean Git checkout, tagged, published, and validated by a hardened `csg.verify_release` (whole-tree distribution binding + deterministic-evidence re-derivation; see workstreams 2E-1…2E-8 below). **Caveat:** the MuJoCo physics floats are machine-dependent and *self-attested* on these laptop-cut tags — `verify_release` reports `evidence.complete=false` and exits 1 (a verified-but-not-fully-bound result, not a full verification) until a tag is cut via `.github/workflows/release.yml` and added to `ATTESTED_TAGS`. |

2C deliverable, concretely:

```text
target_csg.json → MJCF scene (table, task objects, fixed-base arm,
parallel-jaw gripper) → scripted task controller → rollout states
(csg.rollout.v0, same whitelist) → robot_csg.json → matcher → hard probes
PASS, no leakage, physicalValidity set by real checks (csg/validity.md:
non-penetration, pose continuity, quasi-static support at release, …)
```

The gated benchmark command is:

```bash
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion
```

The 30-seed randomized benchmark harness now exists, invalid fixtures are frozen
under `gold_invalid/`, and the baseline comparison report is implemented.

### Phase 2E — Credible sim-only benchmark endpoint

Current release-candidate endpoint:

```text
A credible sim-only benchmark and verification framework for fixed-base
robotic-arm manipulation.
```

This is the strongest hardware-free target. It does **not** claim real-robot
transfer or general video understanding. It claims that, in a fully virtual
fixed-base arm setting, target CSGs, solver rollouts, independent extraction,
hard-probe verification, leakage checks, physical-validity checks, randomized
coverage, invalid-fixture diagnostics, failure taxonomy, and baseline
comparison artifacts are packaged as a reproducible benchmark.

Current status: **shipped**. The benchmark machinery, no-op expected-failure
baseline, MIT license metadata, and release rehearsal are implemented, and the
report artifacts have been regenerated from a committed clean checkout and
published as the tagged `v0.3.x` release (latest `v0.3.2`), validated against the
tagged source by `csg.verify_release`. One item remains before the endpoint is
*fully bound*: the machine-dependent MuJoCo physics floats are self-attested on
the laptop-cut tags, so `verify_release` reports `evidence.complete=false` and
exits 1 (not a full verification); cutting a release through
`.github/workflows/release.yml` and adding the tag to `ATTESTED_TAGS` closes that.

Post-2E external-trace pilot status: the narrow **RLBench `open_drawer` pilot**
(see `docs/rlbench_external_trace_pilot.md`) has its offline ingest boundary
implemented. `pilots/rlbench/` lives outside `csg/` and consumes the frozen
verifier like a third party; the converter, live recorder scaffold, hardened
external leakage checks, and 1×N confusion report are implemented and tested with
fakes. Live evidence capture remains pending: real CoppeliaSim/PyRep/RLBench demos
must show that each `open_drawer` rollout PASSes its own target, FAILs
non-equivalent targets, remains leakage-clean, and reports
`physicalValidity: null`.

Acceptance bar for calling Phase 2E done:

```text
python3 -m csg.release_rehearsal --dry-run --out <release-out>
python3 -m csg.release_rehearsal --out <release-out>
python3 -m pytest tests/ -q
python3 -m csg.benchmark gold_tests --confusion --require-pass
.venv-sim/bin/python -m pytest tests/ -q
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco \
  --confusion --randomized --seeds 30 --require-pass
.venv-sim/bin/python -m csg.benchmark gold_tests \
  --compare-backends symbolic,noop,mujoco --confusion --require-pass
.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid \
  --require-pass
python3 -m csg.release_audit \
  --symbolic <symbolic-out> \
  --mujoco <mujoco-out> \
  --randomized <randomized-out> \
  --comparison <comparison-out> \
  --invalid-fixtures <invalid-fixtures-out>
python3 -m csg.release_audit \
  --symbolic <symbolic-out> \
  --mujoco <mujoco-out> \
  --randomized <randomized-out> \
  --comparison <comparison-out> \
  --invalid-fixtures <invalid-fixtures-out> \
  --require-final-metadata \
  --project-root .
python3 -m csg.release_rehearsal --out <release-out> \
  --require-final-metadata \
  --project-root .
```

The release rehearsal fails loudly on matcher mismatch, leakage, physical
invalidity, unexpected off-diagonal confusion, missing diagnostics, or missing
report artifacts. The seeded sweep samples every V0 task; push uses shared x
translation so the non-grasp contact line stays calibrated while the tabletop
start location changes. The no-op baseline is deliberately expected to fail;
release audit checks that it fails with non-`passed` failure classes.

Actionable workstreams, in order:

| ID | Workstream | Done when |
| --- | --- | --- |
| **2E-1** | Git/versioned workspace hygiene | ✅ `.gitignore` excludes Python caches, local venvs, logs, and benchmark outputs. Benchmark, comparison, and invalid-fixture reports include `sourceProvenance` with a deterministic source snapshot and Git commit/status when available. Final reports are regenerated from the committed clean checkout so every report's `sourceProvenance` is Git-backed with `dirty=false`; `csg.verify_release` binds each report's source snapshot and the **entire file tree** of every distribution — the wheel (every member outside `.dist-info`, so a native `*.so`/`*.data` PATH payload or a rogue `console_scripts` entry point is caught), the sdist, and the full source tarball (not just `csg/` — a backdoor outside `csg/`, e.g. a tampered `scripts/` file or a sibling package, is reported `unbound`/`treeMismatch`) — to `git archive` of the in-source-pinned tag commit, then reconciles `RELEASE_SHA256SUMS` + `release_manifest.json` against those anchors (`origin` is not trusted for identity). The machine-dependent MuJoCo physics floats are the one layer it cannot re-derive: for a tag not in `ATTESTED_TAGS` they are self-attested, so the verdict is `evidence.complete=false` and exits 1 (not a full verification). |
| **2E-2** | Packaging cleanup and license | ✅ MIT `LICENSE`, `pyproject.toml` license metadata, `sim` extra, README-backed package metadata, neutral classifiers, and console scripts for the existing CLIs are in place; release checklist exists in `docs/release_checklist.md`. |
| **2E-3** | Reproducibility docs | ✅ `README.md`, this roadmap, `docs/release_checklist.md`, and `docs/sim_only_benchmark_report.md` show current commands, expected pass counts, Python/MuJoCo version notes, output locations, and the exact claim boundary. |
| **2E-4** | Randomized rollouts per task | ✅ Benchmark plumbing exists: `--randomized --seeds N` runs every target per seed and stores seed, sampled layout, solver config, matcher result, leakage result, validity report, and failure class per rollout. Current evidence: 30 seeds/task PASS with `physicalValidity: true`; push seeds now sample distinct x-shifted layouts. |
| **2E-5** | Frozen invalid fixtures | ✅ Nine frozen MuJoCo invalid fixtures live under `gold_invalid/`, run by `--invalid-fixtures gold_invalid`, and cover every physical-validity check plus semantic verifier failures for push contact missing, wrong relation, and wrong event order. |
| **2E-6** | Failure taxonomy and reporting | ✅ `failure_classification.json`, per-case `failureClassification`, summary counts, CSV class column, and report.md class summary are implemented. Classes are derived from status/error, hard mismatches, leakage, and physical-validity evidence without changing PASS criteria. |
| **2E-7** | Baseline solver comparisons | ✅ `--compare-backends symbolic,noop,mujoco` writes `comparison_report.json` and per-baseline benchmark outputs. Current baseline comparison contrasts the physics-unverified symbolic backend, a deliberately failing no-op baseline that makes failure taxonomy visible, and the scripted MuJoCo solver on the same target CSGs. Optional later baselines can add noisy/ablated scripted solvers or learned policies. |
| **2E-8** | Benchmark release artifact | ✅ Draft report exists in `docs/sim_only_benchmark_report.md` with task set, probes, validity checks, leakage guarantees, randomized pass rates, invalid-fixture failure rates, confusion matrix, limitations, exact reproduction commands, release audit, and release rehearsal. Finalized from a Git-backed clean checkout and published as tagged release assets (report artifacts, `RELEASE_SHA256SUMS`, `release_manifest.json`); validated against the tagged source by `csg.verify_release` (report-snapshot binding, whole-tree binding of the wheel, sdist, and full source-tarball to `git archive`, and deterministic-evidence re-derivation, plus checksum/manifest reconciliation — the manifest is consumed and reconciled, not inert; the MuJoCo physics floats can't be re-derived cross-machine, so a tag outside `ATTESTED_TAGS` is self-attested and verifies as `evidence.complete=false`/exit 1) and re-runnable from a clean clone via `scripts/clean_clone_rehearsal.sh` (which re-runs the benchmarks + strict audit; MuJoCo physics floats are machine-dependent, so this re-derives the evidence, not bit-identical bytes). |

Recommended implementation order:

1. Regenerate and finalize `docs/sim_only_benchmark_report.md` from a
   tagged/source-snapshot clean checkout once Git provenance and license
   metadata are in place.
2. Add broader optional baselines later; the symbolic/no-op/MuJoCo comparison
   and nine-fixture invalid suite are already implemented.

Research contribution after Phase 2E:

```text
An open, leakage-clean sim benchmark for fixed-base arm manipulation where
object-centric target CSGs are evaluated across symbolic and MuJoCo rollouts,
with independent extraction, hard-probe matching, real physical-validity
checks, randomized coverage, invalid-fixture diagnostics, and reproducible
baseline comparisons.
```

### Phase 3 — Cheap perception (legacy name: "Phase 7 perception compiler")

Constrained capture, not general video understanding:

```text
phone camera · tripod · fixed table · colored cube · colored tray
fiducial markers (AprilTag/ArUco) if needed · known workspace · good lighting
```

First compiler is deliberately simple: color segmentation, marker poses,
object centroids, relation thresholds **imported from `csg/predicates.py`**
(one grammar for target and rollout words — schema audit note #3), event
boundary heuristics, contact likelihood from hand-object proximity. Output:
`target_csg.json` + `ucv_hypotheses.json` (hidden-physics estimates —
mass/friction/grasp-stability — stay out of the CSG; see
`Causal_Skill_Graph_V0.md`). Do **not** start with Sapiens2 or general video
models. If the compiler cannot produce a correct CSG for "put cube in tray,"
do not move on.

### Phase 4 — DK1 data campaign

The DK1 is the real-arm backend, not the project. The 24 hours of access is a
**data-acquisition and grounding campaign**, not a live autonomy build. The
proof sought:

```text
DK1 teleop success episode  → extractor → robot CSG → hard probes PASS
DK1 teleop failure episode  → extractor → robot CSG → the RIGHT probe FAILS
both leakage-clean; physical validity honestly reported from available traces
```

Hour-by-hour plan, prerequisites checklist, and the failure modes to
intentionally collect are in §7.

### Phase 5 — DK1 control adapter (legacy name: "Phase 8C/8D")

```text
CSG subgoal → skill route → end-effector waypoints → IK / controller
→ joint-position actions → recorded rollout → extractor → verifier
```

Backend package `csg/backends/dk1/` — see §7 for the module layout. Only the
CSG-binding level is CSG-specific; everything below is DK1-specific.

Skill routing by CSG structure (implemented for the symbolic backend in
`csg/skills.py`; reuse for DK1):

```text
relation NEAR → INSIDE                      pick_place / insert
relation NEAR → ON_TOP_OF                   place_on
planar pose delta / CONTACT_MODE_CONSTRAINT
  with TOUCHING_LIKELY / SLIDING_LIKELY     push
articulation value changes                  open / close
ALIGNED_WITH + INSIDE                       insert
```

If learned policies ever enter here: dense solver rewards (pose distance,
relation achievement, contact timing, collision penalty) for training; the
matcher **only** as terminal verifier / curriculum / benchmark. Never
`reward = -matcher_distance`.

### Phase 6 — Optional autonomy

One scripted DK1 pick-place with known object pose and fixed camera, hard
probes PASS, no leakage. Only after this should learning-based execution be
considered. Autonomy is the last layer, not the first proof.

### Legacy phase labels

Older docs, code comments, and commit messages use the pre-rename numbering.
Mapping:

| Legacy label | Now |
| --- | --- |
| 6A gold tests, 6B symbolic harness, 6D leakage tests | Phase 2A / 2B (all done) |
| **6C MuJoCo harness** | **Phase 2C** |
| Phase 7 / 7A perception compiler, 7B UCV hypotheses | Phase 3 |
| Phase 8 control (8A/8B sim solver; 8C/8D DK1 executor) | sim solver shipped in Phase 2; DK1 adapter is Phase 5 |
| Phase 9 real world (9A calibration, 9B primitives, 9C eval) | Phase 4 (data) + Phase 5/6 (control) |

---

## 6. Harness levels

```text
Level 0  symbolic trace harness     ✅ proves JSON → rollout → CSG → matcher
                                      plumbing; physicalValidity = None by
                                      contract (never claims physics)
Level 1  kinematic simulator        (optional stepping stone)
Level 2  physics simulator          🟡 MuJoCo (Phase 2C): contacts, gripper,
                                      friction — all five V0 gold tasks pass
                                      gated tests/benchmark with real
                                      physicalValidity true
```

MuJoCo first (controlled manipulation, fast, simple). Isaac later, only if
GPU-scale or photorealism is ever justified.

---

## 7. DK1 playbook (Phases 4–5)

### Adapter package layout

```text
csg/backends/dk1/
  dk1_robot_profile.json     robot-specific assumptions (fixed_base_arm,
                             parallel_jaw, joint_position control, workspace
                             bounds, cameras, supported capabilities)
  dk1_camera_calibration.py  intrinsics/extrinsics, camera↔world↔robot
                             transforms, table plane, workspace bounds
  dk1_kinematics.py          IK, reachability
  dk1_workspace.py           frames, bounds
  dk1_skill_executor.py      pick_place / place_on / push / open_drawer /
                             insert / home / safe_stop / recover_from_failed_grasp
  dk1_rollout_recorder.py    timestamps, context+wrist frames, joint
                             positions, gripper state, commanded actions,
                             episode metadata
  dk1_rollout_to_csg.py      independent extraction: tracks, poses, relations,
                             contact likelihoods, events, final/failure state.
                             MUST NOT read target_csg.json — same contract as
                             csg/rollout_extract.py, enforced by the same
                             leakage tests. Emit csg.rollout.v0 if possible so
                             the existing extractor is reused.
  dk1_safety.py              limits, e-stop integration
  dk1_dataset_import.py      LeRobot dataset ↔ rollout conversion
```

Five-level control stack — only Level 5 is CSG-specific:

```text
L1 robot connection    connect / get_observation / send_action / home / stop
L2 calibration         camera, base, table, workspace, gripper, object frames
L3 primitive motion    move_to_joint / move_to_ee_pose / open / close /
                       move_linear / lift / lower / retreat
L4 skills              pick, place, push, insert, open_drawer
L5 CSG binding         the §5-Phase-5 routing table
```

### Before the 24 h window (all of this is desk work — do it first)

```text
repo installed & tests green        recording scripts ready
benchmark CLI ready                 calibration checklist ready
target_csg fixtures ready           episode naming convention ready
objects prepared (colored cube, tray, flat target zone, platform, simple
  drawer if possible)               calibration board + fiducials printed
external camera/tripod ready        storage ready
```

### The 24 hours

```text
h 0–2    setup: connect, verify leader/follower, both cameras, recording,
         replay/saving, e-stop
h 2–5    calibration: board poses, known table points, workspace corners,
         gripper open/close, fixed-marker views → intrinsics, extrinsics,
         table plane, world frame, workspace limits, reachability
h 5–12   main task put_cube_in_tray: 20 successful teleop episodes,
         20 failed/near-failed, 5 human demo videos, 5 reset videos.
         Vary: cube start, tray position, approach direction, minor lighting.
h 12–18  second task (place_on_top or push_object — pick the easier one if
         the first was unstable): 10–15 successes, ~10 failures
h 18–22  primitive clips: reach, approach, close-near-cube, lift, place &
         release, push straight/left/right, recover from missed grasp /
         dropped cube
h 22–24  redundancy: re-record gaps; verify files saved, metadata exists,
         videos readable, joint logs exist, timestamps aligned, labels
         written down
```

Failures to collect **intentionally** (this is what makes the verifier
useful): missed grasp, cube pushed but not lifted, cube dropped outside tray,
cube placed on rim (ON_TOP_OF, not INSIDE — the predicates already separate
these), cube near but not inside, tray moved accidentally, gripper collision,
occlusion.

Do **not** spend DK1 time training a policy or attempting full autonomy.

### Budget reality

A ~$200 GPU budget does not change this project — V0 avoids heavy training
by design (scripted solvers, fiducials, known geometry, deterministic tests).
Spend it on: tripod/camera, lighting, markers, calibration board, objects,
storage, cables, occasional cloud bursts/API calls. The advantage is rigor,
not compute.

---

## 8. Failure taxonomy

Every failed rollout gets classified — a verifier that explains failure is
the product; a bare success rate is not. Classes:

```text
perception_failure          object_dropped
wrong_object_selected       object_outside_workspace
relation_not_achieved       collision
contact_missing             grasp_failed
event_order_wrong           physical_invalidity
extractor_uncertainty       target_leakage_detected
verifier_mismatch
```

These are derived from the per-probe agreement vector + leakage + validity
reports, and benchmark runs write an explicit `failure_classification.json`.
Reports must include failures, never hide them.

## 9. What NOT to build

```text
full VLA model · diffusion policy · general robot learner · Sapiens2-heavy
pipeline · Unreal synthetic-human factory · internet-video parser ·
world-model rollout predictor · RL from matcher distance · multi-robot
generalization beyond sim · natural-language-only planner
```

These are tempting and premature. The project dies competing with foundation
-model labs; it survives as the clean compiler/verifier layer those models
could eventually plug into.

---

## 10. History — how the verifier got honest (V0.1–V0.3)

Condensed; the authoritative override list is `physical_quotient.md` §0 and
the schema audit notes at the top of `Causal_Skill_Graph_V0.md`.

- **V0.1 (first audit):** the original harness was leaky end to end — the
  rollout converter deep-copied the target CSG, the solver hardcoded success,
  and the scalar-distance KPI had an empty honest-zero set. Rebuilt as the
  single-source `csg/` package; PASS became the per-probe agreement vector;
  leakage tests added. `CSG_Matcher/` and `CSG_Solver_Harness/` are
  deprecated shims (see `CSG_Solver_Harness/DEPRECATED.md`).
- **V0.2 (second audit, executed attacks):** vacuity gate (sparse targets no
  longer accept everything); event order = injective order-preserving
  embedding; converse-relation normalization; directional promoted
  contact-word; open-cavity container compilation (INSIDE physically
  reachable); `physicalValidity` contract (symbolic = None, never true);
  structural-leakage hardening (rollout body whitelist, neutral ids,
  physical_kind-only carrier signature).
- **V0.3 (priorities 4–6):** object mapping via 1-WL role fingerprints with
  symmetry-orbit reporting (10 identical cubes align mover-to-mover in ~5 ms);
  push skill + non-grasp contact extraction; tray fixture fixed per schema
  note #5; gold tasks grew to 5 with failure variants; benchmark confusion
  matrix with `KNOWN_EQUIVALENT_TASKS`; subsumption-preorder semantics
  documented; `csg/rollout_schema.md` written.

Current state: the core suite is green (symbolic loop, leakage, adversarial,
validity-checks), **5/5 gold tasks PASS, clean confusion diagonal, leakage
clean**. The **MuJoCo backend** (Phase 2C) has gated tests/benchmark coverage
for all five V0 gold tasks, each with `physicalValidity: true`; a sabotaged
rollout (`early_release`) is correctly rejected.

Known open items: articulation magnitude is only checked via
`goal_satisfaction` (a dedicated probe is a candidate). The next concrete
research item is the live RLBench external-trace run: install CoppeliaSim +
PyRep + RLBench, record bottom/middle/top `OpenDrawer` demos, run the external
verifier + confusion check, and write up whether the result is clean success,
leak-to-PASS, or structurally unmappable. Broader ablated/noisy baselines remain
optional later extensions, not blockers for the current Phase 2E minimum.

```text
python3 -m pytest tests/ -q                          # core suite; mujoco tests skip without extra
pip install -e '.[sim]' && python3 -m pytest tests/ -q   # includes gated mujoco backend tests
python3 -m csg.benchmark gold_tests --confusion      # 5/5, clean matrix
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco \
  --confusion --randomized --seeds 30 --require-pass
.venv-sim/bin/python -m csg.benchmark gold_tests \
  --compare-backends symbolic,noop,mujoco --confusion --require-pass
.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid \
  --require-pass
```
