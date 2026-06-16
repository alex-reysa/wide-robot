# Arm-Bounded Demonstration Compiler — Project Roadmap

> **Renamed 2026-06-10** (scope decision, third audit). This project was
> previously titled *"The Universal Demonstration Compiler."* That framing is
> retired: it is too broad, invites impossible expectations (arbitrary videos,
> arbitrary robots, autonomous execution), and is not provable by a small team
> with limited hardware access. The long-horizon vision survives as background
> reading in `thesis.md`, which is **not** the current scope.

The project claim, stated exactly:

```text
The same semantic task description can be used to judge a human demo, a
simulator rollout, an external simulator trace, and a real robot episode —
without changing the verifier — and the system reports pass/fail, failure
reason, leakage status, and evidence/validity status.
```

```text
semantic task card
→ source binding / calibration
→ concrete target CSG
→ source episode evidence
→ csg.rollout.v0
→ independently extracted rollout CSG
→ unchanged hard-probe verifier
→ pass / fail / failure diagnosis / leakage / validity
```

The goal is **not** general robot intelligence, a robot controller, or a VLA
model. The goal is a **source-independent verifier for embodied task
completion**: what was supposed to happen, what actually happened, whether they
match, why they fail when they do not, whether the evidence was clean, and
whether physical validity was checked. The system maps **object-state
transitions, relations, contacts, articulation, and event order** — never human
joints to robot joints.

```text
The project separates task execution from task verification. That is the wedge.
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

**Allowed now / after Phase 2F RLBench evidence:**

```text
The unchanged verifier can judge an external RLBench OpenDrawer trace through
the same rollout/extraction/matcher path: the original uncalibrated gold target
fails leakage-clean, while a separate RLBench-calibrated value-only target
passes fresh live traces. This shows the verifier is not rubber-stamping
external evidence and that source bindings are required for cross-source
calibration.
```

**Allowed now (Phase 3A real-camera ingestion):**

```text
Real Sony/tripod (and iPhone-top) object_inside_container episodes are converted
to tracks and csg.rollout.v0 and judged by the unchanged verifier: across 78 real
clips, 0 false PASSes on 30 genuine-failure clips, born-inside rejected by
relation_event 8/8, with success recall 27/32 after a frozen manual tray-corner
calibration. It is a marker-based, calibration-bounded pilot demonstrating that the
verifier's safety survives real-video evidence — not a high-recall or marker-free
perception system.
```

**Allowed after the One Task, Four Worlds report (Phase 6):**

```text
The same semantic task card can be bound to multiple evidence sources and
judged by the unchanged verifier across internal simulation, external
simulation, real camera video, and real robot recordings, with pass/fail,
failure diagnosis, leakage status, and evidence/validity status.
```

**Allowed after DK1 / real-robot recordings (Phase 5):**

```text
The same target CSG can be evaluated across symbolic, simulated, and real
recorded robotic-arm rollouts.
```

**Allowed after the human-demo compiler (Phase 3B):**

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
semantic task card              (human-level invariant)
        ↓
source binding / calibration    (MuJoCo, RLBench, Sony, DK1, ...)
        ↓
observable target CSG           (schema: Causal_Skill_Graph_V0.md)
        ↓
source evidence adapter         (internal sim, external sim, camera, robot log)
        ↓
rollout traces                  (csg.rollout.v0 — csg/rollout_schema.md)
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
likelihood, event segmentation, simulator state adapters, robot log adapters,
and source calibration (source-specific evidence). The CSG itself and the
verifier stay source-agnostic; MuJoCo, RLBench/ManiSkill, Sony/tripod video,
and DK1/robot recordings enter only through bindings and rollout adapters.

The RLBench result established the key architectural rule:

```text
"same task" does not mean "same raw number"
```

A task card defines semantic success. A source binding defines how one evidence
source measures that success. The verifier never changes per source.

Example:

```text
Task card: open_drawer
Semantic success:
  drawer starts mostly closed
  drawer articulation increases
  terminal drawer extension is open enough

Bindings:
  MuJoCo: existing gold/mujoco drawer range
  RLBench: terminal extension ≈ 0.234 m
  Sony/tripod: marker- or vision-calibrated open threshold
  DK1/robot: logged joint or calibrated camera estimate
```

Future task-card structure:

```text
tasks/
  open_drawer/
    task_card.md
    bindings/
      mujoco.json
      rlbench.json
      real_camera.json
      dk1.json
    targets/
      open_drawer_mujoco.json
      open_drawer_rlbench_value_only.json
      open_drawer_rlbench_articulation_event.json

  object_inside_container/
    task_card.md
    bindings/
      mujoco.json
      external_sim.json
      sony_marker_table.json
      dk1.json
    targets/
```

The task card is the human-level invariant. The binding says how a source
measures it. The concrete target is what the unchanged verifier reads.

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
| **2F** | External trace verification | 🟡 **RLBench OpenDrawer value-only target is a 9/9 positive result; gold target rejects the same traces 9/9 leakage-clean; a mutation/negative suite proves the calibration is not too permissive; an articulation-event target adds an articulation-increase + event-present check (9/9, strictly stronger than value-only). Next: external object-inside-container.** |
| **3A** | Real-camera episode ingestion (video → rollout evidence) | ✅ **DONE — real Sony/iPhone `object_inside_container` clips judged by the frozen verifier.** 78 clips ingested (`datasets/sony_object_inside_container_v0/`); **0 false PASSes on 30 genuine-failure clips**, born-inside→relation_event FAIL 8/8, success recall **27/32** after a frozen manual tray-corner calibration (was 18/32 marker-fit), 5 remaining misses are conservative UNCERTAIN. `pilots/real_camera/` (author calibration→tracks→rollout→verify, + `visualize_episode.py` overlay diagnostic + `manual_calibration.py`); 107 real_camera tests; `csg/` byte-frozen; raw mp4s off-repo, derived JSON committed. Marker-based + calibration-bounded — not marker-free or high-recall. |
| **3A.5** | RH20T external-source smoke test | ✅ **DONE — real RH20T episode PASSES the frozen verifier.** `pilots/rh20t/` (annotation→tracks→rollout→frozen verifier) passes 22 synthetic tests; a real episode (`task_0017` pen→holder, cfg3 scene rating 9/10, reviewed from a global camera) PASSes both targets — relation-event non-vacuously — leakage-clean, `physicalValidity` null, source-blind rollout; a derived negative FAILs leakage-clean. `csg/` byte-frozen; no raw RH20T media committed (data obtained via an own-OAuth-client Drive copy bypass and kept off-repo). See `datasets/rh20t_object_inside_container_v0/{manifest.json,reports/eligibility_report.md}`. Gates only RH20T-as-external-source evidence; not a Sony/tripod Phase 3A replacement, not a 3B compiler. |
| **3B** | Human-demo compiler (video → target CSG) | ⬜ pending, after 3A |
| **4** | Cross-source flagship task | 🟡 **Three of four worlds landed for object_inside_container: MuJoCo internal-sim (2C), Sony/tripod real-camera (3A), and external-sim (2F-4 — RLBench PutItemInDrawer, 9 live demos PASS the unchanged verifier 9/9).** Remaining: the unified cross-source report (Phase 6) tying object_inside_container across MuJoCo + external sim + Sony/tripod (+ RH20T real-robot-video) through the unchanged verifier; DK1 real-arm is Phase 5. |
| **5** | DK1 / real robot recorded evidence | ⬜ pending (hardware-gated, data campaign first) |
| **6** | One Task, Four Worlds report | ⬜ pending |
| **7** | Verifier-as-a-service / dataset audit tool | ⬜ pending |

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
(see `docs/rlbench_external_trace_pilot.md`) has its offline ingest boundary, one
live Runpod capture, and a value-only diagnostic target complete. `pilots/rlbench/`
lives outside `csg/` and consumes the frozen verifier like a third party; the
converter, live recorder, hardened external leakage checks, and 1×N confusion report
are implemented and tested with fakes. The 2026-06-14 live CoppeliaSim/PyRep/RLBench
run recorded bottom/middle/top demos and emitted leakage-clean rollouts with
`physicalValidity: null`, now promoted to committed fixtures
(`pilots/rlbench/fixtures/live_runpod_20260614/`). Two committed results stand on
those traces: **(A)** the gold `open_drawer` target does **not** accept them
(`event_order`, `goal_satisfaction`); **(B)** a value-only diagnostic target
(`pilots/rlbench/targets/open_drawer_rlbench_value_only.json`), asserting only the
terminal drawer extension, **PASSes** them leakage-clean and non-vacuously. A deliberate
reproducibility rerun (3 fresh demos × bottom/middle/top = 9, committed under
`fixtures/live_runpod_20260614_rerun/` and aggregated by `summarize_reruns`) makes (B) a
**9/9 strong result**: value-only PASS 9/9, gold FAIL-leakage-clean 9/9, off-task-clean
9/9. The negative/mutation suite **(C)** proves the calibration is not too permissive, and
the articulation-event target **(D)** adds an articulation-increase + event-present check
(9/9, strictly stronger than value-only). Contact/order semantics stay deferred (no honest
RLBench evidence source yet). **Result (E)** lands the external-sim leg of the flagship
`object_inside_container` task with a second RLBench task, `PutItemInDrawer` (2F-4): a
two-body converter, three targets (`terminal_only` / `relation_event` / `placed_from_outside`),
a 29-test no-RLBench suite, and **9 live Runpod demos** (3×bottom/middle/top) that PASS
`terminal_only` 9/9 leakage-clean (`physicalValidity: null`) through the unchanged verifier,
with the strong targets partitioning the episodes by observed initial relation (6 FAR, 3 NEAR).
`csg/` stays byte-frozen.

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

### Phase 2F — External trace verification

Purpose: prove that `csg/` is not only accepting rollouts generated by its own
MuJoCo backend. External traces must enter through `pilots/` adapters, source
bindings, and `csg.rollout.v0`; the verifier stays frozen.

```text
external simulator episode
→ source adapter
→ csg.rollout.v0
→ extract_robot_csg
→ frozen matcher + leakage report
→ PASS / FAIL / failure reason
```

Current flagship result:

```text
RLBench OpenDrawer, 9 fresh live demos:
  value-only RLBench target          PASS 9/9
  original gold open_drawer target   FAIL 9/9, leakage-clean
  off-task confusion                 clean 9/9
  physicalValidity                   null 9/9
```

That result is the correct scientific split: the verifier rejects an
uncalibrated target, accepts a calibrated source binding, rejects off-task
targets, and keeps core `csg/` unchanged.

| Sub | Deliverable | Done when |
| --- | --- | --- |
| **2F-1** | RLBench OpenDrawer value-only result | ✅ 9/9 fresh demos PASS the calibrated value-only target; the original gold target FAILs 9/9 leakage-clean; artifacts and docs preserve both results. |
| **2F-2** | RLBench mutation/negative suite | ✅ `tests/test_rlbench_mutations.py` (39 tests, no RLBench): real traces PASS 9/9, gold FAILs 9/9 leakage-clean, off-task confusion clean 9/9, kinematically-wrong (leakage-clean) traces FAIL `goal_satisfaction`, a mis-calibrated `0.18 m` target FAILs all 9, and leaky traces are rejected before matcher success — `csg/` frozen. Answers "is value-only too permissive?" — no. |
| **2F-3** | RLBench articulation-event target | ✅ `open_drawer_rlbench_articulation_event.json` + `tests/test_rlbench_articulation_event.py` (25 tests): enforces terminal ≈ `0.234` (goal_satisfaction), an articulation **increase** (articulation_transitions — direction only), and an `ARTICULATION_CHANGE` event present (event_presence); `event_order` support 0. The numeric initial value is authoring, NOT enforced (a `0.10 → 0.234` trace also PASSes — tripwire test). PASSes 9/9 non-vacuously, strictly stronger than value-only (a flat born-open drawer FAILs it); no handle contact or contact-before-motion order. `csg/` frozen. |
| **2F-4** | External object-inside-container task | ✅ **Offline + live both landed.** RLBench `PutItemInDrawer` is the external-sim container source: a two-body converter (`pilots/rlbench/adapter_object_inside_container.py`), three pilot targets (`object_inside_container_{terminal_only,relation_event,placed_from_outside}.json`), a recorder (`record_put_item_in_drawer.py`), and `tests/test_rlbench_object_inside_container.py` (29 tests, no RLBench). **Live (Runpod/CoppeliaSim, 2026-06-16):** 9 real demos (3×bottom/middle/top) committed under `fixtures/live_runpod_20260616_put_item/` — `terminal_only` PASSes **9/9** leakage-clean, `physicalValidity: null`; the strong targets partition by observed initial relation (6 FAR→`placed_from_outside`, 3 NEAR→`relation_event`), each PASSing its match and rejecting born-inside via `initial_state`. Reproducible from a clean clone with no RLBench; `csg/` byte-frozen. The compact cross-source confusion/leakage report is 2F-5. |
| **2F-5** | External confusion + leakage report | ⬜ A compact report shows positives, negatives, off-task confusion, leak rejection, `physicalValidity: null`, and unchanged `csg/`. |

### Phase 3A — Real-camera episode ingestion

Build video as an **evidence source** before trying to compile video into target
CSGs. The first camera pipeline should judge recorded episodes, not author new
task descriptions.

Constrained capture, not general video understanding:

```text
phone camera · tripod · fixed table · colored cube · colored tray
fiducial markers (AprilTag/ArUco) if needed · known workspace · good lighting
```

Build the first real-camera adapter under `pilots/real_camera/`:

```text
calibrate_table.py
marker_tracker.py
video_to_tracks.py
tracks_to_rollout.py
verify_episode.py
README.md
```

Dataset shape:

```text
datasets/sony_object_inside_container_v0/
  raw_videos/
  calibration/
  tracks/
  rollouts/
  reports/
  manifest.json
```

Start marker-based. Record a small success/failure set for
`object_inside_container`: successes, near-not-inside, rim placement, dropped
outside, missed grasp, wrong object if practical, and occlusion/uncertain
evidence. The output is `csg.rollout.v0`, not `target_csg.json`.

Done when:

```text
Sony/tripod success episodes PASS
Sony/tripod failure episodes FAIL with useful classes
uncertain tracking is surfaced as uncertainty, not hidden
leakage remains clean
```

Current status (2026-06-16): **DONE — real Sony/iPhone `object_inside_container` episodes are
judged by the frozen verifier, conservatively and leakage-clean.** 40 episodes × 2 cameras
(`sony_front` 45°, `iphone_top`) = 80 clips; 78 task clips ingested (2 calibration clips excluded),
**0 errors** (`datasets/sony_object_inside_container_v0/`: `verdicts_all.json`, `INGESTION_RESULTS.md`).
Headline (the point of the phase): the source-independent verifier's *safety* survives real video —
**0 false PASSes across 30 genuine-failure clips** (near-not-inside / left-on-rim / dropped-outside /
inside→outside / static), and **born-inside → relation_event FAIL 8/8**. The pipeline lives entirely
under `pilots/real_camera/` (`calibrate_table.py`, `marker_tracker.py`, `author_calibration.py`,
`video_to_tracks.py`, `track_postprocess.py`, `tracks_to_rollout.py`, `verify_episode.py`) and consumes
the frozen verifier; `csg/` is byte-frozen; raw mp4s are kept off-repo (derived JSON committed). Targets:
the RLBench-parity bundle (`terminal_only` + `relation_event`) plus a sibling `placed_from_outside`
(FAR-start) OR-combined at ingest into a put-in **transition** (real put-ins start NEAR *or* FAR).

Success recall was lifted from **18/32 → 27/32** (Sony 10→15, iPhone 8→12) by a **frozen manual
tray-corner calibration** (commit `3ece64e`): the marker-fit tray center was ~1–2 cm off the physical
cardboard, so genuinely-inside cubes read NEAR. A diagnostic overlay tool
(`pilots/real_camera/visualize_episode.py`) made the offset visible; `pilots/real_camera/manual_calibration.py`
turns four clicked inner-floor tray corners on one reference frame into the marker-7→tray-center offset
**in marker 7's own frame** — a camera-independent physical constant reapplied per clip via each clip's
marker 7 (no per-clip tuning; the top-down iPhone view measures the boundary, Sony adopts that offset
since its 45° depth back-projection is unreliable). The recovery preserved every safety invariant (0
false PASS, born-inside transition 8/8 FAIL, 0 regressions); the 5 remaining success misses are honest
UNCERTAIN, and the physical footprint stays the measured ~18×18 (center-only fix). A global footprint
expansion (18→20 cm) was tested earlier and **rejected** for causing 1 false PASS. Honest limits
(perception, not bugs): born-inside *terminal* is unjudgeable (≈0 net cube displacement → no figure for
the motion-based extractor; relation_event still correct), and the iPhone-top vs Sony-45° trade-off
(top keeps the cube tag but loses the tray floor tag when filled; 45° resolves the terminal relation but
occludes the cube during the place) — sensor fusion is future work. Full method + per-class numbers in
`datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md` (see "Update 2"). This is the **Sony/tripod
leg of the Phase 4 flagship**; it is **not** a Phase 3B target compiler (it judges episodes, it does not
author target CSGs from video).

### Phase 3A.5 — RH20T external-source smoke test

RH20T is a plausible **separate external source**, not a substitute for the
Sony/tripod proof. It offers calibrated multi-camera robot episodes, RGB/RGBD
video, low-dimensional robot data, task descriptions, and containment-like
tasks. Use it to test source-adapter discipline before the full human-demo
compiler, not to claim the Sony/ArUco capture path works.

Good first candidates:

```text
task_0017  Put the pen into the pen holder
task_0072  Drop coins into a piggy bank
task_0073  Put things in the drawer
task_0091  Move an object from one box to another
```

Operational guidance: inspect/download only a minimal RH20T shard on RunPod
storage (`/workspace` or a network volume), not the local machine. Do not commit
raw media.

Build a new source adapter that converts one RH20T episode into neutral tracks
and `csg.rollout.v0`. The smoke test should run the frozen verifier against an
existing or hand-bound source-specific `object_inside_container` target. It must
not build a video-to-target compiler, and it must not infer or author target CSG
from the RH20T episode.

Done when:

```text
one selected containment episode converts RH20T data → tracks → csg.rollout.v0
one positive RH20T episode PASSes the frozen verifier
one negative/corrupted RH20T episode FAILs or reports UNCERTAIN if available
physicalValidity is null, not claimed true
leakage remains clean
git diff -- csg is empty
raw RH20T media is not committed
```

Current status (2026-06-15): **DONE — a real RH20T episode PASSES the frozen verifier.**
The real positive episode is `task_0017_user_0010_scene_0005_cfg_0003` (pen → pen holder,
cfg3 scene rating 9/10), reviewed from global camera `cam_104122062823` and visually confirmed
NEAR → INSIDE. Against the unchanged verifier it PASSes both targets — relation-event
**non-vacuously** (goal_satisfaction, initial_state, terminal_state, relation_transitions,
event_presence all support 1) — leakage-clean, `physicalValidity` null, with a source-blind
rollout; a derived near-not-inside negative FAILs leakage-clean. The data was obtained via an
own-OAuth-client Google Drive `files.copy` bypass (direct download and shared-client copy were
24h-quota-walled), streamed and extracted with raw media kept off-repo; poses are honest
human/assistant review estimates (single oblique 640x360 camera, no depth — the pen is tracked
by centroid; see the annotation `review` and `reports/eligibility_report.md` for the modeling
caveats). `csg/` is byte-frozen and no raw RH20T media is committed. Below is the historical
seam-status detail.

`pilots/rh20t/` implements
annotation→tracks→rollout→frozen-verifier (`annotations_to_tracks`, `tracks_to_rollout`,
`verify_episode` + two source-bound `object_inside_container` targets) and passes 22
synthetic tests (`tests/test_rh20t_rollout.py`, `tests/test_rh20t_cli.py`): a real put-in
PASSes both targets non-vacuously, born-inside is strictly-stronger-rejected on
`initial_state`, near/rim/dropped FAIL leakage-clean, and the rollout is **fully
source-blind** — an RH20T `episodeId` *is* the source identity, so the door drops it (only
a one-way `episodeRef` hash + a fail-closed-validated `archiveSha256` survive into
diagnostics). An adversarial review caught and fixed a real leak (an unchecked
`archiveSha256` could carry a pasted scene path into the rollout); `validate_tracks_v0` now
rejects a non-hex sha fail-closed. `csg/` is byte-frozen and no raw media is committed.

Data acquisition was initially blocked (the cfg3 Drive shard was globally download-quota-walled
and the server-side `files.copy` was per-user rate-limited on the shared rclone client —
recorded under `blocked_data_acquisition_drive_quota` in
`datasets/rh20t_object_inside_container_v0/reports/eligibility_report.md`). It was **resolved on
2026-06-15** via a personal own-OAuth-client Google Drive `files.copy` bypass (the per-user copy
limit cleared after a short cooldown, not the worst-case 24h); the private same-content copy was
streamed by id, one scene + calibration extracted (raw media kept off-repo), the poses
established by human/assistant frame review, and the copy deleted afterward. This checkpoint is
**not** a Sony/tripod Phase 3A result and **not** a Phase 3B target compiler.

### Phase 3B — Human-demo compiler

Only after 3A works, and after the RH20T smoke test if that path is pursued,
build `video → target CSG`. The compiler can reuse the marker/track pipeline,
relation thresholds **imported from `csg/predicates.py`** (one grammar for
target and rollout words — schema audit note #3), event boundary heuristics,
and contact likelihood from hand-object proximity. Output: `target_csg.json` +
`ucv_hypotheses.json` (hidden-physics estimates — mass/friction/grasp-stability
— stay out of the CSG; see `Causal_Skill_Graph_V0.md`). Do **not** start with
Sapiens2, marker-free perception, or general internet video. Target generation
comes after episode verification.

### Phase 4 — Cross-source flagship task

Use two task tracks, not all five V0 tasks:

```text
Track A: open_drawer
  purpose: articulation task
  sources: MuJoCo, RLBench, later real drawer / DK1 if available

Track B: object_inside_container
  purpose: flagship real-world relation task
  sources: MuJoCo put_cube_in_tray, RLBench/ManiSkill container task,
           Sony/tripod video, later DK1/robot recordings
```

`object_inside_container` should be the flagship "One Task, Four Worlds" task:
it is easy to film, easy to understand, meaningful in the real world, has clear
visible failures, and can eventually be executed or replayed on robot hardware.

Done when:

```text
one object_inside_container task card has source bindings for:
  MuJoCo
  external simulator (RLBench or ManiSkill)
  Sony/tripod video
and every source produces PASS successes, FAIL failures, clean leakage, and
interpretable failure classes through the unchanged verifier
```

### Phase 5 — DK1 / real robot recorded evidence

The DK1 is a recorded-evidence source first, not an autonomy milestone. The 24
hours of access is a **data-acquisition and grounding campaign**. The proof
sought:

```text
DK1 / robot success episode  → csg.rollout.v0 → extractor → robot CSG → PASS
DK1 / robot failure episode  → csg.rollout.v0 → extractor → robot CSG → FAIL
both leakage-clean; physical validity honestly reported from available traces
```

Build under `pilots/dk1_recorded/` before any autonomy:

```text
episode_schema.md
logs_to_rollout.py
camera_tracks_to_rollout.py
fuse_robot_and_camera.py
verify_dk1_episode.py
```

Collect `object_inside_container` success and failure episodes: near not
inside, rim placement, dropped object, missed grasp, wrong object, and
occlusion/uncertain evidence. Hour-by-hour checklist and failure modes remain
in §7, updated to treat DK1 as recorded evidence first.

### Phase 6 — One Task, Four Worlds report

Create one report command:

```bash
python3 -m pilots.cross_source.verify_task \
  --task tasks/object_inside_container/task_card.md \
  --sources mujoco,external_sim,sony,dk1 \
  --out reports/object_inside_container_cross_source/
```

Output:

```text
cross_source_report.md
cross_source_report.json
summary.csv
failure_classification.json
source_manifest.json
leakage_report.json
```

Done when the project can honestly say:

```text
For a simple object-inside-container task, the same semantic task card can be
bound to multiple sources and judged by the unchanged verifier across internal
simulation, external traces, real camera video, and real robot recordings.
```

### Phase 7 — Verifier-as-a-service / dataset audit tool

After the cross-source proof, package the verifier as a practical QA layer:

```text
upload rollout/video/logs
choose task card
get PASS / FAIL / failure diagnosis / leakage report / evidence status
```

This is the real product direction: robot/dataset QA for manipulation tasks,
teleoperation datasets, learned-policy rollouts, warehouse pick/place episodes,
and home-robot attempts. The question is always: did the episode actually
complete the task, or did it only look successful?

### Legacy phase labels

Older docs, code comments, and commit messages use the pre-rename numbering.
Mapping:

| Legacy label | Now |
| --- | --- |
| 6A gold tests, 6B symbolic harness, 6D leakage tests | Phase 2A / 2B (all done) |
| **6C MuJoCo harness** | **Phase 2C** |
| Phase 7 / 7A perception compiler, 7B UCV hypotheses | Phase 3A (camera episode ingestion) + Phase 3B (human-demo compiler) |
| Phase 8 control (8A/8B sim solver; 8C/8D DK1 executor) | sim solver shipped in Phase 2; DK1 control/autonomy deferred behind recorded evidence |
| Phase 9 real world (9A calibration, 9B primitives, 9C eval) | Phase 5 recorded robot evidence + Phase 6 cross-source report |

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

## 7. DK1 playbook (Phase 5 first; control later)

The first DK1 proof is recorded evidence, not autonomous execution. Put the
recorded-evidence adapter under `pilots/dk1_recorded/`; only add a robot-control
backend after the evidence path is passing.

### Recorded-evidence package layout

```text
pilots/dk1_recorded/
  episode_schema.md
  logs_to_rollout.py
  camera_tracks_to_rollout.py
  fuse_robot_and_camera.py
  verify_dk1_episode.py
```

Later control/backend package layout, if needed:

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
L5 CSG binding         the Phase 5 recorded-evidence/task-binding route
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

## 8A. Success metrics

The project should report the following metrics across every source binding:

```text
success recall:        true successes that PASS
failure rejection:     true failures that FAIL
failure-class accuracy: failures assigned to the right reason
leakage rejection:     target-copy / planner contamination rejected
off-task confusion:    wrong task target does not pass
source robustness:     same task card works across source bindings
core stability:        verifier unchanged across sources
evidence honesty:      physicalValidity true / false / null is reported honestly
```

A strong result is not simply "20/20 passed." A strong result is: successes
pass, failures fail, wrong tasks do not pass, leakage is rejected, uncertainty
is surfaced, and the verifier remains unchanged.

## 9. What NOT to build

```text
full VLA model · diffusion policy · general robot learner · Sapiens2-heavy
pipeline · Unreal synthetic-human factory · internet-video parser ·
world-model rollout predictor · RL from matcher distance · multi-robot
generalization beyond sim · natural-language-only planner
```

Also do **not** do more identical RLBench OpenDrawer demos, chase all five V0
tasks across every source, make RLBench pass by changing the matcher, claim real
physical validity for RLBench when it remains `null`, start marker-free
perception before marker-based evidence works, or make DK1 autonomy the next
goal.

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

Known open items: articulation **magnitude** is only checked at the terminal value
(via `goal_satisfaction`); the `articulation_transitions` probe compares only the
change **direction** (INCREASE/DECREASE/FLAT), not numeric endpoints, so the
articulation-event target (Result D) enforces "increased to ≈ 0.234 + event present"
but cannot pin the initial value — a dedicated initial-value or magnitude-delta probe
is a candidate if that ever matters. The RLBench evidence is now committed every way:
the 2026-06-14 live run is leakage-clean and unmapped against the gold `open_drawer`
target (Result A); a value-only diagnostic target PASSes the same traces non-vacuously
(Result B); a negative/mutation suite proves the calibration is not too permissive
(Result C); and an articulation-event target adds the increase + event check, strictly
stronger than value-only (Result D) — all reproducible from committed fixtures without
Runpod. The cross-source flagship task `object_inside_container` has since advanced: the
**Sony/tripod real-camera leg is DONE (Phase 3A)** — 78 real clips judged by the frozen
verifier, 0 false PASS on 30 genuine failures, success recall 27/32 after a frozen manual
tray-corner calibration. So the next concrete research items are the **external-sim container
binding (2F-4)** and the **unified cross-source report (Phase 6)** tying MuJoCo + external sim
+ Sony/tripod through the unchanged verifier, then DK1/robot recorded evidence. Broader
ablated/noisy baselines remain optional later extensions, not blockers for the current
Phase 2E/2F minimum.

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
