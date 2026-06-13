# Phase 2E Sim-Only Benchmark State

Last updated: 2026-06-13

## Orchestration Contract

- Main thread acts as orchestrator and integration owner.
- Planner maps task IDs, scopes, risks, and acceptance criteria before implementation edits.
- Developer work is scoped by Task ID and file ownership.
- Auditor verifies diffs and raw test/command evidence before any task is marked complete.
- Reviewer checks overall integration at phase boundaries.
- This file is updated after every material step so the work is resumable.

## Environment

- Workspace: `/Users/alejandro/Desktop/999. PROJECTS/wide-robot`
- Git: no `.git` repository detected in this directory; commit/SHA-based review is unavailable.
- Python split:
  - `python3`: Python 3.12.8, no `mujoco` module.
  - `.venv-sim/bin/python`: Python 3.12.8 with `mujoco==3.9.0`.

## Current Release Snapshot

- MuJoCo Phase 2C coverage is implemented for all five V0 gold tasks:
  `put_cube_in_tray`, `place_on_top`, `insert_object`, `push_object`, and
  `open_drawer`.
- Latest recorded full verification:
  - Core `python3` suite: `114 passed, 2 skipped`.
  - Symbolic gold benchmark: `passed=5/5`.
  - MuJoCo `.venv-sim` suite: `132 passed, 1 skipped`.
  - MuJoCo gold benchmark: `passed=5/5`.
  - MuJoCo randomized benchmark: `150/150` across 30 seeds/task with
    `physicalValidity=True` and distinct sampled layouts for all five tasks.
  - Symbolic-vs-MuJoCo comparison: both baselines `5/5`; symbolic validity is
    labeled `unverified`, MuJoCo validity is labeled `valid`.
  - Invalid fixtures: `matched=9/9` across six physical-invalidity fixtures and
    three semantic verifier failures.
- Package metadata now includes the `sim` extra, README-backed package
  metadata, neutral classifiers, and console scripts for the existing module
  CLIs (`csg-benchmark`, `csg-solver`, `csg-to-sim`, `csg-rollout-extract`,
  `csg-matcher`, `csg-skills`, `csg-release-audit`,
  `csg-release-rehearsal`).
- Benchmark, comparison, failure-classification, and invalid-fixture reports
  now include `sourceProvenance`: Git metadata when available plus a
  deterministic SHA-256 source snapshot.
- Benchmark summaries now include failure-class, physical-validity, and leakage
  clean/dirty counts at top level for release auditability.
- Public sim-only benchmark report draft exists at
  `docs/sim_only_benchmark_report.md`; it still needs regeneration/finalization
  from a Git-backed clean checkout after license metadata is chosen.
- Remaining public-release gates:
  - Initialize or restore Git provenance so reports include commit-backed
    source identity, not only a source snapshot.
  - Project owner selects a license, commits `LICENSE`, and adds matching
    `pyproject.toml` license metadata.
  - Run the release checklist from a clean checkout and attach report artifacts
    to the tagged/source-snapshot release.

## Global Constraints

- Do not touch the frozen verifier surface: matcher, extractor, canon, leakage gate.
- Keep symbolic backend behavior green.
- Keep existing `put_cube_in_tray` MuJoCo pass green.
- Route through `csg.rollout.v0`; no target leakage in rollout/extractor.
- Every MuJoCo PASS must carry a real `physicalValidity=True` verdict.
- Confusion matrix: only `insert_object ~ put_cube_in_tray` may pass off diagonal.

## Task Map

### T0: Baseline And Current Failure Evidence

Files: read-only.

Scope:
- Run core suite under `python3`.
- Run current MuJoCo gated suite and/or targeted probes under `.venv-sim/bin/python`.
- Capture current failure modes for the four missing gold tasks.

Risks:
- Existing output files under `csg_benchmark_out/` may reflect an old run.
- No Git history means diff/audit must rely on filesystem inspection and test output.

Acceptance:
- Raw command evidence recorded in this file.
- Existing `put_cube_in_tray` status known before refactor.

### T1: Runner Dispatch And Pick-Family Generalization

Files:
- `csg/backends/mujoco/runner.py`
- `csg/backends/mujoco/arm.py` only if exports/imports need adjustment
- `tests/test_mujoco_backend.py`

Scope:
- Factor `_Runner.run()` into skill dispatch.
- Move current pick-place logic into `_run_pick_place()`.
- Add `_Segment.report_closed`.
- Add skill-aware `_finish()` fields.
- Add `_squeeze_q_for_width()` using finger geometry constants.
- Route `insert` and `place_on` through pick-place.

Risks:
- Regressing `put_cube_in_tray` event order or validity.
- Generalized squeeze may fail gripper feasibility for tall/narrow peg.
- `ON_TOP_OF` placement can be confused with `INSIDE` if z target or settling is wrong.

Acceptance:
- `put_cube_in_tray`, `insert_object`, and `place_on_top` MuJoCo end-to-end PASS.
- `physicalValidity=True` for all three.
- Pick-family validity checks applicable as expected.
- No leakage violations.

### T2: Push MuJoCo Path

Files:
- `csg/backends/mujoco/runner.py`
- `csg/backends/mujoco/scene_mjcf.py`
- `tests/test_mujoco_backend.py`

Scope:
- Add `_run_push()` with open-gripper fixed-aperture cradle.
- Add push-aware layout so puck starts FAR from goal and can end NEAR.
- No weld, no release, no grasp interval.
- Trace fields: `figure_id=puck`, `ground_id=goal_block`, `grasped_object=None`, `release_indices=[]`.

Risks:
- Extractor drops non-grasp touch unless gripper is open, object moves >5 mm, and co-motion >= 0.6.
- Effector must remain within `touching_gap_m=0.012` of puck surface during push.
- Puck can penetrate, rotate away, or be pushed into the goal block too hard.

Acceptance:
- `push_object` MuJoCo end-to-end PASS.
- Robot CSG contact word is `TOUCHING_LIKELY` and not `GRASP_LIKELY`.
- Final puck relation to goal block is `NEAR`.
- `physicalValidity=True`; applicable checks are non-penetration, continuity, reachability.
- Push row/column remains separated in confusion matrix.

### T3: Open Drawer Articulation

Files:
- `csg/backends/mujoco/scene_mjcf.py`
- `csg/backends/mujoco/runner.py`
- `tests/test_mujoco_backend.py`

Scope:
- Emit articulated drawer as static cabinet plus child slide body named by object id.
- Add prismatic slide joint with range `0 0.22` and initial value `0.02`.
- Add graspable handle bar on robot-facing face.
- Record per-frame `articulation[drawer_id]` from slide qpos.
- Add `_run_open()` handle grasp, weld, hold, pull, release.
- Trace fields: `grasped_object=drawer`, `figure_id=None`, `articulation_limits={drawer:(0,0.22)}`.

Risks:
- MJCF freejoint removal can break object pose recording if body names/joint names drift.
- IK reach to handle and pull displacement may fail unless layout is skill-aware.
- Slide friction/damping can prevent >0.05 m articulation increase.
- Gripper feasibility must pass on handle/drawer grasp interval.

Acceptance:
- `open_drawer` MuJoCo end-to-end PASS.
- Extracted robot CSG includes `ARTICULATION_CHANGE` with PRISMATIC / EXTENSION_M and increase >0.05.
- Extracted contact includes handle/drawer `GRASP_LIKELY` and `RELEASE_INFERRED`.
- `physicalValidity=True`; articulation limits applicable and passing.
- Open row/column remains separated in confusion matrix.

### T4: Benchmark CLI, Docs, And Full Confusion

Files:
- `csg/benchmark.py`
- `tests/test_mujoco_backend.py`
- `README.md`
- `roadmap.md`
- `csg/validity.md`

Scope:
- Add `--backend/--engine` flag to benchmark CLI.
- Pass `SolverConfig(backend=args.backend)` into `run_benchmark`.
- Add gated MuJoCo per-task and confusion tests.
- Update status docs to reflect full V0 MuJoCo coverage.

Risks:
- CLI default must remain symbolic.
- Docs must not overclaim randomized seeded coverage.
- Confusion tests can be slow; keep MuJoCo tests gated by importorskip.

Acceptance:
- `python3 -m csg.benchmark gold_tests --confusion` remains symbolic 5/5.
- `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion` reports 5/5, all physicalValidity true, no unexpected off-diagonal passes.
- `python3 -m pytest tests/ -q` remains green without MuJoCo.
- `.venv-sim/bin/python -m pytest tests/ -q` green with MuJoCo tests.

## Current Status

- T0: completed.
- T1: completed.
- T2: completed.
- T3: completed.
- T4: completed.

## Phase 2E Continuation

Persistent objective:
- Reach a reproducible public benchmark for fixed-base robotic-arm
  manipulation, not merely a five-task MuJoCo prototype.
- Current release-candidate status: seeded randomized MuJoCo rollouts for every
  V0 task, frozen invalid fixtures, benchmark failure taxonomy/reporting,
  symbolic-vs-MuJoCo baseline comparison, source-snapshot provenance, package
  console entry points, release audit, release rehearsal, and reproducibility
  docs are implemented.
- Required final public-release gates still open: Git/versioned workspace
  provenance, owner-selected license/package metadata, final clean-checkout
  report regeneration, and attachment of the resulting release artifacts.

Known blockers/gaps from current-state audit:
- This workspace has no `.git` directory, so commit/SHA-backed frozen-file and
  diff evidence is unavailable until the project is initialized or restored as a
  Git checkout.
- There is no `LICENSE` file yet.
- `pyproject.toml` has install metadata, README-backed package metadata,
  neutral classifiers, the `sim` extra, and console entry points for the
  existing module CLIs, but no license metadata because no `LICENSE` file
  exists yet.
- Frozen MuJoCo invalid fixtures now cover all six physical-validity checks
  plus semantic verifier failures for push contact missing, wrong relation,
  and wrong event order.
- Randomized benchmark plumbing and failure taxonomy exist; push seeded sweeps
  now sample distinct shared-x layouts while preserving the calibrated contact
  line.
- First baseline comparison mode exists for `symbolic` vs `mujoco`; broader
  ablated/no-op/noisy baselines are optional future extensions, not blockers
  for the current Phase 2E minimum.

### 2E Task Map

#### 2E-0: Phase 2E Audit And Plan

Files:
- `state.md`
- `docs/superpowers/plans/2026-06-12-phase-2e-sim-only-benchmark.md` if a
  detailed execution plan is needed.

Acceptance:
- Requirement-to-evidence map exists in this file.
- Next implementation slice is identified with clear tests and no weakened
  matcher/extractor/leakage gates.

#### 2E-1: Benchmark Failure Taxonomy

Files:
- `csg/benchmark.py`
- `tests/test_confusion.py` or new focused benchmark-report tests.
- `README.md`, `roadmap.md` if public command/report shape changes.

Acceptance:
- Every benchmark case includes an explicit `failureClassification` object.
- Benchmark output includes a top-level failure-class summary and writes
  `failure_classification.json`.
- PASS cases classify as `passed`; failures are derived from matcher probes,
  leakage report, solver errors, and physical-validity checks without changing
  verifier semantics.

#### 2E-2: Seeded Randomized MuJoCo Benchmark Plumbing

Files:
- `csg/backends/mujoco/scene_mjcf.py`
- `csg/backends/mujoco/runner.py` if diagnostics need sampled layout metadata.
- `csg/benchmark.py`
- `tests/test_mujoco_backend.py`
- `README.md`, `roadmap.md`, `csg/validity.md`

Acceptance:
- `csg.benchmark --backend mujoco --randomized --seeds N` runs each target for
  deterministic seed values.
- Reports store seed, solver config, sampled scene/layout parameters, matcher
  result, leakage result, and physical-validity verdict per rollout.
- Small gated tests prove at least multiple seeds/task pass and produce distinct
  sampled layouts while preserving leakage cleanliness and expected confusion.
- The 30-seed command is documented but only claimed after fresh evidence.

#### 2E-3: Frozen Physically Invalid Fixtures

Files:
- `gold_tests/**`
- `csg/benchmark.py` if fixture discovery/output needs extensions.
- `tests/test_mujoco_backend.py` and/or new invalid-fixture tests.

Acceptance:
- Physically invalid MuJoCo-generated fixtures are checked in or reproducibly
  generated as benchmark fixtures.
- Each fixture has expected verifier or validity failure reason.
- Invalid fixtures fail for the expected reason and cannot be hidden by PASS
  aggregation.

#### 2E-4: Packaging, License, And Release Hygiene

Files:
- `.gitignore`
- `LICENSE`
- `pyproject.toml`
- `README.md`
- `roadmap.md`

Acceptance:
- Generated artifacts, local venvs, caches, logs, and benchmark outputs are
  ignored once the workspace is in Git.
- License is explicit.
- Clean-checkout setup commands for base and sim extras are documented.
- Package metadata is sufficient for editable install.
- Existing module CLIs have package console entry points.

#### 2E-5: Baseline Solver Comparison

Files:
- `csg/benchmark.py`
- `csg/solver.py` if a deliberately weak baseline backend/config is added.
- Tests and docs for the chosen baseline.

Acceptance:
- A single command benchmarks the current scripted solver against at least one
  weaker baseline on the same targets.
- Report groups pass rate, failure classes, leakage, validity, and confusion per
  baseline.
- The baseline is diagnostic and clearly not overclaimed as a learned policy.

## Evidence Log

- 2026-06-12: Initialized planner state before implementation edits.
- 2026-06-12 T0 baseline:
  - `python3 -m pytest tests/ -q` -> `99 passed, 1 skipped in 0.43s`.
  - `python3 -m csg.benchmark gold_tests --confusion --out /tmp/wide_robot_symbolic_baseline` -> `passed=5/5`; expected off-diagonal equivalence only `insert_object <-> put_cube_in_tray`.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `7 passed in 0.63s`.
- 2026-06-12 T0 current MuJoCo probes:
  - `put_cube_in_tray`: matcher PASS, `physicalValidity=True`.
  - `insert_object`: matcher PASS, `physicalValidity=False`; `gripper_feasibility: no bilateral finger contact on grasped object`.
  - `place_on_top`: `physicalValidity=True`, matcher FAIL on `initial_state`.
  - `push_object`: `physicalValidity=True`, matcher FAIL on `initial_state`, `terminal_state`, `goal_satisfaction`, `contact_word`; extractor saw `GRASP_LIKELY`, proving current runner incorrectly uses pick-place for push.
  - `open_drawer`: `physicalValidity=False`, matcher FAIL on `articulation_transitions`, `event_presence`, `event_order`, `goal_satisfaction`; current scene uses freejoint/body pose rather than per-frame prismatic articulation.
- 2026-06-12 T1 Developer:
  - Changed `csg/backends/mujoco/runner.py`, `csg/backends/mujoco/scene_mjcf.py`, `tests/test_mujoco_backend.py`.
  - Reported red test after adding pick-family tests: `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `2 failed, 7 passed`.
  - Reported final T1 targeted test: `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `9 passed`.
  - Direct probe reported `put_cube_in_tray`, `insert_object`, `place_on_top` all `physicalValidity=True`, matcher PASS, leakage clean.
- 2026-06-12 T1 main-thread verification:
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `9 passed in 0.79s`.
- 2026-06-12 T1 Auditor:
  - Verdict: `SPEC COMPLIANT`.
  - Reproduced `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `9 passed in 0.78s`.
  - Confirmed put/insert spawn gap `0.050` (NEAR) and place_on spawn gap `0.130` (FAR), with predicate `near_gap_m=0.10`.
- 2026-06-12 T1 Reviewer:
  - No critical issues.
  - Important issues evaluated: dispatch fallback kept because the user design explicitly specified pick-place default; program-aware layout anchoring accepted and fixed; generic pick-place `NEAR`/`ALIGNED_WITH` kept out of scope for full-V0 gold task completion.
- 2026-06-12 T1 follow-up Developer:
  - Changed `csg/backends/mujoco/scene_mjcf.py`, `tests/test_mujoco_dispatch.py`, `tests/test_mujoco_backend.py`.
  - Reported red test before layout fix: `python3 -m pytest tests/test_mujoco_dispatch.py -q` -> `1 failed, 5 passed`.
  - Reported final tests: `python3 -m pytest tests/test_mujoco_dispatch.py -q` -> `6 passed`; `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `9 passed`.
- 2026-06-12 T1 final main-thread verification:
  - `python3 -m pytest tests/test_mujoco_dispatch.py -q` -> `6 passed in 0.02s`.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `9 passed in 0.87s`.
  - `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped in 0.41s`.
- 2026-06-12 T2 Developer:
  - Changed `tests/test_mujoco_backend.py`, `csg/backends/mujoco/runner.py`, `csg/backends/mujoco/scene_mjcf.py`.
  - Reported red push test first: focused push test failed on `_run_push()` `NotImplementedError`.
  - Initial implementation passed behavior tests but used a temporary push-pad geom, which main-thread review rejected as outside the `arm.py` no-structural-change constraint.
- 2026-06-12 T2 main-thread correction:
  - Removed all `push_pad` / `enable_push_pad` references and restored `arm.py` to no push-specific structure.
  - Implemented push using open aperture (`width + 0.0025`) plus push-only contact margin/friction on existing finger geoms.
  - Direct probe: `physicalValidity=True`; relations `['FAR_FROM', 'NEAR']`; contact `TOUCHING_LIKELY` with motion correlation `0.9798`; events `CONTACT_BEGIN`, `HAND_OBJECT_CO_MOTION`; matcher PASS; leakage clean; puck width `0.040`; contact aperture `0.04333..0.04395`; max touch gap `0.011976`.
- 2026-06-12 T2 verification:
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_push_object_end_to_end_passes_with_touching_contact -q` -> `1 passed in 0.30s`.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_push_object_touching_contact_uses_open_aperture_wider_than_puck -q` -> `1 passed in 0.30s`.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `11 passed in 1.18s`.
  - `python3 -m pytest tests/test_mujoco_dispatch.py -q` -> `6 passed in 0.02s`.
  - `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped in 0.42s`.
- 2026-06-12 T2 Auditor:
  - Initial audit caught the push-pad/open-aperture issue.
  - Re-audit verdict after correction: `SPEC COMPLIANT`.
  - Reproduced push focused tests and `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `11 passed in 1.20s`.
- 2026-06-12 T3 Developer:
  - Changed `tests/test_mujoco_backend.py`, `csg/backends/mujoco/scene_mjcf.py`, `csg/backends/mujoco/runner.py`.
  - Reported red open test first: focused open test failed on `_run_open()` `NotImplementedError`.
  - Reported final tests: focused open `1 passed`, `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `12 passed`, `python3 -m pytest tests/test_mujoco_dispatch.py -q` -> `6 passed`.
  - Probe: q start `0.020000`, end `0.189084`, delta `0.169084`; contact `GRASP_LIKELY`; events `CONTACT_BEGIN`, `RELEASE_INFERRED`, `ARTICULATION_CHANGE`; matcher PASS; leakage clean.
- 2026-06-12 T3 main-thread verification:
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_open_drawer_end_to_end_passes_with_articulation -q` -> `1 passed in 0.20s`.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `12 passed in 1.23s`.
  - Direct probe: `physicalValidity=True`; q `0.02 -> 0.1890835475`; contact `GRASP_LIKELY` with motion correlation `0.9615`; events `CONTACT_BEGIN`, `RELEASE_INFERRED`, `ARTICULATION_CHANGE`; matcher PASS; leakage clean.
  - Validity checks: non-penetration, continuity, gripper, reachability, articulation limits applicable/pass; quasi-static support non-applicable/pass.
- 2026-06-12 T3 Auditor:
  - Verdict: `SPEC COMPLIANT`.
  - Reproduced focused open `1 passed`, `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `12 passed`, `.venv-sim/bin/python -m pytest -q` -> `112 passed, 1 skipped`.
- 2026-06-12 T4 Developer:
  - Changed `csg/benchmark.py`, `tests/test_mujoco_backend.py`, `README.md`, `roadmap.md`, `csg/validity.md`.
  - Red CLI check before implementation: `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --out /tmp/wide_robot_mujoco_t4_red` -> exit 2, `unrecognized arguments: --backend mujoco`.
  - Reported final commands:
    - `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped`.
    - `python3 -m csg.benchmark gold_tests --confusion --out /tmp/wide_robot_symbolic_t4` -> `passed=5/5`.
    - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `13 passed`.
    - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --out /tmp/wide_robot_mujoco_t4` -> `passed=5/5`.
  - MuJoCo confusion: failed `0`; all five cases `physicalValidity=true`, leakage clean; off-diagonal passes only `insert_object -> put_cube_in_tray` and `put_cube_in_tray -> insert_object`.
- 2026-06-12 T4 main-thread verification:
  - `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped in 0.50s`.
  - `python3 -m csg.benchmark gold_tests --confusion --out /tmp/wide_robot_symbolic_final` -> `passed=5/5`; expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `13 passed in 1.90s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --out /tmp/wide_robot_mujoco_final` -> `passed=5/5`.
  - `/tmp/wide_robot_mujoco_final/report.json`: every case `PASS`, `physicalValidity=True`, `leakageClean=True`, no hard mismatches; missed diagonal `[]`, unexpected off-diagonal `[]`, off-diagonal passes `[["insert_object","put_cube_in_tray"], ["put_cube_in_tray","insert_object"]]`.
- 2026-06-12 T4 Auditor:
  - Found no code/doc compliance issues.
  - Reported `ISSUES FOUND` only because this directory is not a Git repository, so frozen-file untouched status cannot be verified by VCS diff/SHA.
  - Reproduced: `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped`; symbolic benchmark `passed=5/5`; `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q` -> `13 passed`; MuJoCo benchmark `passed=5/5` with all `physicalValidity=true`, leakage clean, expected off-diagonal only.
- 2026-06-12 final verification:
  - `python3 -m pytest tests/ -q` -> `101 passed, 1 skipped in 0.45s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_require_final` -> exit 0, `passed=5/5`; all cases `physicalValidity=None`, leakage clean; off-diagonal passes only insert/put both directions.
  - `.venv-sim/bin/python -m pytest tests/ -q` -> `113 passed, 1 skipped in 2.25s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_require_final` -> exit 0, `passed=5/5`; every case `physicalValidity=True`, leakage clean, no hard mismatches; off-diagonal passes only insert/put both directions.
- 2026-06-12 roadmap follow-up:
  - Updated `roadmap.md` claim discipline to make Phase 2C gold-task MuJoCo coverage an allowed-now claim for all five V0 gold tasks.
  - Clarified remaining Phase 2C/2D work as 30 seeded randomized rollouts/task plus frozen physically-invalid gold fixtures.
- 2026-06-12 2E continuation start:
  - Main-thread audit confirmed no `.git` directory, no `LICENSE`, no
    `--randomized`/`--seeds` benchmark CLI, no explicit benchmark failure
    classification, no frozen MuJoCo invalid fixtures, and no baseline comparison
    mode.
  - Planner sidecar (read-only) independently reported the same gaps and
    identified the seeded randomized MuJoCo harness as the highest-leverage next
    implementation slice because `SolverConfig.seed` and `build_arm_scene_xml(...,
    seed=...)` already exist but the seed is not used.
  - Auditor sidecar (read-only) confirmed: `seed` is threaded from
    `SolverConfig` to the MuJoCo scene builder but has no behavioral effect;
    only MuJoCo sabotage mode is `early_release`; baseline variants are limited
    to backend selection (`symbolic`/`mujoco`); validity sidecars are written by
    `solve_to_files` but not by `csg.benchmark`; benchmark reports do not yet
    normalize failures into taxonomy classes.
  - Developer worker dispatched for 2E-1 failure taxonomy with write scope limited
    to `csg/benchmark.py` and benchmark-report tests.
  - Added resumable execution plan:
    `docs/superpowers/plans/2026-06-12-phase-2e-sim-only-benchmark.md`.
- 2026-06-12 2E-1 failure taxonomy:
  - Developer worker changed `csg/benchmark.py` and added
    `tests/test_benchmark_failure_classification.py`.
  - Initial spec review found two issues: `passed=True` without `status=PASS`
    could classify as `passed`, and `initial_state`/`terminal_state` were mapped
    too broadly to `relation_not_achieved`. Main thread fixed both and added
    regression assertions.
  - Code-quality review found soft-only `probeAgreement=False` could drive a
    primary class. Main thread changed classification to use only hard mismatch
    probes for verifier categories and added a soft-only regression assertion.
  - Verification after fixes:
    `python3 -m pytest tests/test_benchmark_failure_classification.py -q` ->
    `2 passed in 0.04s`.
  - Verification after fixes:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_loop.py tests/test_validity.py tests/test_confusion.py -q` ->
    `22 passed in 0.28s`.
  - Verification after fixes:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_taxonomy_symbolic_quality_fix` ->
    exit 0, `passed=5/5`, expected insert/put off-diagonal only.
  - Generated `/tmp/wide_robot_taxonomy_symbolic_quality_fix/failure_classification.json`
    exists with schema `csg.benchmark_failure_classification.v1`, summary
    `{"passed": 5}`, and all five gold cases categorized as `passed`.
  - Spec re-review after first fixes: `SPEC COMPLIANT`.
  - Code-quality re-review after hard-mismatch-only fix: `APPROVED`; reviewer
    confirmed soft-only `probeAgreement=False` cannot drive the primary class.
- 2026-06-12 2E-2 seeded randomized benchmark plumbing
  (historical, superseded by the later push randomized-start retuning entry):
  - Added conservative seeded layout sampling in
    `csg/backends/mujoco/scene_mjcf.py`: non-push tasks got a small shared
    y-translation; at this point push returned the calibrated layout because
    sub-millimeter x/y perturbations caused contact-word or terminal-goal
    failures. Later push retuning replaced this with shared x jitter.
  - Added MuJoCo `sampled_layout` propagation through `SimResult` and rollout
    diagnostics when `SolverConfig.seed` is set.
  - Added `csg.benchmark --randomized --seeds N`, unique per-seed case names,
    seed/base-case/solver-config/sampled-layout report fields, base-case-aware
    confusion expectations, and compact CLI confusion output for large
    randomized matrices.
  - Added MuJoCo-gated tests for same-seed determinism, different seeded
    layouts, and all-five-task randomized smoke.
  - Intermediate evidence: y offsets caused push failures; x-only push offsets
    still failed at seeds 13 and 25 for `contact_word`/event probes, so push
    randomized starts are explicitly left for controller retuning.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_seeded_layouts_are_reproducible_and_distinct tests/test_mujoco_backend.py::test_seeded_rollouts_same_seed_are_deterministic tests/test_mujoco_backend.py::test_benchmark_mujoco_randomized_seeded_smoke -q` ->
    `3 passed in 2.81s`.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_confusion.py tests/test_validity.py -q` ->
    `11 passed in 0.22s`.
  - Verification:
    `python3 -m pytest tests/ -q` -> `103 passed, 1 skipped in 0.51s`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/ -q` -> `118 passed, 1 skipped in 5.18s`.
  - Verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_taxonomy_randomized_work` ->
    exit 0, `passed=5/5`, expected insert/put off-diagonal only.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_taxonomy_randomized_work` ->
    exit 0, `passed=5/5`, expected insert/put off-diagonal only.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests/push_object --backend mujoco --randomized --seeds 30 --require-pass --out /tmp/wide_robot_push_randomized_30_calibrated` ->
    exit 0, `passed=30/30`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_calibrated` ->
    exit 0. Report JSON summary: `passed=150/150`, `physicalValidity=True`
    for all 150 cases, leakage clean for all 150 cases, failure classification
    `{"passed": 150}`, no missed diagonal, no unexpected off-diagonal passes.
    Distinct sampled layouts at this historical checkpoint: 30 each for
    insert/open/place/put; 1 for push pending the later randomized-push
    retuning.
  - Docs updated: `README.md`, `roadmap.md`, `csg/validity.md`.
- 2026-06-12 final verification after CLI/docs edits:
  - `python3 -m pytest tests/ -q` -> `103 passed, 1 skipped in 0.50s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_phase2e_final` ->
    exit 0, `passed=5/5`, expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` -> `118 passed, 1 skipped in 5.14s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_phase2e_final` ->
    exit 0, `passed=5/5`; report JSON shows every case
    `physicalValidity=True`, leakage clean, and no unexpected off-diagonal
    passes.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_final` ->
    exit 0, `passed=150/150`; report JSON shows seeds 0..29, 30 cases per
    base task, `physicalValidity=True` for all 150, leakage clean for all 150,
    failure classification `{"passed": 150}`, no missed diagonal, no unexpected
    off-diagonal passes, and `failure_classification.json` schema
    `csg.benchmark_failure_classification.v1`.
- 2026-06-12 2E baseline comparison:
  - Added `run_benchmark_comparison()` and CLI `--compare-backends` in
    `csg/benchmark.py`; comparison reports write `comparison_report.json` plus
    per-baseline benchmark outputs.
  - Added core comparison test `tests/test_benchmark_comparison.py`.
  - Added MuJoCo-gated comparison test comparing symbolic baseline against the
    scripted MuJoCo solver in `tests/test_mujoco_backend.py`.
  - Red evidence before implementation:
    `python3 -m pytest tests/test_benchmark_comparison.py -q` failed with
    `ImportError: cannot import name 'run_benchmark_comparison'`.
  - Red evidence before implementation:
    `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_benchmark_comparison_symbolic_baseline_vs_scripted_mujoco -q`
    failed with the same missing import.
  - Verification:
    `python3 -m pytest tests/test_benchmark_comparison.py -q` ->
    `1 passed in 0.07s`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_benchmark_comparison_symbolic_baseline_vs_scripted_mujoco -q` ->
    `1 passed in 0.90s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_symbolic_mujoco` ->
    exit 0. `comparison_report.json` schema `csg.benchmark_comparison.v1`;
    `symbolic` passed `5/5` with physical validity `{"unverified": 5}`;
    `mujoco` passed `5/5` with physical validity `{"valid": 5}`; both had
    failure classification `{"passed": 5}` and no unexpected off-diagonal
    confusion.
  - Docs updated: `README.md`, `roadmap.md`.
- 2026-06-12 release hygiene progress:
  - Confirmed this workspace still has no `.git` directory.
  - Expanded `.gitignore` from only `.venv-sim` to cover local Python envs,
    `__pycache__`, `.pytest_cache`, coverage output, benchmark/solver output
    directories, logs, `.DS_Store`, and common editor directories.
  - Added `docs/release_checklist.md` with required metadata, reproduction
    commands, report artifacts, and known non-release gaps.
  - Updated `README.md` and `roadmap.md` to point at the release checklist and
    keep license selection explicit as a project-owner decision.
  - Remaining for this workstream: initialize/restore Git, select and commit a
    license, and add matching `pyproject.toml` license metadata.
- 2026-06-12 final verification after baseline comparison:
  - `python3 -m pytest tests/ -q` -> `104 passed, 1 skipped in 0.45s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_baseline_compare_final` ->
    exit 0, `passed=5/5`, expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` -> `120 passed, 1 skipped in 5.51s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_baseline_compare_final` ->
    exit 0, `passed=5/5`; report JSON shows every case
    `physicalValidity=True`, leakage clean, and no unexpected off-diagonal
    passes.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_baseline_final` ->
    exit 0; comparison schema `csg.benchmark_comparison.v1`, order
    `["symbolic", "mujoco"]`, symbolic `5/5` with `{"unverified": 5}`,
    MuJoCo `5/5` with `{"valid": 5}`, no unexpected off-diagonal passes.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_baseline_compare_final` ->
    exit 0, `passed=150/150`; report JSON shows 30 cases per base task,
    `physicalValidity=True` for all 150, leakage clean for all 150, failure
    classification `{"passed": 150}`, no missed diagonal, no unexpected
    off-diagonal passes.
- 2026-06-12 invalid physical fixture:
  - Added frozen invalid fixture manifest
    `gold_invalid/put_cube_in_tray_early_release.json`. It targets
    `gold_tests/put_cube_in_tray/target.json`, runs MuJoCo with
    `sabotage=early_release`, and expects `physicalValidity=false`, failure
    category `physical_invalidity`, and failed validity check
    `quasi_static_support_at_release`.
  - Added `sabotage` to `SolverConfig` so invalid fixtures flow through
    `solve()` and the normal benchmark extraction, matcher, leakage, and
    validity path.
  - Added `run_invalid_fixtures()` and CLI `--invalid-fixtures` in
    `csg/benchmark.py`; report schema is
    `csg.invalid_fixture_report.v1`.
  - Added MuJoCo-gated test `tests/test_invalid_fixtures.py`.
  - Red evidence before implementation:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q`
    failed with `ImportError: cannot import name 'run_invalid_fixtures'`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    `1 passed in 0.14s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_final` ->
    exit 0, `matched=1/1`; report JSON shows fixture
    `put_cube_in_tray__early_release`, `expectedFailureMatched=true`,
    `status=FAIL`, `physicalValidity=false`, failure category
    `physical_invalidity`, and failed validity checks
    `["pose_continuity", "quasi_static_support_at_release"]`.
  - Docs updated: `README.md`, `roadmap.md`, and
    `docs/release_checklist.md`.
- 2026-06-12 final verification after invalid-fixture/docs update:
  - `python3 -m pytest tests/ -q` ->
    `104 passed, 2 skipped in 0.48s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_invalid_fixture_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` ->
    `121 passed, 1 skipped in 5.84s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_invalid_fixture_final` ->
    exit 0, `passed=5/5`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_invalid_fixture_final` ->
    exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_invalid_fixture_final` ->
    exit 0; comparison report schema `csg.benchmark_comparison.v1`;
    symbolic `5/5` with physical validity `{"unverified": 5}`; MuJoCo
    `5/5` with physical validity `{"valid": 5}`; no unexpected
    off-diagonal passes.
  - `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_post_docs_final` ->
    exit 0, `matched=1/1`.
  - JSON audit:
    `/tmp/wide_robot_symbolic_invalid_fixture_final/report.json` schema
    `csg.benchmark_report.v2`, `5/5` PASS, physical validity
    `{"None": 5}`, leakage clean for all, failure classes `{"passed": 5}`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_invalid_fixture_final/report.json` schema
    `csg.benchmark_report.v2`, `5/5` PASS, physical validity
    `{"True": 5}`, leakage clean for all, failure classes `{"passed": 5}`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_randomized_30_invalid_fixture_final/report.json`
    schema `csg.benchmark_report.v2`, `150/150` PASS, seeds `0..29`,
    30 cases per base task, physical validity `{"True": 150}`, leakage clean
    for all, failure classes `{"passed": 150}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`.
  - JSON audit:
    `/tmp/wide_robot_invalid_fixtures_post_docs_final/invalid_fixtures_report.json`
    schema `csg.invalid_fixture_report.v1`, summary
    `{"matched": 1, "mismatched": 0, "total": 1}`; fixture
    `put_cube_in_tray__early_release` matched expected failure with
    `status=FAIL`, `physicalValidity=false`, and failed checks
    `["pose_continuity", "quasi_static_support_at_release"]`.
- 2026-06-12 expanded frozen invalid fixture matrix:
  - Red evidence before implementation:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    failed because the test expected six fixtures but the report still had
    `{"matched": 1, "mismatched": 0, "total": 1}`.
  - Added five fixture manifests:
    `gold_invalid/open_drawer_overlimit_articulation.json`,
    `gold_invalid/place_on_top_penetrate_goal.json`,
    `gold_invalid/put_cube_in_tray_impossible_reach.json`,
    `gold_invalid/put_cube_in_tray_teleport_after_release.json`, and
    `gold_invalid/put_cube_in_tray_wide_grasp.json`.
  - Added MuJoCo sabotage modes in `csg/backends/mujoco/runner.py`:
    `wide_grasp`, `impossible_reach`, `teleport_after_release`,
    `penetrate_goal`, and `overlimit_articulation`. These are reachable only
    through explicit fixture `solverConfig.sabotage` values.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    `1 passed in 0.57s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_expanded_tuned` ->
    exit 0, `matched=6/6`.
  - Expanded fixture report:
    `open_drawer__overlimit_articulation` -> `["articulation_limits"]`;
    `place_on_top__penetrate_goal` ->
    `["non_penetration", "pose_continuity"]`;
    `put_cube_in_tray__early_release` ->
    `["pose_continuity", "quasi_static_support_at_release"]`;
    `put_cube_in_tray__impossible_reach` ->
    `["workspace_reachability"]`;
    `put_cube_in_tray__teleport_after_release` ->
    `["pose_continuity"]`;
    `put_cube_in_tray__wide_grasp` -> `["gripper_feasibility"]`.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_confusion.py tests/test_validity.py -q` ->
    `11 passed in 0.22s`.
  - Docs updated: `README.md`, `roadmap.md`, and
    `docs/release_checklist.md` now distinguish completed physical-invalidity
    fixture coverage from remaining semantic-invalid fixture coverage.
  - Auditor sidecar reviewed the fixture matrix and flagged the intended
    non-exclusivity contract: fixtures assert that the expected failed check is
    present, not that it is the only failed check.
- 2026-06-12 final verification after expanded invalid fixtures:
  - `python3 -m pytest tests/ -q` ->
    `104 passed, 2 skipped in 0.44s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_expanded_invalid_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` ->
    `121 passed, 1 skipped in 6.27s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_expanded_invalid_final` ->
    exit 0, `passed=5/5`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_expanded_invalid_final` ->
    exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_expanded_invalid_final` ->
    exit 0; symbolic `5/5` with physical validity `{"unverified": 5}`;
    MuJoCo `5/5` with physical validity `{"valid": 5}`; no unexpected
    off-diagonal passes.
  - `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_expanded_final` ->
    exit 0, `matched=6/6`.
  - JSON audit:
    `/tmp/wide_robot_symbolic_expanded_invalid_final/report.json` schema
    `csg.benchmark_report.v2`, `5/5` PASS, physical validity
    `{"None": 5}`, leakage clean for all, failure classes `{"passed": 5}`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_expanded_invalid_final/report.json` schema
    `csg.benchmark_report.v2`, `5/5` PASS, physical validity
    `{"True": 5}`, leakage clean for all, failure classes `{"passed": 5}`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_randomized_30_expanded_invalid_final/report.json`
    schema `csg.benchmark_report.v2`, `150/150` PASS, seeds `0..29`,
    30 cases per base task, physical validity `{"True": 150}`, leakage clean
    for all, failure classes `{"passed": 150}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`.
  - JSON audit:
    `/tmp/wide_robot_invalid_fixtures_expanded_final/invalid_fixtures_report.json`
    schema `csg.invalid_fixture_report.v1`, summary
    `{"matched": 6, "mismatched": 0, "total": 6}`; every fixture matched its
    expected failure and reported `physicalValidity=false`.
- 2026-06-12 semantic invalid fixtures:
  - Red evidence before implementation:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    failed because the expanded test expected 9 fixtures but the report still
    had `{"matched": 6, "mismatched": 0, "total": 6}`.
  - Added three semantic fixtures:
    `gold_invalid/push_object_missing_contact.json`,
    `gold_invalid/place_on_top_wrong_relation.json`, and
    `gold_invalid/place_on_top_wrong_event_order.json`.
  - Added MuJoCo sabotage modes in `csg/backends/mujoco/runner.py` for
    `push_missing_contact` and `wrong_relation`. `wrong_relation` first
    creates the correct support event, then moves the cube to a nearby table
    pose before release so event order remains reproducible while terminal
    goal satisfaction fails.
  - Added benchmark-only `targetMutation: release_before_relation` support in
    `csg/benchmark.py` for semantic event-order fixtures. The mutated target is
    generated under the fixture output directory; normal gold targets and the
    frozen verifier are unchanged.
  - Intermediate failure after first implementation:
    the report had `{"matched": 7, "mismatched": 2, "total": 9}` because
    `wrong_event_order` caused `quasi_static_support_at_release` physical
    invalidity and `wrong_relation` returned the cube too close to its initial
    pose, so extraction emitted no contact/events. Root causes were inspected
    from the fixture `robot_csg.json`, `matcher_report.json`, and
    `validity_report.json`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    `1 passed in 0.80s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_semantic` ->
    exit 0, `matched=9/9`.
  - JSON audit of `/tmp/wide_robot_invalid_fixtures_semantic/invalid_fixtures_report.json`:
    schema `csg.invalid_fixture_report.v1`; summary
    `{"matched": 9, "mismatched": 0, "total": 9}`; failure classes
    `{"physical_invalidity": 6, "event_order_wrong": 1,
    "relation_not_achieved": 1, "contact_missing": 1}`; physical validity
    counts `{"False": 6, "True": 3}`.
  - Docs updated: `README.md`, `roadmap.md`, and
    `docs/release_checklist.md` now describe a nine-fixture invalid suite
    covering both physical-validity checks and semantic verifier failures.
- 2026-06-12 final verification after semantic invalid fixtures:
  - `python3 -m pytest tests/ -q` ->
    `104 passed, 2 skipped in 0.44s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_semantic_invalid_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` ->
    `121 passed, 1 skipped in 6.33s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_semantic_invalid_final` ->
    exit 0, `passed=5/5`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_semantic_invalid_final` ->
    exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_semantic_invalid_final` ->
    exit 0; symbolic `5/5` with physical validity `{"unverified": 5}`;
    MuJoCo `5/5` with physical validity `{"valid": 5}`; no unexpected
    off-diagonal passes.
  - `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_semantic_final` ->
    exit 0, `matched=9/9`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_randomized_30_semantic_invalid_final/report.json`
    schema `csg.benchmark_report.v2`, `150/150` PASS, seeds `0..29`,
    30 cases per base task, physical validity `{"True": 150}`, leakage clean
    for all, failure classes `{"passed": 150}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`.
  - JSON audit:
    `/tmp/wide_robot_invalid_fixtures_semantic_final/invalid_fixtures_report.json`
    schema `csg.invalid_fixture_report.v1`, summary
    `{"matched": 9, "mismatched": 0, "total": 9}`; classes
    `{"physical_invalidity": 6, "event_order_wrong": 1,
    "relation_not_achieved": 1, "contact_missing": 1}`.
  - Post test-assertion cleanup verification:
    `.venv-sim/bin/python -m pytest tests/test_invalid_fixtures.py -q` ->
    `1 passed in 0.80s`.
  - Post test-assertion cleanup verification:
    `python3 -m pytest tests/ -q` -> `104 passed, 2 skipped in 0.45s`;
    `.venv-sim/bin/python -m pytest tests/ -q` ->
    `121 passed, 1 skipped in 6.38s`.
- 2026-06-12 push randomized-start retuning:
  - Added red MuJoCo-gated test
    `tests/test_mujoco_backend.py::test_benchmark_push_randomized_seeds_sample_distinct_layouts`.
    Red evidence:
    `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_benchmark_push_randomized_seeds_sample_distinct_layouts -q` ->
    failed because all four push seeds shared one sampled layout.
  - Initial shared-y push sampling reproduced the historical instability:
    `.venv-sim/bin/python -m csg.benchmark gold_tests/push_object --backend mujoco --randomized --seeds 30 --require-pass --out /tmp/wide_robot_push_randomized_retune_probe` ->
    `passed=21/30`. Failures were contact-word drops, terminal NEAR misses,
    and marginal workspace/continuity physical invalidity.
  - Systematic debugging evidence: even y shifts below 0.25 mm could drop the
    non-grasp contact word. A shared x translation preserved the calibrated
    lateral contact line but still changed tabletop start positions.
  - Retuned push controller:
    `csg/backends/mujoco/scene_mjcf.py` now applies push-specific shared x
    jitter; `csg/backends/mujoco/runner.py` uses a slightly deeper terminal
    gap, a 0.8 mm forward site bias, and a slower 10 s push segment so the
    grasp site remains inside the 12 mm touch band during moving co-motion.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py::test_benchmark_push_randomized_seeds_sample_distinct_layouts -q` ->
    `1 passed in 1.10s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests/push_object --backend mujoco --randomized --seeds 30 --require-pass --out /tmp/wide_robot_push_randomized_xjitter_bias08_slow_probe` ->
    exit 0, `passed=30/30`. JSON audit: 30 distinct push sampled layouts,
    physical validity `{"True": 30}`, leakage clean for all, classes
    `{"passed": 30}`.
  - Docs updated: `README.md`, `roadmap.md`, `csg/validity.md`, and
    `docs/release_checklist.md` no longer carry the calibrated-push caveat.
- 2026-06-12 final verification after push randomized-start retuning:
  - `python3 -m pytest tests/ -q` ->
    `104 passed, 2 skipped in 0.48s`.
  - `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_push_randomized_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - `.venv-sim/bin/python -m pytest tests/ -q` ->
    `122 passed, 1 skipped in 7.47s`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_push_randomized_final` ->
    exit 0, `passed=5/5`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_push_randomized_final` ->
    exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_push_randomized_final` ->
    exit 0; symbolic `5/5` with physical validity `{"unverified": 5}`;
    MuJoCo `5/5` with physical validity `{"valid": 5}`; no unexpected
    off-diagonal passes.
  - `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_push_randomized_final` ->
    exit 0, `matched=9/9`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_randomized_30_push_randomized_final/report.json`
    schema `csg.benchmark_report.v2`, `150/150` PASS, seeds `0..29`,
    30 cases per base task, distinct sampled layouts
    `{"insert_object": 30, "open_drawer": 30, "place_on_top": 30,
    "push_object": 30, "put_cube_in_tray": 30}`, physical validity
    `{"True": 150}`, leakage clean for all, failure classes
    `{"passed": 150}`.
  - JSON audit:
    `/tmp/wide_robot_invalid_fixtures_push_randomized_final/invalid_fixtures_report.json`
    schema `csg.invalid_fixture_report.v1`, summary
    `{"matched": 9, "mismatched": 0, "total": 9}`; classes
    `{"physical_invalidity": 6, "event_order_wrong": 1,
    "relation_not_achieved": 1, "contact_missing": 1}`.
- 2026-06-12 package metadata release-hygiene slice:
  - Added red metadata test
    `tests/test_package_metadata.py::test_pyproject_exposes_command_line_entry_points`.
    Red evidence:
    `python3 -m pytest tests/test_package_metadata.py -q` -> failed with
    `KeyError: 'scripts'`.
  - Added `[project.scripts]` entries in `pyproject.toml` for the existing
    module CLIs: `csg-benchmark`, `csg-solver`, `csg-to-sim`,
    `csg-rollout-extract`, `csg-matcher`, and `csg-skills`.
  - Verification:
    `python3 -m pytest tests/test_package_metadata.py -q` ->
    `1 passed in 0.00s`.
  - Docs updated: `README.md`, `roadmap.md`, and
    `docs/release_checklist.md` now record package console-script coverage
    while leaving Git provenance and owner-selected license metadata as open
    release gates.
- 2026-06-12 release-hygiene docs/comment cleanup and final verification:
  - Auditor sidecar found stale wording in README/roadmap/MuJoCo comments:
    missing `report.json` in the README artifact list, `csg/validity.md`
    described as a future deliverable, failure taxonomy described as future
    work, and MuJoCo seeded-layout/runner comments still implying pick-place or
    y-only sampling. These were corrected without changing runtime behavior.
  - Text audit: the auditor-reported stale phrases for future failure
    taxonomy, future validity deliverable, old y-only seeded-layout wording,
    pick-place-only runner wording, invalid-fixture freeze work, pending
    push-retune caveats, and missing console-entry metadata no longer appear in
    current docs/code.
  - Verification:
    `python3 -m pytest tests/ -q` ->
    `105 passed, 2 skipped in 0.43s`.
  - Verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_release_hygiene_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/ -q` ->
    `123 passed, 1 skipped in 7.93s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_release_hygiene_final` ->
    exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_release_hygiene_final` ->
    exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_release_hygiene_final` ->
    exit 0; symbolic `5/5` with physical validity `{"unverified": 5}`;
    MuJoCo `5/5` with physical validity `{"valid": 5}`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_release_hygiene_final` ->
    exit 0, `matched=9/9`.
  - JSON audit:
    `/tmp/wide_robot_mujoco_randomized_30_release_hygiene_final/report.json`
    schema `csg.benchmark_report.v2`, `150/150` PASS, distinct sampled
    layouts
    `{"insert_object": 30, "open_drawer": 30, "place_on_top": 30,
    "push_object": 30, "put_cube_in_tray": 30}`, per-case
    `physicalValidity=True` for all 150, leakage clean for all 150, failure
    classes `{"passed": 150}`.
  - JSON audit:
    `/tmp/wide_robot_compare_release_hygiene_final/comparison_report.json`
    schema `csg.benchmark_comparison.v1`, baselines `["symbolic", "mujoco"]`,
    each `5/5`; symbolic physical validity `{"unverified": 5}`, MuJoCo
    physical validity `{"valid": 5}`.
  - JSON audit:
    `/tmp/wide_robot_invalid_fixtures_release_hygiene_final/invalid_fixtures_report.json`
    schema `csg.invalid_fixture_report.v1`, summary
    `{"matched": 9, "mismatched": 0, "total": 9}`; classes
    `{"physical_invalidity": 6, "event_order_wrong": 1,
    "relation_not_achieved": 1, "contact_missing": 1}`; physical validity
    counts `{"False": 6, "True": 3}`.
- 2026-06-12 source-provenance report slice:
  - Added red tests for report-level source provenance:
    `python3 -m pytest tests/test_benchmark_failure_classification.py::test_run_benchmark_records_source_provenance tests/test_benchmark_comparison.py::test_benchmark_comparison_groups_baselines_and_writes_report -q`
    -> failed with missing `sourceProvenance` keys in `report.json` and
    `comparison_report.json`.
  - Implemented `csg.source_provenance.v1` in `csg/benchmark.py`: reports now
    include Git commit/status when `.git` exists and always include a
    deterministic SHA-256 source snapshot over package, test, docs, gold, and
    invalid-fixture source files.
  - Added provenance to `report.json`, `report.md`,
    `failure_classification.json`, `comparison_report.json`, and
    `invalid_fixtures_report.json`.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py tests/test_invalid_fixtures.py -q`
    -> `4 passed, 1 skipped in 0.12s`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py tests/test_invalid_fixtures.py -q`
    -> `5 passed in 0.95s`.
  - Verification:
    `python3 -m pytest tests/ -q` ->
    `106 passed, 2 skipped in 0.50s`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/ -q` ->
    `124 passed, 1 skipped in 7.91s`.
  - Verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_source_provenance_final`
    -> exit 0, `passed=5/5`; expected insert/put off-diagonal only.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_source_provenance_final`
    -> exit 0, `passed=5/5`, all cases `physicalValidity=True`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_source_provenance_retry`
    -> exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`; JSON audit found `sourceProvenance` schema
    `csg.source_provenance.v1`, source snapshot digest length `64`,
    75 included source files, per-case `physicalValidity=True` for all 150,
    and 30 distinct sampled layouts for every V0 task.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --compare-backends symbolic,mujoco --confusion --require-pass --out /tmp/wide_robot_compare_source_provenance_final`
    -> exit 0; `comparison_report.json` has `sourceProvenance`, symbolic
    `5/5` with `{"unverified": 5}`, MuJoCo `5/5` with `{"valid": 5}`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid --require-pass --out /tmp/wide_robot_invalid_fixtures_source_provenance_final`
    -> exit 0, `matched=9/9`; `invalid_fixtures_report.json` has
    `sourceProvenance` and classes `{"physical_invalidity": 6,
    "event_order_wrong": 1, "relation_not_achieved": 1,
    "contact_missing": 1}`.
- 2026-06-12 sim-only benchmark report draft:
  - Added `docs/sim_only_benchmark_report.md` as a release-candidate draft
    covering claim boundaries, task set, evaluation pipeline, physical-validity
    checks, reproduction commands, expected counts, report artifacts,
    `sourceProvenance`, invalid fixtures, failure taxonomy, baseline
    comparison, limitations, and remaining release gates.
  - Updated `README.md`, `roadmap.md`, and `docs/release_checklist.md` to link
    the draft report while explicitly stating that final public release still
    requires Git-backed clean-checkout evidence and owner-selected license
    metadata.
  - Text audit:
    `rg -n "Produce a tagged V0 sim-only release|no report|report draft|sim_only_benchmark_report|Benchmark release artifact|Final public benchmark" README.md roadmap.md docs/release_checklist.md docs/sim_only_benchmark_report.md state.md`
    -> expected links/status lines only; no stale "missing report" claim.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py tests/test_package_metadata.py -q`
    -> `5 passed in 0.11s`.
  - Verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_report_draft_check`
    -> exit 0, `passed=5/5`; expected insert/put off-diagonal only.
- 2026-06-13 non-license package metadata cleanup:
  - Added red package metadata test:
    `python3 -m pytest tests/test_package_metadata.py -q` -> failed because
    `pyproject.toml` lacked `project.readme`.
  - Added `readme = "README.md"`, package keywords, and neutral classifiers to
    `pyproject.toml` while keeping license metadata absent until the project
    owner chooses a license.
  - Updated `README.md`, `roadmap.md`, `docs/release_checklist.md`,
    `docs/sim_only_benchmark_report.md`, and this state file to distinguish
    completed non-license package metadata from the remaining owner-selected
    license gate.
  - Verification:
    `python3 -m pytest tests/test_package_metadata.py -q` ->
    `2 passed in 0.00s`.
  - Focused verification:
    `python3 -m pytest tests/test_package_metadata.py tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py -q`
    -> `6 passed in 0.11s`.
  - Focused verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_package_metadata_check`
    -> exit 0, `passed=5/5`; expected insert/put off-diagonal only.
- 2026-06-13 report summary auditability:
  - Added red report test requiring normal benchmark summaries and
    `failure_classification.json` sidecars to include physical-validity and
    leakage clean/dirty counts. Red evidence:
    `python3 -m pytest tests/test_benchmark_failure_classification.py::test_run_benchmark_writes_failure_classification_sidecar -q`
    -> failed with missing `summary["physicalValidity"]`.
  - Added top-level `summary.physicalValidity` and `summary.leakage` to normal
    benchmark reports, plus `physicalValiditySummary` and `leakageSummary` to
    `failure_classification.json`; `report.md` now prints those counts near the
    source provenance line.
  - Updated `README.md`, `docs/sim_only_benchmark_report.md`, and
    `docs/release_checklist.md` to document the summary-count contract.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py::test_run_benchmark_writes_failure_classification_sidecar -q`
    -> `1 passed in 0.05s`.
  - Verification:
    `python3 -m pytest tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py -q`
    -> `4 passed in 0.11s`.
  - Verification:
    `python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_symbolic_summary_audit`
    -> exit 0, `passed=5/5`; JSON audit:
    `summary.failureClassification={"passed": 5}`,
    `summary.physicalValidity={"unverified": 5}`, and
    `summary.leakage={"clean": 5, "dirty": 0}`.
  - Verification:
    `python3 -m pytest tests/ -q` ->
    `107 passed, 2 skipped in 0.74s`.
  - Verification:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass --out /tmp/wide_robot_mujoco_summary_audit`
    -> exit 0, `passed=5/5`; JSON audit:
    `summary.failureClassification={"passed": 5}`,
    `summary.physicalValidity={"valid": 5}`,
    `summary.leakage={"clean": 5, "dirty": 0}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/test_benchmark_failure_classification.py tests/test_benchmark_comparison.py tests/test_invalid_fixtures.py -q`
    -> `5 passed in 1.18s`.
- 2026-06-13 release artifact audit CLI:
  - Added red tests for `audit_release_artifacts()`: initial evidence
    `python3 -m pytest tests/test_release_audit.py -q` failed during collection
    with `ModuleNotFoundError: No module named 'csg.release_audit'`.
  - Added `csg/release_audit.py`, a stdlib release artifact validator for the
    five generated output directories: symbolic benchmark, MuJoCo benchmark,
    randomized MuJoCo benchmark, symbolic-vs-MuJoCo comparison, and invalid
    fixtures.
  - The audit checks required files, report schemas, `sourceProvenance`, pass
    counts, physical-validity summaries, leakage summaries, confusion
    unexpected/missed diagonal counts, randomized seed/layout coverage,
    comparison summaries, and invalid-fixture failure categories.
  - Added console script `csg-release-audit` and documented
    `python3 -m csg.release_audit` in the release checklist/report draft.
  - Verification:
    `python3 -m pytest tests/test_release_audit.py tests/test_package_metadata.py -q`
    -> `4 passed in 0.02s`.
  - First audit against older randomized output correctly failed:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_symbolic_summary_audit --mujoco /tmp/wide_robot_mujoco_summary_audit --randomized /tmp/wide_robot_mujoco_randomized_30_source_provenance_retry --comparison /tmp/wide_robot_compare_source_provenance_final --invalid-fixtures /tmp/wide_robot_invalid_fixtures_source_provenance_final`
    -> exit 2 because the older randomized report predated
    `summary.physicalValidity` and `summary.leakage`.
  - Regenerated randomized evidence:
    `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass --out /tmp/wide_robot_mujoco_randomized_30_summary_audit`
    -> exit 0, `passed=150/150`, missed diagonal `0`, unexpected
    off-diagonal passes `0`.
  - Release artifact audit:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_symbolic_summary_audit --mujoco /tmp/wide_robot_mujoco_summary_audit --randomized /tmp/wide_robot_mujoco_randomized_30_summary_audit --comparison /tmp/wide_robot_compare_source_provenance_final --invalid-fixtures /tmp/wide_robot_invalid_fixtures_source_provenance_final`
    -> exit 0, `release audit ok=True checks=71/71`.
  - JSON audit of regenerated randomized report:
    `summary.physicalValidity={"valid": 150}`,
    `summary.leakage={"clean": 150, "dirty": 0}`,
    `summary.failureClassification={"passed": 150}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`, and distinct sampled layouts
    `{"insert_object": 30, "open_drawer": 30, "place_on_top": 30,
    "push_object": 30, "put_cube_in_tray": 30}`.
  - Focused verification:
    `python3 -m pytest tests/test_release_audit.py tests/test_package_metadata.py tests/test_benchmark_failure_classification.py -q`
    -> `7 passed in 0.07s`.
- 2026-06-13 strict release metadata audit mode:
  - Added red tests for `require_final_metadata=True`: initial evidence
    `python3 -m pytest tests/test_release_audit.py -q` failed because
    `audit_release_artifacts()` did not accept the `require_final_metadata`
    argument.
  - Added strict audit mode to `csg.release_audit`: when enabled, it requires
    every report to carry Git-backed `sourceProvenance` and requires `.git`,
    `LICENSE`, plus `pyproject.toml` license metadata under the project root.
  - Verification:
    `python3 -m pytest tests/test_release_audit.py -q` ->
    `4 passed in 0.02s`.
  - Normal release audit remains green:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_symbolic_summary_audit --mujoco /tmp/wide_robot_mujoco_summary_audit --randomized /tmp/wide_robot_mujoco_randomized_30_summary_audit --comparison /tmp/wide_robot_compare_source_provenance_final --invalid-fixtures /tmp/wide_robot_invalid_fixtures_source_provenance_final`
    -> exit 0, `release audit ok=True checks=71/71`.
  - Strict final-metadata audit correctly failed in this workspace at this
    checkpoint (superseded by the later `.git` final-metadata gate):
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_symbolic_summary_audit --mujoco /tmp/wide_robot_mujoco_summary_audit --randomized /tmp/wide_robot_mujoco_randomized_30_summary_audit --comparison /tmp/wide_robot_compare_source_provenance_final --invalid-fixtures /tmp/wide_robot_invalid_fixtures_source_provenance_final --require-final-metadata --project-root .`
    -> exit 2, `release audit ok=False checks=71/78`, failing exactly the
    five source-provenance Git checks plus missing `LICENSE` and missing
    `pyproject.toml` license metadata.
- 2026-06-13 stricter `.git` final-metadata audit:
  - Added red assertion that strict final metadata requires a `.git` directory
    under the project root. Red evidence:
    `python3 -m pytest tests/test_release_audit.py::test_release_audit_final_metadata_mode_requires_git_and_license tests/test_release_audit.py::test_release_audit_final_metadata_mode_accepts_git_and_license -q`
    -> failed because the missing `.git` directory was not reported.
  - Added `final_metadata:git_dir` to strict release audit checks.
  - Verification:
    `python3 -m pytest tests/test_release_audit.py::test_release_audit_final_metadata_mode_requires_git_and_license tests/test_release_audit.py::test_release_audit_final_metadata_mode_accepts_git_and_license -q`
    -> `2 passed in 0.01s`.
  - Verification:
    `python3 -m pytest tests/test_release_audit.py -q` ->
    `4 passed in 0.02s`.
  - Current strict final-metadata audit:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_phase2e_rehearsal_full/symbolic --mujoco /tmp/wide_robot_phase2e_rehearsal_full/mujoco --randomized /tmp/wide_robot_phase2e_rehearsal_full/mujoco_randomized_30 --comparison /tmp/wide_robot_phase2e_rehearsal_full/comparison --invalid-fixtures /tmp/wide_robot_phase2e_rehearsal_full/invalid_fixtures --require-final-metadata --project-root .`
    -> exit 2, `release audit ok=False checks=71/79`, failing exactly:
    five source-provenance Git checks, missing `.git`, missing `LICENSE`, and
    missing `pyproject.toml` license metadata.
- 2026-06-13 release rehearsal CLI:
  - Added red tests for `build_rehearsal_plan()`: initial evidence
    `python3 -m pytest tests/test_release_rehearsal.py -q` failed during
    collection with `ModuleNotFoundError: No module named 'csg.release_rehearsal'`.
  - Added `csg/release_rehearsal.py`, which builds and optionally runs the
    Phase 2E checklist sequence into stable output subdirectories:
    `symbolic`, `mujoco`, `mujoco_randomized_30`, `comparison`, and
    `invalid_fixtures`, followed by `csg.release_audit`.
  - Added dry-run support, strict final-metadata forwarding, and console script
    `csg-release-rehearsal`.
  - Updated `README.md`, `docs/release_checklist.md`, and
    `docs/sim_only_benchmark_report.md` to document the rehearsal command.
  - Verification:
    `python3 -m pytest tests/test_release_rehearsal.py tests/test_package_metadata.py -q`
    -> `4 passed in 0.01s`.
  - Dry-run verification:
    `python3 -m csg.release_rehearsal --dry-run --out /tmp/wide_robot_phase2e_rehearsal_dry --sim-python .venv-sim/bin/python`
    -> exit 0 and printed the eight-step checklist sequence:
    core tests, symbolic gold, MuJoCo tests, MuJoCo gold, MuJoCo randomized,
    backend comparison, invalid fixtures, and release audit.
  - JSON dry-run verification:
    `python3 -m csg.release_rehearsal --dry-run --json --out /tmp/wide_robot_phase2e_rehearsal_dry --sim-python .venv-sim/bin/python`
    -> exit 0, schema `csg.release_rehearsal_plan.v1`, output directories
    under `/tmp/wide_robot_phase2e_rehearsal_dry`, seeds `30`, and
    `strictFinalMetadata=false`.
  - Focused verification:
    `python3 -m pytest tests/test_release_rehearsal.py tests/test_package_metadata.py tests/test_release_audit.py -q`
    -> `8 passed in 0.02s`.
- 2026-06-13 full non-strict Phase 2E release rehearsal:
  - Command:
    `python3 -m csg.release_rehearsal --out /tmp/wide_robot_phase2e_rehearsal_full --sim-python .venv-sim/bin/python`
    -> exit 0, `release rehearsal ok=True steps=8/8`.
  - Rehearsal steps all returned `0`: `core_tests`, `symbolic_gold`,
    `mujoco_tests`, `mujoco_gold`, `mujoco_randomized`,
    `backend_comparison`, `invalid_fixtures`, and `release_audit`.
  - JSON audit:
    `/tmp/wide_robot_phase2e_rehearsal_full/release_rehearsal_result.json`
    schema `csg.release_rehearsal_result.v1`, `ok=true`, 8 result steps.
  - Randomized report audit:
    `/tmp/wide_robot_phase2e_rehearsal_full/mujoco_randomized_30/report.json`
    summary `{"total": 150, "passed": 150, "failed": 0,
    "failureClassification": {"passed": 150},
    "physicalValidity": {"valid": 150},
    "leakage": {"clean": 150, "dirty": 0}}`, missed diagonal `0`,
    unexpected off-diagonal passes `0`, and distinct sampled layouts
    `{"insert_object": 30, "open_drawer": 30, "place_on_top": 30,
    "push_object": 30, "put_cube_in_tray": 30}`.
  - Comparison/invalid audit:
    comparison physical validity `{"symbolic": {"unverified": 5},
    "mujoco": {"valid": 5}}`; invalid fixtures summary
    `{"matched": 9, "mismatched": 0, "total": 9}`.
  - Release audit step stdout:
    `release audit ok=True checks=71/71`.
  - This rehearsal is non-strict: it proves current report artifacts and
    commands are internally consistent, but does not satisfy the final Git and
    license gates enforced by `--require-final-metadata`.
- 2026-06-13 current final-gate audit:
  - Filesystem checks:
    `test -d .git && echo git-present || echo git-missing` -> `git-missing`;
    `test -f LICENSE && echo license-present || echo license-missing` ->
    `license-missing`.
  - Strict final-metadata audit:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_phase2e_rehearsal_full/symbolic --mujoco /tmp/wide_robot_phase2e_rehearsal_full/mujoco --randomized /tmp/wide_robot_phase2e_rehearsal_full/mujoco_randomized_30 --comparison /tmp/wide_robot_phase2e_rehearsal_full/comparison --invalid-fixtures /tmp/wide_robot_phase2e_rehearsal_full/invalid_fixtures --require-final-metadata --project-root .`
    -> exit 2, `release audit ok=False checks=71/79`.
  - The eight failing strict checks are exactly:
    source provenance is `source_snapshot` rather than `git` for symbolic,
    MuJoCo, randomized, comparison, and invalid-fixture reports; missing
    `.git`; missing `LICENSE`; missing `pyproject.toml` license metadata.
  - No matcher, extractor, leakage, MuJoCo, randomized-rollout,
    invalid-fixture, failure-taxonomy, baseline-comparison, source-snapshot,
    package-entry-point, release-audit, or release-rehearsal work remains known
    from the current audit. Completion now requires owner decisions: initialize
    or restore Git, and choose the project license.
- 2026-06-13 roadmap/status wording alignment:
  - Updated `roadmap.md` so Phase 2E is no longer described as merely the
    "next target"; it now records release-candidate tooling as implemented and
    identifies final public-release gates as Git-backed provenance,
    owner-selected license metadata, and clean-checkout artifact finalization.
  - Updated the Phase 2E acceptance command block in `roadmap.md` to include
    release rehearsal, backend comparison, invalid fixtures, normal release
    audit, and strict final-metadata audit.
  - Updated `README.md` to call Phase 2E a current release-candidate endpoint
    and changed the remaining-work wording to "before public release."
  - Updated `docs/sim_only_benchmark_report.md` expected local test counts
    from fresh evidence.
  - Verification:
    `python3 -m pytest tests/ -q` ->
    `113 passed, 2 skipped in 0.54s`.
  - Verification:
    `.venv-sim/bin/python -m pytest tests/ -q` ->
    `131 passed, 1 skipped in 7.81s`.
  - Incorporated sidecar Planner/Auditor findings:
    roadmap history wording now points to Git/license/final-artifact gates
    rather than broader baseline variants as the next step; the Phase 2E
    continuation block distinguishes implemented release-candidate machinery
    from remaining public-release gates; README readiness now mentions source
    provenance plus release audit/rehearsal; and the release checklist's final
    section is titled as status notes instead of non-release gaps.
  - Targeted stale-text scan:
    `rg -n "next target|Near-term aspirational|What remains to get there|release hygiene and broader baseline|Known Non-Release Gaps|106 passed|107 passed|124 passed|2026-06-12\\)" README.md roadmap.md docs/*.md state.md`
    -> only historical `state.md` evidence-log hits remain.
  - Focused verification:
    `python3 -m pytest tests/test_release_audit.py tests/test_release_rehearsal.py tests/test_package_metadata.py -q`
    -> `8 passed in 0.02s`.
- 2026-06-13 repeated final-gate audit:
  - Filesystem checks:
    `test -d .git && echo git-present || echo git-missing` -> `git-missing`;
    `test -f LICENSE && echo license-present || echo license-missing` ->
    `license-missing`.
  - Package metadata check:
    `pyproject.toml` project table has neither `license` nor `license-files`.
  - Strict final-metadata audit:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_phase2e_rehearsal_full/symbolic --mujoco /tmp/wide_robot_phase2e_rehearsal_full/mujoco --randomized /tmp/wide_robot_phase2e_rehearsal_full/mujoco_randomized_30 --comparison /tmp/wide_robot_phase2e_rehearsal_full/comparison --invalid-fixtures /tmp/wide_robot_phase2e_rehearsal_full/invalid_fixtures --require-final-metadata --project-root .`
    -> exit 2, `release audit ok=False checks=71/79`.
  - The eight failing strict checks remain exactly:
    source provenance is `source_snapshot` rather than `git` for symbolic,
    MuJoCo, randomized, comparison, and invalid-fixture reports; missing
    `.git`; missing `LICENSE`; missing `pyproject.toml` license metadata.
  - No safe non-owner substitution is available for these gates: initializing
    or restoring Git requires explicit approval, and license selection must be
    made by the project owner.
- 2026-06-13 minimal public release ownership transfer:
  - User delegated full ownership to decide the license, initialize Git, set up
    GitHub, regenerate final provenance-backed reports, package a minimal
    public release, add one deliberately dumb baseline, and rewrite the README
    around verification discipline rather than robot capability.
  - Chosen license: MIT. Added `LICENSE`; updated `pyproject.toml` with
    `license = "MIT"`, `license-files = ["LICENSE"]`, and MIT classifier.
  - Added `backend="noop"` as a deliberately dumb diagnostic baseline. It
    instantiates the scene, emits static open-gripper rollout frames, and is
    expected to fail frozen hard probes without changing matcher, extractor,
    canon, or leakage semantics.
  - Updated comparison reports so `noop` is marked `expectedFailure: true`.
    `--require-pass` allows this baseline only when it actually fails with
    non-`passed` failure classes; release audit now requires that behavior.
  - Updated release rehearsal default comparison to
    `symbolic,noop,mujoco`.
  - Rewrote README/release docs around the unique wedge: verification
    discipline and failure diagnosis, not robot capability.
  - Red/green evidence:
    `python3 -m pytest tests/test_benchmark_comparison.py::test_benchmark_comparison_includes_deliberately_dumb_noop_baseline -q`
    initially failed because `backend="noop"` fell through to symbolic and
    passed; after implementation it passed.
  - Red/green evidence:
    `python3 -m pytest tests/test_package_metadata.py::test_pyproject_declares_public_package_metadata_with_mit_license -q`
    initially failed with missing `project.license`; after MIT metadata it
    passed.
  - Focused verification:
    `python3 -m pytest tests/test_benchmark_comparison.py tests/test_release_audit.py tests/test_release_rehearsal.py tests/test_package_metadata.py tests/test_benchmark_failure_classification.py -q`
    -> `13 passed in 0.17s`.
  - Baseline contract probe:
    `python3 -m csg.benchmark gold_tests --compare-backends symbolic,noop --confusion --require-pass --out /tmp/wide_robot_noop_release_contract`
    -> exit 0; symbolic `passed=5/5`, noop `passed=0/5`, noop classes
    `{"contact_missing": 1, "event_order_wrong": 4}`, unexpected confusion
    `0`, missed diagonal `5`.
- 2026-06-13 public release packaging:
  - Initialized Git on `main`, created public GitHub repository
    `https://github.com/alex-reysa/wide-robot`, and published release tag
    `v0.3.0`.
  - Strict release rehearsal command:
    `python3 -m csg.release_rehearsal --out /tmp/wide_robot_phase2e_release_final --sim-python .venv-sim/bin/python --require-final-metadata --project-root .`
    -> `release rehearsal ok=True steps=8/8`; steps
    `core_tests`, `symbolic_gold`, `mujoco_tests`, `mujoco_gold`,
    `mujoco_randomized`, `backend_comparison`, `invalid_fixtures`, and
    `release_audit` all returned `0`.
  - Strict release audit:
    `python3 -m csg.release_audit --symbolic /tmp/wide_robot_phase2e_release_final/symbolic --mujoco /tmp/wide_robot_phase2e_release_final/mujoco --randomized /tmp/wide_robot_phase2e_release_final/mujoco_randomized_30 --comparison /tmp/wide_robot_phase2e_release_final/comparison --invalid-fixtures /tmp/wide_robot_phase2e_release_final/invalid_fixtures --require-final-metadata --project-root .`
    -> `release audit ok=True checks=84/84`.
  - Final report evidence:
    symbolic `passed=5/5`, `physicalValidity={"unverified": 5}`;
    MuJoCo `passed=5/5`, `physicalValidity={"valid": 5}`;
    randomized MuJoCo `passed=150/150`, `physicalValidity={"valid": 150}`,
    leakage `{"clean": 150, "dirty": 0}`;
    invalid fixtures `matched=9/9`;
    comparison order `["symbolic", "noop", "mujoco"]`.
  - No-op diagnostic baseline evidence:
    `expectedFailure=true`, `passed=0/5`, failure classes
    `{"contact_missing": 1, "event_order_wrong": 4}`, leakage clean `5/5`,
    physical validity unverified `5/5`.
  - Release artifacts assembled under
    `/tmp/wide_robot_phase2e_release_final/package/`:
    `phase2e-report-artifacts.tar.gz`, `csg-0.3.0-py3-none-any.whl`,
    `csg-0.3.0.tar.gz`, and `wide-robot-0.3.0-source.tar.gz`.
