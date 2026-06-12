# Phase 2E Sim-Only Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the five-task MuJoCo-backed prototype into a reproducible public sim-only benchmark for fixed-base robotic-arm manipulation.

**Architecture:** Keep the frozen verifier boundary intact: targets compile through `csg.to_sim`, rollouts stay `csg.rollout.v0`, extractor/matcher/leakage gates remain unchanged. Add benchmark orchestration, reporting, randomized scene sampling, invalid fixture handling, packaging hygiene, and baseline comparison around the existing solver seam.

**Tech Stack:** Python standard library for core benchmark/reporting, optional `mujoco>=3.9` extra for physics rollouts, pytest for regression gates.

---

## Current Evidence

- `python3 -m pytest tests/ -q` and symbolic benchmark were green in the Phase 2C final verification recorded in `state.md`.
- `.venv-sim/bin/python -m pytest tests/ -q` and `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass` were green in the Phase 2C final verification recorded in `state.md`.
- Current workspace is not a Git checkout; `state.md` records this as a Phase 2E hygiene blocker.
- `SolverConfig.seed` exists, and MuJoCo runner passes it to `build_arm_scene_xml`, but `scene_mjcf.py` does not use it yet.

## Task 2E-1: Benchmark Failure Taxonomy

**Files:**
- Modify: `csg/benchmark.py`
- Test: `tests/test_confusion.py` or `tests/test_benchmark_reporting.py`

- [ ] **Step 1: Add failing tests for classification schema**

Create tests that assert:
- `run_benchmark(...).cases[*].failureClassification` exists.
- PASS cases classify as `{"primaryClass": "passed"}`.
- synthetic classification inputs map leakage to `target_leakage_detected`, physical validity false to `physical_invalidity`, contact mismatches to `contact_missing`, relation/goal mismatches to `relation_not_achieved`, order mismatches to `event_order_wrong`, and unknown hard mismatches to `verifier_mismatch`.
- `failure_classification.json` is written with `summary` and `cases`.

Run:

```bash
python3 -m pytest tests/test_confusion.py -q
```

Expected before implementation: failure on missing classification fields or output file.

- [ ] **Step 2: Implement classifier helpers in `csg/benchmark.py`**

Add pure functions near `leakage_report`:

```python
def classify_failure(case: Json) -> Json:
    ...

def summarize_failure_classes(cases: Sequence[Json]) -> Json:
    ...
```

The classifier must only read existing benchmark facts: error status, matcher result, hard mismatches, leakage report, `physicalValidity`, and physical-validity reason/report if available.

- [ ] **Step 3: Wire classification into reports**

Set `case["failureClassification"]` for every case and add `report["failureClassificationSummary"]`. Write `failure_classification.json` alongside `report.json`, `summary.csv`, and `report.md`.

- [ ] **Step 4: Verify**

Run:

```bash
python3 -m pytest tests/test_confusion.py tests/test_validity.py -q
python3 -m pytest tests/ -q
python3 -m csg.benchmark gold_tests --confusion --require-pass --out /tmp/wide_robot_taxonomy_symbolic
```

Expected: all commands exit 0; `/tmp/wide_robot_taxonomy_symbolic/failure_classification.json` exists and all five gold cases classify as `passed`.

## Task 2E-2: Seeded Randomized MuJoCo Rollouts

**Files:**
- Modify: `csg/backends/mujoco/scene_mjcf.py`
- Modify: `csg/solver.py` if diagnostics need sampled layout metadata
- Modify: `csg/benchmark.py`
- Test: `tests/test_mujoco_backend.py`
- Docs: `README.md`, `roadmap.md`, `csg/validity.md`

- [ ] **Step 1: Add failing tests for seed behavior**

In MuJoCo-gated tests, assert:
- Two runs with the same non-null seed produce identical frame JSON.
- Two different seeds produce different initial sampled object layout metadata.
- Each V0 task passes for a small seed set such as `[0, 1, 2]`.

Run:

```bash
.venv-sim/bin/python -m pytest tests/test_mujoco_backend.py -q
```

Expected before implementation: same/different seed tests fail because seed has no effect.

- [ ] **Step 2: Sample bounded layout perturbations**

Use `random.Random(seed)` in `scene_mjcf.py` to derive `pose_overrides` from the deterministic deconflicted layout. Keep perturbations conservative:
- jitter x/y by a few millimeters inside reach.
- preserve task-specific initial relation constraints: put/insert stay NEAR, place/push stay FAR, open handle remains reachable.
- store sampled layout data in the runner or rollout diagnostics.

- [ ] **Step 3: Add CLI/report plumbing**

Add `--randomized` and `--seeds N` to `csg.benchmark`. For randomized mode, run every target once per seed and name output directories as `<case>/seed_<n>` or `<case>__seed_<n>`. Store seed, solver config, sampled scene metadata, matcher result, leakage report, validity verdict, and classification for every trial.

- [ ] **Step 4: Verify small seeded run**

Run:

```bash
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 3 --require-pass --out /tmp/wide_robot_randomized_3
```

Expected: 15/15 seeded trials pass, every trial has `physicalValidity: true`, leakage clean, no unexpected off-diagonal confusion.

## Task 2E-3: Frozen Physically Invalid Fixtures

**Files:**
- Modify/create under `gold_tests/`
- Modify: `tests/test_mujoco_backend.py` or new invalid-fixture tests
- Modify: `csg/benchmark.py` only if fixture discovery needs a new mode

- [ ] **Step 1: Define invalid fixture manifest shape**

Each fixture should declare:
- source task.
- sabotage or generator config.
- expected primary failure class.
- expected failed physical-validity check or hard probe.

- [ ] **Step 2: Generate and freeze first real MuJoCo invalid fixture**

Use `sabotage="early_release"` for `put_cube_in_tray` first. Store enough data to reproduce or verify it without target leakage.

- [ ] **Step 3: Verify expected failure**

Run a focused test that confirms the fixture fails for `quasi_static_support_at_release` or the expected hard probe and classifies as `object_dropped` or `physical_invalidity`.

## Task 2E-4: Packaging, License, And Release Hygiene

**Files:**
- Modify: `.gitignore`
- Add: `LICENSE` after user confirms license choice
- Modify: `pyproject.toml`
- Modify: `README.md`
- Modify: `roadmap.md`

- [ ] **Step 1: Clean ignore rules**

Add ignore entries for Python caches, pytest cache, local virtualenvs, benchmark output directories, logs, and OS metadata.

- [ ] **Step 2: Confirm license choice**

Ask the project owner to choose MIT, Apache-2.0, BSD-3-Clause, or another license. Do not invent a license claim without owner confirmation.

- [ ] **Step 3: Update package metadata after license is known**

Add license metadata to `pyproject.toml` and ensure editable install commands remain current.

## Task 2E-5: Baseline Solver Comparison

**Files:**
- Modify: `csg/benchmark.py`
- Modify: `csg/solver.py` if adding a named weak baseline
- Test: new benchmark comparison tests
- Docs: `README.md`, `roadmap.md`

- [ ] **Step 1: Pick the first baseline**

Use a deliberately weaker baseline that does not require new dependencies, such as symbolic backend versus MuJoCo scripted solver, or a no-op/ablated scripted backend that produces valid schema but fails task probes.

- [ ] **Step 2: Add comparison report mode**

Add a CLI option such as `--compare-baselines scripted_mujoco,symbolic` or a narrower `--baseline symbolic`. Reports must group pass rate, failure class summary, leakage, validity, and confusion by baseline.

- [ ] **Step 3: Verify**

Run comparison on gold tasks and confirm the scripted MuJoCo solver is the reference baseline while the weaker baseline is clearly labeled and diagnostically useful.

## Completion Gate

Phase 2E is not complete until fresh current-state evidence proves all of:

```bash
python3 -m pytest tests/ -q
python3 -m csg.benchmark gold_tests --confusion --require-pass
.venv-sim/bin/python -m pytest tests/ -q
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --require-pass
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion --randomized --seeds 30 --require-pass
```

Additional required evidence:
- Git checkout with usable ignore rules and source snapshot provenance.
- Explicit license/package setup.
- Frozen invalid fixtures fail for expected taxonomy/check reasons.
- Baseline comparison report exists and is reproducible from a clean checkout.
