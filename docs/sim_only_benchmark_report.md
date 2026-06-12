# Phase 2E Sim-Only Benchmark Report Draft

Status: release report for the minimal public Phase 2E benchmark. Final
release artifacts are regenerated from a committed Git checkout so JSON
reports carry commit-backed provenance.

## Claim Boundary

This benchmark evaluates fixed-base robotic-arm manipulation in simulation. It
does not claim real-robot transfer, arbitrary video understanding, robot
learning from video, or general policy competence.

Allowed claim for this report:

```text
Object-centric target CSGs for the five V0 tabletop tasks are evaluated across
symbolic, no-op, and MuJoCo fixed-base-arm rollouts with independent rollout
extraction, frozen hard-probe matching, leakage checks, real physical-validity
checks for MuJoCo, randomized seeded coverage, invalid-fixture diagnostics, and
baseline comparison reports.
```

## Task Set

The V0 benchmark contains five gold tasks:

| Task | Primary skill | Required distinguishing signal |
| --- | --- | --- |
| `put_cube_in_tray` | pick/place into container | `INSIDE` goal relation |
| `insert_object` | pick/place into container | `INSIDE` goal relation; quotient-equivalent to `put_cube_in_tray` |
| `place_on_top` | pick/place onto block | `ON_TOP_OF` goal relation, distinct from `INSIDE` |
| `push_object` | non-grasp push | `TOUCHING_LIKELY` contact, no grasp, terminal `NEAR` |
| `open_drawer` | grasp and pull prismatic drawer | `ARTICULATION_CHANGE` with prismatic extension increase |

The only documented off-diagonal confusion equivalence is:

```text
insert_object ~ put_cube_in_tray
```

All other off-diagonal PASS cells are treated as benchmark failures.

## Evaluation Pipeline

```text
target_csg.json
  -> compile_scene
  -> solver rollout (symbolic or MuJoCo)
  -> csg.rollout.v0 frames
  -> independent extract_robot_csg
  -> frozen matcher hard probes
  -> leakage gate
  -> physical-validity gate where available
  -> benchmark report and failure classification
```

The rollout extractor reads rollout frames only. The matcher, extractor, canon,
and leakage gate are not weakened for this benchmark.

## Validity Checks

Symbolic rollouts report `physicalValidity: null` and are labeled
physics-unverified. MuJoCo rollouts report a real Boolean verdict from:

1. Non-penetration.
2. Pose continuity.
3. Quasi-static support at release.
4. Gripper feasibility.
5. Workspace reachability.
6. Articulation limits.

A MuJoCo benchmark PASS requires matcher PASS, leakage clean, and
`physicalValidity: true`.

## Reproduction Commands

Base environment:

```bash
python3 -m csg.release_rehearsal --dry-run --out <release-out>
python3 -m csg.release_rehearsal --out <release-out>
python3 -m pytest tests/ -q
python3 -m csg.benchmark gold_tests --confusion --require-pass
```

MuJoCo environment:

```bash
pip install -e '.[sim]'
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

Expected current counts from the latest recorded local verification:

| Command family | Expected result |
| --- | --- |
| Core tests | `114 passed, 2 skipped` |
| Symbolic gold benchmark | `passed=5/5` |
| MuJoCo tests | `132 passed, 1 skipped` |
| MuJoCo gold benchmark | `passed=5/5`, all cases `physicalValidity=True` |
| MuJoCo randomized sweep | `passed=150/150`, 30 seeds per task |
| Backend comparison | symbolic `5/5`, no-op `0/5` expected failure, MuJoCo `5/5` |
| Invalid fixtures | `matched=9/9` |

## Report Artifacts

Each normal benchmark run writes:

- `report.json`
- `report.md`
- `summary.csv`
- `failure_classification.json`
- Per-case rollout, extracted robot CSG, matcher report, and validity report
  where applicable.

Comparison runs write `comparison_report.json` plus per-baseline benchmark
subdirectories. Invalid-fixture runs write `invalid_fixtures_report.json`.

Every JSON report includes `sourceProvenance`:

- Git commit/status when a `.git` checkout exists.
- A deterministic SHA-256 source snapshot over package, tests, docs, gold
  targets, and invalid fixtures.

This workspace currently lacks `.git`, so current local reports use
`kind: source_snapshot`. A public release should be regenerated from a Git
checkout so the same field carries commit-backed provenance.

The `csg.release_audit` command validates that generated release output
directories contain the required artifact files, expected schemas,
`sourceProvenance`, pass counts, physical-validity counts, leakage counts,
randomized layout coverage, comparison summaries, and invalid-fixture
categories.

The `--require-final-metadata` mode additionally requires a `.git` checkout,
Git-backed `sourceProvenance`, `LICENSE`, and `pyproject.toml` license
metadata.

The `csg.release_rehearsal` command runs the full checklist sequence into
stable output subdirectories and then invokes `csg.release_audit`. Use
`--dry-run` first to inspect the commands.

## Invalid Fixtures

The frozen invalid suite under `gold_invalid/` covers all six physical-validity
checks plus three semantic verifier failures.

| Fixture class | Expected count |
| --- | ---: |
| `physical_invalidity` | 6 |
| `contact_missing` | 1 |
| `relation_not_achieved` | 1 |
| `event_order_wrong` | 1 |

The fixture harness checks the expected failed validity check or hard probe and
fails nonzero under `--require-pass` when a fixture passes unexpectedly or fails
for the wrong reason.

## Failure Taxonomy

Every benchmark case includes a `failureClassification` object. PASS cases are
classified as `passed`. Failure categories are derived from existing evidence:
solver errors, leakage report, physical-validity report, and hard-probe
mismatches. The taxonomy does not change the verifier or PASS criteria.

Top-level benchmark summaries also include:

- `failureClassification`: count by failure class.
- `physicalValidity`: count by `valid`, `invalid`, and `unverified`.
- `leakage`: count by `clean` and `dirty`.

## Baseline Comparison

The baseline comparison is:

```text
symbolic vs noop vs mujoco
```

The symbolic backend demonstrates interface-level task equivalence but is
physics-unverified. The `noop` backend instantiates the scene and does nothing;
it is an expected-failure baseline, and release audit requires it to fail with
non-`passed` failure classes. The MuJoCo backend uses the scripted fixed-base
arm and must satisfy real physical validity. Later baselines can add noisy,
ablated scripted solvers, or learned policies, but they are not required for
the current Phase 2E minimum.

## Limitations And Release Gates

Release gates:

- JSON reports attached to a public release must show `sourceProvenance.kind:
  git` and `dirty: false`.
- The project is MIT-licensed; `LICENSE` and `pyproject.toml` metadata must
  remain aligned.
- The report artifacts should be regenerated from a clean Git checkout and
  attached to the tagged release.
