# Sim-Only Benchmark Release Checklist

This project is not ready for a public Phase 2E release until every item below
has current evidence from a clean checkout.

## Required Metadata

- [x] Git repository initialized or restored.
- [x] Generated artifacts ignored: Python caches, local venvs, benchmark output
      directories, logs, and OS/editor files.
- [x] Existing module CLIs exposed as package console scripts:
      `csg-benchmark`, `csg-solver`, `csg-to-sim`, `csg-rollout-extract`,
      `csg-matcher`, `csg-skills`, `csg-release-audit`, and
      `csg-release-rehearsal`.
- [x] Non-license package metadata declares README, keywords, and neutral
      Python/package classifiers.
- [x] License selected and committed as `LICENSE` (MIT).
- [x] `pyproject.toml` license metadata matches `LICENSE`.
- [x] Source snapshot recorded in benchmark, comparison, and invalid-fixture
      reports via `sourceProvenance`; Git commit/status is included when a
      `.git` checkout exists.
- [x] Public sim-only benchmark report draft exists at
      `docs/sim_only_benchmark_report.md`.

## Reproducibility Commands

```bash
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

## Required Report Artifacts

- [ ] `report.json`, `report.md`, `summary.csv`, and
      `failure_classification.json` for symbolic gold benchmark.
      `report.json` must expose summary counts for failure class,
      physical-validity state, and leakage cleanliness.
- [ ] Same artifacts for MuJoCo gold benchmark.
- [ ] Same artifacts for 30-seed MuJoCo randomized benchmark.
- [ ] `comparison_report.json` plus per-baseline outputs for symbolic, no-op
      expected-failure, and MuJoCo.
- [ ] Frozen invalid fixture report with expected failed probe/check
      for every invalid fixture (`gold_invalid/`).
- [ ] Final public benchmark report regenerated from clean-checkout evidence
      and linked to a tagged/source-snapshot release.
- [ ] Release artifact audit passes for the generated output directories.
- [ ] Strict release artifact audit passes with `--require-final-metadata`
      after `.git`, Git-backed report provenance, and license metadata exist.
- [ ] Release rehearsal result (`release_rehearsal_result.json`) is attached to
      the final tagged/source-snapshot release.

## Current Status Notes

- The frozen MuJoCo invalid-fixture suite covers all six physical-validity
  checks plus semantic verifier failures for push contact missing, wrong
  relation, and wrong event order.
- The no-op baseline is deliberately expected to fail; release audit requires
  it to fail with non-`passed` failure classes so the taxonomy is visible.
- Final public reports should show `sourceProvenance.kind: git` and
  `dirty: false`.
