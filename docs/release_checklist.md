# Sim-Only Benchmark Release Checklist

Status: the public Phase 2E release is **shipped** (latest `v0.3.2`); every item
below has current evidence from a clean checkout. This checklist is now the
maintenance gate — re-run it before cutting any future tag. (A release is not
considered *fully bound* until its MuJoCo physics is CI-attested: a laptop-cut tag
verifies as `evidence.complete=false`/exit 1 until added to `ATTESTED_TAGS`.)

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
# Reproduce from a clean clone (base + sim), then verify the published release:
bash scripts/clean_clone_rehearsal.sh <ref>
# Pack the report tarball deterministically (reproducible SHA), record the sim
# environment (mujoco/numpy versions), then pin everything in the manifest:
python3 -m csg.release_manifest --tag <tag> \
  --asset-dir <assets> --reports-root <release-out> \
  --build-report-tarball phase2e-report-artifacts.tar.gz \
  --sim-python .venv-sim/bin/python --project-root . --write-checksums
python3 -m csg.verify_release --tag <tag>
```

## Cutting a release in CI (attested) vs. on a laptop (self-attested)

Prefer the CI path: pushing a `v*` tag triggers `.github/workflows/release.yml`,
which runs the full rehearsal **inside GitHub Actions** (so the machine-dependent
MuJoCo numbers are produced there), packs the deterministic tarball + checksums +
manifest via `csg.release_manifest`, signs every asset with a build-provenance
**attestation** bound to the workflow's OIDC identity, and publishes the release.
This is what lets `csg.verify_release` trust the MuJoCo evidence it cannot re-derive
locally. A laptop-cut release (the manual commands above) has no attestation: its
MuJoCo numbers are self-attested and `verify_release` reports `attestation:skipped`.

What `verify_release` now checks for every release:
- **source identity** — reports + wheel/sdist `csg/` bound to `git archive` of the
  in-source-pinned commit;
- **deterministic evidence** — it re-runs the symbolic / noop / invalid benchmarks
  from the tagged source and diffs the numbers (a fabricated number fails);
- **MuJoCo evidence** — covered by the CI attestation for attested tags, or flagged
  self-attested otherwise;
- **integrity** — `RELEASE_SHA256SUMS` + `release_manifest.json` reconciled.

### After tagging a NEW release — update the in-source anchors

In a **follow-up commit on the branch** (the pins are read from the verifier's own
checkout, not from the tagged commit, so they cannot live inside the tagged commit):

1. Add `tag -> commit` to `KNOWN_TAG_COMMITS` in `csg/verify_release.py`.
2. If the tag was cut by `release.yml` (attested), also add it to `ATTESTED_TAGS`.

Until those lines land, the tag is "provisional": it still verifies via the unpinned
(but git-archive-bound) path, and attestation is reported as `skipped`.

## Required Report Artifacts

- [x] `report.json`, `report.md`, `summary.csv`, and
      `failure_classification.json` for symbolic gold benchmark.
      `report.json` must expose summary counts for failure class,
      physical-validity state, and leakage cleanliness.
- [x] Same artifacts for MuJoCo gold benchmark.
- [x] Same artifacts for 30-seed MuJoCo randomized benchmark.
- [x] `comparison_report.json` plus per-baseline outputs for symbolic, no-op
      expected-failure, and MuJoCo.
- [x] Frozen invalid fixture report with expected failed probe/check
      for every invalid fixture (`gold_invalid/`).
- [x] Final public benchmark report regenerated from clean-checkout evidence
      and linked to a tagged/source-snapshot release.
- [x] Release artifact audit passes for the generated output directories.
- [x] Strict release artifact audit passes with `--require-final-metadata`
      after `.git`, Git-backed report provenance, and license metadata exist.
- [x] Release rehearsal result (`release_rehearsal_result.json`) is attached to
      the final tagged/source-snapshot release.

## Current Status Notes

- The frozen MuJoCo invalid-fixture suite covers all six physical-validity
  checks plus semantic verifier failures for push contact missing, wrong
  relation, and wrong event order.
- The no-op baseline is deliberately expected to fail; release audit requires
  it to fail with non-`passed` failure classes so the taxonomy is visible.
- Final public reports should show `sourceProvenance.kind: git` and
  `dirty: false`.
