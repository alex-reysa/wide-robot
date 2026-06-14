# Arm-Bounded Demonstration Compiler

A **leakage-clean compiler/verifier loop for fixed-base robotic-arm
manipulation.** The claim is verification discipline, not robot capability.

```text
human tabletop demonstration
→ observable target CSG (Causal Skill Graph)
→ robotic-arm solver attempt
→ simulated or recorded rollout
→ independently extracted rollout CSG
→ unchanged hard-probe verifier
→ pass / fail / failure diagnosis
```

This repository is a benchmark for whether a manipulation rollout can be
judged honestly. The system compiles **what changed in the world** — object
state transitions, relations, contacts, event order — into an inspectable,
embodiment-agnostic graph, then judges whether a solver rollout reproduced
that task-level transformation **without ever letting the answer key leak into
the rollout side**. A simple scripted arm can pass; a deliberately dumb no-op
baseline fails with named failure classes. That contrast is the point.

> Formerly "The Universal Demonstration Compiler." Renamed 2026-06-10 when the
> scope was deliberately narrowed to fixed-base arms — see `roadmap.md` for
> the rationale, the claims discipline, and the phase plan. `thesis.md` is the
> retired long-horizon vision, kept as background reading only.

## Quick start

```bash
python3 -m pytest tests/ -q                          # core suite, all green (mujoco tests skip)
python3 -m csg.benchmark gold_tests                  # 5/5 gold tasks PASS
python3 -m csg.benchmark gold_tests --confusion      # + cross-task matrix
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco \
  --confusion --randomized --seeds 30 --require-pass
.venv-sim/bin/python -m csg.benchmark gold_tests \
  --compare-backends symbolic,noop,mujoco --confusion --require-pass
.venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid \
  --require-pass
```

The verifier loop has **no dependencies beyond the Python 3 standard library**
(pytest to run tests). The benchmark writes `report.json`, `summary.csv`,
`report.md`, and `failure_classification.json` per run; `--require-pass` makes
it exit nonzero on any failure, leakage, or unexpected cross-task PASS (for CI).
Report summaries include pass/fail counts, failure classes, physical-validity
counts, and leakage clean/dirty counts.
JSON reports include `sourceProvenance`: Git commit/status when a `.git`
checkout exists, plus a deterministic SHA-256 source snapshot either way. The
package is MIT-licensed. The public benchmark report lives at
`docs/sim_only_benchmark_report.md`; final release artifacts are generated from
a Git checkout so reports carry commit-backed provenance.

The **MuJoCo physics backend** (roadmap Phase 2C) is an opt-in extra:

```bash
pip install -e '.[sim]'                               # adds mujoco (+ numpy)
python3 -m csg.solver gold_tests/put_cube_in_tray/target.json --backend mujoco
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion
```

Editable installs include README-backed package metadata and expose console
scripts for the existing module CLIs: `csg-benchmark`, `csg-solver`,
`csg-to-sim`, `csg-rollout-extract`, `csg-matcher`, `csg-skills`, and
`csg-release-audit`, plus `csg-release-rehearsal` for running the Phase 2E
checklist sequence. The `python3 -m csg...` forms remain the canonical commands
in docs and CI because they also work from an uninstalled checkout.

mujoco's macOS/Linux wheels target CPython ≤ 3.13 today; if your default
`python3` is newer, run the sim in a 3.12 venv (`python3.12 -m venv .venv-sim`).
The mujoco-dependent tests `pytest.importorskip("mujoco")`, so the core suite
still runs (and stays green) without it.

## Current status (V0.3.1, 2026-06-13)

| What | State |
| --- | --- |
| Verifier (hard-probe matcher, frozen) | ✅ done, audited three times |
| Leakage gate + anti-cheating tests | ✅ done (executed-attack hardened) |
| Symbolic solver backend (Level 0) | ✅ done — proves the loop, reports `physicalValidity: None` honestly |
| Gold tasks | ✅ 5/5: put_cube_in_tray, place_on_top, push_object, open_drawer, insert_object — each with failure variants |
| Cross-task confusion matrix | ✅ clean diagonal; one documented quotient equivalence (insert_object ~ put_cube_in_tray) |
| MuJoCo physics backend (Level 2) | ✅ **all five V0 gold tasks pass gated MuJoCo tests/benchmark with real `physicalValidity: true`** (`csg/backends/mujoco/`); seeded 30-rollout/task benchmark samples every V0 task, including x-shifted push starts; covered by an optional manual CI workflow (`.github/workflows/mujoco.yml`). **Release-evidence caveat:** the MuJoCo numbers are machine-dependent floats that `verify_release` cannot re-derive cross-machine, so for any tag **not** in `ATTESTED_TAGS` (currently none — releases were cut on a laptop, not in CI) the published physics evidence is **self-attested**: taken on the publisher's word, not independently verified. `verify_release` reports `mujocoCoverage: self-attested`, sets `evidence.complete=false`, and **exits 1** (not 0) for such a release; `release_audit` likewise *asserts* the MuJoCo summaries against expected constants rather than verifying the physics. Binding them requires cutting a release in `.github/workflows/release.yml` and listing the tag in `ATTESTED_TAGS`. Scope: verification discipline on a fixed-base arm, not general robot capability. |
| Sim-only benchmark readiness | ✅ seeded randomized reports, failure taxonomy, symbolic/no-op/MuJoCo baseline comparison, a nine-fixture invalid suite, source provenance, release audit, release rehearsal, Git hygiene, and MIT package metadata are in place; release artifacts are regenerated from the committed clean checkout (Git-backed `sourceProvenance`, `dirty=false`) and verified by `python -m csg.verify_release`, which re-derives the deterministic (symbolic/no-op/invalid) numbers from `git archive <pinned tag commit>`, binds every report's source snapshot, binds the wheel's `csg/` source and the sdist's + full source-tarball's **entire file tree** to that archive (a backdoor outside `csg/` cannot ride along unbound), and reconciles `RELEASE_SHA256SUMS` + `release_manifest.json` against those anchors. The MuJoCo physics floats are the one layer it cannot re-derive — see the caveat above; a fully-bound (`evidence.complete`, exit 0) verdict requires a CI-attested tag |
| RLBench external-trace pilot | 🟡 offline `open_drawer` ingest/verifier path implemented outside `csg/`: hardened external leakage gate, real converter, live recorder, synthetic leakage-clean fixture, and 1×N confusion all pass without RLBench installed. A 2026-06-14 Runpod live capture recorded bottom/middle/top RLBench demos and produced leakage-clean `physicalValidity: null` rollouts, now promoted to committed fixtures. Two committed results: **(A)** the gold target does **not** accept them (`event_order`, `goal_satisfaction`); **(B)** a value-only diagnostic target (terminal extension only; contact/event semantics deferred) **PASSes** them leakage-clean. Next: a follow-on articulation-event target. `csg/` byte-frozen throughout. |
| Perception compiler (video → target CSG) | ⬜ Phase 3 |
| DK1 real-arm data campaign + adapter | ⬜ Phases 4–5 (playbook in `roadmap.md` §7) |

## Reproducibility

Continuous integration runs the dependency-free suite on every push/PR
(`.github/workflows/ci.yml`, Python 3.11–3.13) — including an integration test
that drives `csg.verify_release` over a release built from the live git history,
so the verifier's real git/tar/snapshot path is exercised on every push. A
separate **manual** workflow exercises the MuJoCo backend
(`.github/workflows/mujoco.yml`).

Every tagged release ships `RELEASE_SHA256SUMS` and a `release_manifest.json`
(commit, tag, asset SHA-256s, recorded sim environment, expected benchmark
summaries, exact commands). `csg.verify_release` re-verifies a published release
*against the tagged source*:

```bash
python3 -m csg.verify_release --tag v0.3.1       # verify the canonical release
bash scripts/clean_clone_rehearsal.sh v0.3.1     # reproduce from a clean clone
```

What that actually proves — the trust model is documented at the top of
`csg/verify_release.py`:

- **Source identity anchored to the commit (the publisher cannot forge it).** The
  expected commit comes from an in-source pin (`KNOWN_TAG_COMMITS`), not from the
  release, and `origin` is never trusted to choose which repo to verify. `git
  archive <commit>` reconstructs the committed source, and the verifier requires
  (a) every distribution to be byte-bound to it — the `csg/` Python source in every
  wheel/sdist (defeats a trojan wheel), and the *whole* file tree of the sdist, the
  source tarball, **and the wheel** (every member outside the `.dist-info` metadata dir
  must match the archive, so a native `*.so`, a `*.data` install payload that pip would
  drop onto `PATH`/into site-packages, or a `[console_scripts]` entry point pointing
  outside `csg` cannot ride along unbound) — and (b) every report's `sourceProvenance.snapshot` to
  equal the snapshot recomputed from that tree. Note (b) binds *which commit* a
  report claims, **not** its numbers — the snapshot is computable from the public
  source, so a report citing the real commit with fabricated results would pass (b)
  alone. The numbers are bound separately:
- **Benchmark numbers bound to the source.** The deterministic evidence (symbolic /
  no-op / invalid-fixture benchmarks — pure stdlib) is **re-derived**: `verify_release`
  re-runs them from the `git archive` tree and diffs the results against the published
  reports, so a fabricated number diverges (this is what actually defeats a fabricated
  report; `--no-rederive` opts out). MuJoCo numbers are machine-dependent floats that
  cannot be re-derived cross-machine, so releases cut by `.github/workflows/release.yml`
  carry a GitHub **build-provenance attestation** binding every asset to that CI run's
  OIDC identity; `verify_release` checks it (`gh attestation verify`) against an
  in-source-pinned signer workflow (`ATTESTED_TAGS`). A tag predating attestation
  reports its MuJoCo evidence as `attestation:skipped` (self-attested), never silently
  blessed.
- **Checksum-pinned for tamper-evidence only.** `RELEASE_SHA256SUMS` and the
  manifest are publisher-supplied, so they are *reconciled* against the anchored
  facts and the recomputed asset bytes — never trusted on their own. A wheel's
  `.dist-info` *compiled metadata* (`METADATA`, `WHEEL`, `RECORD`, …) is the one
  layer not reconstructable from the archive, so it is pinned for transit integrity
  rather than re-derived — except `entry_points.txt`, whose `[console_scripts]`/
  `[gui_scripts]` targets are checked to stay inside the byte-bound `csg` package;
  byte-for-byte rebuild reproducibility of the wheel itself is out of scope.

Reproducibility scope (be precise about what "reproducible" means here):

- The release tooling packs the report-artifacts tarball **deterministically**
  (`csg.release_manifest --build-report-tarball`: sorted entries, fixed
  mtime/uid/gid), so re-packing an identical report tree reproduces its SHA-256.
  This applies to releases cut with the current tooling; the **v0.3.1** tarball
  was packed before that builder existed, so its bytes are checksum-pinned for
  tamper-evidence, not byte-reproducible.
- The **symbolic / no-op / invalid-fixture** evidence is deterministic and is
  re-derived and diffed by `verify_release` (above), so those numbers are bound to
  the tagged source, not merely asserted.
- MuJoCo report **content** carries machine-dependent physics floats (MuJoCo /
  numpy / BLAS), so a clean re-run does not bit-match across machines and cannot be
  re-derived. For these, the trust root is the CI **build-provenance attestation**
  (attested tags) plus the recorded `environment` (manifest `environment.sim`) and
  the bounded `mujoco>=3.9,<3.10` pin. Tags predating attestation (e.g. **v0.3.1**,
  **v0.3.2**) are self-attested for the MuJoCo evidence.

So a self-attested release must not read like a fully-verified one. Every verdict
reports its **coverage** (`evidence.deterministicReDerived`, `evidence.mujocoCoverage`,
`evidence.complete`): such a release still passes by default but prints a loud
`WARNING: evidence coverage INCOMPLETE — MuJoCo/randomized numbers are self-attested`,
and `--strict` turns an incomplete binding (self-attested MuJoCo, or any skipped layer
such as `--no-rederive`) into a hard failure (exit 2). The genuine v0.3.1/v0.3.2
releases pass today; under `--strict` they fail until re-cut through `release.yml`.

`verify_release` exits 0 (ok), 2 (the release fails verification — bad or forged
content: a published number that diverges from re-derivation, a *refuted* attestation,
or — under `--strict` — self-attested/skipped evidence), or 3 (operational error:
`gh`/`git` missing, tag/commit unresolved, download / `git archive` / re-derivation
failure, an attestation that cannot be *reached* (offline/unauthenticated `gh`), or a
filesystem/environment failure such as an unwritable work dir). Hostile release bytes
are classified as 2, never a traceback; environment failures (incl. an unreachable
attestation) are 3 — being unable to *complete* a check must never read as "the release
is bad". The claim boundary is unchanged: this hardens *verification discipline*, not
robot capability.

## Repository map

```text
csg/                     THE package — single source of truth
  common.py              JSON / pose / id helpers
  predicates.py          versioned geometric semantics of every relation &
                         contact word (NEAR, INSIDE, TOUCHING, …). Target
                         compilers and rollout extractors must both import
                         this — one grammar, no private vocabularies.
  canon.py               canonical form: strips TaskSpec from rollouts,
                         normalizes converse relations, confidence masking
  matcher.py             the verifier: hard/soft probe split, vacuity gate,
                         1-WL role-fingerprint object mapping, symmetry orbits
  to_sim.py              target PlannerView → simulator scene (open-cavity
                         containers, rollout body sanitization whitelist)
  skills.py              CSG structure → skill skeleton routing
                         (pick_place / place_on / push / insert / open-close)
  solver.py              backend dispatch: scene + skill → rollout frames
                         (symbolic inline; mujoco via the seam at solve())
  backends/mujoco/       MuJoCo arm backend (Phase 2C, opt-in extra):
                           arm.py        hand-written 6-DoF arm + parallel-jaw
                                         gripper MJCF (no external assets)
                           scene_mjcf.py compiled scene → full MJCF, shared
                                         cavity geom, initial-pose deconfliction
                           controller.py damped-least-squares 6-DoF IK
                           runner.py     scripted task controller → frames + SimTrace
                           validity.py   the six checks → physicalValidity verdict
                           trace.py      stdlib SimTrace seam (no mujoco import)
  rollout_extract.py     INDEPENDENT extractor: frames → robot CSG.
                         Reads the rollout only. Never the target.
  benchmark.py           frozen loop runner: per-task PASS, leakage report,
                         validity labeling, confusion matrix
  rollout_schema.md      csg.rollout.v0 — the information-flow contract
  validity.md            the six physical-validity checks (now implemented by
                         the MuJoCo backend; symbolic stays None)

gold_tests/              5 tasks × (target + success & failure rollout
                         fixtures + expected.json) — end-to-end regression
tests/                   core suite + validity-checks + mujoco backend (gated):
                         gold, loop, leakage, adversarial, metamorphic,
                         separation, mapping, confusion, validity, mujoco
pilots/                  research pilots outside the package. They consume the
                         frozen csg verifier as external users; RLBench lives here,
                         not in the released `csg` package.

Causal_Skill_Graph_V0.md CSG schema (observable facts only) + audit notes
physical_quotient.md     the math: observable quotient Q̂*_CSG; §0 lists the
                         authoritative implementation overrides
roadmap.md               scope, claims discipline, phase plan, DK1 playbook
thesis.md                RETIRED broad vision (background reading only)

CSG_Matcher/             deprecated shims from the pre-audit V0 — kept for
CSG_Solver_Harness/      history; see CSG_Solver_Harness/DEPRECATED.md
```

## What This Measures

This is not a robot-capability leaderboard. It measures whether a claimed
manipulation success survives a disciplined verifier:

- Can the target task be represented as observable object-centric CSG facts?
- Can a solver produce only rollout evidence, with no target facts copied into
  the extraction side?
- Can the independent extractor recover the right contacts, relations, events,
  and articulation changes from that rollout?
- Do the unchanged hard probes accept scripted successful rollouts, reject
  invalid fixtures, and make a no-op baseline fail for intelligible reasons?

The scripted MuJoCo arm is intentionally modest. Its job is to make the
verification loop physically grounded enough to be falsifiable.

## The three rules that make this project credible

1. **No target leakage.** The robot CSG is generated from rollout traces
   only. The rollout artifact (`csg.rollout.v0`) is the information-flow
   boundary: it carries only what a simulator with no access to the
   demonstration could honestly report. Default answer to "can the rollout
   carry X?" is **no**. Enforced by `tests/test_leakage.py` and the benchmark
   leakage gate.
2. **The verifier is frozen.** Solvers, perception, and extractors improve
   until the *unchanged* hard probes pass. Nobody weakens a probe to make a
   rollout pass. Acceptance = every hard probe agrees + leakage-clean +
   physical validity true or honestly "not checked". The scalar distance is
   a diagnostic, never the criterion.
3. **Honesty over impressiveness.** The symbolic backend never claims
   physical validity (`physicalValidity: None`, labeled "physics-unverified").
   Failure variants and failure taxonomy are part of every report. Claims
   stay within the allowed list in `roadmap.md` §1.

## For the team taking over

Read in this order:

1. `roadmap.md` — scope, claims, phases, what NOT to build
2. `physical_quotient.md` §0 + §0.b — verifier semantics and overrides
   (where prose and code disagree, **code wins**)
3. `csg/rollout_schema.md` — the leakage contract you must not break
4. `csg/validity.md` — implemented physical-validity checks and reporting contract
5. Top of `Causal_Skill_Graph_V0.md` — schema audit notes

**Phase 2C gold-task coverage is implemented.** The MuJoCo backend (`csg/backends/mujoco/`) now takes
all five V0 gold tasks end-to-end — compiled scene → MJCF arm → scripted task
controller → `csg.rollout.v0` frames → independent extraction → frozen matcher
PASS — with a *real* `physicalValidity: true` verdict from the checks in
`csg/validity.md`, verified by gated tests and:

```bash
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion
.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco \
  --confusion --randomized --seeds 30 --require-pass
```

It slots in at `csg/solver.py`'s seam (`backend="mujoco"`) without touching the
matcher, extractor, or leakage gate. Pick/insert/place/open grasps are
weld-assisted (fingers close on the object for genuine gripper feasibility and
finger contact; a weld holds the scripted transport; the weld is released before
placement so quasi-static support is judged honestly) — `stable_grasp_quality`
stays on `hiddenVariablesNotUsed`.

Current Phase 2E release endpoint (see `roadmap.md`):

```text
A credible sim-only benchmark and verification framework for fixed-base
robotic-arm manipulation.
```

Release discipline:

- Keep the workspace under Git and generated artifacts out of versioned source,
  so diffs and frozen-file status are auditable.
- Keep the MIT `LICENSE` and `pyproject.toml` license metadata aligned.
- Regenerate the benchmark report artifacts from the committed checkout before
  tagging a release.
- Keep reproducibility docs current with exact symbolic and MuJoCo commands,
  expected results, Python/MuJoCo notes, and output locations.
- Keep randomized rollout evidence current; the 30-seed MuJoCo sweep now
  samples all five V0 tasks, including push starts via shared x translation.

Working agreements:

- Keep `python3 -m pytest tests/ -q` green and
  `python3 -m csg.benchmark gold_tests --confusion --require-pass` exit-0 on
  every change.
- Any addition to the rollout format requires a schema-version bump and a
  review of the sanitization whitelist + leakage tests
  (`csg/rollout_schema.md` §Versioning).
- New tasks get a gold fixture set (success + failure variants +
  `expected.json`) and a confusion-matrix check; genuine quotient
  equivalences are *documented* in `KNOWN_EQUIVALENT_TASKS`, not silenced.
- Old phase labels (6C, "Phase 7", …) in code comments map to the new plan
  via the table in `roadmap.md` §5.
