# Cross-source report — `object_inside_container`, One Task / Four Worlds

The same semantic task — **did the object end up inside the container, having been *put* there
(a real outside→inside transition), not merely born inside?** — bound to four worlds and judged
by the **same frozen verifier core** (`csg.matcher.match` + `csg.rollout_extract.extract_robot_csg`):

| World | What it is | Verifier path | physicalValidity |
|---|---|---|---|
| **MuJoCo** | internal physics sim (`put_cube_in_tray`) | `extract_robot_csg → match → leakage_report` (== `csg.benchmark.run_one`) | **true** (physics re-checked) |
| **RLBench** | external sim (`PutItemInDrawer`, 9 live demos) | `verify_external_rollout` | null (physics-unverified) |
| **Sony/iPhone** | real camera (78 marker-tracked clips) | `pilots.real_camera.verify_episode` (UNCERTAIN gate → verifier) | null |
| **RH20T** | real-robot video (pen→holder, +derived negative) | `verify_external_rollout` | null |

**Headline:** **0 false-PASSes on every non-success clip in every world**, leakage-clean throughout,
and `physicalValidity` reported honestly — `true` only where physics was actually re-checked (the
internal sim), `null` for the three external worlds (a recorded trace cannot be physics-revalidated).

## Why this is one proof, not four pilots

- **One frozen core.** `verify_external_rollout` (the three external worlds) and `csg.benchmark.run_one`
  (MuJoCo) call the *identical* `csg.matcher.match` + `extract_robot_csg`. `csg/` is byte-frozen.
- **Same semantic task, instantiated per source.** Each world has its OWN target card (different
  `graphId`, object labels, geometry provenance) — they are NOT one shared file. `target_equivalence.json`
  proves every card reduces to the *same verifier-enforced semantic core* per tier (all ids canonicalised
  to roles, authoring-only fields stripped). The claim is "same semantics, per-source card", not "same file".
- **Two comparison tiers.** `terminal_only` ("did it merely END inside?" — what a born-inside episode
  passes) and the STRUCTURED tier (`relation_event` near-start OR `placed_from_outside` far-start — a real
  put-in). The gap between them is the whole point: the structured tier rejects born-inside via `initial_state`.

## Regenerate

```bash
python3 -m scripts.build_cross_source_report
```

Recomputes every verdict live from committed inputs — **no MuJoCo / RLBench / cv2 install needed** —
and rewrites the artifacts below. Output is timestamp-free, so a re-run is byte-identical.

## Files

| file | what it is |
|---|---|
| `cross_source_report.md` | the public-facing narrative + master scoreboard + honest caveats |
| `cross_source_report.json` | the full structured record (every clip, every leg) |
| `summary.csv` | one row per clip across all four worlds (skeptic spreadsheet) |
| `target_equivalence.json` | proof the per-source target cards share one enforced semantic core |
| `leakage_report.json` | per-world leakage cleanliness |
| `source_manifest.json` | provenance + committed input paths + clip counts per world |
| `per_world/<key>.json` | per-leg detail dump (MuJoCo / RLBench / Sony / RH20T) |
| `mujoco_fixture/` | the committed real MuJoCo solver rollout + validity report (see its `PROVENANCE.md`) |

Pinned by `tests/test_cross_source_report.py` (regenerated values, not hand-copied), including the
`csg/`-byte-frozen guard and the same-core identity check.

## Honest caveats (also in the report)

- **Two verifier paths, one core** — MuJoCo skips the external-trace leakage *door* (its legitimate
  internal `objectIdMap` would be rejected there) but runs the identical matcher + extractor.
- **physicalValidity is honest, not uniform** — `true` only for MuJoCo; a `null` PASS is *physics-unverified*, never *valid*.
- **Failure data is uneven** — Sony (40 non-success) + RH20T (1 negative) carry explicit failures; the two
  sim worlds are success-only in their committed live data, so discrimination is shown via the wrong-tier
  rejection (RLBench) and the committed sabotage corpus (MuJoCo: 4/4 sabotaged variants correctly FAIL).
- **Sony hard-FAIL false-negatives are real** — a few genuine successes hard-FAIL when brief obstruction
  corrupts the terminal relation without tripping the evidence gate. Reported, not hidden; a perception
  limit of marker-only 3A capture, not a verifier claim.
