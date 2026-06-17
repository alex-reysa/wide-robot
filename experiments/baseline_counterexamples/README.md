# Baseline counterexamples — terminal predicates encode a weaker task

> **These examples do not show that wide-robot beats a bad predicate.**
> They show that common *terminal* predicates encode a **weaker question** than the
> task actually asks. wide-robot checks the whole object-state story — role
> existence, **initial** state, **terminal** containment, the **transition** between
> them, **evidence confidence**, and **leakage** discipline. A single-condition
> terminal predicate checks only the last of those, so it answers "is the object
> at a spot that looks like success in the final frame?" — not "did the *task*
> happen?"
>
> This is **single-condition terminal predicate vs. structured, leakage-clean
> verifier** — *not* learned-verifier vs. hand-coded predicate. Every predicate
> here, including the baselines, is hand-coded; they differ only in **how much of
> the task definition they encode**. To make that undeniable we include **B5**, the
> *strongest possible single-frame terminal predicate* (the verifier's own
> `csg.is_inside` on the last frame). B5 closes the rim case — and **still** cannot
> see the two things that actually define a put-in:
>
> 1. the **transition** (born-inside) — *irreducibly structural*: it needs two time
>    points and the target-aware initial-state/transition check inside the matcher;
>    no single-frame predicate can ever close it;
> 2. the **evidence quality** (occlusion) — caught by a **fail-closed evidence
>    gate**. Honest caveat: that gate (`assess_evidence_quality`) is *target-blind*
>    and runs *before* the matcher, so it is **separable** — any baseline could bolt
>    on the same check. wide-robot bundles it; a bare terminal predicate has it by
>    default no more than it has the transition check.
>
> So the *irreducibly structural* residue is **born-inside**; occlusion is the
> residue of **fail-closed evidence discipline**. A single-condition terminal
> predicate has neither.

All evidence below is real: 78 committed Sony/iPhone `object_inside_container`
clips (`datasets/sony_object_inside_container_v0/`), scored by the ladder in
[`baseline_predicates.py`](baseline_predicates.py) and by the **frozen**
`csg.matcher` through `pilots.real_camera.verify_episode`. `csg/` is read-only
here. Reproduce — with **no OpenCV and no raw video** (the tests pass with cv2
import-blocked and the mp4s removed):

```bash
python3 -m scripts.build_baseline_counterexamples          # full (renders overlays; needs cv2 + local mp4s)
python3 -m scripts.build_baseline_counterexamples --no-overlays   # JSON/CSV/MD only
python3 -m pytest tests/test_baseline_counterexamples.py -q       # no video / no cv2 required
```

## The baseline ladder

Five hand-coded "did the cube end up in the tray?" checks, climbing in strictness.
Each reads at most the **first and last** frame and uses the *same* box geometry as
the verifier (`csg.predicates`), so the comparison is fair:

| id | question it answers |
|----|---------------------|
| **B1** center-in-footprint | terminal cube **center** within the tray's outer footprint (2D, ignores rim height) |
| **B2** footprint-overlap | terminal cube **footprint** overlaps the tray footprint ≥ 0.5 (2D) |
| **B3** full-inner-containment | terminal cube footprint **fully** inside the shrunk inner region (2D, 5 mm margin) |
| **B4** contained + started-outside | B3 **and** the cube started outside the footprint (2D + initial state) |
| **B5** terminal-3D-containment | **`csg.is_inside` on the last frame**: shrunk footprint **and** rim height (3D) — the maximal single-frame terminal predicate |

What no baseline can see: a real **outside→inside transition** vs. born-inside
(none of B1–B5), or whether the evidence was good enough to certify anything
(none of B1–B5). B1/B2 additionally ignore the **rim height**; **B5 fixes that**,
which is the whole point of including it — the gap that survives B5 is structural,
not a 2D-vs-3D artifact.

## Three lessons, escalating

| clip | human | B1 | B2 | B3 | B4 | **B5 (max terminal)** | wr terminal_only | wr structured | lesson |
|---|---|---|---|---|---|---|---|---|---|
| `oic_fail_on_rim_001__iphone_top` | rim FAIL | **PASS** | **PASS** | reject | reject | **reject** | FAIL/LEFT_ON_RIM | FAIL/LEFT_ON_RIM | **dimensionality**: 2D center → 3D containment |
| `oic_control_inside_to_inside_001__sony_front` | born-inside FAIL | **PASS** | **PASS** | reject | reject | **PASS** ❗ | **PASS** ❗ | FAIL/BORN_INSIDE_NO_TRANSITION | **transition**: only the initial-state check rejects it |
| `oic_success_005__iphone_top` | success (occluded) | PASS | PASS | PASS | PASS | **PASS** ❗ | UNCERTAIN | UNCERTAIN | **evidence**: 50-frame dropout, fail-closed |

Read top to bottom, the gap that survives gets deeper:

1. **Rim — a dimensionality lesson, stated honestly.** A single-condition center
   predicate (B1) calls it a success because the cube's center, projected straight
   down, lands inside the footprint. The verifier rejects it (`ON_TOP_OF`, not
   `INSIDE`) — and so does **B5** and even the verifier's *weakest* `terminal_only`
   target. So the rim does **not** require "structure"; it requires **rim-height
   awareness**. We keep it as the visual flagship (below) because it is the most
   legible failure, but the honest claim is "a *single-condition* terminal predicate
   is insufficient," not "only a structured verifier can catch this."

2. **Born-inside — the first genuinely structural gap.** The cube is inside the
   whole time; it is never *placed*. It ends inside, so **B5 — the strongest
   single-frame terminal predicate — PASSES**, and so does the verifier's
   `terminal_only` target (which encodes the same weak definition). Only the
   structured targets, which require the cube to have **started outside** and
   crossed in, reject it (`initial_state`). No terminal predicate, however
   sophisticated, can close this — it needs two points in time.

3. **Occlusion — an evidence-discipline gap (separable, not "structure").** A
   genuine success, but the cube marker is occluded for **50 consecutive frames**.
   The first and last frames look perfect, so **every baseline including B5
   certifies it**. wide-robot returns UNCERTAIN via its fail-closed evidence gate
   (`assess_evidence_quality`). Crucially — and we say so plainly — that gate is a
   **target-blind preprocessing check** that runs *before* the matcher and depends
   only on `(tracks, thresholds)`; it is **separable** and any baseline could adopt
   the identical check. So occlusion shows the value of **fail-closed evidence
   discipline**, which wide-robot bundles in, not the value of structure per se.

**The sharpened thesis:** even the maximal single-frame terminal predicate (B5)
false-passes the born-inside clips and the occluded successes. The *irreducibly
structural* residue — *did a real outside→inside transition happen?* — can never be
reached by any single-frame predicate; the *evidence* residue — *did we actually
see the trajectory?* — is reached by a fail-closed gate that wide-robot includes by
default and a bare terminal predicate does not. wide-robot has both; a
single-condition terminal predicate has neither.

## The flagship visual: `oic_fail_on_rim_001__iphone_top`

![rim overlay](cases/rim_edge/overlay_final_frame.png)

The cube is **left balanced on the rim** (green wireframe on the near edge,
terminal frame 379). Its center lands inside the tray's outer footprint (red), so
B1/B2 say success; the verifier sees `ON_TOP_OF` and says **FAIL / LEFT_ON_RIM**.

### Robustness — the rejection is solid, the spurious PASS is not
Quantified in [`cases/rim_edge/robustness_perturbation.json`](cases/rim_edge/robustness_perturbation.json)
(14 calibration perturbations: tray center ±5/±10 mm in x and y, cube size 4–6 cm):

- **wide-robot `terminal_only` is NEVER PASS** across all 14 perturbations — the
  cube sits **+26 mm above the rim+slack**, a large margin. B5 likewise rejects in
  all 14. The verifier's rejection is **calibration-robust**.
- **B1's PASS is knife-edge**: the cube center is only **+5.4 mm** inside the
  nearest (back) footprint edge, so a single ~10 mm shift (`tray_y-10mm`) flips B1
  to reject. We disclose this rather than imply the PASS is robust — and it
  *reinforces* the lesson: a height-blind center test is not just wrong on the rim,
  it is **unstably** wrong, resolving a boundary case by ignoring the very quantity
  (height) that decides containment.

This is also the **only** one of the six `left_on_rim` clips where B1 PASSes (the
other five have the cube center *outside* the footprint, so there is no false
certification to overturn) — the calibration sensitivity is load-bearing and we
say so.

## Aggregate over all 78 clips

(see [`results_table.md`](results_table.md) / [`results_table.csv`](results_table.csv))

- **40** clips are human-non-successes (rim, near-not-inside, dropped, born-inside, static, removed).
- naive **B1** falsely PASSes **11** of them (every born-inside / inside-to-inside variant, plus the rim clip).
- **B5** — the maximal single-frame terminal predicate — *still* falsely PASSes **10** of them: the born-inside-family clips (`born_inside*` + `inside_to_inside`, all "cube ends inside without a valid placement"). It correctly drops the rim; everything else a terminal check can't see remains.
- wide-robot **`terminal_only` falsely PASSes 3** — born-inside-family clips that end inside (including the `inside_to_inside` clip). *This is the verifier asked the weak question*, and it is the point.
- wide-robot **structured** (`relation_event` ∨ `placed_from_outside`) **falsely PASSes 0**.
- On the 38 human-successes, structured **certifies 27**, fail-closes **5 to UNCERTAIN** (occlusion), and **hard-FAILs 6** (see limitation).

### Reproducibility + independent corroboration
- **Reproducibility** ([`reproducibility_check.json`](reproducibility_check.json)):
  our recompute reproduces the verdicts stored in the committed dataset on
  **78/78 clips, 0 disagreements**. *Honest scope:* `verdicts_all.json` is produced
  by `scripts/ingest_recordings.py` calling the **same** `verify_episode`, so this is
  a regression/reproducibility check (the experiment uses the production verifier
  unchanged) — **not** independent corroboration.
- **Independent geometry** ([`independent_geometry_check.json`](independent_geometry_check.json)):
  a **from-scratch** reimplementation of the terminal-containment geometry (separate
  code, no `csg.predicates` logic; threshold values pinned to `csg.predicates.DEFAULT`
  and asserted equal in the tests) reproduces the verifier's extracted terminal
  relation on **57/57** clips where the verifier emits one (14 are occlusion-gated
  before extraction; 7 have no cube motion, so the verifier's figure-ground step emits
  no relation). This is genuine two-implementation agreement on the containment core.

### Honest limitation
The 6 hard-FAILed successes are the `hand_obstruction` / `tag_obstruction` variants:
a brief occlusion corrupts the terminal marker pose without crossing the evidence
gate's dropout thresholds, so the structured target sees a wrong terminal relation
and FAILs (a real false-negative, not UNCERTAIN). This is a known limitation of the
marker-only 3A pipeline under partial occlusion — it is **not** part of the thesis
(which is about false *positives* / weak definitions), and it is reported here
rather than hidden. It is also the opposite error from the baselines: the verifier
errs toward *not* certifying, never toward a false success.

### Provenance (not rigged)
Each featured clip's `source_info.json` records the `videoSha256` and
`calibrationHash`; the tray geometry is the committed, frozen calibration derived
from the marker-7 physical offset (`calibration/manual_tray_corners_*_v0.json`,
anchored on `oic_success_001` frame 0) — the experiment scripts only **read** it.
The experiment uses the **same** targets (`pilots/real_camera/targets/`) and the
**same** thresholds (`max_consecutive_missing=30, max_dropout_frac=0.35`) as the
dataset ingest pipeline; the only experiment-specific constant is B2's overlap
fraction (0.5), documented in `baseline_predicates.py`.

## Files

```
README.md                       <- this file
baseline_predicates.py          <- pure B1..B5 (no cv2 / no video), reused by the tests
results_table.csv / .md         <- all 78 clips, ladder B1..B5 + 3 wide-robot targets
reproducibility_check.json      <- 78/78 recompute == committed dataset verdicts (SAME verifier; regression check)
independent_geometry_check.json <- 57/57 from-scratch geometry reimpl == verifier terminal relation (genuine 2nd impl)
cases/<case>/
  source_info.json              <- clip identity, hashes, geometry, human label, provenance
  naive_predicate_results.json  <- B1..B5 verdicts + the question each asks
  wide_robot_report.json        <- headline + full records for all 3 structured targets
  overlay_final_frame.png        <- terminal frame: tray footprint (red) / inner region (yellow) / cube (green)
cases/rim_edge/robustness_perturbation.json   <- 14-perturbation table: wide-robot FAIL never flips to PASS
wide_robot_reports/             <- full verifier dumps for the featured clips
```

Raw mp4s are **not** committed (repo `*.mp4` ignore rule); they are present locally
for overlay regeneration. The tracked proof is the JSON/CSV/MD/PNG, the source
hashes in each `source_info.json`, and the reproducible scripts above.
