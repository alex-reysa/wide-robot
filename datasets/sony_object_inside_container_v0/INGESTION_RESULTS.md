# Phase 3A — Real-Camera Ingestion Results (`object_inside_container`)

> **Status: conservative Phase 3A pilot — NOT "fully-solved real-camera ingestion."**
> The defensible claim is narrow and strong: **across 78 real clips the frozen verifier produced
> 0 false PASSes on 30 genuine-failure clips**, with **partial success recall** (terminal target
> ~50–62%; the misses are conservative — borderline placements → UNCERTAIN/FAIL, never a wrong-way
> PASS). It is a marker-based, approximate-calibration pilot that demonstrates the
> source-independent verifier's *safety* survives real evidence — not a high-recall perception system.

Real video → `real_camera.tracks.v0` → `csg.rollout.v0` → the **unchanged** frozen verifier
(`pilots.external_verify.verify_external_rollout`; `csg/` byte-frozen). 40 episodes × 2 cameras
(`sony_front` 45°, `iphone_top`) = 80 clips; 78 task clips judged (2 calibration clips excluded),
**0 errors**. Both bundled targets run per clip: `object_inside_container_terminal_only` and
`object_inside_container_relation_event`.

Reproduce: `python -m scripts.ingest_recordings --select all`
(needs the `[camera]` extra; artifacts under `tracks/`, `rollouts/`, `calibration/perclip/`,
`verdicts_all.json`).

## Headline: the verifier's safety survives the real-evidence path

- **0 false PASSes** across **30 genuine-failure clips** (near-not-inside, left-on-rim,
  dropped-outside, inside→outside, static-no-task). Every genuine failure resolved to **FAIL or
  UNCERTAIN** on both targets — the verifier never green-lights a failure from real camera input.
- **born-inside → relation_event FAIL: 8/8.** The relation-event target correctly rejects a cube
  that was inside from the start (the `initial_state` probe), exactly as designed.

The errors are all **conservative** (false negatives on success → UNCERTAIN/FAIL), driven by
*approximate* marker calibration — never permissive. That is the correct bias for a verifier.

## Confusion matrix (terminal status; "fit" = matches the recorded oracle)

| expectedClass | cam | n | terminal status | term fit | rel fit |
|---|---|--:|---|--:|--:|
| success | iphone | 16 | PASS 8 / FAIL 4 / UNC 4 | 8/16 | 3/16 |
| success | sony | 16 | PASS 10 / FAIL 5 / UNC 1 | 10/16 | 2/16 |
| near_not_inside | iphone | 4 | FAIL 2 / UNC 2 | 2/4 | 2/4 |
| near_not_inside | sony | 4 | FAIL 3 / UNC 1 | 3/4 | 3/4 |
| left_on_rim | iphone | 3 | FAIL 2 / UNC 1 | 2/3 | 2/3 |
| left_on_rim | sony | 3 | FAIL 3 | 3/3 | 3/3 |
| dropped_or_left_outside | iphone | 5 | FAIL 2 / UNC 3 | 2/5 | 2/5 |
| dropped_or_left_outside | sony | 5 | FAIL 5 | 5/5 | 5/5 |
| inside_to_outside | both | 2+2 | FAIL (1 UNC on iphone) | 3/4 | 3/4 |
| static_no_task | both | 1+1 | FAIL | 2/2 | 2/2 |
| born_inside(+hand_motion) | both | 8 | FAIL (terminal) | 0/8 | **8/8** |
| inside_to_inside | both | 1+1 | FAIL/UNC | 0/2 | 1/2 |
| success_tag/hand_obstruction | both | 6 | FAIL | 0/6 | — |

(Full per-clip records: `verdicts_all.json`. Genuine-failure classes show **no PASS** anywhere.)

## What works well

- **All clear failures are caught.** dropped-outside, left-on-rim, near-not-inside,
  inside→outside, static — 0/30 false PASS; Sony resolves nearly all to a clean FAIL with the
  right `cameraFailureClass`.
- **Successes are recognised** on the terminal target ~50–62% (sony 10/16, iphone 8/16); the
  rest are borderline placements (cube ending near a tray wall) that approximate calibration
  reads as NEAR — conservative, not wrong-way.
- **born-inside is rejected** by relation_event on both cameras (8/8).

## Honest limitations (verifier/perception properties, not calibration bugs)

1. **born-inside terminal is unjudgeable.** The recorded born cubes have ~**0.000 m net**
   (first→last) displacement, so the frozen extractor's motion-based figure-ground finds no
   "figure" → no terminal relation → terminal FAIL. The relation-event verdict is still correct.
   A born demo is only terminal-judgeable if the cube has net intra-tray motion.
2. **relation_event requires a NEAR start — RESOLVED, see the Update below.** Real put-ins start
   FAR *or* NEAR (measured: 18 FAR / 9 NEAR across the minted successes), and the frozen matcher's
   `initial_state` can't express "NEAR-or-FAR" in one target. We keep `relation_event` (NEAR) and
   add a sibling `placed_from_outside` (FAR); the combined put-in **transition = relation_event
   PASS OR placed_from_outside PASS** now equals terminal recall while born-inside fails both.
3. **iPhone top vs Sony front trade-off.** The top view keeps the cube tag visible but loses the
   tray's only floor tag (7) when the cube is placed inside and is noisy on the vertical (rim)
   axis → more UNCERTAIN. The 45° Sony view resolves the terminal relation more decisively (more
   clean FAIL/PASS) but occludes the cube during the place motion. Neither alone is ideal;
   fusing both is future work.
4. **Obstruction controls (tag/hand) → FAIL.** Deliberate occlusion degrades cube tracking enough
   that the cube reads near/outside; the oracle's PASS_OR_UNCERTAIN isn't met. Honest.

## Calibration approach (all pilot-side; `csg/` untouched)

- **Intrinsics** derived from lens/sensor at each clip's real resolution (Sony 4K @ 24 mm
  APS-C ≈ 36 mm-equiv; iPhone 1080p ≈ 26 mm-equiv), zero distortion, `calibrationQuality:
  approximate`. Scale comes from the printed AprilTag sizes; `fx` was **not** tuned to fit verdicts.
- **Extrinsic (world Z = table-up)** recovered per clip from the flat markers 0/1/7 (averaged
  normals), **origin anchored at marker 7** (near the objects, so the residual tilt acts over
  ~10 cm not ~0.5 m — this is what keeps the cube's z from falsely dropping below the tray floor),
  **in-plane yaw aligned to the tray** so the extractor's axis-aligned box matches the rotated tray.
- **Tray geometry** from the marker6↔7 midpoint (≈ tray depth), shared across cameras via the
  co-anchored (markers 0/1) world frame; box extended ~3 cm below the floor to tolerate top-view
  vertical noise (rim unchanged → INSIDE-vs-ON_RIM preserved).
- **Tracks post-processing** (`pilots/real_camera/track_postprocess.py`): trim to the
  cube-observed span, interpolate short cube-occlusion gaps, hold the STATIC tray at its fitted
  center. Evidence-quality thresholds relaxed for 30 fps video (consecutive-missing 5→30,
  dropout 0.2→0.35); confidence/endpose/static-jitter gates unchanged (fail-closed preserved).

## Update — target semantics decided + false-negative audit

**Target semantics (decided).** Added `pilots/real_camera/targets/object_inside_container_placed_from_outside.json`
(initial **FAR_FROM** → INSIDE), the FAR-start sibling of `relation_event` (initial NEAR). It is
evaluated in the ingest layer (the canonical bundle stays the RLBench-parity pair). Combined
**transition = relation_event OR placed_from_outside**:

| | successes (32) | every genuine failure (30) |
|---|---|---|
| terminal_only PASS | 18 | **0** |
| relation_event (NEAR) PASS | 5 | 0 |
| placed_from_outside (FAR) PASS | 13 | 0 |
| **combined transition PASS** | **18 (= terminal recall)** | **0** |

born-inside → transition **8/8 FAIL**. Net: transition recall now equals terminal recall with the
0-false-PASS safety intact. Overall terminal-AND-transition matches: **40/78** (was 27/78). Three
cv2-free unit tests pin the NEAR/FAR/born behaviour; `csg/` byte-frozen; 69 real_camera tests pass.

**False-negative audit (5 terminal-FAIL / UNCERTAIN successes, classified from the final frame +
geometry).** The misses are **not** genuine ambiguity and **not** rim cases — in every frame the
cube is *visibly, clearly inside the tray*:

| clip | cam | model says | verdict |
|---|---|---|---|
| success_005 | iPhone | `is_inside=True`, cube missing 50 frames | **OCCLUSION** (correct conservative UNCERTAIN) |
| success_013 | iPhone | 1.8 cm past modeled back wall | **CALIBRATION** (tray y off ~2 cm) |
| success_004 | iPhone | 0.2 cm past wall, touching left wall | **CALIBRATION at the margin** |
| success_014 | Sony | 1.3 cm past wall (oblique view) | **CALIBRATION** |
| success_001 | Sony | 0.2 cm past shrunk footprint, at back wall | **CALIBRATION / borderline** |

**Root cause:** the modeled tray footprint (18×18 cm, axis-aligned, marker-derived center) is
**~1–2 cm too small/forward**, so cubes placed toward the back or against a wall fall just past the
model.

**Effective-footprint expansion — tested and REJECTED.** Per a fixed acceptance protocol (one global
expansion, no per-clip tuning, no `csg/` change, rerun all 78, accept only if failure false-PASSes
stay 0), a single global **18 → 20 cm** ("effective/calibrated footprint", not the physical tray
size) was evaluated. Result: **+2 terminal success recall (18 → 20) but ONE false PASS** —
`near_not_inside_001 sony`, where the cube sits *clearly outside, beside the tray*, gets pulled
INSIDE by the enlarged box (its gap goes from −2.5 cm-at-edge to −3.4 cm-inside; `left_on_rim` min
gap also reaches 0.0 cm). The ~1–2 cm calibration error is comparable to the spacing between "inside
against the wall" and "outside against the wall", so **no global footprint can separate them**.
→ Reverted to **18 cm**; the capture is **conservative but calibration-limited**. Recovering the
footprint false-negatives would require better calibration (e.g. proper per-camera intrinsics, a
tray-edge fit instead of a marker-center+nominal-size box), not a footprint fudge.

`relation_event`'s NEAR-vs-FAR limitation is tracked separately above (resolved via the combined
transition); it is independent of this footprint result.

## Calibration-clip note

`oic_calibration_caliper75mm_cube50mm_001` contains the marked **50 mm** task cube (tags 2/3), a
separate **unmarked ~30 mm** scale cube, and a 75 mm vernier caliper. The cube we track is the
marked 50 mm one — consistent with the manifest; **no label fix needed**.
