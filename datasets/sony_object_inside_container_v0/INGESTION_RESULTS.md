# Phase 3A — Real-Camera Ingestion Results (`object_inside_container`)

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
2. **relation_event requires a NEAR start.** Real put-ins often start FAR (cube ~15–20 cm away),
   so `initial_state` (which needs the robot's first relation to be NEAR) fails — terminal_only
   passes. This is the authored target's deliberate "started near AND achieved inside" claim; for
   far-start captures it is strict. Revisiting the target's initial relation (accept "not
   already inside" rather than exactly NEAR) is a **target-design** decision, left to the user.
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

## Calibration-clip note

`oic_calibration_caliper75mm_cube50mm_001` contains the marked **50 mm** task cube (tags 2/3), a
separate **unmarked ~30 mm** scale cube, and a 75 mm vernier caliper. The cube we track is the
marked 50 mm one — consistent with the manifest; **no label fix needed**.
