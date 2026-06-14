# Real-camera external-trace pilot (Phase 3A)

Feed constrained **Sony/tripod `object_inside_container` video** through the *frozen* csg
verifier as an **evidence source that JUDGES recorded episodes** — PASS / FAIL /
**UNCERTAIN** — not a compiler that authors task descriptions (that is Phase 3B). The
camera matters only through calibration metadata; the rollout + verifier interface is
device-agnostic. `csg/` is never imported into or modified.

```
video ─▶ marker observations ─▶ real_camera.tracks.v0 ─▶ csg.rollout.v0 ─▶ frozen verifier
        marker_tracker.py        video_to_tracks.py       tracks_to_rollout.py   verify_episode.py
        (ArUco, observations      (role-map + pose          (ONLY place evidence    (UNCERTAIN gate,
         only)                     estimate, occlusion)      is minted; neutralise)  then match)
```

## Module responsibilities (strict boundaries)

| module | does | does NOT |
|---|---|---|
| `marker_tracker.py` | detect ArUco markers in one frame → `MarkerObservation` (id + pixel corners) | estimate poses, build tracks, judge anything |
| `video_to_tracks.py` | frames → `real_camera.tracks.v0` (role-map markers, estimate world poses, allow occlusion) | mint rollout evidence |
| `calibrate_table.py` | author/validate/hash `real_camera.calibration.v0` (intrinsics, table→world, marker map, object geometry) | — |
| `tracks_to_rollout.py` | **the only place rollout evidence is minted**: tracks → leakage-clean `csg.rollout.v0` via the shared `pilots.external_rollout` door | decide PASS/FAIL/UNCERTAIN |
| `verify_episode.py` | fail-closed UNCERTAIN gate, then run the frozen verifier vs the two targets; derive useful failure classes | modify `csg/` or the shared verifier |

`cv2`/`numpy` are an **optional** extra (`pip install -e ".[camera]"`), imported lazily and
isolated to the detector + PnP pose estimator. The whole **tracks → rollout → verifier**
seam (and the entire test suite) runs with **neither installed** — detection and pose
estimation are injectable, so synthetic fixtures + a fake detector cover the pipeline; the
real OpenCV path is smoke-tested only (`pytest.importorskip("cv2")`).

## The two targets (a strictly-stronger pair)

`targets/object_inside_container_terminal_only.json` asserts only the cube's **terminal**
relation is INSIDE (`goal_satisfaction`). `targets/object_inside_container_relation_event.json`
adds: started **NEAR** (`initial_state`), ended INSIDE (`terminal_state` +
`relation_transitions`), and a **CONTAINMENT_CHANGE** event present (`event_presence`).

A **born-inside** clip (cube inside the whole time, but moving) PASSes terminal-only and
**FAILs relation-event on `initial_state`** — *not* on the event/transition. This is the
load-bearing subtlety: the frozen extractor (`csg/rollout_extract.py`) seeds
`prev_rel="NEAR"` unconditionally, so any cube ending inside emits a NEAR→INSIDE
CONTAINMENT_CHANGE delta; only the robot's **first relation** (INSIDE vs NEAR) distinguishes
born-inside from a real put-in. The relation-event target therefore authors an explicit
initial-NEAR relation so `initial_state` does the rejecting.

Neither target lives in `gold_tests/` — they are pilot diagnostics.

## Three disciplines

- **Quarantine / leakage.** Source identity (tag ids, role names like `cube`/`tray`,
  colours) lives in tracks + the dataset manifest and is **dropped** at `tracks_to_rollout`:
  the rollout carries only neutral `body_NNN` ids, an empty `objectIdMap`, and whitelisted
  body fields. The shared `assert_rollout_leakage_clean` re-checks this at the door.
- **UNCERTAIN, fail-closed.** `verify_episode` runs an evidence-quality gate *before* the
  verifier: low marker confidence, high dropout, a weak/occluded **endpose** (the verdict
  hinges on BOTH the initial-NEAR and terminal-INSIDE frames), or an over-jittery static
  container → `status:"UNCERTAIN"` (`perception_failure` / `extractor_uncertainty`), never a
  PASS. The frozen extractor rewrites pose confidence to 1.0, so this gate is the only place
  perception uncertainty is honored — hence fail-closed, before the rollout is minted.
  Thresholds in `verify_episode.DEFAULT_THRESHOLDS` are provisional — recalibrate on the
  first real capture.
- **Physics-unverified.** A camera trace cannot re-check physics, so `physicalValidity` is
  always `null` (honest "physics-unverified", never "valid").

## CLI

```bash
# (real video, needs the camera extra) video -> tracks
python3 -m pilots.real_camera.video_to_tracks \
  --video datasets/sony_object_inside_container_v0/raw_videos/<ep>.MP4 \
  --calibration datasets/sony_object_inside_container_v0/calibration/sony_table_v0.calibration.json \
  --out datasets/sony_object_inside_container_v0/tracks/<ep>.tracks.json

# tracks -> leakage-clean rollout (no cv2 needed)
python3 -m pilots.real_camera.tracks_to_rollout \
  --tracks datasets/sony_object_inside_container_v0/tracks/<ep>.tracks.json \
  --out datasets/sony_object_inside_container_v0/rollouts/<ep>.rollout.json

# judge an episode (PASS / FAIL / UNCERTAIN) vs both targets
python3 -m pilots.real_camera.verify_episode \
  --tracks datasets/sony_object_inside_container_v0/tracks/<ep>.tracks.json --json
```

## Capture protocol (for the real campaign)

- Tripod; fixed zoom, focus, exposure, white balance; stable diffuse lighting; ~28 mm if
  the whole workspace fits.
- One ArUco marker on a **known face** of the cube (record the fixed marker→center offset);
  fixed markers on/around the tray and table for the table→world board.
- **Recalibrate** (new `calibration.v0`, new `markerMapHash`) whenever zoom, focus distance,
  resolution, camera position, or the table/marker layout changes.
- Record a success/failure set: successes, near-not-inside, rim placement, dropped outside,
  missed grasp, wrong object (if a second marked object is available), and occluded/uncertain
  clips. The output is `csg.rollout.v0`, **not** a target CSG.

## Dataset

`datasets/sony_object_inside_container_v0/` holds `raw_videos/` (gitignored), `calibration/`,
`tracks/`, `rollouts/`, `reports/`, and `manifest.json` (episode index + per-target expected
verdicts + raw-video sha256/path). The committed episodes are currently **synthetic**
(hand-authored marker tracks, no real video) to exercise the pipeline; real captures append
entries with a real `videoSha256`.
