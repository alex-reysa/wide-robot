# Phase 3A Real-Camera Handoff

Date: 2026-06-16  
Branch: `phase-3a-real-camera-ingestion`  
Current head observed locally: `75a8e2d CI: skip numpy-requiring rlbench recorder test without the [camera] extra`

## Current State

This repo is in Phase 3A: real-camera episode ingestion for `object_inside_container`.

The Phase 3A camera work has already produced and pushed two key commits on this branch:

```text
f52a205 Phase 3A: real-camera batch ingestion + frozen-verifier validation (78 clips)
c964111 Phase 3A: put-in transition target + false-negative audit (20cm footprint tested, rejected)
```

There is also a later CI commit:

```text
75a8e2d CI: skip numpy-requiring rlbench recorder test without the [camera] extra
```

The defensible Phase 3A claim is:

```text
Conservative marker-based real-camera pilot complete:
real Sony/iPhone videos are converted to tracks/rollouts and judged by the unchanged verifier.
The failure side is safe: 0 false PASSes on genuine failures.
Success recall is partial and calibration-limited.
```

Do **not** claim fully solved real-camera ingestion.

## Important Result

Primary result file:

```text
datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md
```

Full verdict artifact:

```text
datasets/sony_object_inside_container_v0/verdicts_all.json
```

Key facts from the final run:

```text
78 task clips judged
0 errors
0 false PASSes on genuine-failure classes
terminal success recall:
  Sony:   10/16
  iPhone:  8/16
combined placed-from-outside transition recall: 18/32
born-inside transition rejection: 8/8 FAIL
csg/ byte-frozen
```

The attempted global tray-footprint expansion from `18 cm` to `20 cm` was tested and rejected because it created a genuine false PASS on `near_not_inside_001 sony`. The final state is reverted to `18 cm` and documented as conservative but calibration-limited.

## Current Local Dirt

At the time this handoff was written, `git status --short` showed unrelated local changes:

```text
 M README.md
 M docs/rlbench_external_trace_pilot.md
 M pilots/rlbench/targets/open_drawer_rlbench_articulation_event.json
 M tests/test_rlbench_articulation_event.py
?? docs/dataset_publishing_runbook.md
?? docs/superpowers/plans/2026-06-16-phase-3a-next-steps.md
?? output/
?? recordings/SHA256SUMS
```

Treat those as separate concerns unless the user explicitly asks about them. Do not revert them without permission.

## Python Runtime

Do not use bare `python` or Homebrew `python3` for the camera path.

The user tried:

```bash
python -m pilots.real_camera.visualize_episode ...
python3 -m pilots.real_camera.visualize_episode ...
```

and hit:

```text
zsh: command not found: python
/opt/homebrew/opt/python@3.14/bin/python3.14: No module named ...
```

Homebrew Python is also known to have a `pyexpat` / `libexpat` mismatch on this machine.

Use the bundled Python:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3
```

The camera extra was installed there and OpenCV worked:

```text
cv2 4.13.0
numpy 2.3.5
DICT_APRILTAG_36h11 available
```

Run camera tests with:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_*.py -q
```

Expected from the recent state:

```text
66 passed
```

## What The User Wants Next

The user wants a way to visualize how the computer maps the real clips:

```text
overlay the virtual tray / cube / inside boundary over video frames
show detected tag IDs and corners
show how the computer detects/maps each element
support manual calibration if possible
avoid a second recording if honest manual source binding can salvage the current capture
```

The command the user tried does **not** exist yet:

```bash
python -m pilots.real_camera.visualize_episode \
  --episode oic_success_001 \
  --camera sony_front \
  --frame terminal \
  --out output/overlays/
```

Next task is to implement this module.

## Existing Useful Artifacts

Scratch overlays already exist but are not productized:

```text
output/frames/audit_oic_success_001__sony_front.jpg
output/frames/audit_oic_success_004__iphone_top.jpg
output/frames/audit_oic_success_005__iphone_top.jpg
output/frames/audit_oic_success_007__iphone_top.jpg
output/frames/audit_oic_success_013__iphone_top.jpg
output/frames/audit_oic_success_014__sony_front.jpg
output/frames/fp_near001_sony.jpg
```

Scratch scripts may contain reusable ideas:

```text
output/scratch_project.py
output/scratch_geom.py
output/scratch_inside.py
output/preview_footprint.py
```

These are scratch only. Prefer creating a clean module under `pilots/real_camera/`.

## Files To Read First

Read these before implementing visualization:

```text
pilots/real_camera/author_calibration.py
pilots/real_camera/video_to_tracks.py
pilots/real_camera/track_postprocess.py
scripts/ingest_recordings.py
pilots/real_camera/tracks_to_rollout.py
pilots/real_camera/verify_episode.py
csg/predicates.py
datasets/sony_object_inside_container_v0/verdicts_all.json
datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md
recordings/manifest.json
```

Keep `csg/` byte-frozen.

## Proposed Visualization Module

Create:

```text
pilots/real_camera/visualize_episode.py
```

Suggested CLI:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  -m pilots.real_camera.visualize_episode \
  --episode oic_success_001 \
  --camera sony_front \
  --frame terminal \
  --out output/overlays/
```

Useful options:

```text
--episode       episodeId from recordings/manifest.json, e.g. oic_success_001
--camera        sony_front or iphone_top
--frame         terminal | start | middle | <integer frame index>
--out           output directory
--show-shrunk-inside-footprint
--show-tags
--show-centers
```

Output:

```text
output/overlays/oic_success_001__sony_front__terminal.jpg
```

The overlay should draw:

```text
detected AprilTag corners and IDs
virtual tray center
virtual tray footprint
virtual tray rim/top rectangle
shrunk INSIDE footprint from csg.predicates.DEFAULT.inside_footprint_margin_m
cube center
cube bounding box projection if practical
terminal primary relation: INSIDE / NEAR / ON_TOP_OF / none
verdict status from verdicts_all.json if available
```

Minimum viable version:

```text
draw tag corners + IDs
draw tray footprint rectangle
draw shrunk INSIDE footprint
draw cube center
write relation/verdict text
```

If projecting full 3D boxes is slow, start with footprint and centers.

## Where To Get Data

Raw video path comes from:

```text
recordings/manifest.json
```

For a given `episodeId` + `camera`, use:

```text
relativePath
```

Tracks are under:

```text
datasets/sony_object_inside_container_v0/tracks/<episodeId>__<camera>.tracks.json
```

Per-clip calibration is under:

```text
datasets/sony_object_inside_container_v0/calibration/perclip/<episodeId>__<camera>.calibration.json
```

Rollout, if not UNCERTAIN, may be under:

```text
datasets/sony_object_inside_container_v0/rollouts/<episodeId>__<camera>.rollout.json
```

Verdicts:

```text
datasets/sony_object_inside_container_v0/verdicts_all.json
```

## Implementation Notes

Use OpenCV for video frame extraction and drawing. Import it lazily inside the real CLI path.

For projecting world points into image pixels:

The calibration stores `cameraToWorld`, but drawing needs world-to-camera. Invert the 4x4 matrix:

```python
M = np.array(calibration["cameraToWorld"], dtype=np.float64)
world_to_camera = np.linalg.inv(M)
```

Then for world point `p_world`:

```python
p = np.array([x, y, z, 1.0])
pc = world_to_camera @ p
u = fx * pc[0] / pc[2] + cx
v = fy * pc[1] / pc[2] + cy
```

Guard against `pc[2] <= 0`.

Tray box dimensions come from the calibration object whose `sourceRole == "tray"`:

```text
sizeM = [x, y, z]
```

The static tray pose comes from the selected frame in tracks:

```text
frame["poses"]["tray"]["positionM"]
```

The cube pose comes from:

```text
frame["poses"]["cube"]["positionM"]
```

To draw the tray footprint, use four world corners at `z = tray_center_z + tray_size_z / 2` or `z = tray_center_z` depending on readability. To draw the actual INSIDE footprint, shrink X/Y by:

```python
from csg.predicates import DEFAULT
margin = DEFAULT.inside_footprint_margin_m
```

So the shrunk half extents are:

```python
hx_inside = tray_size_x / 2 - margin
hy_inside = tray_size_y / 2 - margin
```

Use colors:

```text
green:  cube center / cube
red:    virtual tray outer footprint
yellow: shrunk INSIDE footprint
cyan:   detected tags
white:  text labels
```

## Tests To Add First

Follow TDD. Add tests before implementation.

Create or extend:

```text
tests/test_real_camera_visualize.py
```

Suggested cv2-free tests:

1. `select_frame_index`:

```text
terminal chooses the last frame containing a cube pose
start chooses the first frame containing a cube pose
integer frame selects exact frame
```

2. `tray_footprint_corners`:

```text
given center (0,0,0), size (0.18,0.18,0.07), returns 4 corners with ±0.09 x/y
```

3. `inside_footprint_corners`:

```text
given margin 0.005, returns ±0.085 x/y for a 0.18 m tray
```

4. `project_world_point`:

```text
identity camera/world, simple camera matrix, point at z=1 projects to principal point
```

Then run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_visualize.py -q
```

After implementation, also run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_*.py -q
```

## Manual Calibration / Annotation Idea

The user asked whether manual work can avoid a second recording.

Answer: yes, possibly, but the manual input should be tray boundary annotation, not just caliper scale.

Most useful manual annotation:

```text
pick one calibration frame
click the four inner tray corners
label front/back/left/right
optionally click the 75 mm caliper endpoints as a scale sanity check
freeze that calibration before rerunning all clips
```

Do not tune per clip.

If implementing manual annotation, prefer a source-binding sidecar:

```text
datasets/sony_object_inside_container_v0/calibration/manual_tray_corners_v0.json
```

Example schema:

```json
{
  "schemaVersion": "real_camera.manual_tray_corners.v0",
  "sourceVideo": "recordings/raw_videos/calibration/oic_calibration_caliper75mm_cube50mm_001__sony_front.mp4",
  "frameIndex": 0,
  "camera": "sony_front",
  "innerTrayCornersPx": {
    "frontLeft": [0, 0],
    "frontRight": [0, 0],
    "backRight": [0, 0],
    "backLeft": [0, 0]
  },
  "caliper75mmPx": {
    "endpointA": [0, 0],
    "endpointB": [0, 0]
  },
  "notes": "Replace placeholder coordinates with clicked pixel coordinates."
}
```

Build a separate tool for this later:

```text
pilots/real_camera/manual_calibration.py
```

Do not mix this into `visualize_episode.py` initially.

## Why Caliper Alone Is Not Enough

The printed AprilTags already provide metric scale. The current limiting issue is not mainly gross scale; it is the mapped tray boundary:

```text
tray center
tray yaw
tray inner footprint
camera-to-world alignment
occlusion of tray marker 7
```

The 75 mm caliper is useful as a sanity check for scale, but the higher-value manual input is the actual inner tray boundary.

## Acceptance Criteria For Visualization

A fresh agent should consider the visualization task done when:

```text
python -m pilots.real_camera.visualize_episode ...   # with bundled python path
produces a JPEG overlay for at least:
  oic_success_001 sony_front terminal
  oic_fail_near_not_inside_001 sony_front terminal
  oic_success_013 iphone_top terminal

overlay includes tag IDs, tray footprint, inside footprint, cube center, and relation/verdict text
tests/test_real_camera_visualize.py passes
tests/test_real_camera_*.py passes
csg/ diff is empty
```

## Suggested First Command In A New Session

Use:

```bash
cd "/Users/alejandro/Desktop/999. PROJECTS/wide-robot"
git status --short
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_*.py -q
```

Then implement `pilots.real_camera.visualize_episode`.

