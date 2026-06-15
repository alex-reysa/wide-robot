# RH20T Annotation Sidecar (`rh20t.annotation.v0`)

The first RH20T smoke test uses a **reviewed annotation sidecar** because RH20T does
not ship ready-made task-object pose tracks. This file is **source evidence, not target
authoring**: it records estimated/depth-backed object + container world poses, per-pose
confidence, and provenance for selected frames of one episode. It must never be derived
from a target CSG (that would be leakage); it is derived from the RH20T video/depth/
calibration by a human reviewer on RunPod.

## Required fields

- `schemaVersion`: `"rh20t.annotation.v0"`
- `episodeId`: RH20T scene id (e.g. `task_0017_user_0001_scene_0001_cfg_0003`)
- `source`: object with `dataset`, `taskId`, `taskDescription`, `scenePath`,
  `archiveSha256` (from `RH20T_cfg3.sha256.txt`)
- `fps`: track sampling rate (number > 0)
- `objects`: ORDERED list of source objects; index `i` maps to neutral `body_{i:03d}`.
  For `object_inside_container` the convention is `mover` = `body_000`,
  `container` = `body_001`. Each object needs:
  - `sourceRole` (e.g. `"mover"`, `"container"` — quarantined; never reaches the rollout)
  - `physicalKind` (e.g. `"RIGID_OBJECT"`)
  - `mobility` (`"MOVABLE"` for the mover, `"STATIC"` for the container)
  - `isContainer` (bool)
  - `sizeM` (`[x, y, z]` extent in metres; estimated sizes are honest — the rollout marks
    `sizeApproximate=true`)
- `frames`: sampled frames, each with `frameIndex`, `timeS`, and a `poses` map carrying
  EVERY object's `positionM` (`{x, y, z}` world metres) plus a numeric `confidence`.
- `review`: free-form provenance — `annotator`, `date`, `cameraSerial`, `method`
  (how poses were estimated, e.g. depth deprojection vs. manual), `uncertaintyNotes`.

## Endpoint coverage required for the smoke test

For the relation-event target to be judged non-vacuously, the sidecar must include:

```text
>= 2 initial frames with the mover NEAR and NOT inside the container
1 transition/midpoint frame
>= 3 terminal frames with the mover INSIDE the container
the container pose in every frame
a numeric confidence for every pose
review.method explaining how positions were estimated
source.archiveSha256 from RH20T_cfg3.sha256.txt
```

## Fail-closed conversion

`annotations_to_tracks` (then `validate_tracks_v0`) rejects sidecars with fewer than
three frames, a missing object pose in any frame, non-numeric confidence, non-monotonic
timestamps, duplicate source roles, or missing object sizes. A rejected sidecar surfaces
as an UNCERTAIN `source_evidence_invalid` verdict from `verify_episode`, never a PASS.

Do **not** tune the sidecar to force a PASS. If the endpoint evidence is incomplete or
ambiguous, record the honest FAIL/UNCERTAIN verdict and the reason.
