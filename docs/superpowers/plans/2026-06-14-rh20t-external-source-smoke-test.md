# RH20T External-Source Smoke Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove whether RH20T can act as a source-independent recorded-evidence input to the frozen verifier without downloading raw RH20T media locally or claiming Sony/ArUco Phase 3A is complete.

**Architecture:** Treat RH20T as a new pilot source, not as `real_camera` and not as Phase 3B target compilation. The pilot converts selected RH20T episode evidence into neutral object tracks, then into `csg.rollout.v0` through `pilots.external_rollout.assemble_rollout`, and verifies with the unchanged frozen verifier.

**Tech Stack:** Python 3.11+ stdlib for the verifier seam, optional `numpy` for RH20T `.npy` metadata, optional OpenCV/ffmpeg on RunPod for video inspection, pytest for regression gates, RunPod network volume for raw RH20T media.

---

## Current Facts And Constraints

- RH20T provides real robot manipulation episodes with multi-view RGB/RGBD, robot low-dimensional data, calibration, task metadata, and corresponding human-demo video.
- The official page lists containment-like tasks suitable for this smoke test: `task_0017` "Put the pen into the pen holder", `task_0072` "Drop coins into a piggy bank", `task_0073` "Put things in the drawer", and `task_0091` "Move an object from one box to another".
- RH20T does not appear to provide ready-made task-object pose tracks. The smoke test must include an honest source extraction or human-assisted annotation step for object/container poses; it must not synthesize passing poses from the target.
- Raw RH20T video is large and may include faces/voices. Keep raw media on RunPod storage and commit only derived JSON plus provenance hashes.
- `csg/` is frozen for this work. The new code lives under `pilots/rh20t/`, tests under `tests/test_rh20t_*.py`, and derived fixture JSON under `datasets/rh20t_object_inside_container_v0/`.
- This is a Phase 3A.5 style checkpoint: it may validate the external-source evidence seam, but it does not replace Sony/tripod recording and does not begin Phase 3B target generation.

## File Structure

- Create: `pilots/rh20t/__init__.py`
  - Package marker and source-contract docstring.
- Create: `pilots/rh20t/README.md`
  - Explain RH20T source boundaries, RunPod-only raw media, annotation/extraction contract, and verification commands.
- Create: `pilots/rh20t/annotations_schema.md`
  - Define the human-assisted annotation sidecar used for the first smoke test.
- Create: `pilots/rh20t/annotations_to_tracks.py`
  - Convert reviewed RH20T annotation sidecars into `rh20t.tracks.v0`.
- Create: `pilots/rh20t/tracks_to_rollout.py`
  - Convert `rh20t.tracks.v0` into leakage-clean `csg.rollout.v0` with `backend="rh20t_external"` and `skill_source="rh20t"`.
- Create: `pilots/rh20t/verify_episode.py`
  - Run an RH20T track or rollout against RH20T pilot targets using `pilots.external_verify.verify_external_rollout`.
- Create: `pilots/rh20t/targets/object_inside_container_terminal_only.json`
  - RH20T-specific metadata copy of the terminal INSIDE target.
- Create: `pilots/rh20t/targets/object_inside_container_relation_event.json`
  - RH20T-specific metadata copy of the initial NEAR -> terminal INSIDE/event target.
- Create: `datasets/rh20t_object_inside_container_v0/manifest.json`
  - Derived fixture manifest. No raw video paths that point outside the RunPod volume are committed.
- Create: `datasets/rh20t_object_inside_container_v0/{annotations,tracks,rollouts,reports}/.gitkeep`
  - Derived-data layout matching the Sony pilot shape.
- Modify: `.gitignore`
  - Ignore `datasets/rh20t_object_inside_container_v0/raw/`, downloaded archives, extracted RH20T trees, frame dumps, and local RunPod logs.
- Modify: `pyproject.toml`
  - Add optional extra `rh20t = ["numpy>=1.24"]` only if implementation reads `.npy` directly in repo code. Do not add OpenCV unless the final extractor actually imports it.
- Test: `tests/test_rh20t_rollout.py`
  - Synthetic RH20T tracks prove target behavior and leakage discipline without raw media.
- Test: `tests/test_rh20t_cli.py`
  - CLI and manifest smoke tests.

## Task 0: RunPod Dataset Triage

**Files:**
- No repo changes.
- Raw downloads live outside git at `/workspace/datasets/rh20t_raw/`.

- [ ] **Step 1: Create a RunPod pod with persistent storage**

Use a RunPod network volume or a pod volume mounted at `/workspace`. Size it for the first attempt:

```text
Minimum for cfg3 RGB + cfg3 depth + extraction scratch: 120 GB
Comfortable for retries and frame dumps: 250 GB
```

Expected: SSH/Jupyter terminal opens and `df -h /workspace` shows enough free space.

- [ ] **Step 2: Install repo and lightweight download tools on RunPod**

Run on RunPod:

```bash
cd /workspace
git clone https://github.com/alex-reysa/wide-robot.git wide-robot
cd /workspace/wide-robot
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip pytest gdown
python3 -m pip install -e ".[dev]"
mkdir -p /workspace/datasets/rh20t_raw
```

Expected: `python3 -m pytest tests/test_real_camera_rollout.py -q` exits 0 or shows only environment-gated skips already known in the repo.

- [ ] **Step 3: Download only the candidate RH20T shard to RunPod**

Start with the 640x360 cfg3 RGB+robot-info archive and cfg3 depth archive because the official RH20T page exposes direct Google Drive file links for both and cfg3 is one of the smaller 640x360 configs.

```bash
cd /workspace/datasets/rh20t_raw
gdown --fuzzy "https://drive.google.com/file/d/1uwieq-EbA_eTXE668ekypQV1cO9PDfES/view?usp=sharing" -O RH20T_cfg3.tar.gz
gdown --fuzzy "https://drive.google.com/file/d/1aekLEcX1ruS9f2z6900ys5t_U_OJnEzQ/view?usp=sharing" -O RH20T_cfg3_depth.tar.gz
sha256sum RH20T_cfg3.tar.gz RH20T_cfg3_depth.tar.gz | tee RH20T_cfg3.sha256.txt
```

If Google Drive quota blocks `gdown`, configure `rclone` with the user's Google Drive or use the RH20T Baidu links from the official page. Expected: archives exist under `/workspace/datasets/rh20t_raw/` and hashes are recorded.

- [ ] **Step 4: Check whether cfg3 contains a candidate containment task**

Run:

```bash
cd /workspace/datasets/rh20t_raw
tar -tzf RH20T_cfg3.tar.gz \
  | rg "task_(0017|0072|0073|0091).*metadata\\.json" \
  | tee cfg3_candidate_metadata.txt
```

Expected success case: at least one `task_0017`, `task_0072`, `task_0073`, or `task_0091` metadata path appears.

Expected stop case: no candidate task appears. Record `cfg3_candidate_metadata.txt` in the report and stop this attempt with status `blocked_no_candidate_task_in_cfg3`; choose the next smallest cfg archive from the RH20T page before writing adapter code around real data.

- [ ] **Step 5: Extract one candidate scene and calibration only**

Choose one metadata path from `cfg3_candidate_metadata.txt` and export it as `RH20T_SCENE`, for example:

```bash
export RH20T_SCENE=RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003
mkdir -p /workspace/datasets/rh20t_raw/extracted_cfg3
tar --wildcards -xzf RH20T_cfg3.tar.gz -C /workspace/datasets/rh20t_raw/extracted_cfg3 \
  "RH20T_cfg3/calib/*" \
  "$RH20T_SCENE/*"
tar --wildcards -xzf RH20T_cfg3_depth.tar.gz -C /workspace/datasets/rh20t_raw/extracted_cfg3 \
  "$RH20T_SCENE/cam_*/depth.mp4" \
  "$RH20T_SCENE/cam_*/timestamps.npy"
```

Expected: `/workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE` contains `metadata.json`, one or more `cam_*` folders, `color.mp4`, `timestamps.npy`, and, if depth was extracted successfully, `depth.mp4`.

- [ ] **Step 6: Produce an eligibility report before implementation**

Create `/workspace/wide-robot/datasets/rh20t_object_inside_container_v0/reports/eligibility_report.md` with:

```markdown
# RH20T Eligibility Report

- Source archive hashes: recorded in `/workspace/datasets/rh20t_raw/RH20T_cfg3.sha256.txt`
- Candidate scene: value of `$RH20T_SCENE`, for example `RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003`
- Task id: derived from `$RH20T_SCENE`, for example `task_0017`
- License class: `scene_0001`-`scene_0005` = RH20T-C / CC BY-SA; `scene_0006`-`scene_0010` = RH20T-NC / non-commercial
- Available cameras: output of `find /workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE -maxdepth 1 -type d -name 'cam_*' -print`
- Has color video: yes/no
- Has depth video: yes/no
- Has calibration folder: yes/no
- Object/container visible in at least one global camera: yes/no
- Initial relation visually not inside: yes/no
- Terminal relation visually inside: yes/no
- Decision: `usable_for_smoke_test`, `blocked_no_visible_container`, `blocked_missing_depth`, or `blocked_ambiguous_task`
```

Expected: decision is `usable_for_smoke_test` before any PASS claim is attempted.

## Task 1: Add Synthetic RH20T Rollout Tests

**Files:**
- Create: `tests/test_rh20t_rollout.py`
- Create: `pilots/rh20t/__init__.py`
- Create: `pilots/rh20t/targets/object_inside_container_terminal_only.json`
- Create: `pilots/rh20t/targets/object_inside_container_relation_event.json`

- [ ] **Step 1: Write failing tests for source identity and target semantics**

Create `tests/test_rh20t_rollout.py` with synthetic `rh20t.tracks.v0` fixtures. The tests should mirror the proven `tests/test_real_camera_rollout.py` behavior but assert RH20T provenance:

```python
from pathlib import Path

from csg.common import load_json
from pilots.external_rollout import assert_rollout_leakage_clean
from pilots.external_verify import verify_external_rollout
from pilots.rh20t.tracks_to_rollout import tracks_to_rollout

REPO = Path(__file__).resolve().parents[1]
TARGETS = REPO / "pilots" / "rh20t" / "targets"
TERMINAL = TARGETS / "object_inside_container_terminal_only.json"
REL_EVENT = TARGETS / "object_inside_container_relation_event.json"

TX, TY, TZ = 0.30, 0.0, 0.015
CUBE = [0.04, 0.04, 0.04]
TRAY = [0.24, 0.18, 0.03]
START_NEAR = (TX + 0.16, TY, 0.05)
INSIDE = (TX, TY, 0.03)
NEAR_NOT_INSIDE = (TX + 0.13, TY, 0.05)


def tracks(cube_seq):
    return {
        "schemaVersion": "rh20t.tracks.v0",
        "episodeId": "task_0017_user_0001_scene_0001_cfg_0003",
        "source": {
            "dataset": "RH20T",
            "taskId": "task_0017",
            "taskDescription": "Put the pen into the pen holder",
            "scenePath": "RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003",
            "archiveSha256": "0" * 64,
        },
        "fps": 10.0,
        "objects": [
            {"sourceRole": "mover", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
             "isContainer": False, "sizeM": CUBE},
            {"sourceRole": "container", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
             "isContainer": True, "sizeM": TRAY},
        ],
        "frames": [
            {"frameIndex": i, "timeS": i / 10.0, "poses": {
                "mover": {"positionM": {"x": x, "y": y, "z": z}, "confidence": 0.95},
                "container": {"positionM": {"x": TX, "y": TY, "z": TZ}, "confidence": 0.99},
            }}
            for i, (x, y, z) in enumerate(cube_seq)
        ],
    }


def approach_then(end_xyz):
    sx, sy, sz = START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz), ((sx + ex) / 2, (sy + ey) / 2, (sz + ez) / 2),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def verify(target_path, rollout):
    return verify_external_rollout(load_json(target_path), rollout, case_name="rh20t_oic")


def test_rh20t_success_passes_both_targets_leakage_clean():
    rollout = tracks_to_rollout(tracks(approach_then(INSIDE)))
    assert rollout["backend"] == "rh20t_external"
    assert rollout["skillProgram"]["source"] == "rh20t"
    assert rollout["diagnostics"]["physicalValidity"] is None
    assert rollout["objectIdMap"] == {}
    assert_rollout_leakage_clean(rollout)
    assert verify(TERMINAL, rollout)["passed"] is True
    assert verify(REL_EVENT, rollout)["passed"] is True


def test_rh20t_near_not_inside_fails_both_targets():
    rollout = tracks_to_rollout(tracks(approach_then(NEAR_NOT_INSIDE)))
    assert verify(TERMINAL, rollout)["passed"] is False
    rel = verify(REL_EVENT, rollout)
    assert rel["passed"] is False
    assert "goal_satisfaction" in rel["hardMismatches"]


def test_rh20t_source_identity_is_quarantined():
    rollout = tracks_to_rollout(tracks(approach_then(INSIDE)))
    blob = __import__("json").dumps(rollout)
    for forbidden in ("task_0017", "pen", "holder", "mover", "container", "RH20T_cfg3"):
        assert forbidden not in blob
```

- [ ] **Step 2: Add minimal RH20T package and targets**

Create `pilots/rh20t/__init__.py`:

```python
"""RH20T external-source pilot.

This package treats RH20T as recorded episode evidence. It converts selected,
reviewed RH20T episodes into neutral rollouts for the frozen verifier. It does
not prove the Sony/ArUco camera path and does not compile target CSGs.
"""
```

Create RH20T targets by copying the semantic structure of `pilots/real_camera/targets/object_inside_container_*.json`, but change only metadata fields:

```json
"graphId": "rh20t_object_inside_container_relation_event"
"pilotMetadata": {
  "diagnostic": "rh20t-object-inside-container-relation-event",
  "source": "RH20T selected containment episode; raw media stays on RunPod",
  "notAGoldTask": "Pilot diagnostic, deliberately NOT added to gold_tests/. It asserts relation evidence only and does not assert contact causality."
}
```

- [ ] **Step 3: Run tests to verify failure**

Run:

```bash
python3 -m pytest tests/test_rh20t_rollout.py -q
```

Expected: import failure for `pilots.rh20t.tracks_to_rollout` or missing targets.

## Task 2: Implement `rh20t.tracks.v0 -> csg.rollout.v0`

**Files:**
- Create: `pilots/rh20t/tracks_to_rollout.py`
- Test: `tests/test_rh20t_rollout.py`

- [ ] **Step 1: Implement RH20T track validation**

Create `pilots/rh20t/tracks_to_rollout.py` with:

```python
#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from csg.common import Json, load_json, write_json
from pilots.external_rollout import _IDENTITY_WXYZ, assemble_rollout

TRACKS_SCHEMA_VERSION = "rh20t.tracks.v0"
RH20T_BACKEND = "rh20t_external"
MIN_TRACK_FRAMES = 3
MOTION_EPS_M = 0.005
NEUTRAL_ID_FMT = "body_{:03d}"


class RH20TTracksError(ValueError):
    pass


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise RH20TTracksError(msg)


def _pos_xyz(pose: Mapping[str, Any], where: str) -> List[float]:
    _require(isinstance(pose, Mapping), f"{where}: pose must be an object")
    p = pose.get("positionM")
    _require(isinstance(p, Mapping), f"{where}: positionM missing")
    try:
        return [float(p["x"]), float(p["y"]), float(p["z"])]
    except (KeyError, TypeError, ValueError) as exc:
        raise RH20TTracksError(f"{where}: positionM must contain numeric x/y/z: {exc}")


def _roles(tracks: Mapping[str, Any]) -> List[str]:
    return [str(obj["sourceRole"]) for obj in tracks["objects"]]


def validate_tracks_v0(tracks: Mapping[str, Any]) -> None:
    _require(isinstance(tracks, Mapping), "tracks must be an object")
    _require(tracks.get("schemaVersion") == TRACKS_SCHEMA_VERSION,
             f"schemaVersion must be {TRACKS_SCHEMA_VERSION!r}")
    _require(bool(tracks.get("episodeId")), "episodeId is required")
    fps = tracks.get("fps")
    _require(isinstance(fps, (int, float)) and fps > 0, "fps must be a positive number")
    objects = tracks.get("objects")
    _require(isinstance(objects, list) and len(objects) >= 2, "objects must contain mover and container")
    roles = []
    for i, obj in enumerate(objects):
        _require(isinstance(obj, Mapping), f"objects[{i}] must be an object")
        for key in ("sourceRole", "physicalKind", "mobility", "sizeM"):
            _require(key in obj, f"objects[{i}] missing {key}")
        size = obj["sizeM"]
        _require(isinstance(size, Sequence) and not isinstance(size, str) and len(size) >= 3,
                 f"objects[{i}].sizeM must be [x,y,z]")
        roles.append(str(obj["sourceRole"]))
    _require(len(set(roles)) == len(roles), "sourceRole values must be unique")
    frames = tracks.get("frames")
    _require(isinstance(frames, list) and len(frames) >= MIN_TRACK_FRAMES,
             f"frames must contain at least {MIN_TRACK_FRAMES} entries")
    last_t: Optional[float] = None
    for fi, frame in enumerate(frames):
        _require(isinstance(frame, Mapping), f"frames[{fi}] must be an object")
        t = frame.get("timeS")
        _require(isinstance(t, (int, float)), f"frames[{fi}].timeS must be numeric")
        if last_t is not None:
            _require(float(t) >= last_t, f"frames[{fi}].timeS is not monotonic")
        last_t = float(t)
        poses = frame.get("poses")
        _require(isinstance(poses, Mapping), f"frames[{fi}].poses must be an object")
        for role in roles:
            _require(role in poses and poses[role] is not None,
                     f"frames[{fi}] missing pose for {role!r}")
            _pos_xyz(poses[role], f"frames[{fi}].poses[{role}]")
            conf = poses[role].get("confidence", 1.0)
            _require(isinstance(conf, (int, float)) and not isinstance(conf, bool),
                     f"frames[{fi}].poses[{role}].confidence must be numeric")
```

- [ ] **Step 2: Implement neutral rollout assembly**

Add to the same file:

```python
def _neutral_body(index: int, obj: Mapping[str, Any]) -> Json:
    size = list(obj["sizeM"])
    bid = NEUTRAL_ID_FMT.format(index)
    return {
        "objectId": bid,
        "bodyId": bid,
        "physicalKind": str(obj["physicalKind"]),
        "mobility": str(obj["mobility"]),
        "sizeM": [float(size[0]), float(size[1]), float(size[2])],
        "sizeApproximate": bool(obj.get("sizeApproximate", True)),
        "isContainer": bool(obj.get("isContainer", False)),
    }


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    return ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def tracks_to_rollout(tracks: Mapping[str, Any]) -> Json:
    validate_tracks_v0(tracks)
    objects = list(tracks["objects"])
    roles = _roles(tracks)
    frames_in = list(tracks["frames"])
    bodies = [_neutral_body(i, obj) for i, obj in enumerate(objects)]
    role_to_bid = {role: NEUTRAL_ID_FMT.format(i) for i, role in enumerate(roles)}

    static_clamp: Dict[str, List[float]] = {}
    static_excursion: Dict[str, float] = {}
    for obj, role in zip(objects, roles):
        if str(obj["mobility"]) != "STATIC":
            continue
        xyzs = [_pos_xyz(frame["poses"][role], f"poses[{role}]") for frame in frames_in]
        med = [_median([xyz[k] for xyz in xyzs]) for k in range(3)]
        static_clamp[role] = med
        static_excursion[role] = max(
            (sum((xyz[k] - med[k]) ** 2 for k in range(3)) ** 0.5 for xyz in xyzs),
            default=0.0,
        )

    min_conf = 1.0
    out_frames: List[Json] = []
    for frame in frames_in:
        object_poses: Json = {}
        for role in roles:
            pose = frame["poses"][role]
            xyz = static_clamp.get(role) or _pos_xyz(pose, f"poses[{role}]")
            min_conf = min(min_conf, float(pose.get("confidence", 1.0)))
            object_poses[role_to_bid[role]] = {
                "frameId": "world",
                "positionM": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
                "orientationWxyz": dict(_IDENTITY_WXYZ),
                "confidence": float(pose.get("confidence", 1.0)),
            }
        out_frames.append({
            "timeS": float(frame["timeS"]),
            "phase": "external",
            "effectorPose": {
                "frameId": "world",
                "positionM": {"x": 0.0, "y": 0.0, "z": 1.0},
                "orientationWxyz": dict(_IDENTITY_WXYZ),
                "confidence": 0.0,
            },
            "gripperClosed": False,
            "objectPoses": object_poses,
            "articulation": {},
        })

    diagnostics = {
        "episodeId": str(tracks["episodeId"]),
        "sourceDataset": "RH20T",
        "sourceTaskId": (tracks.get("source") or {}).get("taskId"),
        "archiveSha256": (tracks.get("source") or {}).get("archiveSha256"),
        "fps": float(tracks["fps"]),
        "numTrackFrames": len(out_frames),
        "minPoseConfidence": min_conf,
        "maxStaticBodyMotionM": max(static_excursion.values(), default=0.0),
        "staticBodyClampApplied": sorted(role_to_bid[r] for r in static_clamp),
        "source": "rh20t",
    }
    return assemble_rollout(
        bodies=bodies,
        frames=out_frames,
        object_id_map={},
        backend=RH20T_BACKEND,
        skill_source="rh20t",
        physical_validity_reason=(
            "external RH20T trace: csg cannot re-check physics from dataset tracks; "
            "physics-unverified by contract (csg/validity.md)"
        ),
        extra_diagnostics=diagnostics,
    )
```

- [ ] **Step 3: Add CLI**

Add:

```python
def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Convert rh20t.tracks.v0 into csg.rollout.v0")
    parser.add_argument("--tracks", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    rollout = tracks_to_rollout(load_json(Path(args.tracks)))
    write_json(Path(args.out), rollout)
    print(f"rh20t tracks_to_rollout: wrote {args.out} backend={rollout['backend']} "
          f"frames={len(rollout['frames'])} physicalValidity={rollout['diagnostics']['physicalValidity']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Verify synthetic tests pass**

Run:

```bash
python3 -m pytest tests/test_rh20t_rollout.py -q
```

Expected: all tests in `tests/test_rh20t_rollout.py` pass.

## Task 3: Add RH20T Annotation Sidecar Conversion

**Files:**
- Create: `pilots/rh20t/annotations_schema.md`
- Create: `pilots/rh20t/annotations_to_tracks.py`
- Test: `tests/test_rh20t_cli.py`

- [ ] **Step 1: Document the sidecar schema**

Create `pilots/rh20t/annotations_schema.md`:

```markdown
# RH20T Annotation Sidecar

The first RH20T smoke test uses a reviewed annotation sidecar because RH20T
does not ship ready-made task-object tracks. This file is source evidence, not
target authoring: it records estimated object/container world poses, confidence,
and provenance for selected frames.

Required fields:

- `schemaVersion`: `rh20t.annotation.v0`
- `episodeId`: RH20T scene id
- `source`: dataset, task id, task description, scene path, archive sha256
- `fps`: track sampling rate
- `objects`: ordered source objects with `sourceRole`, `physicalKind`,
  `mobility`, `isContainer`, `sizeM`
- `frames`: sampled frames with `frameIndex`, `timeS`, and per-role pose
  `positionM` plus numeric `confidence`
- `review`: annotator, date, camera serial, extraction method, uncertainty notes

The converter rejects sidecars with fewer than three frames, missing endpoint
poses, non-numeric confidence, non-monotonic timestamps, or missing object sizes.
```

- [ ] **Step 2: Write CLI test over a minimal sidecar**

Create `tests/test_rh20t_cli.py`:

```python
import json
from pathlib import Path

from csg.common import load_json
from pilots.rh20t.annotations_to_tracks import annotations_to_tracks
from pilots.rh20t.verify_episode import verify_episode_both


def sidecar():
    return {
        "schemaVersion": "rh20t.annotation.v0",
        "episodeId": "task_0017_user_0001_scene_0001_cfg_0003",
        "source": {
            "dataset": "RH20T",
            "taskId": "task_0017",
            "taskDescription": "Put the pen into the pen holder",
            "scenePath": "RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003",
            "archiveSha256": "0" * 64,
        },
        "fps": 10.0,
        "objects": [
            {"sourceRole": "mover", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
             "isContainer": False, "sizeM": [0.04, 0.04, 0.04]},
            {"sourceRole": "container", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
             "isContainer": True, "sizeM": [0.24, 0.18, 0.03]},
        ],
        "frames": [
            {"frameIndex": 0, "timeS": 0.0, "poses": {
                "mover": {"positionM": {"x": 0.46, "y": 0.0, "z": 0.05}, "confidence": 0.9},
                "container": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.015}, "confidence": 0.99}}},
            {"frameIndex": 1, "timeS": 0.1, "poses": {
                "mover": {"positionM": {"x": 0.38, "y": 0.0, "z": 0.04}, "confidence": 0.9},
                "container": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.015}, "confidence": 0.99}}},
            {"frameIndex": 2, "timeS": 0.2, "poses": {
                "mover": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.03}, "confidence": 0.9},
                "container": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.015}, "confidence": 0.99}}},
            {"frameIndex": 3, "timeS": 0.3, "poses": {
                "mover": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.03}, "confidence": 0.9},
                "container": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.015}, "confidence": 0.99}}},
        ],
        "review": {"method": "unit-test synthetic sidecar"},
    }


def test_annotations_to_tracks_then_verify_both():
    tracks = annotations_to_tracks(sidecar())
    assert tracks["schemaVersion"] == "rh20t.tracks.v0"
    result = verify_episode_both(tracks=tracks)
    assert result["object_inside_container_terminal_only"]["status"] == "PASS"
    assert result["object_inside_container_relation_event"]["status"] == "PASS"
```

- [ ] **Step 3: Implement annotation conversion**

Create `pilots/rh20t/annotations_to_tracks.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Mapping, Optional

from csg.common import Json, load_json, write_json
from pilots.rh20t.tracks_to_rollout import TRACKS_SCHEMA_VERSION, validate_tracks_v0

ANNOTATION_SCHEMA_VERSION = "rh20t.annotation.v0"


class RH20TAnnotationError(ValueError):
    pass


def annotations_to_tracks(annotation: Mapping[str, Any]) -> Json:
    if not isinstance(annotation, Mapping):
        raise RH20TAnnotationError("annotation must be an object")
    if annotation.get("schemaVersion") != ANNOTATION_SCHEMA_VERSION:
        raise RH20TAnnotationError(f"schemaVersion must be {ANNOTATION_SCHEMA_VERSION!r}")
    tracks = {
        "schemaVersion": TRACKS_SCHEMA_VERSION,
        "episodeId": str(annotation["episodeId"]),
        "source": dict(annotation.get("source") or {}),
        "fps": float(annotation["fps"]),
        "objects": list(annotation["objects"]),
        "frames": list(annotation["frames"]),
        "review": dict(annotation.get("review") or {}),
    }
    validate_tracks_v0(tracks)
    return tracks


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Convert rh20t.annotation.v0 into rh20t.tracks.v0")
    parser.add_argument("--annotation", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    tracks = annotations_to_tracks(load_json(Path(args.annotation)))
    write_json(Path(args.out), tracks)
    print(f"rh20t annotations_to_tracks: wrote {args.out} frames={len(tracks['frames'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Verify**

Run:

```bash
python3 -m pytest tests/test_rh20t_cli.py tests/test_rh20t_rollout.py -q
```

Expected: all RH20T synthetic tests pass.

## Task 4: Add RH20T Verify CLI And Dataset Layout

**Files:**
- Create: `pilots/rh20t/verify_episode.py`
- Create: `pilots/rh20t/README.md`
- Create/modify: `datasets/rh20t_object_inside_container_v0/manifest.json`
- Create: `datasets/rh20t_object_inside_container_v0/{annotations,tracks,rollouts,reports}/.gitkeep`
- Modify: `.gitignore`
- Test: `tests/test_rh20t_cli.py`

- [ ] **Step 1: Implement verify CLI**

Create `pilots/rh20t/verify_episode.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Optional

from csg.common import Json, load_json
from pilots.external_rollout import ExternalTraceLeakage
from pilots.external_verify import verify_external_rollout
from pilots.rh20t.tracks_to_rollout import RH20TTracksError, tracks_to_rollout

TARGETS_DIR = Path(__file__).resolve().parent / "targets"
BUNDLED_TARGETS = ("object_inside_container_terminal_only", "object_inside_container_relation_event")


def verify_episode(target: Mapping[str, object], *, tracks: Optional[Mapping[str, object]] = None,
                   rollout: Optional[Mapping[str, object]] = None, case_name: str = "rh20t_episode") -> Json:
    if (tracks is None) == (rollout is None):
        raise ValueError("verify_episode requires exactly one of tracks= or rollout=")
    if tracks is not None:
        try:
            rollout = tracks_to_rollout(tracks)
        except (RH20TTracksError, ExternalTraceLeakage) as exc:
            return {"case": case_name, "status": "UNCERTAIN", "passed": False,
                    "failureClass": "source_evidence_invalid",
                    "uncertaintyReasons": [str(exc)], "physicalValidity": None,
                    "traceSource": "rh20t_external"}
    try:
        return verify_external_rollout(target, rollout, case_name=case_name)
    except ExternalTraceLeakage as exc:
        return {"case": case_name, "status": "UNCERTAIN", "passed": False,
                "failureClass": "leakage_violation", "uncertaintyReasons": [str(exc)],
                "physicalValidity": None, "traceSource": "rh20t_external"}


def verify_episode_both(*, tracks: Optional[Mapping[str, object]] = None,
                        rollout: Optional[Mapping[str, object]] = None,
                        targets_dir: Path = TARGETS_DIR) -> Json:
    out: Json = {}
    for name in BUNDLED_TARGETS:
        out[name] = verify_episode(load_json(targets_dir / f"{name}.json"),
                                   tracks=tracks, rollout=rollout, case_name=name)
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Verify an RH20T external-source episode")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--tracks")
    src.add_argument("--rollout")
    parser.add_argument("--target")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    tracks = load_json(Path(args.tracks)) if args.tracks else None
    rollout = load_json(Path(args.rollout)) if args.rollout else None
    if args.target:
        name = Path(args.target).stem
        results = {name: verify_episode(load_json(Path(args.target)), tracks=tracks, rollout=rollout, case_name=name)}
    else:
        results = verify_episode_both(tracks=tracks, rollout=rollout)
    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        for name, rec in results.items():
            print(f"{name}: {rec['status']}" + (f" mismatches={rec.get('hardMismatches')}" if rec.get("hardMismatches") else ""))
    return 0 if all(rec["status"] == "PASS" for rec in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Create dataset manifest**

Create `datasets/rh20t_object_inside_container_v0/manifest.json`:

```json
{
  "datasetId": "rh20t_object_inside_container_v0",
  "schemaVersion": "rh20t.dataset.v0",
  "task": "object_inside_container",
  "note": "Derived RH20T external-source smoke-test artifacts only. Raw RH20T archives, extracted videos, frame dumps, faces, and voices stay on RunPod storage and are not committed.",
  "episodes": []
}
```

- [ ] **Step 3: Add ignore rules**

Modify `.gitignore`:

```gitignore
# RH20T raw media and extraction scratch are too large/sensitive for git.
datasets/rh20t_object_inside_container_v0/raw/
datasets/rh20t_object_inside_container_v0/frame_dumps/
datasets/rh20t_object_inside_container_v0/**/*.mp4
datasets/rh20t_object_inside_container_v0/**/*.tar.gz
```

- [ ] **Step 4: Add README**

Create `pilots/rh20t/README.md` with commands:

```markdown
# RH20T External-Source Pilot

This pilot treats RH20T as recorded evidence. It is not the Sony/tripod Phase 3A
capture path and it is not the Phase 3B target compiler.

Raw RH20T archives and videos stay on RunPod. Commit only:

- reviewed `rh20t.annotation.v0`
- derived `rh20t.tracks.v0`
- derived `csg.rollout.v0`
- reports and provenance hashes

Commands:

```bash
export EP=task_0017_user_0001_scene_0001_cfg_0003
python3 -m pilots.rh20t.annotations_to_tracks --annotation datasets/rh20t_object_inside_container_v0/annotations/$EP.annotation.json --out datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json
python3 -m pilots.rh20t.tracks_to_rollout --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json --out datasets/rh20t_object_inside_container_v0/rollouts/$EP.rollout.json
python3 -m pilots.rh20t.verify_episode --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json --json
```
```

- [ ] **Step 5: Verify CLI tests**

Extend `tests/test_rh20t_cli.py` to write a sidecar to `tmp_path`, run the three module `main()` functions in process, and assert JSON outputs exist.

Run:

```bash
python3 -m pytest tests/test_rh20t_cli.py tests/test_rh20t_rollout.py -q
python3 -m compileall -q pilots/rh20t tests/test_rh20t_cli.py tests/test_rh20t_rollout.py
```

Expected: all commands exit 0.

## Task 5: Produce One Real RH20T Derived Episode On RunPod

**Files:**
- Create: `datasets/rh20t_object_inside_container_v0/annotations/task_0017_user_0001_scene_0001_cfg_0003.annotation.json`
- Create: `datasets/rh20t_object_inside_container_v0/tracks/task_0017_user_0001_scene_0001_cfg_0003.tracks.json`
- Create: `datasets/rh20t_object_inside_container_v0/rollouts/task_0017_user_0001_scene_0001_cfg_0003.rollout.json`
- Create: `datasets/rh20t_object_inside_container_v0/reports/task_0017_user_0001_scene_0001_cfg_0003.verify.json`
- Modify: `datasets/rh20t_object_inside_container_v0/manifest.json`

- [ ] **Step 1: Inspect the selected camera stream**

On RunPod:

```bash
cd /workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE
find . -maxdepth 2 -name color.mp4 -o -name depth.mp4 -o -name metadata.json
```

Pick a global camera where the object and container are visible from initial approach through terminal state.

Expected: selected camera serial is recorded in the eligibility report.

- [ ] **Step 2: Dump review frames**

Run:

```bash
find /workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE -maxdepth 1 -type d -name 'cam_*' -print | sort
export RH20T_CAM=$(find /workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE -maxdepth 1 -type d -name 'cam_*' -print | sort | head -1 | xargs basename)
mkdir -p /workspace/wide-robot/datasets/rh20t_object_inside_container_v0/frame_dumps/$RH20T_SCENE/$RH20T_CAM
ffmpeg -i /workspace/datasets/rh20t_raw/extracted_cfg3/$RH20T_SCENE/$RH20T_CAM/color.mp4 \
  -vf "fps=2" \
  /workspace/wide-robot/datasets/rh20t_object_inside_container_v0/frame_dumps/$RH20T_SCENE/$RH20T_CAM/frame_%04d.jpg
```

Expected: a small set of JPEG frames for human review exists under ignored `frame_dumps/`.

- [ ] **Step 3: Write the reviewed annotation sidecar**

Create `datasets/rh20t_object_inside_container_v0/annotations/task_0017_user_0001_scene_0001_cfg_0003.annotation.json` using measured or depth-backed world poses for at least:

```text
2 initial frames with mover NEAR and not INSIDE
1 transition/midpoint frame
3 terminal frames with mover INSIDE container
container pose in every frame
numeric confidence for every pose
review.method explaining how positions were estimated
source.archiveSha256 from RH20T_cfg3.sha256.txt
```

The sidecar may use source roles `mover` and `container`; `tracks_to_rollout` must remove them from the rollout.

Expected: the sidecar itself is committed as evidence provenance; raw frames are not committed.

- [ ] **Step 4: Convert annotation to tracks and rollout**

Run:

```bash
export EP=task_0017_user_0001_scene_0001_cfg_0003
python3 -m pilots.rh20t.annotations_to_tracks \
  --annotation datasets/rh20t_object_inside_container_v0/annotations/$EP.annotation.json \
  --out datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json
python3 -m pilots.rh20t.tracks_to_rollout \
  --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json \
  --out datasets/rh20t_object_inside_container_v0/rollouts/$EP.rollout.json
python3 -m pilots.rh20t.verify_episode \
  --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json \
  --json | tee datasets/rh20t_object_inside_container_v0/reports/$EP.verify.json
```

Expected success case: both RH20T targets report `PASS`, `leakageClean=true`, `physicalValidity=null`.

Expected honest non-success case: if endpoint evidence is incomplete, verdict is `FAIL` or `UNCERTAIN`; do not tune the sidecar to force PASS.

- [ ] **Step 5: Add a negative or uncertain comparison episode**

Prefer a real RH20T failure/low-quality episode from the same task if one is easy to identify from metadata rating or review frames. If none is found in the extracted shard, create a derived negative by taking the same annotation sidecar and moving only the terminal mover pose to a visually documented near-not-inside/rim/far position in a separate `*_negative.annotation.json` file with `review.method="derived negative from reviewed RH20T geometry"`.

Run the same conversion and verification commands.

Expected: terminal-only and relation-event targets fail or become uncertain; leakage remains clean.

## Task 6: Final Verification And Roadmap Status

**Files:**
- Modify: `roadmap.md` only after the real RH20T result exists.
- Modify: `README.md` only if the pilot should be discoverable from the top-level docs.

- [ ] **Step 1: Run focused tests**

Run:

```bash
python3 -m pytest tests/test_rh20t_rollout.py tests/test_rh20t_cli.py -q
python3 -m compileall -q pilots/rh20t tests/test_rh20t_rollout.py tests/test_rh20t_cli.py
```

Expected: all commands exit 0.

- [ ] **Step 2: Run external-source regression tests**

Run:

```bash
python3 -m pytest tests/test_real_camera_rollout.py tests/test_real_camera_tracks.py tests/test_real_camera_cli.py -q
python3 -m pytest tests/test_rlbench_pilot.py tests/test_rlbench_mutations.py tests/test_rlbench_articulation_event.py -q
```

Expected: no regressions beyond known environment-gated skips.

- [ ] **Step 3: Confirm frozen verifier boundary**

Run:

```bash
git diff --name-only -- csg
```

Expected: no output.

- [ ] **Step 4: Confirm no raw RH20T media is staged**

Run:

```bash
git status --short --untracked-files=all \
  | rg "rh20t|RH20T|\\.mp4|\\.tar\\.gz|frame_dumps" || true
```

Expected: only derived JSON, README, tests, and `.gitkeep` files appear; no `.mp4`, `.tar.gz`, extracted raw folders, or frame dumps appear.

- [ ] **Step 5: Update roadmap with result status**

Only after real RH20T verification is run, update the roadmap status:

```text
Phase 3A.5 RH20T external-source smoke test:
  PASS result exists / blocked with reason / deferred
  raw media stayed on RunPod
  csg/ unchanged
  physicalValidity null
  not a replacement for Sony/tripod Phase 3A
  not a Phase 3B target compiler
```

## Completion Gate

The RH20T checkpoint is complete only when fresh evidence shows:

```bash
python3 -m pytest tests/test_rh20t_rollout.py tests/test_rh20t_cli.py -q
python3 -m pytest tests/test_real_camera_rollout.py tests/test_real_camera_tracks.py tests/test_real_camera_cli.py -q
python3 -m pytest tests/test_rlbench_pilot.py tests/test_rlbench_mutations.py tests/test_rlbench_articulation_event.py -q
python3 -m compileall -q pilots/rh20t tests/test_rh20t_rollout.py tests/test_rh20t_cli.py
git diff --name-only -- csg
export EP=task_0017_user_0001_scene_0001_cfg_0003
python3 -m pilots.rh20t.verify_episode --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json --json
```

Required result:

```text
At least one selected real RH20T-derived episode has an honest PASS, FAIL, or UNCERTAIN record.
If PASS is claimed: relation-event target passes non-vacuously, leakageClean=true, physicalValidity=null.
At least one negative/uncertain comparison exists or the report explicitly says the extracted shard did not provide one.
No raw RH20T video/archive/frame dump is committed.
`git diff --name-only -- csg` is empty.
```

## Self-Review Notes

- This plan intentionally avoids Phase 3B target generation.
- This plan does not claim Sony/tripod recording is complete.
- The first RH20T extraction path is human-assisted because object tracks are not provided directly by RH20T.
- The target and rollout vocabularies stay aligned with the existing object-inside-container semantics.
- The plan has an early stop condition before implementation if cfg3 does not contain a usable candidate scene.
