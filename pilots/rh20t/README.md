# RH20T External-Source Pilot (Phase 3A.5)

This pilot treats **RH20T as recorded episode evidence** — a *separate external source*
that the FROZEN verifier judges. It is **not** the Sony/ArUco real-camera capture path
(Phase 3A) and it is **not** the Phase 3B video→target compiler. It does not author
target CSGs from RH20T; it converts a reviewed RH20T episode into a neutral
`csg.rollout.v0` and asks the unchanged verifier "did the object end up inside the
container, and (strictly stronger) did it start near and get put in?"

```text
RH20T episode  →  rh20t.annotation.v0  →  rh20t.tracks.v0  →  csg.rollout.v0  →  frozen verifier
(RunPod, raw)     (annotations_to_tracks)  (tracks_to_rollout)  (verify_episode)   PASS/FAIL/UNCERTAIN
```

## Raw media stays on RunPod

RH20T archives and videos are large and may contain faces/voices. They stay on RunPod
storage (`/workspace/datasets/rh20t_raw/`). **Commit only:**

- reviewed `rh20t.annotation.v0` sidecars
- derived `rh20t.tracks.v0`
- derived `csg.rollout.v0`
- reports and provenance hashes (`reports/*.verify.json`, `reports/eligibility_report.md`)

`.gitignore` blocks raw archives, extracted trees, frame dumps, and `*.mp4`/`*.tar.gz`
under `datasets/rh20t_object_inside_container_v0/`.

## Leakage discipline (stronger than real-camera)

An RH20T `episodeId` *is* the source identity (`task_0017_user_..._scene_...`). The
rollout door (`tracks_to_rollout`) drops it: the rollout is **fully source-blind** — no
task id, description, scene path, or source role name appears anywhere in the rollout,
not even in diagnostics (only a one-way `episodeRef` hash + the content-derived
`archiveSha256`). `tests/test_rh20t_rollout.py` asserts this. Human-readable provenance
lives in the committed tracks/annotation/report, which the verifier never reads.

## Commands

```bash
export EP=task_0017_user_0010_scene_0005_cfg_0003   # the committed real episode
python3 -m pilots.rh20t.annotations_to_tracks \
  --annotation datasets/rh20t_object_inside_container_v0/annotations/$EP.annotation.json \
  --out datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json
python3 -m pilots.rh20t.tracks_to_rollout \
  --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json \
  --out datasets/rh20t_object_inside_container_v0/rollouts/$EP.rollout.json
python3 -m pilots.rh20t.verify_episode \
  --tracks datasets/rh20t_object_inside_container_v0/tracks/$EP.tracks.json --json
```

`verify_episode` exits 0 only if BOTH bundled targets PASS. A structurally-broken
sidecar surfaces as UNCERTAIN `source_evidence_invalid`; a leaky rollout as UNCERTAIN
`leakage_violation`. `physicalValidity` is always `null` (physics-unverified by contract).

## Tests (no raw media, no numpy/cv2/RLBench)

```bash
python3 -m pytest tests/test_rh20t_rollout.py tests/test_rh20t_cli.py -q
python3 -m compileall -q pilots/rh20t tests/test_rh20t_rollout.py tests/test_rh20t_cli.py
```

The synthetic fixtures prove target semantics, the strictly-stronger pair, the failure
modes, and the leakage quarantine without touching RH20T raw data or `csg/`.
