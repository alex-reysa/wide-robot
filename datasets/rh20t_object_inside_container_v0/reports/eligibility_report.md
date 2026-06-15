# RH20T Eligibility Report

Status of Task 0 (RunPod dataset triage) for the Phase 3A.5 external-source smoke test.

## Triage attempt — 2026-06-14

- **Code seam (Tasks 1–4):** COMPLETE and verified independent of any raw data —
  `tests/test_rh20t_rollout.py` + `tests/test_rh20t_cli.py` pass (21 tests),
  external-source regressions green, `git diff --name-only -- csg` empty, no raw media
  committed. The pilot can ingest a reviewed RH20T episode the moment one is in hand.
- **Data acquisition:** BLOCKED. The plan's candidate shard (cfg3, 640×360) is served
  only via Google Drive / Baidu from rh20t.github.io, and the Google Drive links are
  currently returning a **global per-file quota error**:

  ```text
  $ gdown 1uwieq-EbA_eTXE668ekypQV1cO9PDfES   # cfg3 RGB
  $ gdown 1aekLEcX1ruS9f2z6900ys5t_U_OJnEzQ   # cfg3 depth
  Failed to retrieve file url:
    Too many users have viewed or downloaded this file recently. Please
    try accessing the file again later ... it may take up to 24 hours ...
  → returncode 1, 0 bytes pulled (gdown 6.1.0), both files.
  ```

  This is a per-file daily cap on the public Drive object; it is **not** bypassable by
  changing IP/region, so re-running the same `gdown` from a RunPod pod would reproduce
  the identical block. **No pod was provisioned** (RunPod balance preserved at ~$9.76);
  spending pod-hours on a download that is globally throttled would be wasted budget.

## cfg3 archive sizes (from rh20t.github.io)

| Variant | RGB | Depth | LowDim | Calib |
| --- | --- | --- | --- | --- |
| 640×360 (plan's links) | 26 GB | 26 GB | 11.3 GB | 334.7 MB |
| 320×180 (lossless depth) | 4.4 GB | 71.3 GB | — | — |

## Alternative sources assessed (no Drive quota)

- **HuggingFace `hainh22/rh20t`** (LeRobot, Apache-2.0, no gating): only **1 episode /
  158 frames / 268 MB**, UR5, 6× RGB 360×640, robot state/action/force-torque, **no
  depth**, and the single task's identity is **not confirmed to be a containment task**.
  Too thin and uncertain to anchor the smoke test; no depth weakens honest pose
  annotation.
- **HuggingFace `fredfang/RH20T`**: empty placeholder (2.33 kB, no data).
- **Baidu Cloud** (official mirror): requires a Baidu account + client; not automatable
  from this environment.

## Honest-annotation boundary (independent of the data block)

RH20T does **not** ship task-object pose tracks. Even once raw media is in hand, the
mover/container world poses must be established honestly from the video/depth/calibration
(human review of frames, or a real depth-backed perception step) — they must **not** be
synthesized from the target. This is the plan's human-assisted step and cannot be
fabricated.

## Decision

`blocked_data_acquisition_drive_quota`. The candidate cfg3 shard is currently inaccessible
via the plan's Google Drive links (global per-file quota; not IP-bypassable; resets up to
24h). Renting a pod now would reproduce the same block, so no pod was provisioned.

**Resolution (user-directed, 2026-06-14):** ship the verified+hardened code seam plus this
report as the Phase 3A.5 checkpoint (the plan explicitly permits an honest documented-
blocked outcome); retry the real episode later. When the data lands, the **mover/container
world-pose annotation is done by human frame review** (the chosen honest-annotation method)
— not synthesized from the target.

## Post-review hardening (2026-06-14)

An adversarial review of the seam found a real leak and it was fixed before shipping:
`source.archiveSha256` was copied into the (source-blind) rollout's diagnostics **unchecked**,
so a paste-error (a scene path / task id pasted where the 64-hex digest belongs) could
smuggle RH20T source identity into the committed rollout — the leakage gate does not inspect
diagnostics. `validate_tracks_v0` now rejects any `archiveSha256` that is not a 64-char
lowercase hex digest or null, fail-closed (→ UNCERTAIN `source_evidence_invalid`, never
PASS), and the quarantine test was made structural/task-agnostic. Regression:
`tests/test_rh20t_rollout.py::test_poisoned_archive_sha_is_rejected_not_leaked`.

## Resuming the real run (when Drive quota resets, or via an authenticated Drive copy)

1. Provision a **CPU pod** (not GPU — the work is download/tar/ffmpeg/depth, all CPU/IO
   bound; GPU would sit idle) with a ~120-250 GB volume at `/workspace`.
2. Follow plan Task 0 (download cfg3 RGB+depth, check for a candidate containment task,
   extract one scene + calibration, write the eligibility decision) and Task 5 (dump review
   frames → **you review them** → encode the `rh20t.annotation.v0` sidecar with
   measured/depth-backed world poses → run the three `pilots.rh20t` CLIs → record the
   verdict). Tear the pod down promptly. RunPod balance at checkpoint: ~$9.76.

## Deeper data-acquisition investigation (2026-06-14, follow-up) — definitive 24h wall

We pursued the RH20T-maintainer-recommended rclone path end-to-end (own Google OAuth
client, server-side copy bypass, gcloud quota-project override) and hit a definitive
Google-Drive 24h quota wall on BOTH download and copy. Storage is NOT the blocker (the
100GB plan was restored, ~41GB free); the blocker is purely Drive API quota/rate-limits:

- **Direct download** (authenticated Drive API, via rclone's shared client AND a personal
  own-project OAuth client `gdw2`): cfg3 and all 7 configs return 403 `downloadQuotaExceeded`.
  This quota is **per-file-global** — a fresh client/project does NOT reset it. Early in the
  session cfg1/cfg6 showed brief seconds-long download windows; all closed.
- **Server-side `files.copy` bypass** (the standard `downloadQuotaExceeded` workaround):
  returns 403 `userRateLimitExceeded` even with the personal `gdw2` client and even on a
  DIFFERENT file (cfg7). So the copy limit is **per-user** (reysanchezalex@gmail.com), not
  per-file or per-client. The `X-Goog-User-Project` quota-redirect (tested with reysanchezalex
  granted serviceUsageConsumer on `alien-container`, Drive API enabled) does NOT move the
  Drive copy quota. Per rclone's Drive docs: server-side copies have a separate rate limit;
  "wait at least 24 hours if you hit user-rate-limit errors."

**Setup left ready for a one-command resume once the ~24h quota resets** (or off-peak China
hours, when the file's global download quota is freer):
- rclone remote `gdw2` (account reysanchezalex) on a **personal OAuth client** with its own
  un-shared project quota: `client_id 321959424413-...apps.googleusercontent.com` (project
  `rclone-rh20t` / 321959424413, Drive API enabled). Also remote `gdrive` (readonly) and
  `gdw` (shared-client write). reysanchezalex has serviceUsageConsumer on `alien-container`.
- Candidate source: cfg3 RGB id `1uwieq-EbA_eTXE668ekypQV1cO9PDfES` (27.4GB, sha256
  `b49b297043f3ccf8386b620e11e9ccebc634ba5704e372ae7243480f6e38b6d3`) — but ANY config that
  contains a containment task (task_0017/0072/0073/0091) works; all four containment task ids
  are confirmed present in RH20T `task_description.json`.

Resume steps (>=24h later):
1. Server-side copy cfg3 into your Drive via the own-client (fresh per-user copy quota):
   `rclone backend copyid gdw2: 1uwieq-EbA_eTXE668ekypQV1cO9PDfES gdw2:`
   (or Drive API `files/{id}/copy` with the gdw2 bearer token).
2. The copy is now your own file -> fresh download quota -> stream it by id through `gtar`,
   extracting ONE containment scene + `*/calib/*` only (keep ~hundreds of MB locally).
3. Dump review frames -> human annotates mover/container world poses -> encode the
   `rh20t.annotation.v0` sidecar -> run `pilots.rh20t.verify_episode` -> record the verdict.
Honest expectation unchanged: the verdict depends on a clear containment + clean pose read;
a FAIL/UNCERTAIN is recorded honestly, never a forced PASS.

Cleanup the user may do later (all reversible, none affect the repo): delete the
`rclone-rh20t` OAuth client + project, remove the `gdw`/`gdw2` rclone remotes, and remove the
serviceUsageConsumer IAM binding on `alien-container`.

Decision (at that point): `blocked_data_acquisition_drive_quota_24h` — download (per-file-global)
and copy (per-user) are BOTH Google-rate-limited; resume after the ~24h reset. The Phase 3A.5
code seam is complete and verified independent of this block.

## RESOLVED (2026-06-15) — real RH20T episode PASSES, `usable_for_smoke_test`

The per-user copy rate-limit cleared after a short cooldown (well under 24h): a retry of the
`files.copy` bypass via the personal own-OAuth-client succeeded on the **first** attempt,
landing a same-content private copy of `RH20T_cfg3.tar.gz` in the user's Drive (`ownedByMe`,
fresh download quota). The copy was then streamed by id through `gtar` at ~22 MB/s, extracting
calibration + the first containment task scenes (~1.4 GB, kept off-repo), and the copy was
deleted afterward (storage freed). cfg3 tar entries are in numeric task order; **task_0017
("Put the pen into the pen holder")** appears early. Scene `task_0017_user_0010_scene_0005_cfg_0003`
(quality rating 9/10) was reviewed from global camera `cam_104122062823` and **visually
confirmed NEAR → INSIDE** (pen lying beside the mesh holder → pen standing inside it).

Result (the frozen verifier, unchanged):

```text
REAL positive episode  task_0017_user_0010_scene_0005_cfg_0003
  object_inside_container_terminal_only   PASS
  object_inside_container_relation_event  PASS (non-vacuous: goal_satisfaction,
                                          initial_state, terminal_state, relation_transitions,
                                          event_presence all support=1, agreement=true;
                                          event_order support 0)
  leakageClean = true   physicalValidity = null
  rollout source-blind: NO task_0017 / pen / holder / RH20T_cfg3 / scene / source-role tokens
    reach it; diagnostics carry only a one-way episodeRef hash + the content-derived
    archiveSha256 + the neutral "RH20T" dataset label / backend / skillProgram.source

DERIVED negative (same geometry, terminal pose -> near-not-inside)
  terminal_only   FAIL (goal_satisfaction)
  relation_event  FAIL (terminal_state, relation_transitions, event_presence, event_order,
                        goal_satisfaction)
  leakageClean = true
```

Honest modeling note (carried in the annotation `review`): cfg3 is 640x360 RGB and the depth
archive was not extracted, so poses are review estimates from a single oblique global camera
(approx +/- 2-3 cm) scaled by the holder's ~8.5 cm diameter; the pen is tracked by its
**centroid** and modeled as a compact rigid object (its elongated shape / rim protrusion are not
box-modeled), so the verdict is at the centroid-relation level. The qualitative NEAR->INSIDE
transition is unambiguous in the video; poses are NOT derived from the target. A strict full-pen
bounding-box model would be a separate, harder question, deliberately out of scope for this
smoke test.

Decision (final): `usable_for_smoke_test` — one real RH20T-derived episode PASSes the frozen
verifier non-vacuously, a derived negative FAILs, leakage is clean, `physicalValidity` is null,
`csg/` is byte-frozen, and no raw RH20T media is committed. Smoke test complete.
