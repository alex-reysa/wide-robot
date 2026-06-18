# MuJoCo `put_cube_in_tray` fixture — provenance

These four files are a **verbatim copy** of a genuine MuJoCo backend solver run, captured at
`phase2e_release_out/mujoco/put_cube_in_tray/` (a gitignored release-output directory). They are
committed here so the cross-source report's **internal-sim leg is reproducible from a clean clone
with no MuJoCo install** — the rollout is re-matched purely in Python (`extract_robot_csg` → `match`),
exactly as `csg.benchmark.run_one` does after a solve.

| file | role | sha256 |
|---|---|---|
| `put_cube_in_tray.rollout.json` | the MuJoCo `csg.rollout.v0` trace (input the report re-extracts + re-matches) | `1bf729ce59153bc1b375ca68151e313191e4692aa0d196f950af625b14482474` |
| `put_cube_in_tray.validity_report.json` | the backend's physical-validity verdict → `physicalValidity: true` (5/5 applicable physics checks passed; `articulation_limits` is N/A — no articulated joints) | `57d4cf418f9775f0f34576f36272a33d2f5af32392ca582941a5297c07dfbcd1` |
| `put_cube_in_tray.robot_csg.json` | the extracted robot CSG (provenance only; the report re-extracts it from the rollout) | `a77438b9962b1598a92ae308eec594c76d73742a9e98dea1fb6ba19a6d9b920d` |
| `put_cube_in_tray.matcher_report.json` | the original match verdict against the gold target (provenance only) | `cacd366a569b276c32bb261a155d74df2d8b8da9a77e601a79cbe4ba43dff906` |

**What it is:** a successful pick-and-place of a 4 cm cube into a tray. `objectIdMap` is
`{h_cube: body_000, h_tray: body_001}` (legitimately bound — this is an *internal* trace, not an
external one), `backend: "mujoco"`, 62 frames, `success: true`. The robot CSG is leakage-clean
(`SIM_STATE_EXTRACTION` is an allowed estimator; no planner-view / target-CSG / solver-metadata).

**Why the internal leg does NOT call `verify_external_rollout`:** that function's first line is the
external-trace leakage door (`assert_rollout_leakage_clean`), which *correctly* rejects a populated
`objectIdMap`. An internal MuJoCo rollout legitimately carries one, so the internal leg uses the same
frozen core (`extract_robot_csg` → `csg.matcher.match` → `leakage_report`) directly — the identical
path `csg.benchmark.run_one` takes — and additionally reports `physicalValidity` from the committed
validity report (physics genuinely re-checked at capture time, which an external trace cannot be).

Regenerate the report with `python3 -m scripts.build_cross_source_report`.
