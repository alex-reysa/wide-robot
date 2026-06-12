# `csg.rollout.v0` — the rollout artifact

The rollout is the honest hand-off between the solver and the independent
extractor (`csg/rollout_extract.py`). It is the **only** thing the extractor
may read, so its contents define the information-flow contract of the
compiler-verifier loop: anything present here is, by construction, available
to the "robot's own perception". Anything target-authored that leaks in makes
the verifier's PASS meaningless (V0.1/V0.2 audits, A4).

Produced by `csg/solver.py::solve`; consumed by
`csg/rollout_extract.py::extract_robot_csg`. Leakage is enforced by
`csg/benchmark.py::leakage_report` and `tests/test_leakage.py`.

## Top-level fields

| field | type | contents |
|---|---|---|
| `schemaVersion` | string | `"csg.rollout.v0"` |
| `backend` | string | solver backend that produced the frames (`symbolic`; MuJoCo in roadmap Phase 2C) |
| `robotEffectorId` | string | id of the effector reported in `frames` (default `robot_gripper`) |
| `objectIdMap` | object | target id → neutral robot id (`body_000`, …). Solver-side bookkeeping; the extractor does not need it and must not use it to recover target identities |
| `sceneBodies` | array | **sanitized** instantiated bodies, see below |
| `skillProgram` | object | the selected skill skeleton (program id, steps). Solver provenance, not extractor input |
| `frames` | array | the continuous state trace, see below |
| `success` | bool | solver claims a plan was produced (not a verifier verdict) |
| `failures` | array of string | solver failure notes (e.g. `no_executable_skill_for:<skill>`) |
| `diagnostics` | object | honest solver diagnostics, see below |

A rollout **never** carries: `targetCsg`, `plannerView`, `solverMetadata`, the
target's observation graph (relations / contacts / events), or any free-text
authored on the target side. `leakage_report` fails a case outright if any of
those keys appear.

## `sceneBodies[]` — sanitized body whitelist

Exactly the fields a simulator could honestly report about a body it
instantiated (`csg/to_sim.py::ROLLOUT_BODY_FIELDS`, applied by
`sanitize_bodies_for_rollout`):

| field | notes |
|---|---|
| `objectId` / `bodyId` | neutral ids (`body_NNN`), never derived from target ids |
| `physicalKind` | the one hard carrier attribute (see `physical_quotient.md` §0.b-11) |
| `sizeM` | `[x, y, z]` full extents in meters |
| `sizeApproximate` | true when the compiler had to invent the size |
| `mobility` | planner-asserted mobility (`MOVABLE` / `STATIC` / `ARTICULATED` / unknown) |
| `articulation` | initial articulated joint state, if any |
| `isContainer`, `containerCavity` | open-cavity compilation parameters (audit A8) |

Stripped by the whitelist: `categoryLabel`, `sourceObjectId`, `geometry`
(notes and source enums), `parts` (part labels), `initialPose*` (the frames
supersede it). The all-string-field canary fuzz in `tests/test_leakage.py`
guards the whitelist against regressions.

## `frames[]` — the state trace

One entry per interpolated step, no teleports:

```json
{
  "timeS": 3.25, "timeNs": "3250000000", "phase": "transport",
  "effectorPose": {"frameId": "world", "positionM": {...}, "orientationWxyz": {...}},
  "gripperClosed": true,
  "objectPoses": {"body_000": {...}, "body_001": {...}},
  "articulation": {"body_000": 0.12}
}
```

* `phase` is solver provenance (approach/grasp/…); the extractor ignores it.
* `gripperClosed` + effector/object poses are what the extractor uses to decide
  contact manner: closed + within grasp reach ⇒ `GRASP_LIKELY`; open but at the
  object surface with co-motion ≥ `PredConfig.co_motion_corr` ⇒
  `TOUCHING_LIKELY` (non-grasp push, V0.3). All relation/contact words are
  decided by `csg/predicates.py`, never copied from anywhere.
* `articulation` maps object id → joint value for articulated bodies.

## `diagnostics` — honest by contract

| field | contents |
|---|---|
| `selectedProgramId`, `skill`, `planProduced`, `numFrames` | solver bookkeeping |
| `hiddenVariablesNotUsed` | explicit list of UCVs the backend cannot ground (force, torque, mass, friction, stable grasp quality) |
| `physicalValidity` | `true` / `false` / `null` — `null` means "backend cannot check"; the symbolic backend never claims `true` (`csg/validity.md`). The benchmark gates on `is not False` and labels `null` PASSes *physics-unverified* |
| `physicalValidityReason` | one-line justification for the verdict |

## Versioning

Additions to the rollout require a schema-version bump and a corresponding
review of `ROLLOUT_BODY_FIELDS` / `leakage_report`: the default answer to "can
the rollout carry X?" is **no** unless a simulator with no access to the
demonstration could have produced X.
