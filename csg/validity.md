# Physical-validity specification (V0.3 — checks implemented in the MuJoCo backend for all five V0 gold tasks, roadmap Phase 2C)

> **Status.** The reporting contract is implemented for every backend; the six
> checks below are now implemented in `csg/backends/mujoco/validity.py` and run
> on a `SimTrace` produced by `csg/backends/mujoco/runner.py`. All five V0 gold
> tasks pass gated MuJoCo tests/benchmark with real `physicalValidity: true`;
> the symbolic backend still reports `None`. Verdict = AND over the
> *applicable* checks; a check that does
> not apply to a task (e.g. articulation limits on a pure pick-place) is reported
> `applicable: false` and never counts toward the verdict. The full per-check
> breakdown is written to a sidecar `validity_report.json` (not the rollout — no
> schema bump). The grasp is **weld-assisted**: fingers close on the object (real
> gripper feasibility + bilateral contact) and a weld holds the scripted
> transport, but the weld is released before placement so quasi-static support is
> judged on an honest, unassisted resting state. `stable_grasp_quality` stays on
> `hiddenVariablesNotUsed`. Gated benchmark invocation:
> `.venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco --confusion`.

The matcher decides *observable task equivalence*. It does **not** decide
whether a rollout is physically realizable. The roadmap's solver-pass rule
(`roadmap.md` §4) is:

```
A solver pass requires:
  all HARD probes agree (match.passed)
  AND no target leakage == true
  AND physical_validity == true, or explicitly reported "not checked"
      (symbolic backend: always None, labeled "physics-unverified";
       never silently true)
```

`match.passed` and leakage are implemented (`csg.matcher`, `csg.benchmark`).
The symbolic backend sets `rollout["diagnostics"]["physicalValidity"] = None`
("backend cannot check", never `true`); `csg.benchmark` gates `passed` on
`validity is not False` and labels `None` as *interface-valid,
physics-unverified* in reports. The **MuJoCo backend** sets a real `true`/`false`
verdict from the checks below (`csg/backends/mujoco/validity.py`).

Container geometry is also fixed for Phase 2C (audit A8): bodies flagged
`isContainer` compile to an open cavity (floor slab + four walls,
`CONTAINER_WALL_M` / `CONTAINER_FLOOR_M` in `csg/to_sim.py`), and the solver
rests inserted objects on the cavity floor — so INSIDE is reachable without
interpenetration. `tests/test_validity.py` asserts goal-pose admissibility
(no goal pose penetrates a collision geom by more than `penetration_tol_m`).

The MuJoCo backend (roadmap Phase 2C, legacy label "6C") implements the checks
below and sets a real `physicalValidity` verdict on the rollout. Phase 2C now
covers all five V0 gold tasks in gated tests/benchmark. The Phase 2E benchmark
runner now supports `--randomized --seeds N` and the 30-seed/task MuJoCo command
passes with real `physicalValidity: true` and sampled layouts for all five V0
tasks, including x-shifted push starts. Frozen invalid fixtures live under
`gold_invalid/`.

## Checks the physics backend implements

1. **Non-penetration.** Max interpenetration depth between any two collision
   geoms ≤ `penetration_tol_m` (e.g. 5 mm) at every step.
2. **Pose continuity (no teleports).** Per-step object translation ≤
   `max_step_translation_m` and rotation ≤ `max_step_rotation_rad`, consistent
   with the control rate. (The symbolic backend already interpolates to satisfy
   continuity; physics must verify it under contact.)
3. **Quasi-static support at release.** At each `RELEASE_INFERRED`, the released
   object's net wrench is supported (resting on a surface / inside a container)
   and it does not subsequently fall out of its terminal relation within a
   settle window.
4. **Gripper feasibility.** Commanded gripper aperture spans the grasped object
   dimension (object min-width ≤ aperture ≤ object max-width) and the grasp is
   within the gripper's force/width limits.
5. **Workspace reachability.** Every commanded effector pose lies within the
   robot's reachable workspace and joint limits; no IK failures.
6. **Articulation limits.** Articulated joint values stay within `[α_min, α_max]`
   and move along the modeled axis; no hinge/slider over-travel.

## Reporting contract

- `rollout["diagnostics"]["physicalValidity"]`: `true` / `false` / `null`.
- `null` means "backend cannot check" (symbolic). A benchmark PASS with a
  `null` validity must be labeled **"interface-valid, physics-unverified"** in
  reports — it is not a claim of physical success.
- The benchmark `PASS` gate is `match.passed AND leakage.clean AND
  physicalValidity is not false`: symbolic `null` is surfaced as
  physics-unverified, while MuJoCo gold-task coverage must carry real `true`
  verdicts in gated tests/benchmark.

## Known V0 geometric limitations (predicate registry)

- Objects are axis-aligned boxes; yaw is ignored in footprint tests
  (`csg/predicates.py`). Rotated containers / thin angled objects are
  approximate.
- Containment is rim-based, not true cavity volume; nested or compartmented
  containers need real geometry (`FROM_6D_POSE_AND_CAD` / multiview).
- These are acceptable for V0 rigid pick/place/insert/push/open; deformables and
  fluids remain Unobservable-Critical-Variable territory (see
  `physical_quotient.md` §10).
