# DEPRECATED — superseded by the `csg/` package

The compiler-verifier loop now lives in the top-level `csg/` package, which is
the single source of truth. The scripts in this directory are the **old V0
harness** and are kept only for historical reference.

Why they were replaced (see the adversarial audit):

- `rollout_to_csg.py` deep-copied the target CSG (`robotize_csg`) — total target
  leakage. The rollout frames were never read.
- `csg_solver.py` embedded `targetCsg` into the scene and rollout, hardcoded
  `success=True`, and fabricated an `objectiveHistory` convergence curve.
- `csg_matcher.py` (1416 lines) used a weighted DTW distance whose honest-zero
  set was empty (confidence weights, plannerView-vs-plannerView comparison,
  temporal-edge penalties) and had confirmed bugs (TOPO_ART mapping, all-time
  terminal probe, nondeterministic tie-break, dilution-vulnerable normalization).

New entry points:

| Old | New |
|---|---|
| `python csg_matcher.py a b` | `python -m csg.matcher a b` |
| `python csg_solver.py t --out o` | `python -m csg.solver t --out o` |
| `python rollout_to_csg.py r` | `python -m csg.rollout_extract r` |
| `python benchmark_runner.py ...` | `python -m csg.benchmark gold_tests` |

`csg_matcher.py` here is now a thin shim re-exporting `csg.matcher`.

Run the loop and tests:

```bash
python -m csg.benchmark gold_tests --json
python -m pytest tests/
```
