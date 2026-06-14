"""RLBench external-trace pilot — see ``docs/rlbench_external_trace_pilot.md``.

Feeds external RLBench demonstration traces through the frozen csg verifier to test
whether the leakage-clean hard-probe discipline holds on traces csg's own solver did
not produce. The RLBench dependency is optional (``pip install -e ".[rlbench]"`` plus
PyRep/CoppeliaSim); the rollout-assembly + verifier seam runs with neither, against
the committed ``fixtures/*.rollout.json`` stand-ins.
"""
# NB: do NOT import .run_external here — it is runnable via ``python -m
# pilots.rlbench.run_external``, and a package-level import would double-import it
# under -m (RuntimeWarning). Import it by its full path instead:
#     from pilots.rlbench.run_external import verify_external_rollout
from .adapter import (
    ExternalTraceLeakage,
    assemble_rollout,
    assert_rollout_leakage_clean,
    rlbench_demo_to_rollout,
)

__all__ = [
    "ExternalTraceLeakage",
    "assemble_rollout",
    "assert_rollout_leakage_clean",
    "rlbench_demo_to_rollout",
]
