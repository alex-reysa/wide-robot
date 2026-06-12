"""Arm-Bounded Demonstration Compiler — Causal Skill Graph (CSG) V0 package.

Single source of truth for the compiler-verifier loop:

    target CSG  --to_sim-->  scene  --solver-->  rollout (frames)
              --rollout_extract-->  robot CSG  --matcher-->  probe agreement

Module map:
    common          JSON / pose / id helpers (deduplicated).
    predicates      Versioned geometric semantics of RelationKind / ContactMode.
    canon           Canonical form: strip TaskSpec from rollouts, normalize
                    converse relations, recompute temporal edges, mask by
                    confidence, deterministic ordering.
    matcher         Probe-based observable quotient checker (hard/soft split).
    to_sim          Compile target PlannerView into a simulator scene.
    skills          Infer candidate skill skeletons from a target CSG.
    solver          Symbolic/kinematic solver -> rollout frames (no leakage).
    rollout_extract Independent extractor: rollout frames -> robot CSG.
    benchmark       Run the frozen loop and report probe-vector PASS.
"""

__all__ = [
    "common",
    "predicates",
    "canon",
    "matcher",
    "to_sim",
    "skills",
    "solver",
    "rollout_extract",
    "benchmark",
]
