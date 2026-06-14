#!/usr/bin/env python3
"""Run an external RLBench ``csg.rollout.v0`` trace through the FROZEN csg verifier.

The verifier-driver functions are source-agnostic and now live in
:mod:`pilots.external_verify` (so the real-camera pilot reuses the identical
``extract_robot_csg -> match -> leakage_report`` path). They are re-exported here so
existing imports (``from pilots.rlbench.run_external import verify_external_rollout``,
``record_open_drawer``'s ``from .run_external import …``) and the
``python -m pilots.rlbench.run_external`` CLI keep working unchanged.
"""
from __future__ import annotations

# Re-export the shared, source-agnostic verifier driver + CLI. ``# noqa: F401`` keeps a
# linter from stripping the "unused" re-exports that downstream imports depend on.
from pilots.external_verify import (  # noqa: F401
    verify_external_rollout,
    load_gold_targets,
    external_confusion_report,
    main,
)


if __name__ == "__main__":
    raise SystemExit(main())
