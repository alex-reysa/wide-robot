#!/usr/bin/env python3
"""MuJoCo arm backend (roadmap Phase 2C).

target CSG -> compiled scene -> MJCF (fixed-base arm + parallel-jaw gripper)
-> scripted pick-place controller -> recorded frames (``csg.rollout.v0``)
-> real ``physicalValidity`` verdict (``csg/validity.md``).

Importing this package never imports ``mujoco``: the heavy modules
(``controller``, ``runner``) import it lazily, and ``trace`` / ``validity`` /
``scene_mjcf`` / ``arm`` are stdlib-only. ``run_skill`` raises an informative
:class:`RuntimeError` if ``mujoco`` is not installed — it never silently falls
back to the symbolic backend, which would let a physics-unverified rollout
masquerade as physics-checked.
"""
from __future__ import annotations

from typing import Any, Mapping


def mujoco_available() -> bool:
    import importlib.util

    return importlib.util.find_spec("mujoco") is not None


MUJOCO_AVAILABLE = mujoco_available()


def run_skill(scene: Mapping[str, Any], program: Mapping[str, Any], cfg: Any) -> Any:
    """Simulate ``program`` against ``scene`` and return a ``SimResult``.

    Raises ``RuntimeError`` (not a silent fallback) when ``mujoco`` is absent.
    """
    if not mujoco_available():
        raise RuntimeError(
            "mujoco backend requested but the 'mujoco' package is not installed. "
            "Install it with: pip install 'mujoco>=3.9'  (or  pip install -e '.[sim]')."
        )
    from .runner import run_skill as _run_skill

    return _run_skill(scene, program, cfg)


__all__ = ["run_skill", "mujoco_available", "MUJOCO_AVAILABLE"]
