"""Execution backends for the CSG solver.

A backend turns a compiled scene + selected skill program into rollout frames
(``csg.rollout.v0``) and, where it can, a real ``physicalValidity`` verdict.

  * ``symbolic`` (the default, implemented inline in ``csg/solver.py``):
    kinematic interpolation, no contact dynamics, ``physicalValidity = None``.
  * ``mujoco`` (roadmap Phase 2C, ``csg/backends/mujoco/``): a fixed-base arm
    and parallel-jaw gripper in MuJoCo, scripted pick-place controller, real
    validity checks per ``csg/validity.md``.

The roadmap §7 reserves ``csg/backends/dk1/`` here for the real-arm adapter.
"""
