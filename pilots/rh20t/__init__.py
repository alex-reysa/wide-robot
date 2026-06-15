"""RH20T external-source pilot (roadmap Phase 3A.5) — see ``pilots/rh20t/README.md``.

This package treats RH20T as **recorded episode evidence**, a *separate external
source* — not the Sony/ArUco real-camera capture path (Phase 3A) and not a target
compiler (Phase 3B). It converts selected, human-reviewed RH20T episodes into neutral
``csg.rollout.v0`` traces for the FROZEN verifier and reports PASS / FAIL / UNCERTAIN.
It does not prove the Sony/tripod camera path and it does not author target CSGs from
the RH20T episode.

Pipeline (mirrors ``pilots/real_camera`` but with a human-reviewed annotation seam,
because RH20T does not ship ready-made task-object pose tracks):

    RH20T episode  →  rh20t.annotation.v0  →  rh20t.tracks.v0  →  csg.rollout.v0  →  frozen verifier
    (RunPod, raw)     (annotations_to_tracks)  (tracks_to_rollout)   (verify_episode)

Raw RH20T archives, extracted video, frame dumps, faces, and voices stay on RunPod
storage; only derived JSON (annotations, tracks, rollouts, reports) plus provenance
hashes are committed. The rollout door reuses the source-agnostic
:mod:`pilots.external_rollout` contract, so the RH20T trace meets the exact leakage
discipline the RLBench and real-camera pilots proved. ``csg/`` is never imported in an
altered form or modified.
"""
