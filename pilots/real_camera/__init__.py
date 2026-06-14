"""Real-camera external-trace pilot (roadmap Phase 3A) — see ``pilots/real_camera/README.md``.

Feeds constrained Sony/tripod ``object_inside_container`` video through the frozen csg
verifier as an **evidence source that JUDGES recorded episodes** (PASS / FAIL / UNCERTAIN),
NOT a compiler that authors target CSGs (that is Phase 3B). The pipeline is:

    video → marker observations → real_camera.tracks.v0 → csg.rollout.v0 → frozen verifier
            (marker_tracker)      (video_to_tracks)       (tracks_to_rollout)  (verify_episode)

The camera (OpenCV/ArUco) dependency is optional (``pip install -e ".[camera]"``); the
tracks→rollout→verifier seam runs with neither numpy nor cv2 installed, against synthetic
in-memory fixtures + a fake detector. ``tracks_to_rollout`` is the FIRST and ONLY place
rollout evidence is minted; the rollout reuses the source-agnostic
:mod:`pilots.external_rollout` door so the camera trace meets the exact leakage contract
the RLBench pilot proved. ``csg/`` is never imported into or modified.
"""
