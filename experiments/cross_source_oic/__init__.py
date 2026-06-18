"""Cross-source "One Task, Four Worlds" report for ``object_inside_container``.

The SAME semantic task — *did the object end up inside the container, having been put
there?* — bound to four worlds (MuJoCo internal sim, RLBench external sim, Sony/iPhone
real camera, RH20T real-robot video) and judged by the SAME frozen verifier core
(``csg.matcher.match`` + ``csg.rollout_extract.extract_robot_csg``). Every leg is
recomputed live from committed inputs — no MuJoCo / RLBench / cv2 install needed.

This package is READ-ONLY over ``csg/`` and the pilot datasets; it adds no new
verifier logic. See ``legs.py`` (the four source resolvers) and ``target_equivalence.py``
(the structural-equivalence proof that the per-source target cards share one enforced
semantic core). Build the artifact with ``python3 -m scripts.build_cross_source_report``.
"""
