> **DEPRECATED.** This document describes the original V0 matcher (weighted
> DTW distance, `same_quotient_class`, `distance == 0` acceptance). That
> design was retired in the V0.1 audit — the scalar-distance criterion had an
> empty honest-zero set and the module was replaced by the probe-based
> verifier in `csg/matcher.py` (`CSG_Matcher/csg_matcher.py` is now a thin
> shim re-exporting it). Kept for historical reference only; see
> `CSG_Solver_Harness/DEPRECATED.md`, `roadmap.md` §10, and
> `physical_quotient.md` §0 for the current semantics.

---

Implemented `CSG_Matcher` as a Python module.

Download: [csg_matcher.py](sandbox:/mnt/data/csg_matcher.py)

I also included tiny sanity-check fixtures:

[human_csg_min.json](sandbox:/mnt/data/human_csg_min.json)
[robot_csg_min.json](sandbox:/mnt/data/robot_csg_min.json)
[robot_csg_diverge.json](sandbox:/mnt/data/robot_csg_diverge.json)

The matcher follows the observable CSG quotient definition: it only compares schema-observable objects, states, relations, contacts, events, temporal edges, planner constraints, evidence/confidence, and explicitly avoids hidden variables like true force, torque, friction, mass, intent, and full physical causality. That matches the discrete problem statement you provided for `\widehat Q^\star_{\mathrm{CSG}}`. 

# Core API

```python
from csg_matcher import match_csg_files, MatcherConfig

cfg = MatcherConfig(
    same_class_threshold=1e-9,
    missing_is_unknown=False,
)

result = match_csg_files(
    "human_csg.json",
    "robot_sim_csg.json",
    cfg,
)

print(result.distance)
print(result.same_quotient_class)
print(result.object_mapping)
print(result.component_distances)
```

CLI:

```bash
python csg_matcher.py human_csg.json robot_sim_csg.json --json
```

# What the algorithm computes

The implemented distance is:

```text
D(G_H, G_R)
=
min_π [
    w_graph   D_graph(G_H^π, G_R)
  + w_event   DTW(events(G_H^π), events(G_R))
  + w_traj    DTW(object_trajectories(G_H^π), object_trajectories(G_R))
  + w_final   D_terminal(G_H^π, G_R)
  + w_plan    D_planner(G_H^π, G_R)
  + w_po      D_event_partial_order(G_H^π, G_R)
  + w_topo    DTW(observable_topological_word(G_H^π), observable_topological_word(G_R))
]
```

Where:

```text
π = best object-role isomorphism
```

and agent parts are abstracted as:

```text
LEFT_HAND, RIGHT_HAND, ROBOT_GRIPPER, ROBOT_TOOL → EFFECTOR
```

This matches your quotient normalizations: time reparameterization, object-ID permutation, effector abstraction, metric tolerance, and confidence masking. 

# Object-role isomorphism

The module constructs a typed directed role graph:

```text
nodes:
  object category
  physical kind
  geometry source/type
  parts
  visual attributes
  mobility

edges:
  relation states
  relation transitions
  object-object contacts
  planner relation constraints

unary role features:
  effector contact with object
  grasp/contact modes
  event participation
  articulation transitions
```

It solves:

```text
π* = argmin_π [
    node_profile_cost
  + edge_consistency_cost
  + unary_role_consistency_cost
]
```

For small graphs, it uses exact permutation search. For larger graphs, it uses beam-search graph matching.

This is the object-ID quotient:

```text
cube_human_demo ↔ cube_robot_sim
tray_human_demo ↔ tray_robot_sim
```

rather than raw `objectId` equality.

# Time reparameterization

The matcher builds reduced event/contact/relation sequences and aligns them with Dynamic Time Warping.

This allows:

```text
human:
  contact → grasp → move → containment_change → release

robot:
  contact --------→ grasp → move ----→ containment_change → release
```

to score as equivalent if the causal/event structure is the same, even if durations differ.

The event sequence includes schema-observable event kinds such as:

```text
CONTACT_BEGIN
HAND_OBJECT_CO_MOTION
RELATION_CHANGE
CONTAINMENT_CHANGE
SUPPORT_CHANGE
ARTICULATION_CHANGE
GRASP_INFERRED
RELEASE_INFERRED
```

Those are the event modes defined in your CSG observable carrier. 

# Observable topology

The code also computes a separate DTW score over the observable topological word:

```text
relations:
  INSIDE
  CONTAINS
  ON_TOP_OF
  SUPPORTED_BY
  ALIGNED_WITH

contacts:
  SUPPORT_CONTACT_LIKELY
  CONTAINMENT_CONTACT_LIKELY
  SLIDING_LIKELY
  GRASP_LIKELY

events:
  RELATION_CHANGE
  CONTAINMENT_CHANGE
  SUPPORT_CHANGE
  ARTICULATION_CHANGE
```

This intentionally implements only the finite observable approximation to topology, not exact knot/linking/homotopy invariants, which your spec correctly marks as unavailable in V0. 

# Return object

The result is:

```python
@dataclass
class MatchResult:
    distance: float
    same_quotient_class: bool
    object_mapping: Dict[str, Optional[str]]
    reverse_object_mapping: Dict[str, Optional[str]]
    dtw_event_cost: float
    dtw_event_path: List[Tuple[int, int]]
    component_distances: Dict[str, float]
    diagnostics: Dict[str, Any]
```

Example output for matching human hand vs robot gripper on the same “put cube in tray” CSG:

```json
{
  "distance": 0.0,
  "same_quotient_class": true,
  "object_mapping": {
    "h_cube": "r_cube",
    "h_tray": "r_tray"
  },
  "component_distances": {
    "object_role_graph": 0.0,
    "event_dtw": 0.0,
    "object_trajectory_dtw": 0.0,
    "terminal_state": 0.0,
    "planner_constraints": 0.0,
    "event_partial_order": 0.0,
    "observable_topological_word_dtw": 0.0
  }
}
```

If the robot puts the cube **on top of** the tray instead of **inside** the tray, the distance becomes positive. In my sanity fixture it returns:

```text
distance=0.475449
same_quotient_class=False
```

# Important implementation choices

The matcher treats low-confidence probes as unknown, not false. This follows your confidence admissibility rule: a low-confidence or missing contact is not automatically equivalent to `NO_CONTACT_OBSERVED`. 

By default, `missing_is_unknown=False`, which is appropriate for robot simulation outputs where the simulator should produce complete relation/contact/event facts. For noisy human-video CSGs, switch to:

```python
MatcherConfig(missing_is_unknown=True)
```

That makes the matcher conservative: it does not punish unobserved facts as hard divergences.

# Distance = 0 condition

`distance == 0` when all of the following hold after best object-role mapping:

```text
1. object role graph matches
2. terminal observable state matches
3. planner constraints match
4. event partial order matches
5. event/contact/relation sequence matches under DTW
6. observable topological word matches under DTW
7. object trajectories match within tolerance, when present
8. numeric fields are equal within tolerance
9. low-confidence facts are masked consistently
10. human/robot effector differences collapse to EFFECTOR
```

So this is not a raw JSON diff. It is a quotient-class matcher over observable causal structure.
