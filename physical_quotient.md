> ## 0. Audit revisions (V0.1) — authoritative overrides
>
> The implemented checker is `csg/matcher.py`; where this document and the code
> disagree, the code is the source of truth. The following overrides apply to
> everything below:
>
> 1. **`s_plan` is NOT a probe.** Including planner constraints in the probe
>    family `S_CSG` makes the quotient depend on the *compiler* rather than the
>    world path, contradicting the definition of `Q̂*` as a function on
>    `Path(W^obs)`. The planner view is (a) the solver's input and (b) a set of
>    *predicates the verifier evaluates against the rollout's terminal state*
>    (`goal_satisfaction`). It is removed from `S_CSG` in §5.
> 2. **Hard / soft probe split.** Equivalence (PASS) requires agreement on the
>    HARD probes only: object carrier, initial state, terminal state, structural
>    relation achievement, articulation transitions, event presence, event
>    partial order, goal satisfaction. SOFT probes (contact-mode word, object
>    trajectory, contact evidence) are graded diagnostics and do not gate PASS.
>    The contact-mode word is promoted to HARD iff the target carries an explicit
>    `CONTACT_MODE_CONSTRAINT` / `CONTACT_REQUIRED`. (Rationale: the thesis says
>    reproduce the *causal transformation*, not the *manner* — so push≠grasp does
>    not fail unless the demo constrains the manner.)
> 3. **Acceptance is the probe-agreement vector, not a scalar.** `w ~ w'` iff all
>    HARD probes agree (matching the "all probes agree" definition in §5/§11).
>    The scalar distance is retained only as a curriculum/diagnostic signal. The
>    checker is therefore an equivalence test plus a graded score; it is NOT a
>    metric (no triangle inequality is claimed; symmetry holds for
>    structurally-identical graphs but the directional probes below are
>    asymmetric by design).
> 4. **Directional event/relation probes.** A faithful rollout must reproduce
>    every demonstrated event and ordering (target ⊆ robot) but MAY add extra
>    events of its own (a richer rollout is fine). Likewise structural relations
>    achieved and endpoint relations are checked as target ⊆ robot.
> 5. **Image-frame relations and pixel evidence are excluded** from quotient
>    probes (`LEFT_OF_IMAGE/…`, `min_2d_distance_px`, `mask_overlap_area_px`):
>    they are viewpoint-dependent and the human and robot cameras differ.
> 6. **One shared monotone time warp.** Event/contact/relation words and object
>    trajectories are aligned under a single monotone reparameterization
>    (event-order is computed from disjoint time spans, which is invariant under
>    any monotone warp), not per-component independent warps.
> 7. **Confidence is a mask, never a weight.** A fact below threshold is dropped
>    (treated as unknown); facts above threshold are compared without their
>    confidence value entering the cost. This makes honest distance-0 reachable.
> 8. **Relation semantics are defined by `csg/predicates.py`** (a versioned,
>    executable registry), not by prose. Both the rollout extractor and the
>    future perception compiler must use it so the matcher compares one grammar.
>
> ## 0.b Audit revisions (V0.2 / V0.3) — additional overrides
>
> 9. **What the checker implements is a subsumption preorder, not the symmetric
>    equivalence of §5.** The directional probes (override 4) define
>    `w ≼ w'` — "rollout `w'` reproduces demonstration `w`" (every demonstrated
>    fact present, extras allowed). PASS = `target ≼ robot`. The §5 relation
>    `w ~ w'` is recovered as *mutual* subsumption (`w ≼ w'` and `w' ≼ w`); the
>    benchmark's `--confusion` matrix probes exactly this cross-task: two tasks
>    are in the same quotient class iff each target PASSes the other's rollout
>    (e.g. `insert_object` ~ `put_cube_in_tray`). Where this document says
>    "equivalence" about a single directed check, read "subsumption".
> 10. **Vacuity gate.** A target with zero support on every task-defining probe
>    (goal satisfaction, relation transitions, articulation transitions)
>    constrains nothing and may not PASS anything: probe agreement with zero
>    target-side facts is agreement about nothing (V0.2, audit A1/A2).
> 11. **Object-ID permutation is implemented with 1-WL role fingerprints**
>    (`csg/matcher.py::_role_fingerprints`), and the hard mappability signature
>    is `physical_kind` only: category label, geometry kind/source, parts,
>    mobility, and visual attributes are estimator-dependent and demoted to
>    soft diagnostics (V0.2, audit A4). Role preservation in §6.2 comes from
>    the fingerprint (the object's position in the relation/contact/event fact
>    graph), not from those attributes. Equal-fingerprint objects form a
>    symmetry orbit; the matcher reports orbit membership
>    (`object_orbit_ambiguous`, `diagnostics['target_symmetry_orbits']`)
>    instead of pretending the identity within the orbit is observable.
> 12. **Physical validity is a separate verdict, not a probe.** The rollout
>    diagnostics carry `physicalValidity: true | false | null` (`null` = the
>    backend cannot check; the symbolic backend never claims `true`). The
>    benchmark gates on `is not False` and labels such PASSes
>    physics-unverified (see `csg/validity.md`).
> 13. **The rollout artifact is specified** in `csg/rollout_schema.md`
>    (`csg.rollout.v0`): the information-flow contract for what a rollout may
>    carry into the independent extractor.
>
> ---

The full physical quotient

[
Q^\star:\mathsf{Path}(\mathcal W)\to \mathsf{Task}^\star
]

cannot be implemented directly from video. The implementable object is the **observable restriction**

[
\widehat Q^\star_{\mathrm{CSG}}:
\mathsf{Path}(\mathcal W^{obs}*{\mathrm{CSG}})
\to
\mathsf{Task}^{obs}*{\mathrm{CSG}},
]

where (\mathcal W^{obs}_{\mathrm{CSG}}) is exactly the world carrier induced by the Causal Skill Graph schema. The schema explicitly says V0 is an estimator-grounded representation of observable world changes, with objects, observations, object states, relations, contacts, events, temporal edges, planner view, evidence, confidence, and covariance; it also explicitly excludes hidden physical quantities such as true force, torque, friction, mass, hidden geometry, stable grasp quality, full physical causality, and unobserved topological invariants. 

Below is the discretized formalization.

---

# 1. Observable world carrier

Let a parsed Causal Skill Graph be

[
G =
(
\texttt{objects},
\texttt{agent_parts},
\texttt{observations},
\texttt{object_states},
\texttt{relations},
\texttt{contacts},
\texttt{events},
\texttt{temporal_edges},
\texttt{planner_view},
\texttt{evidence}
).
]

The schema makes these top-level arrays explicit: `objects`, `object_states`, `relations`, `contacts`, `events`, `temporal_edges`, `planner_view`, and `evidence`. 

Let

[
\mathcal O={o_1,\ldots,o_N}
]

be the set of observed objects.

Let

[
\mathcal A={a_1,\ldots,a_M}
]

be the set of observed agent parts: human hands, human arms, robot gripper, robot tool, and so on.

Let

[
\mathcal C={c_1,\ldots,c_K}
]

be the set of cameras.

Let

[
\mathcal T={t_0,\ldots,t_T}
]

be the discrete set of timestamps induced by `Observation.time_ns`, `ObjectState.time_ns`, `RelationState.time_ns`, `Contact.time_span`, and `Event.time_span`.

At each (t\in\mathcal T), define an observable world state

[
x_t\in\mathcal W^{obs}_{\mathrm{CSG}}.
]

The state is the tuple

[
x_t =
\left(
P_t,\dot P_t,
B_t,
A_t,
D_t,
H_t,
R_t,
C_t,
E_t,
\Theta_t
\right).
]

Each component is strictly schema-derived.

---

## 1.1 Object pose array

For every object (o_i), define

[
P_t[i]=
\left(
p_i(t),q_i(t),\Sigma_i(t),m_i^P(t),\gamma_i^P(t)
\right),
]

where:

[
p_i(t)\in\mathbb R^3
]

is `Pose3D.position_m`,

[
q_i(t)\in S^3
]

is `Pose3D.orientation_wxyz`,

[
\Sigma_i(t)\in\mathbb R^{6\times 6}
]

is `Pose3D.covariance_6x6` when present,

[
m_i^P(t)\in{0,1}
]

is a missingness boolean indicating whether the 3D pose is present, and

[
\gamma_i^P(t)\in[0,1]
]

is `Pose3D.confidence`.

The schema permits `Pose3D` in object observations, object states, planner constraints, and geometry proxies; it also includes covariance and confidence fields. 

If no 3D pose is available, set

[
P_t[i]=\bot
]

and (m_i^P(t)=0).

---

## 1.2 Object twist array

For every object (o_i),

[
\dot P_t[i]=
\left(
v_i(t),\omega_i(t),m_i^{\dot P}(t),\gamma_i^{\dot P}(t)
\right),
]

where

[
v_i(t)\in\mathbb R^3
]

is `Twist3D.linear_mps`,

[
\omega_i(t)\in\mathbb R^3
]

is `Twist3D.angular_radps`.

If absent, use (\bot).

---

## 1.3 Body and geometry array

For every object (o_i), define

[
B_t[i]=
\left(
k_i,
g_i,
b_i,
\mu_i,
\gamma_i^B
\right),
]

where

[
k_i\in
{
\texttt{RIGID_OBJECT},
\texttt{ARTICULATED_OBJECT},
\texttt{DEFORMABLE_OBJECT},
\texttt{FLUID_LIKE},
\texttt{STATIC_SCENE_SURFACE},
\texttt{UNKNOWN_OBJECT_KIND}
}
]

is `ObjectPhysicalKind`.

[
g_i
]

is the `GeometryProxy`, whose source is one of:

[
\texttt{FROM_2D_MASK_ONLY},
\texttt{FROM_6D_POSE_AND_CAD},
\texttt{FROM_REFERENCE_IMAGES},
\texttt{FROM_MULTIVIEW_RECONSTRUCTION},
\texttt{FROM_DEPTH_OR_POINTMAP},
\texttt{MANUAL_APPROXIMATION}.
]

The geometry itself may be an oriented box, cylinder, mesh, point cloud, or mask-only geometry. 

[
b_i
]

is the set of observed object parts:

[
b_i\subseteq
{
\texttt{HANDLE},
\texttt{RIM},
\texttt{OPENING},
\texttt{LID},
\texttt{BUTTON},
\texttt{EDGE},
\texttt{SURFACE},
\texttt{GRASPABLE_REGION_CANDIDATE}
}.
]

The schema explicitly warns that `GRASPABLE_REGION_CANDIDATE` is only a visual candidate, not guaranteed graspability. 

[
\mu_i
]

is `BodyMobility` from `PlannerBody`, when available:

[
\mu_i\in
{
\texttt{STATIC},
\texttt{MOVABLE},
\texttt{ARTICULATED},
\texttt{UNKNOWN_MOBILITY}
}.
]

---

## 1.4 Articulation and deformable array

For every object (o_i),

[
A_t[i]=
\left(
j_i,
u_i,
\alpha_i(t),
\eta_i,
d_i(t)
\right).
]

Here

[
j_i\in
{\texttt{PRISMATIC},\texttt{REVOLUTE},\texttt{UNKNOWN_JOINT}}
]

is `JointKind`.

[
u_i\in
{
\texttt{EXTENSION_M},
\texttt{ANGLE_RAD},
\texttt{OPEN_FRACTION_0_TO_1}
}
]

is `ArticulationValueKind`.

[
\alpha_i(t)\in\mathbb R
]

is `joint_value`.

[
\eta_i\in\mathbb R^3
]

is `axis_unit` when present.

For deformables,

[
d_i(t)
]

contains only the observable descriptors permitted by the schema: visible mask, 2D keypoints, 3D keypoints, confidence, and evidence. The schema explicitly states that V0 does not store full cloth physics, only visible low-dimensional descriptors. 

---

## 1.5 Detection, mask, visibility, and observation-status tensor

For object (o_i), camera (c_k), and time (t),

[
D_t[i,k]=
\left(
r_{i,k}(t),
m_{i,k}(t),
\nu_{i,k}(t),
s_{i,k}(t),
\delta_{i,k}(t),
\tau_{i,k}(t)
\right).
]

Where:

[
r_{i,k}(t)
]

is the 2D region: bbox, mask, or polygon.

[
m_{i,k}(t)
]

is the mask reference when available.

[
\nu_{i,k}(t)\in[0,1]
]

is `visibility`.

[
s_{i,k}(t)
\in
{
\texttt{DETECTED},
\texttt{TRACKED},
\texttt{OCCLUDED},
\texttt{LOST},
\texttt{REAPPEARED}
}
]

is `ObservationStatus`.

[
\delta_{i,k}(t)
]

is detection confidence.

[
\tau_{i,k}(t)
]

is tracking confidence.

These fields are explicitly part of `ObjectObservation`. 

---

## 1.6 Agent-part state array

For each agent part (a_m),

[
H_t[m]=
\left(
h_m^{2D}(t),
h_m^{3D}(t),
\dot h_m(t),
K_m^{2D}(t),
K_m^{3D}(t),
\nu_m^H(t),
\gamma_m^H(t)
\right).
]

This comes from `AgentPartState`, which stores a 2D region, optional 3D pose, optional twist, 2D keypoints, 3D keypoints, visibility, confidence, and evidence. 

Agent parts are not task-world objects. They are effectors that may produce contacts. For the quotient, human hands and robot grippers are later collapsed into an abstract `EFFECTOR` role unless the task semantics require distinguishing them.

---

## 1.7 Relation matrix

For objects (o_i,o_j), define a relation tensor

[
R_t[i,j,r]\in[0,1],
]

where

[
r\in\mathcal R,
]

and

[
\mathcal R =
{
\texttt{NEAR},
\texttt{FAR_FROM},
\texttt{LEFT_OF_IMAGE},
\texttt{RIGHT_OF_IMAGE},
\texttt{ABOVE_IMAGE},
\texttt{BELOW_IMAGE},
\texttt{ABOVE_3D},
\texttt{BELOW_3D},
\texttt{INSIDE},
\texttt{CONTAINS},
\texttt{ON_TOP_OF},
\texttt{SUPPORTED_BY},
\texttt{TOUCHING_LIKELY},
\texttt{OCCLUDES},
\texttt{PARTIALLY_OCCLUDED_BY},
\texttt{ALIGNED_WITH}
}.
]

If the relation is present in a `RelationState` with confidence (\gamma), set

[
R_t[i,j,r]=\gamma.
]

Otherwise set it to (0) or (\bot), depending on whether absence means “not observed” or “observed false.”

The schema defines exactly this relation family, including optional metric distance and confidence. 

---

## 1.8 Contact matrix

Let

[
\mathcal E =
\mathcal O\cup \mathcal A\cup\mathcal S_{\mathrm{scene}}
]

be all contact-capable entities: objects, human parts, robot parts, and scene regions.

For entities (e_u,e_v), define

[
C_t[u,v,m]\in[0,1],
]

where

[
m\in\mathcal M,
]

and

[
\mathcal M =
{
\texttt{NO_CONTACT_OBSERVED},
\texttt{PROXIMITY_ONLY},
\texttt{TOUCHING_LIKELY},
\texttt{SUPPORT_CONTACT_LIKELY},
\texttt{CONTAINMENT_CONTACT_LIKELY},
\texttt{GRASP_LIKELY},
\texttt{SLIDING_LIKELY}
}.
]

Each contact also carries the observable evidence vector

[
\varepsilon^{contact}*{u,v}(t)=
\left(
d^{2D}*{min},
A_{overlap},
d^{3D}*{min},
\rho*{motion},
b_{statechange}
\right),
]

where these are respectively:

* `min_2d_distance_px`,
* `mask_overlap_area_px`,
* `min_3d_distance_m`,
* `motion_correlation`,
* `state_change_near_contact_boundary`.

The last one is a boolean. The schema explicitly defines these contact-evidence fields and the contact modes. 

Contact also has:

[
\lambda_{u,v}(t)
\in
{
\texttt{STICKING_LIKELY},
\texttt{SLIDING_LIKELY_RELATIVE},
\texttt{SEPARATING},
\texttt{APPROACHING},
\texttt{UNKNOWN_RELATIVE_MOTION}
}.
]

---

## 1.9 Event tensor

Define an event array

[
E_t[\ell]
]

for all events active at time (t).

Each event has:

[
E_t[\ell]=
\left(
\epsilon_\ell,
I_\ell^O,
I_\ell^A,
I_\ell^C,
B_\ell,
A_\ell,
\Delta_\ell,
\gamma_\ell
\right).
]

Where:

[
\epsilon_\ell
]

is one of:

[
\begin{aligned}
\mathcal E_v =
{&
\texttt{OBJECT_APPEARS},
\texttt{OBJECT_DISAPPEARS},
\texttt{AGENT_PART_APPROACHES_OBJECT},
\texttt{CONTACT_BEGIN},
\texttt{CONTACT_END},
\texttt{OBJECT_MOTION_BEGIN},
\texttt{OBJECT_MOTION_END},
\texttt{OBJECT_POSE_CHANGE},
\texttt{RELATION_CHANGE},
\texttt{CONTAINMENT_CHANGE},
\texttt{SUPPORT_CHANGE},
\texttt{ARTICULATION_CHANGE},
\texttt{HAND_OBJECT_CO_MOTION},
\texttt{GRASP_INFERRED},
\texttt{RELEASE_INFERRED},
\texttt{VISUAL_KEYFRAME}
}.
\end{aligned}
]

The event contains involved objects, involved agent parts, contact IDs, before-state IDs, after-state IDs, observed state deltas, generated planner constraints, confidence, and evidence. 

Each state delta is one of:

[
\Delta_\ell\in
{
\texttt{PoseDelta3D},
\texttt{RelationTransition},
\texttt{ArticulationTransition},
\texttt{VisibilityTransition}
}.
]

The schema explicitly defines these transition types. 

---

## 1.10 Planner constraint tensor

The planner-facing projection is

[
\Pi^{plan}(G)=\texttt{PlannerView}.
]

It contains:

[
\texttt{bodies},
\quad
\texttt{stages},
\quad
\texttt{preconditions},
\quad
\texttt{path_constraints},
\quad
\texttt{goal_constraints},
\quad
\texttt{contact_permissions}.
]

The schema explicitly says the planner should consume only this subset and not the full narrative graph. 

A planner constraint is one of:

[
\begin{aligned}
\mathcal K =
{&
\texttt{OBJECT_POSE_GOAL},
\texttt{OBJECT_REGION_GOAL},
\texttt{OBJECT_RELATION_GOAL},
\texttt{ARTICULATION_GOAL},
\texttt{CONTACT_MODE_CONSTRAINT},
\texttt{CONTACT_PERMISSION_CONSTRAINT},
\texttt{WAYPOINT_SEQUENCE},
\texttt{KEEP_ORIENTATION},
\texttt{KEEP_VISIBLE}
}.
\end{aligned}
]

Each constraint has:

[
(\texttt{kind},\texttt{hard},\texttt{weight},\texttt{confidence},\texttt{evidence_ids})
]

plus one of:

[
\texttt{PoseConstraint},
\texttt{RegionConstraint},
\texttt{RelationConstraint},
\texttt{ArticulationConstraint},
\texttt{ContactConstraint},
\texttt{TrajectoryConstraint},
\texttt{OrientationConstraint},
\texttt{VisibilityConstraint}.
]

These are exactly the constraint types defined in the schema. 

---

# 2. Observable strata

The original continuous theory had strata

[
\mathcal W=\bigsqcup_{\sigma\in\Sigma}\mathcal W_\sigma.
]

In the schema-discrete version, we define

[
\mathcal W^{obs}_{\mathrm{CSG}}
===============================

\bigsqcup_{\sigma\in\Sigma_{\mathrm{CSG}}}
\mathcal W^{obs}_{\sigma}.
]

A stratum is determined by the **discrete mode signature**

[
\sigma_t =
\left(
K,
M,
S_t,
R_t^\sharp,
C_t^\sharp,
L_t,
J,
V_t,
E_t^\sharp
\right).
]

Each term is schema-derived.

---

## 2.1 Object physical-kind mode

[
K[i]=\texttt{ObjectPhysicalKind}(o_i).
]

So (K[i]) is one of:

[
\texttt{RIGID_OBJECT},
\texttt{ARTICULATED_OBJECT},
\texttt{DEFORMABLE_OBJECT},
\texttt{FLUID_LIKE},
\texttt{STATIC_SCENE_SURFACE},
\texttt{UNKNOWN_OBJECT_KIND}.
]

---

## 2.2 Mobility mode

[
M[i]=\texttt{BodyMobility}(o_i)
]

when present in `PlannerBody`.

So

[
M[i]\in
{
\texttt{STATIC},
\texttt{MOVABLE},
\texttt{ARTICULATED},
\texttt{UNKNOWN_MOBILITY}
}.
]

---

## 2.3 Observation-status mode

For each object-camera pair,

[
S_t[i,k]=\texttt{ObservationStatus}(o_i,c_k,t).
]

So

[
S_t[i,k]\in
{
\texttt{DETECTED},
\texttt{TRACKED},
\texttt{OCCLUDED},
\texttt{LOST},
\texttt{REAPPEARED}
}.
]

This makes occlusion/loss not a mere nuisance. It becomes a distinct observable stratum because the quotient must know when state inference has become underdetermined.

---

## 2.4 Relation mode matrix

Choose a confidence threshold

[
\eta_R\in[0,1].
]

Define the boolean relation matrix

[
R_t^\sharp[i,j,r]
=================

\mathbf 1
\left[
R_t[i,j,r]\ge \eta_R
\right].
]

Thus every stratum contains the currently active object-object relations:

[
\texttt{INSIDE},
\texttt{ON_TOP_OF},
\texttt{SUPPORTED_BY},
\texttt{ALIGNED_WITH},
\ldots
]

These relation modes are the primary symbolic topology available in V0.

---

## 2.5 Contact mode matrix

Choose a confidence threshold

[
\eta_C\in[0,1].
]

Define

[
C_t^\sharp[u,v,m]
=================

\mathbf 1
\left[
C_t[u,v,m]\ge \eta_C
\right].
]

This includes modes such as:

[
\texttt{PROXIMITY_ONLY},
\texttt{TOUCHING_LIKELY},
\texttt{SUPPORT_CONTACT_LIKELY},
\texttt{CONTAINMENT_CONTACT_LIKELY},
\texttt{GRASP_LIKELY},
\texttt{SLIDING_LIKELY}.
]

The contact matrix is the discrete replacement for the continuous contact-mode strata in the original theory.

---

## 2.6 Relative-motion mode

For each contacting pair,

[
L_t[u,v]=\texttt{RelativeMotionKind}(u,v,t).
]

So

[
L_t[u,v]\in
{
\texttt{STICKING_LIKELY},
\texttt{SLIDING_LIKELY_RELATIVE},
\texttt{SEPARATING},
\texttt{APPROACHING},
\texttt{UNKNOWN_RELATIVE_MOTION}
}.
]

This is the only schema-supported proxy for contact kinematics. It is not force.

---

## 2.7 Articulation mode

For object (o_i),

[
J[i]=
\left(
\texttt{JointKind}_i,
\texttt{ArticulationValueKind}_i
\right).
]

The continuous joint value

[
\alpha_i(t)
]

lives inside the stratum. The joint-kind class determines the stratum.

Thus a drawer with a prismatic joint and a door with a revolute joint live in different strata.

---

## 2.8 Visibility mode

Choose visibility threshold

[
\eta_V\in[0,1].
]

Define

[
V_t[i,k]=\mathbf 1[\nu_{i,k}(t)\ge \eta_V].
]

This boolean is derived from the schema’s `visibility` scalar.

A task that requires keeping an object visible may preserve this as a semantic probe.

---

## 2.9 Event mode

Define

[
E_t^\sharp[\epsilon]
====================

\mathbf 1
[
\exists \ell:
E_t[\ell].\texttt{event_kind}=\epsilon
\ \wedge
E_t[\ell].\texttt{confidence}\ge \eta_E
].
]

Thus the active event types at time (t) become part of the stratum:

[
\texttt{CONTACT_BEGIN},
\texttt{OBJECT_POSE_CHANGE},
\texttt{RELATION_CHANGE},
\texttt{CONTAINMENT_CHANGE},
\texttt{SUPPORT_CHANGE},
\texttt{ARTICULATION_CHANGE},
\texttt{GRASP_INFERRED},
\texttt{RELEASE_INFERRED}.
]

---

## 2.10 Stratum definition

Now define:

[
\boxed{
\mathcal W^{obs}_{\sigma}
=========================

\left{
x_t\in\mathcal W^{obs}_{\mathrm{CSG}}
:
\operatorname{disc}(x_t)=\sigma
\right}.
}
]

Where

[
\operatorname{disc}(x_t)
========================

\left(
K,
M,
S_t,
R_t^\sharp,
C_t^\sharp,
L_t,
J,
V_t,
E_t^\sharp
\right).
]

Inside a stratum, continuous quantities may vary:

[
p_i(t),
q_i(t),
v_i(t),
\omega_i(t),
\alpha_i(t),
\texttt{distance_m},
\texttt{joint_value},
\texttt{confidence}.
]

Crossing a stratum boundary occurs when a discrete observable mode changes:

[
\texttt{NO_CONTACT_OBSERVED}
\to
\texttt{TOUCHING_LIKELY},
]

[
\texttt{NEAR}
\to
\texttt{INSIDE},
]

[
\texttt{CLOSED}
\to
\texttt{OPEN_FRACTION_0_TO_1}=0.7,
]

[
\texttt{TRACKED}
\to
\texttt{OCCLUDED},
]

[
\texttt{CONTACT_BEGIN}
\to
\texttt{HAND_OBJECT_CO_MOTION}.
]

---

# 3. Observable paths

A path is a finite timestamped sequence:

[
w=
(x_{t_0},x_{t_1},\ldots,x_{t_T}).
]

The path has a stratum word:

[
\operatorname{StrataWord}(w)
============================

\sigma_{t_0}\sigma_{t_1}\cdots\sigma_{t_T}.
]

Compress consecutive duplicate strata:

[
\operatorname{RedStrataWord}(w)
===============================

\operatorname{compress}
(
\operatorname{StrataWord}(w)
).
]

This gives a discrete contact-relation-event word:

[
\texttt{approach}
\to
\texttt{touch}
\to
\texttt{grasp_likely}
\to
\texttt{object_motion}
\to
\texttt{inside}
\to
\texttt{release}.
]

Formally, this is the observable replacement for a continuous path through physical contact manifolds.

---

# 4. Semantic probes

Define the schema-allowable semantic probe family:

[
\mathcal S_{\mathrm{CSG}}
=========================

{
s_1,\ldots,s_L
}.
]

Each probe is a function

[
s_\ell:
\mathsf{Path}(\mathcal W^{obs}*{\mathrm{CSG}})
\to Z*\ell.
]

No probe may use hidden force, hidden mass, true friction, human intent, tactile state, hidden geometry, stable grasp quality, or unobserved topological invariants, because the schema explicitly excludes those. 

---

## 4.1 Object-carrier probe

[
s_{\mathrm{obj}}(w)
===================

\left[
(o_i,k_i,g_i,b_i,\mu_i,\texttt{visual_attributes}*i)
\right]*{i=1}^N.
]

This captures which observable bodies exist and their observable types.

It uses:

* `object_id`,
* `category_label`,
* `category_confidence`,
* `physical_kind`,
* `geometry`,
* `parts`,
* `visual_attributes`,
* `evidence_ids`.

---

## 4.2 Initial-state probe

[
s_{\mathrm{init}}(w)
====================

x_{t_0}
\quad
\text{projected to object states, relations, contacts, and articulations.}
]

This includes:

[
P_{t_0},
A_{t_0},
R_{t_0},
C_{t_0},
D_{t_0}.
]

This is the observable precondition state.

---

## 4.3 Terminal-state probe

[
s_{\mathrm{final}}(w)
=====================

x_{t_T}
\quad
\text{projected to object states, relations, contacts, and articulations.}
]

This is the observable goal-state candidate.

For “put cube in tray”:

[
s_{\mathrm{final}}(w)
\supset
R_{t_T}[\texttt{cube},\texttt{tray},\texttt{INSIDE}].
]

---

## 4.4 Relation-transition probe

[
s_{\Delta R}(w)
===============

\left{
(i,j,r_{before},r_{after},t)
:
\texttt{RelationTransition}
\right}.
]

This uses `StateDelta.relation_transition`.

Example:

[
(\texttt{red_cube},
\texttt{black_tray},
\texttt{NEAR},
\texttt{INSIDE},
t).
]

This is the main symbolic task-success probe.

---

## 4.5 Pose-delta probe

[
s_{\Delta P}(w)
===============

\left{
(i,P_{from},P_{to},\Sigma,\gamma)
:
\texttt{PoseDelta3D}
\right}.
]

This uses `StateDelta.pose_delta_3d`.

If pose is absent, the probe returns (\bot).

---

## 4.6 Object-waypoint probe

For every object (o_i), define a reduced waypoint sequence:

[
s_{\mathrm{traj}}^i(w)
======================

\operatorname{simplify}*{\epsilon}
\left(
P*{t_0}[i],
P_{t_1}[i],
\ldots,
P_{t_T}[i]
\right).
]

This is allowed only when 3D poses exist.

The schema’s `TrajectoryConstraint` explicitly stores sparse object waypoints, not human hand waypoints. 

So the quotient may preserve:

[
\text{object trajectory}
]

but not:

[
\text{human hand trajectory as the task itself}.
]

---

## 4.7 Articulation-transition probe

[
s_{\Delta A}(w)
===============

\left{
(i,j_i,\alpha_{from},\alpha_{to},u_i,\eta_i)
:
\texttt{ArticulationTransition}
\right}.
]

Example:

[
(\texttt{drawer},
\texttt{PRISMATIC},
0.02m,
0.18m,
\texttt{EXTENSION_M},
\texttt{axis})
]

This gives an observable representation of opening, closing, extending, or rotating articulated objects.

---

## 4.8 Contact-mode word probe

For each entity pair ((e_u,e_v)),

[
s_C^{u,v}(w)
============

\operatorname{compress}
\left(
\arg\max_m C_t[u,v,m]
\right)_{t=t_0}^{t_T}.
]

Example:

[
\texttt{NO_CONTACT}
\to
\texttt{PROXIMITY_ONLY}
\to
\texttt{TOUCHING_LIKELY}
\to
\texttt{GRASP_LIKELY}
\to
\texttt{NO_CONTACT}.
]

This is the observable contact grammar of the task.

---

## 4.9 Contact-evidence probe

[
s_{\mathrm{contactEvidence}}(w)
===============================

\left{
d^{2D}*{min},
A*{overlap},
d^{3D}*{min},
\rho*{motion},
b_{statechange}
\right}
]

for every contact interval.

The boolean

[
b_{statechange}
===============

\texttt{state_change_near_contact_boundary}
]

is crucial. It is the schema’s closest proxy for causal intervention.

It does not prove causality. It only says an observable state change occurred near a contact boundary.

---

## 4.10 Grasp/release proxy probe

Define:

[
s_{\mathrm{graspProxy}}(w)
==========================

\left{
(e_u,o_i,t_a,t_b)
:
C_t[u,i,\texttt{GRASP_LIKELY}]\ge\eta_C
\right}.
]

Define:

[
s_{\mathrm{releaseProxy}}(w)
============================

\left{
(e_u,o_i,t)
:
\texttt{CONTACT_END}
\vee
\texttt{RELEASE_INFERRED}
\right}.
]

This uses only:

* `GRASP_LIKELY`,
* `HAND_OBJECT_CO_MOTION`,
* `RELEASE_INFERRED`,
* `CONTACT_END`,
* `motion_correlation`,
* `state_change_near_contact_boundary`.

It does **not** assert force closure or stable grasp.

---

## 4.11 Support and containment probe

[
s_{\mathrm{sup/con}}(w)
=======================

\left{
R_t[i,j,\texttt{SUPPORTED_BY}],
R_t[i,j,\texttt{ON_TOP_OF}],
R_t[i,j,\texttt{INSIDE}],
R_t[i,j,\texttt{CONTAINS}],
C_t[i,j,\texttt{SUPPORT_CONTACT_LIKELY}],
C_t[i,j,\texttt{CONTAINMENT_CONTACT_LIKELY}]
\right}_{t}.
]

This is the schema-level approximation of containment and support topology.

---

## 4.12 Visibility and occlusion probe

[
s_{\mathrm{vis}}(w)
===================

\left{
\nu_{i,k}(t),
S_t[i,k],
R_t[i,j,\texttt{OCCLUDES}],
R_t[i,j,\texttt{PARTIALLY_OCCLUDED_BY}]
\right}_{i,j,k,t}.
]

This matters because one-shot inference can become mathematically underdetermined when the manipulated object is occluded or lost.

---

## 4.13 Event partial-order probe

Let `TemporalEdge` define a relation

[
\prec
]

on events.

Then

[
s_{\mathrm{eventPO}}(w)
=======================

\left(
{E_\ell},
\prec
\right).
]

The relation (\prec) uses:

[
\texttt{BEFORE},
\texttt{AFTER},
\texttt{DURING},
\texttt{OVERLAPS},
\texttt{MEETS}.
]

This gives a partial order over observed task events.

---

## 4.14 Planner-constraint probe

> **V0.1 (§0.1): not a quotient probe.** `s_plan` is removed from `S_CSG`.
> Planner constraints are the solver's input and, for verification, are turned
> into predicates evaluated against the rollout terminal state
> (`goal_satisfaction`). The text below is retained for the projection's
> definition, not as a member of the equivalence-defining probe family.

[
s_{\mathrm{plan}}(w)
====================

\texttt{PlannerView.stages}.
]

For each stage (z), preserve:

[
\left(
\texttt{preconditions},
\texttt{path_constraints},
\texttt{goal_constraints},
\texttt{contact_permissions},
\texttt{hard},
\texttt{weight},
\texttt{tolerance},
\texttt{confidence}
\right).
]

This is the planner-facing quotient projection.

The schema’s planner view explicitly reduces the full graph to bodies, stages, preconditions, path constraints, goal constraints, and contact permissions. 

---

## 4.15 Observable topological word probe

The schema cannot directly store true topology. Therefore the allowed observable topological probe is only:

[
s_{\mathrm{topo}}^{obs}(w)
==========================

\operatorname{compress}
\left(
R_t^{topo},
C_t^{topo},
E_t^{topo}
\right)_{t=t_0}^{t_T},
]

where

[
R_t^{topo}
==========

{
\texttt{INSIDE},
\texttt{CONTAINS},
\texttt{ON_TOP_OF},
\texttt{SUPPORTED_BY},
\texttt{ALIGNED_WITH},
\texttt{OCCLUDES},
\texttt{PARTIALLY_OCCLUDED_BY}
},
]

[
C_t^{topo}
==========

{
\texttt{SUPPORT_CONTACT_LIKELY},
\texttt{CONTAINMENT_CONTACT_LIKELY},
\texttt{SLIDING_LIKELY},
\texttt{GRASP_LIKELY}
},
]

and

[
E_t^{topo}
==========

{
\texttt{RELATION_CHANGE},
\texttt{CONTAINMENT_CHANGE},
\texttt{SUPPORT_CHANGE},
\texttt{ARTICULATION_CHANGE}
}.
]

So the schema supports a **regular-language approximation to topology**:

[
\text{topology as a word over relation/contact/event modes}.
]

It does not support exact linking number, winding number, knot class, braid class, or homotopy class unless additional geometric variables are present.

---

# 5. The observable quotient

Define

[
\mathcal S_{\mathrm{CSG}}
=========================

{
s_{\mathrm{obj}},
s_{\mathrm{init}},
s_{\mathrm{final}},
s_{\Delta R},
s_{\Delta P},
s_{\mathrm{traj}},
s_{\Delta A},
s_C,
s_{\mathrm{contactEvidence}},
s_{\mathrm{graspProxy}},
s_{\mathrm{sup/con}},
s_{\mathrm{vis}},
s_{\mathrm{eventPO}},
s_{\mathrm{topo}}^{obs}
}.
]

**(V0.1 revision, see §0):** `s_plan` is intentionally *removed* from `S_CSG`.
The planner view is not a probe on the world path; the verifier instead
evaluates the target's hard planner goals as predicates against the rollout's
terminal observable state (the `goal_satisfaction` probe). The probes above are
partitioned into HARD (gate PASS) and SOFT (graded only) per §0.2.

Now define equivalence between observable paths.

Two observable paths

[
w,w'\in \mathsf{Path}(\mathcal W^{obs}_{\mathrm{CSG}})
]

are equivalent,

[
w\sim_{\mathrm{CSG}} w',
]

if and only if all semantic probes agree after the allowed normalizations below.

[
\boxed{
w\sim_{\mathrm{CSG}} w'
\iff
\forall s\in\mathcal S_{\mathrm{CSG}},
\quad
s(w)=s(w')
\quad
\text{modulo schema-allowed normalization.}
}
]

The quotient is:

[
\boxed{
\widehat Q^\star_{\mathrm{CSG}}(w)
==================================

[w]*{\sim*{\mathrm{CSG}}}.
}
]

This is the discrete approximation to (Q^\star).

**Implementation note (subsumption preorder vs equivalence, §0.b-9).** The
checker does not evaluate the symmetric relation above directly. Because the
directional probes allow the rollout to be a refinement of the demonstration
(extra events, fragmented relations), the implemented single-shot check is the
*subsumption* `w ≼ w'` ("`w'` reproduces `w`"), and a verifier PASS means
`target ≼ robot`. The symmetric `w ~ w'` is exactly mutual subsumption and is
what the benchmark's cross-task confusion matrix measures: two demonstrations
denote the same quotient class iff each one's target accepts the other's
rollout. Anti-symmetry is *not* claimed — distinct paths can subsume each
other; that is what "same task class" means here.

---

# 6. Allowed quotient normalizations

The quotient may remove nuisance variation, but only if the schema supports doing so.

## 6.1 Time reparameterization

If two paths have the same event partial order and same reduced stratum word, they may differ in duration.

Thus:

[
w(t)
\sim
w'(\phi(t))
]

for any monotone time reparameterization (\phi) preserving event order:

[
s_{\mathrm{eventPO}}(w)=s_{\mathrm{eventPO}}(w').
]

Durations may still be preserved when a `Tolerance.time_s` or timing-sensitive constraint exists.

---

## 6.2 Object-ID permutation

Object IDs are graph-local. They should not define the task.

Let

[
\pi:\mathcal O\to\mathcal O'
]

be a bijection preserving:

[
\texttt{category_label},
\quad
\texttt{physical_kind},
\quad
\texttt{geometry source/type},
\quad
\texttt{parts},
\quad
\texttt{visual_attributes},
\quad
\texttt{role in relation/contact/event graph}.
]

Then

[
w\sim_{\mathrm{CSG}} \pi(w).
]

If two identical objects are symmetry-equivalent, the quotient should return an **orbit**, not a single arbitrary identity.

Example:

[
{\texttt{cube_1},\texttt{cube_2}}/S_2.
]

This prevents the quotient from hallucinating a difference between two indistinguishable red cubes.

**Implementation note (§0.b-11).** Of the attribute list above, only
`physical_kind` is a *hard* mappability constraint; the others are
estimator-dependent and enter only as soft diagnostics. "Role in the
relation/contact/event graph" is implemented as a 1-WL color-refinement
fingerprint (`csg/matcher.py::_role_fingerprints`), and candidate bijections
are generated by a fingerprint-similarity-guided DFS bounded by
`MatcherConfig.max_candidate_mappings` — not N! enumeration, so ten identical
cubes align by role in microseconds. Equal-fingerprint objects are reported as
an orbit (`object_orbit_ambiguous`), realizing the (S_2) example above.

---

## 6.3 Effector abstraction

Define an abstraction map:

[
\phi_{\mathrm{eff}}:
{
\texttt{LEFT_HAND},
\texttt{RIGHT_HAND},
\texttt{ROBOT_GRIPPER},
\texttt{ROBOT_TOOL}
}
\to
\texttt{EFFECTOR}.
]

Then contact events may be compared under:

[
\texttt{human hand contacts object}
\sim
\texttt{robot gripper contacts object},
]

provided the object-state, relation-state, contact-mode, and event probes agree.

This is the mathematical removal of embodiment-specific motion.

However, if the task explicitly requires a specific agent part — for example, “left hand holds object while right hand opens lid” — then `AgentPartKind` must be preserved and not collapsed.

---

## 6.4 Metric tolerance

For any pose, orientation, distance, region, articulation, or waypoint value, equality is tolerance-aware.

For scalar values:

[
a\equiv b
\iff
|a-b|\le \epsilon.
]

For positions:

[
p\equiv p'
\iff
|p-p'|_2\le
\texttt{Tolerance.position_m}.
]

For orientations:

[
q\equiv q'
\iff
d_{SO(3)}(q,q')\le
\texttt{Tolerance.orientation_rad}.
]

For relations:

[
r\equiv r'
\iff
r=r'
]

unless a relation-specific tolerance is provided by `RelationConstraint`.

---

## 6.5 Confidence admissibility

Every probe has a confidence.

For probe (s), define an admissibility threshold (\eta_s).

If confidence is below threshold:

[
\gamma_s<\eta_s,
]

then the probe value is not treated as false. It is treated as unknown:

[
s(w)=\bot.
]

This is crucial. A missing or low-confidence contact is not equivalent to no contact.

---

# 7. Canonical quotient signature

The quotient class can be represented by a canonical signature:

```json
{
  "taskClass": {
    "objectOrbit": "...",
    "initialObservableState": "...",
    "terminalObservableState": "...",
    "reducedStratumWord": "...",
    "relationTransitions": "...",
    "contactModeWords": "...",
    "eventPartialOrder": "...",
    "objectTrajectoryWords": "...",
    "articulationTransitions": "...",
    "goalPredicates": "evaluated against the rollout terminal state, not a probe (§0-1)",
    "observableTopologicalWord": "...",
    "confidencePolicy": "..."
  }
}
```

Mathematically:

[
\boxed{
\widehat Q^\star_{\mathrm{CSG}}(w)
==================================

\operatorname{Canon}
\left(
s_{\mathrm{obj}}(w),
s_{\mathrm{init}}(w),
s_{\mathrm{final}}(w),
s_{\Delta R}(w),
s_{\Delta P}(w),
s_{\mathrm{traj}}(w),
s_{\Delta A}(w),
s_C(w),
s_{\mathrm{contactEvidence}}(w),
s_{\mathrm{eventPO}}(w),
s_{\mathrm{topo}}^{obs}(w)
\right).
}
]

(`s_plan` does not appear: per §0-1 the planner view is not a probe on the
world path. The target's hard goals are carried alongside the signature as
predicates to evaluate against a candidate rollout's terminal state.)

This is the finest observable quotient induced by the current schema.

Any coarser task representation must factor through it:

[
Q'(w)
=====

F
\left(
\widehat Q^\star_{\mathrm{CSG}}(w)
\right).
]

---

# 8. Example: “put cube in tray”

The schema’s own minimal example contains:

* a red cube,
* a black tray,
* hand-cube `GRASP_LIKELY`,
* `motionCorrelation`,
* a `CONTAINMENT_CHANGE`,
* a relation transition from `NEAR` to `INSIDE`,
* and a planner goal constraint requiring the red cube to be inside the tray. 

The quotient signature is therefore not:

```text
"human puts red cube into black tray"
```

because `task_caption` is explicitly not authoritative for planning. 

It is:

[
\widehat Q^\star_{\mathrm{CSG}}(w)
==================================

\left[
\begin{array}{l}
\text{movable rigid object } o_1,\
\text{container/support object } o_2,\
R_{0}[o_1,o_2,\texttt{NEAR}],\
C[o_1,\texttt{effector},\texttt{GRASP_LIKELY}],\
\texttt{HAND_OBJECT_CO_MOTION},\
\texttt{CONTAINMENT_CHANGE},\
R_T[o_1,o_2,\texttt{INSIDE}],\
\texttt{goal: OBJECT_RELATION_GOAL}(o_1,o_2,\texttt{INSIDE}).
\end{array}
\right].
]

This is embodiment-invariant because the human hand is abstracted to an effector, while the world-state transition is preserved.

---

# 9. What topology survives discretization?

Only this:

[
\boxed{
\text{Topology}^{obs}
=====================

\text{a word over observable relation modes, contact modes, support modes, containment modes, and event transitions.}
}
]

So for V0, topological structure means:

```text
outside → contact → supported_by → inside
```

or

```text
near handle → contact → articulation_change → open_fraction increases
```

or

```text
above surface → support_contact_likely → on_top_of
```

The schema does **not** justify exact statements like:

```text
the ring linked around the peg with linking number 1
the rope formed a knot
the cable passed through the loop without crossing
the cloth wrapped twice around the cylinder
```

unless additional geometry and continuous visibility are provided.

Thus the schema supports a **finite automaton of topological relation changes**, not full algebraic topology.

---

# 10. Unobservable Critical Variables

These are variables required by the full (Q^\star) theory but missing or only weakly represented in the current schema.

## 10.1 True force

Needed for:

[
\text{push},
\text{pull},
\text{press},
\text{insert},
\text{scrub},
\text{twist},
\text{deform}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
ContactMode
RelativeMotionKind
state_change_near_contact_boundary
motion_correlation
surface_normal_unit
```

Problem:

[
\texttt{surface_normal_unit}\neq \text{force direction}.
]

The schema explicitly says surface normal should not be treated as force. 

---

## 10.2 Torque

Needed for:

[
\text{turn knob},
\text{unscrew cap},
\text{rotate handle},
\text{twist tool}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
ArticulationTransition
PoseDelta3D
RelativeMotionKind
```

Missing:

[
\tau(t)\in\mathbb R^3.
]

---

## 10.3 Friction coefficient

Needed for:

[
\text{slide},
\text{wipe},
\text{push without slipping},
\text{grasp stability}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
SLIDING_LIKELY
STICKING_LIKELY
motion_correlation
```

Missing:

[
\mu_{ij}
]

for contacting surfaces.

The uploaded schema explicitly excludes friction coefficient. 

---

## 10.4 Mass and inertia

Needed for:

[
\text{lift feasibility},
\text{throwing},
\text{pouring},
\text{dynamic manipulation},
\text{force planning}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
ObjectPhysicalKind
GeometryProxy
motion response from video
```

Missing:

[
m_i,\quad I_i.
]

The schema explicitly excludes mass. 

---

## 10.5 Material and compliance

Needed for:

[
\text{cloth},
\text{sponge},
\text{foam},
\text{food},
\text{deformable packaging}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
VisualAttribute("material_appearance")
ObjectPhysicalKind.DEFORMABLE_OBJECT
DeformableState.visible_mask
DeformableState.keypoints
```

Missing:

[
\text{elasticity},
\quad
\text{plasticity},
\quad
\text{stiffness},
\quad
\text{damping}.
]

The schema stores only visible low-dimensional deformable descriptors, not full cloth or material physics. 

---

## 10.6 Hidden 3D geometry

Needed for:

[
\text{grasp planning},
\text{insertion},
\text{containment volume},
\text{collision-free motion},
\text{topological invariants}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
GeometryProxy
Region3D
MeshRef
PointCloudRef
MaskOnlyGeometry
```

Problem:

If geometry source is only

```text
FROM_2D_MASK_ONLY
```

then the system lacks metric 3D geometry.

This blocks exact reasoning about containment, insertion, and topology.

---

## 10.7 Object topology

Needed for:

[
\text{thread through loop},
\text{ring around peg},
\text{tie knot},
\text{wrap cable},
\text{hook object}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
INSIDE
CONTAINS
ON_TOP_OF
SUPPORTED_BY
ALIGNED_WITH
GeometryProxy.mesh if available
PointCloudRef if available
```

Missing:

[
H_k(\text{object}),
\quad
\pi_1(\text{free space}),
\quad
\text{skeleton graph},
\quad
\text{loop/handle topology}.
]

CV team target:

```text
Predict object topological class:
  solid
  container
  loop
  handle
  hook
  flexible strand
  sheet
  articulated chain
```

This should eventually become a schema field.

---

## 10.8 Continuous swept volume

Needed for:

[
\text{homotopy class},
\text{winding number},
\text{non-crossing path},
\text{threading},
\text{wrapping}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
TrajectoryConstraint.object_waypoints
PoseDelta3D
ObjectState.pose_3d
```

Missing:

[
\text{swept volume }
\bigcup_t \mathrm{Geom}(o_i,t).
]

Sparse waypoints are insufficient for exact topological path invariants.

---

## 10.9 Persistent contact patch

Needed for:

[
\text{rolling},
\text{scrubbing},
\text{pinching},
\text{precision insertion},
\text{stable grasp}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
contact_point_m
surface_normal_unit
region_2d_a
region_2d_b
ContactMode
RelativeMotionKind
```

Missing:

[
\mathcal P_{contact}(t)
\subset \partial o_i \times \partial o_j.
]

The schema has optional contact point/patch proxies, but not a persistent 3D contact manifold.

---

## 10.10 Stable grasp quality

Needed for:

[
\text{whether robot can execute the task}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
GRASP_LIKELY
motion_correlation
STICKING_LIKELY
state_change_near_contact_boundary
GRASPABLE_REGION_CANDIDATE
```

Missing:

[
\text{force closure},
\quad
\text{wrench cone},
\quad
\text{slip margin},
\quad
\text{gripper-object compatibility}.
]

The schema explicitly says visual graspable-region candidates are not guaranteed graspable. 

---

## 10.11 Articulation limits and mechanism model

Needed for:

[
\text{open drawer fully},
\text{avoid breaking hinge},
\text{turn knob},
\text{open door along correct arc}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
JointKind
axis_unit
joint_value
ArticulationValueKind
```

Missing:

[
\alpha_{min},
\quad
\alpha_{max},
\quad
\text{joint stiffness},
\quad
\text{joint damping},
\quad
\text{handle-to-joint kinematic map}.
]

---

## 10.12 Fluid state

Needed for:

[
\text{pour},
\text{stir},
\text{fill},
\text{empty},
\text{mix}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
ObjectPhysicalKind.FLUID_LIKE
mask
visibility
possibly relation INSIDE / CONTAINS
```

Missing:

[
\text{volume},
\quad
\text{free surface},
\quad
\text{flow rate},
\quad
\text{viscosity},
\quad
\text{liquid-container interface}.
]

---

## 10.13 Human intention

Needed only when observable effects are ambiguous.

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
task_caption
VLM_EVENT_LABEL
planner constraints
final state
```

But `task_caption` is explicitly human-readable and not authoritative for planning. 

In the quotient, intention should not be used unless grounded by observable state transitions.

---

## 10.14 Full physical causality

Needed for:

[
\text{distinguishing coincidence from causation}.
]

Schema status:

```text
UNOBSERVABLE CRITICAL VARIABLE
```

Current proxy:

```text
state_change_near_contact_boundary
motion_correlation
temporal_edges
CONTACT_BEGIN
OBJECT_MOTION_BEGIN
RELATION_CHANGE
```

Missing:

[
\text{interventional evidence}.
]

The schema stores observable transitions and contact likelihoods; it does not prove causal mechanisms.

---

# 11. Final discrete problem statement

The implementable version of the North Star is:

[
\boxed{
\begin{aligned}
&\textbf{Observable CSG Quotient Problem} \
\
&\text{Given a Causal Skill Graph }G\text{ obeying the provided schema,}\
&\text{construct a finite observable path }w_G
\in \mathsf{Path}(\mathcal W^{obs}*{\mathrm{CSG}}),\
&\text{where each state }x_t\text{ is composed only of schema fields:}\
&\quad
\texttt{ObjectState},
\texttt{RelationState},
\texttt{Contact},
\texttt{Event},
\texttt{TemporalEdge},
\texttt{PlannerConstraint},
\texttt{Evidence}.\
\
&\text{Define strata }\mathcal W^{obs}*\sigma
\text{ by discrete schema modes:}\
&\quad
\texttt{ObjectPhysicalKind},
\texttt{BodyMobility},
\texttt{ObservationStatus},
\texttt{RelationKind},
\texttt{ContactMode},
\texttt{RelativeMotionKind},
\texttt{JointKind},
\texttt{EventKind}.\
\
&\text{Define semantic probes }\mathcal S_{\mathrm{CSG}}
\text{ from observable transitions,}\
&\quad
\text{relations, contacts, articulations, trajectories, temporal edges,}\
&\quad
\text{planner constraints, visibility, evidence, and confidence.}\
\
&\text{Then define the quotient}\
&\quad
\widehat Q^\star_{\mathrm{CSG}}(w_G)
====================================

[w_G]*{\sim*{\mathrm{CSG}}},\
&\text{where }w\sim_{\mathrm{CSG}}w'
\text{ iff all probes in }\mathcal S_{\mathrm{CSG}}
\text{ agree}\
&\text{up to time reparameterization, object-role isomorphism,}\
&\text{effector abstraction, numeric tolerances, and confidence masking.}
\end{aligned}
}
]

This is the strict discretization:

[
\boxed{
Q^\star
\leadsto
\widehat Q^\star_{\mathrm{CSG}}
===============================

\text{the finest quotient preserving all schema-observable semantic probes.}
}
]

Anything required by the original (Q^\star) but absent from the schema is not allowed into the quotient. It must be recorded as an **Unobservable Critical Variable** and assigned to perception, active probing, simulation identification, or robot-side experimentation.
