> ## Audit revisions (V0.1) — authoritative overrides
>
> Implemented in the `csg/` package. Where this document and the code disagree,
> the code wins.
>
> 1. **ObservationGraph vs TaskSpec split.** A CSG has two parts: the
>    *ObservationGraph* (objects, observations, object_states, relations,
>    contacts, events, evidence — what a rollout may contain) and the *TaskSpec*
>    (`PlannerView` + tolerances — what a compiler emits and a solver consumes).
>    **A robot/rollout CSG must contain ONLY the ObservationGraph.** It must not
>    carry `plannerView`, a `targetCsg` copy, or `solverMetadata`. The verifier
>    strips/forbids these on the rollout side and evaluates the target's planner
>    goals as predicates against the rollout's terminal state. `csg/benchmark.py`
>    enforces this as a leakage gate; `Event.generated_constraints` is TaskSpec
>    and likewise must not appear in a rollout CSG.
> 2. **New estimator kind `SIM_STATE_EXTRACTION`** for evidence produced by the
>    rollout extractor (`csg/rollout_extract.py`). A robot CSG's evidence must be
>    sim-extraction provenance, never `HUMAN_POSE`/`Sapiens2` (which would prove
>    the human graph was copied).
> 3. **`RelationKind` / `ContactMode` semantics are normatively defined by the
>    versioned predicate registry `csg/predicates.py`**, not by prose. Both the
>    rollout extractor and the perception compiler (roadmap Phase 3) must import it so
>    the matcher compares words from one grammar (e.g. INSIDE = object center
>    below the container rim and within its footprint; resting *on the rim* is
>    ON_TOP_OF, not INSIDE).
> 4. **Provenance is required for fabricated fields.** Invented geometry/size or
>    poses (no metric estimate) must be flagged `MANUAL_APPROXIMATION` /
>    `sizeApproximate` / `initialPoseApproximate`, never asserted as observed.
> 5. **Example fix:** the tray below is modeled `STATIC_SCENE_SURFACE`, but a
>    tray is a movable rigid object held static only by planner convenience. Use
>    `physicalKind: RIGID_OBJECT` with `PlannerBody.mobility: STATIC` (a TaskSpec
>    assertion, with provenance) rather than baking "static" into the observed
>    `physicalKind`. `BodyMobility` "configured" is a prior, not an observation.
>    *(Applied in V0.3: the `gold_tests/put_cube_in_tray` fixtures now model the
>    tray as `RIGID_OBJECT` + `mobility: STATIC` with metric `orientedBox`
>    geometry matching the previous fallback size.)*
> 6. **The rollout artifact has its own schema**, `csg.rollout.v0`, documented
>    in `csg/rollout_schema.md`. It is deliberately *much narrower* than this
>    observation-graph schema: the rollout is the information-flow boundary
>    between solver and independent extractor, and may carry only what an
>    instantiated simulator could honestly report (sanitized bodies, frames,
>    honest diagnostics — no labels, no planner view, no target copy).
>
> ---

Yes. For V0, the **Causal Skill Graph** should be a typed, estimator-grounded representation of **observable world changes**, not a symbolic fantasy layer.

Use **Protobuf** as the canonical schema and serialize to JSON when needed. The planner-facing subset should reduce to rigid/articulated bodies, object poses, relation goals, contact-mode hints, waypoints, and tolerances. That maps cleanly to MuJoCo/Isaac-style scenes because MuJoCo models bodies, joints, geoms, and constraints, while Isaac/PhysX/USD-style stacks represent rigid bodies, articulations, joints, and collision properties. ([MuJoCo Documentation][1])

The fields below are limited to things current systems can plausibly estimate from video: human pose/parts from Sapiens2, object detection from open-set detectors such as Grounding DINO, video masks/tracks from SAM 2-style segmentation, and conditional 6D object pose from tools such as FoundationPose when a CAD model or reference images are available. ([arXiv][2])

# Causal Skill Graph V0

Core design rule:

```text
No field may exist unless it is:
  1. directly observed,
  2. estimated by a named model,
  3. derived from observable geometry/tracking,
  4. or explicitly marked as low-confidence / conditional.
```

So V0 includes:

```text
objects
2D detections
masks
tracks
optional 6D poses
human hand/body tracks
contact likelihood
object-object relations
articulation estimates
event boundaries
state transitions
planner constraints
evidence/provenance
confidence/covariance
```

V0 excludes:

```text
true force
true torque
friction coefficient
mass
material parameters
human intent
tactile state
hidden object geometry
stable grasp quality
reward
full physical causality
unobserved topological invariants
```

---

# `csg_v0.proto`

```proto
syntax = "proto3";

package csg.v0;

// -----------------------------------------------------------------------------
// CAUSAL SKILL GRAPH V0
// -----------------------------------------------------------------------------
//
// Design principle:
// This schema stores only observable or estimator-derived facts from video.
// Every nontrivial assertion should reference Evidence.
// Planner-facing fields are separated into PlannerView / SkillStage / Constraint.
// -----------------------------------------------------------------------------

message CausalSkillGraph {
  string schema_version = 1;          // Example: "csg.v0"
  string graph_id = 2;

  // Human-readable label only. Not authoritative for planning.
  string task_caption = 3;

  TimeSpan observed_time = 4;

  repeated SourceVideo source_videos = 5;
  repeated Camera cameras = 6;
  repeated CoordinateFrame frames = 7;

  repeated AgentPart agent_parts = 8;       // Human hands, arms, body parts.
  repeated Object objects = 9;

  repeated Observation observations = 10;   // Raw per-frame/per-camera observations.
  repeated ObjectState object_states = 11;  // Estimated states at keyframes or frames.
  repeated RelationState relations = 12;    // Observable object-object relations.
  repeated Contact contacts = 13;           // Contact/contact-likelihood intervals.
  repeated Event events = 14;               // Observable world-state transitions.

  repeated TemporalEdge temporal_edges = 15;

  PlannerView planner_view = 16;

  repeated Evidence evidence = 17;
}

// -----------------------------------------------------------------------------
// TIME / SOURCE / CALIBRATION
// -----------------------------------------------------------------------------

message TimeSpan {
  int64 start_time_ns = 1;
  int64 end_time_ns = 2;
}

message SourceVideo {
  string video_id = 1;
  string uri = 2;

  int32 width_px = 3;
  int32 height_px = 4;
  double fps = 5;

  string camera_id = 6;

  // Optional synchronization info.
  int64 first_frame_time_ns = 7;
  int64 frame_count = 8;
}

message Camera {
  string camera_id = 1;
  string label = 2;

  CameraIntrinsics intrinsics = 3;

  // Camera pose in a world/table frame, if calibrated.
  Pose3D extrinsics_world_from_camera = 4;

  CalibrationStatus calibration_status = 5;
}

enum CalibrationStatus {
  CALIBRATION_STATUS_UNSPECIFIED = 0;
  CALIBRATION_UNKNOWN = 1;
  INTRINSICS_ONLY = 2;
  EXTRINSICS_ESTIMATED = 3;
  EXTRINSICS_CALIBRATED = 4;
}

message CameraIntrinsics {
  double fx = 1;
  double fy = 2;
  double cx = 3;
  double cy = 4;

  // k1, k2, p1, p2, k3 if available.
  repeated double distortion = 5;
}

message CoordinateFrame {
  string frame_id = 1;
  string parent_frame_id = 2;

  Pose3D parent_from_frame = 3;

  FrameType frame_type = 4;
  float confidence = 5;

  repeated string evidence_ids = 6;
}

enum FrameType {
  FRAME_TYPE_UNSPECIFIED = 0;
  WORLD = 1;
  TABLE = 2;
  CAMERA = 3;
  OBJECT = 4;
  HUMAN_PART = 5;
  ROBOT_BASE = 6;
}

// -----------------------------------------------------------------------------
// BASIC GEOMETRY
// -----------------------------------------------------------------------------

message Vec2 {
  double x = 1;
  double y = 2;
}

message Vec3 {
  double x = 1;
  double y = 2;
  double z = 3;
}

message Quaternion {
  // Convention: w, x, y, z.
  double w = 1;
  double x = 2;
  double y = 3;
  double z = 4;
}

message Pose3D {
  string frame_id = 1;

  Vec3 position_m = 2;
  Quaternion orientation_wxyz = 3;

  // Optional 6x6 covariance row-major:
  // x, y, z, roll, pitch, yaw.
  repeated double covariance_6x6 = 4;

  float confidence = 5;
}

message Twist3D {
  string frame_id = 1;

  Vec3 linear_mps = 2;
  Vec3 angular_radps = 3;

  float confidence = 4;
}

message BBox2D {
  int32 x_min_px = 1;
  int32 y_min_px = 2;
  int32 x_max_px = 3;
  int32 y_max_px = 4;

  float confidence = 5;
}

message MaskRef {
  // URI or key into object storage.
  string uri = 1;

  int32 width_px = 2;
  int32 height_px = 3;

  MaskEncoding encoding = 4;

  float confidence = 5;
}

enum MaskEncoding {
  MASK_ENCODING_UNSPECIFIED = 0;
  RLE = 1;
  PNG_BINARY = 2;
  POLYGON = 3;
  COCO_RLE = 4;
}

message Region2D {
  string camera_id = 1;
  int64 frame_index = 2;

  oneof region {
    BBox2D bbox = 3;
    MaskRef mask = 4;
    Polygon2D polygon = 5;
  }

  float confidence = 6;
}

message Polygon2D {
  repeated Vec2 vertices_px = 1;
}

message Region3D {
  string frame_id = 1;

  oneof region {
    Box3D box = 2;
    Sphere3D sphere = 3;
    Cylinder3D cylinder = 4;
    OrientedBox3D oriented_box = 5;
  }

  float confidence = 6;
}

message Box3D {
  Vec3 min_m = 1;
  Vec3 max_m = 2;
}

message OrientedBox3D {
  Pose3D pose = 1;
  Vec3 size_m = 2;
}

message Sphere3D {
  Vec3 center_m = 1;
  double radius_m = 2;
}

message Cylinder3D {
  Pose3D pose = 1;
  double radius_m = 2;
  double height_m = 3;
}

message Tolerance {
  double position_m = 1;
  double orientation_rad = 2;
  double distance_m = 3;
  double time_s = 4;
}

// -----------------------------------------------------------------------------
// AGENT PARTS
// -----------------------------------------------------------------------------
//
// These are observed human/robot effectors, not task-world objects.
// For human demos, this is where Sapiens2 hand/body outputs land.
// -----------------------------------------------------------------------------

message AgentPart {
  string agent_part_id = 1;

  AgentKind agent_kind = 2;
  AgentPartKind part_kind = 3;

  string label = 4;

  repeated string evidence_ids = 5;
}

enum AgentKind {
  AGENT_KIND_UNSPECIFIED = 0;
  HUMAN = 1;
  ROBOT = 2;
}

enum AgentPartKind {
  AGENT_PART_KIND_UNSPECIFIED = 0;
  LEFT_HAND = 1;
  RIGHT_HAND = 2;
  LEFT_ARM = 3;
  RIGHT_ARM = 4;
  HEAD = 5;
  TORSO = 6;
  ROBOT_GRIPPER = 7;
  ROBOT_TOOL = 8;
}

message AgentPartState {
  string state_id = 1;
  string agent_part_id = 2;

  int64 time_ns = 3;
  string camera_id = 4;
  int64 frame_index = 5;

  Region2D region_2d = 6;

  // Optional, only if estimated from pose/pointmap/depth/multiview.
  Pose3D pose_3d = 7;
  Twist3D twist_3d = 8;

  repeated Keypoint2D keypoints_2d = 9;
  repeated Keypoint3D keypoints_3d = 10;

  float visibility = 11;
  float confidence = 12;

  repeated string evidence_ids = 13;
}

message Keypoint2D {
  string name = 1;
  Vec2 pixel = 2;
  float confidence = 3;
}

message Keypoint3D {
  string name = 1;
  Vec3 point_m = 2;
  string frame_id = 3;
  float confidence = 4;
}

// -----------------------------------------------------------------------------
// OBJECTS
// -----------------------------------------------------------------------------

message Object {
  string object_id = 1;

  // Example: "red cube", "bowl", "drawer", "cup", "handle".
  string category_label = 2;
  float category_confidence = 3;

  ObjectPhysicalKind physical_kind = 4;

  // Conservative geometry proxy usable by planner/simulator.
  GeometryProxy geometry = 5;

  // Observable parts only: handle, rim, opening, lid, button, edge, surface.
  repeated ObjectPart parts = 6;

  // Optional human-readable attributes from detection/VLM.
  repeated VisualAttribute visual_attributes = 7;

  repeated string evidence_ids = 8;
}

enum ObjectPhysicalKind {
  OBJECT_PHYSICAL_KIND_UNSPECIFIED = 0;
  RIGID_OBJECT = 1;
  ARTICULATED_OBJECT = 2;
  DEFORMABLE_OBJECT = 3;
  FLUID_LIKE = 4;
  STATIC_SCENE_SURFACE = 5;
  UNKNOWN_OBJECT_KIND = 6;
}

message GeometryProxy {
  GeometrySource source = 1;

  oneof geometry {
    OrientedBox3D oriented_box = 2;
    Cylinder3D cylinder = 3;
    MeshRef mesh = 4;
    PointCloudRef point_cloud = 5;
    MaskOnlyGeometry mask_only = 6;
  }

  float confidence = 7;
}

enum GeometrySource {
  GEOMETRY_SOURCE_UNSPECIFIED = 0;
  FROM_2D_MASK_ONLY = 1;
  FROM_6D_POSE_AND_CAD = 2;
  FROM_REFERENCE_IMAGES = 3;
  FROM_MULTIVIEW_RECONSTRUCTION = 4;
  FROM_DEPTH_OR_POINTMAP = 5;
  MANUAL_APPROXIMATION = 6;
}

message MeshRef {
  string uri = 1;
  string frame_id = 2;
  double scale_m = 3;
}

message PointCloudRef {
  string uri = 1;
  string frame_id = 2;
}

message MaskOnlyGeometry {
  // Used when no metric 3D geometry is available.
  string note = 1;
}

message ObjectPart {
  string part_id = 1;
  string object_id = 2;

  ObjectPartKind kind = 3;
  string label = 4;

  // At least one of these should be set.
  Region2D region_2d = 5;
  Region3D region_3d = 6;

  float confidence = 7;
  repeated string evidence_ids = 8;
}

enum ObjectPartKind {
  OBJECT_PART_KIND_UNSPECIFIED = 0;
  HANDLE = 1;
  RIM = 2;
  OPENING = 3;
  LID = 4;
  BUTTON = 5;
  EDGE = 6;
  SURFACE = 7;
  GRASPABLE_REGION_CANDIDATE = 8; // Visual candidate, not guaranteed graspable.
}

message VisualAttribute {
  string name = 1;       // Example: "color", "shape", "material_appearance".
  string value = 2;      // Example: "red", "cube-like", "metal-looking".
  float confidence = 3;
  repeated string evidence_ids = 4;
}

// -----------------------------------------------------------------------------
// OBSERVATIONS
// -----------------------------------------------------------------------------

message Observation {
  string observation_id = 1;

  int64 time_ns = 2;
  string source_video_id = 3;
  string camera_id = 4;
  int64 frame_index = 5;

  repeated ObjectObservation object_observations = 6;
  repeated AgentPartState agent_part_states = 7;

  repeated string evidence_ids = 8;
}

message ObjectObservation {
  string observation_id = 1;
  string object_id = 2;

  BBox2D bbox_2d = 3;
  MaskRef mask = 4;

  // Optional 6D pose estimate.
  Pose3D pose_3d = 5;

  float visibility = 6;         // 0 = fully occluded, 1 = fully visible.
  float detection_confidence = 7;
  float tracking_confidence = 8;

  ObservationStatus status = 9;

  repeated string evidence_ids = 10;
}

enum ObservationStatus {
  OBSERVATION_STATUS_UNSPECIFIED = 0;
  DETECTED = 1;
  TRACKED = 2;
  OCCLUDED = 3;
  LOST = 4;
  REAPPEARED = 5;
}

// -----------------------------------------------------------------------------
// STATES
// -----------------------------------------------------------------------------

message ObjectState {
  string state_id = 1;
  string object_id = 2;

  int64 time_ns = 3;

  // 2D observation state.
  Region2D region_2d = 4;

  // 3D state if available.
  Pose3D pose_3d = 5;
  Twist3D twist_3d = 6;

  // For articulated objects like drawers, doors, lids.
  ArticulationState articulation = 7;

  // For deformables, keep only observable low-dimensional descriptors.
  DeformableState deformable = 8;

  float visibility = 9;
  float confidence = 10;

  repeated string evidence_ids = 11;
}

message ArticulationState {
  string articulated_object_id = 1;

  JointKind joint_kind = 2;

  // Estimated visible joint axis if available.
  Vec3 axis_unit = 3;
  string axis_frame_id = 4;

  // Examples:
  // drawer extension in meters,
  // door angle in radians,
  // normalized open fraction.
  double joint_value = 5;
  ArticulationValueKind value_kind = 6;

  float confidence = 7;
  repeated string evidence_ids = 8;
}

enum JointKind {
  JOINT_KIND_UNSPECIFIED = 0;
  PRISMATIC = 1;
  REVOLUTE = 2;
  UNKNOWN_JOINT = 3;
}

enum ArticulationValueKind {
  ARTICULATION_VALUE_KIND_UNSPECIFIED = 0;
  EXTENSION_M = 1;
  ANGLE_RAD = 2;
  OPEN_FRACTION_0_TO_1 = 3;
}

message DeformableState {
  // V0 does not store full cloth physics.
  // It stores only visible descriptors.
  MaskRef visible_mask = 1;
  repeated Keypoint2D keypoints_2d = 2;
  repeated Keypoint3D keypoints_3d = 3;

  float confidence = 4;
  repeated string evidence_ids = 5;
}

message RelationState {
  string relation_id = 1;

  int64 time_ns = 2;

  string subject_object_id = 3;
  string object_object_id = 4;

  RelationKind relation = 5;

  // Optional metric support when poses are available.
  double distance_m = 6;

  float confidence = 7;
  repeated string evidence_ids = 8;
}

enum RelationKind {
  RELATION_KIND_UNSPECIFIED = 0;

  NEAR = 1;
  FAR_FROM = 2;

  LEFT_OF_IMAGE = 3;
  RIGHT_OF_IMAGE = 4;
  ABOVE_IMAGE = 5;
  BELOW_IMAGE = 6;

  ABOVE_3D = 7;
  BELOW_3D = 8;

  INSIDE = 9;
  CONTAINS = 10;

  ON_TOP_OF = 11;
  SUPPORTED_BY = 12;

  TOUCHING_LIKELY = 13;

  OCCLUDES = 14;
  PARTIALLY_OCCLUDED_BY = 15;

  ALIGNED_WITH = 16;
}

// -----------------------------------------------------------------------------
// CONTACTS
// -----------------------------------------------------------------------------

message EntityRef {
  EntityKind kind = 1;
  string id = 2;
}

enum EntityKind {
  ENTITY_KIND_UNSPECIFIED = 0;
  OBJECT_ENTITY = 1;
  HUMAN_PART_ENTITY = 2;
  ROBOT_PART_ENTITY = 3;
  SCENE_REGION_ENTITY = 4;
}

message Contact {
  string contact_id = 1;

  EntityRef a = 2;
  EntityRef b = 3;

  TimeSpan time_span = 4;

  ContactMode mode = 5;

  ContactEvidence contact_evidence = 6;

  // Estimated contact regions, if visible.
  Region2D region_2d_a = 7;
  Region2D region_2d_b = 8;

  // Optional 3D contact point/patch if pose/depth/geometry supports it.
  Vec3 contact_point_m = 9;
  string contact_frame_id = 10;

  // Optional surface normal estimate.
  // Do not treat as force.
  Vec3 surface_normal_unit = 11;

  RelativeMotionKind relative_motion = 12;

  float confidence = 13;
  repeated string evidence_ids = 14;
}

enum ContactMode {
  CONTACT_MODE_UNSPECIFIED = 0;

  NO_CONTACT_OBSERVED = 1;
  PROXIMITY_ONLY = 2;
  TOUCHING_LIKELY = 3;
  SUPPORT_CONTACT_LIKELY = 4;
  CONTAINMENT_CONTACT_LIKELY = 5;

  // Inferred from hand/object contact plus co-motion.
  GRASP_LIKELY = 6;

  // Inferred from contact plus tangential relative motion.
  SLIDING_LIKELY = 7;
}

message ContactEvidence {
  double min_2d_distance_px = 1;
  double mask_overlap_area_px = 2;

  // Only valid if 3D poses/depth are available.
  double min_3d_distance_m = 3;

  // Correlation between hand/object motion over the interval.
  double motion_correlation = 4;

  // True when object starts/stops moving near contact boundary.
  bool state_change_near_contact_boundary = 5;
}

enum RelativeMotionKind {
  RELATIVE_MOTION_KIND_UNSPECIFIED = 0;
  STICKING_LIKELY = 1;
  SLIDING_LIKELY_RELATIVE = 2;
  SEPARATING = 3;
  APPROACHING = 4;
  UNKNOWN_RELATIVE_MOTION = 5;
}

// -----------------------------------------------------------------------------
// EVENTS
// -----------------------------------------------------------------------------
//
// Events are observable changes, not intentions.
// Example:
//   CONTACT_BEGIN
//   OBJECT_POSE_CHANGE
//   RELATION_CHANGE
//   ARTICULATION_CHANGE
//   CONTAINMENT_CHANGE
// -----------------------------------------------------------------------------

message Event {
  string event_id = 1;

  EventKind event_kind = 2;

  TimeSpan time_span = 3;

  repeated string involved_object_ids = 4;
  repeated string involved_agent_part_ids = 5;
  repeated string contact_ids = 6;

  repeated string before_state_ids = 7;
  repeated string after_state_ids = 8;

  repeated StateDelta observed_deltas = 9;

  // Planner constraints derived from this observed event.
  repeated PlannerConstraint generated_constraints = 10;

  float confidence = 11;
  repeated string evidence_ids = 12;
}

enum EventKind {
  EVENT_KIND_UNSPECIFIED = 0;

  OBJECT_APPEARS = 1;
  OBJECT_DISAPPEARS = 2;

  AGENT_PART_APPROACHES_OBJECT = 3;
  CONTACT_BEGIN = 4;
  CONTACT_END = 5;

  OBJECT_MOTION_BEGIN = 6;
  OBJECT_MOTION_END = 7;
  OBJECT_POSE_CHANGE = 8;

  RELATION_CHANGE = 9;
  CONTAINMENT_CHANGE = 10;
  SUPPORT_CHANGE = 11;

  ARTICULATION_CHANGE = 12;

  HAND_OBJECT_CO_MOTION = 13;
  GRASP_INFERRED = 14;
  RELEASE_INFERRED = 15;

  VISUAL_KEYFRAME = 16;
}

message StateDelta {
  string object_id = 1;

  // Optional pose delta.
  PoseDelta3D pose_delta_3d = 2;

  // Optional relation transition.
  RelationTransition relation_transition = 3;

  // Optional articulation transition.
  ArticulationTransition articulation_transition = 4;

  // Optional visibility transition.
  VisibilityTransition visibility_transition = 5;

  float confidence = 6;
  repeated string evidence_ids = 7;
}

message PoseDelta3D {
  Pose3D from_pose = 1;
  Pose3D to_pose = 2;
}

message RelationTransition {
  string subject_object_id = 1;
  string object_object_id = 2;

  RelationKind from_relation = 3;
  RelationKind to_relation = 4;
}

message ArticulationTransition {
  string articulated_object_id = 1;

  ArticulationState from_state = 2;
  ArticulationState to_state = 3;
}

message VisibilityTransition {
  double from_visibility = 1;
  double to_visibility = 2;
}

// -----------------------------------------------------------------------------
// TEMPORAL STRUCTURE
// -----------------------------------------------------------------------------

message TemporalEdge {
  string edge_id = 1;

  string from_event_id = 2;
  string to_event_id = 3;

  TemporalRelation relation = 4;

  float confidence = 5;
  repeated string evidence_ids = 6;
}

enum TemporalRelation {
  TEMPORAL_RELATION_UNSPECIFIED = 0;
  BEFORE = 1;
  AFTER = 2;
  DURING = 3;
  OVERLAPS = 4;
  MEETS = 5;
}

// -----------------------------------------------------------------------------
// PLANNER VIEW
// -----------------------------------------------------------------------------
//
// This is the subset meant to be compiled into MuJoCo/Isaac/MoveIt-style
// constraints. It should not contain VLM-only narrative.
// -----------------------------------------------------------------------------

message PlannerView {
  string world_frame_id = 1;

  repeated PlannerBody bodies = 2;
  repeated SkillStage stages = 3;

  repeated string evidence_ids = 4;
}

message PlannerBody {
  string object_id = 1;

  ObjectPhysicalKind physical_kind = 2;
  GeometryProxy geometry = 3;

  string initial_state_id = 4;

  // Optional static/dynamic hint. This is visually inferred or configured.
  BodyMobility mobility = 5;

  float confidence = 6;
}

enum BodyMobility {
  BODY_MOBILITY_UNSPECIFIED = 0;
  STATIC = 1;
  MOVABLE = 2;
  ARTICULATED = 3;
  UNKNOWN_MOBILITY = 4;
}

message SkillStage {
  string stage_id = 1;

  // Usually points back to the observed event that produced this stage.
  string source_event_id = 2;

  repeated PlannerConstraint preconditions = 3;
  repeated PlannerConstraint path_constraints = 4;
  repeated PlannerConstraint goal_constraints = 5;

  repeated ContactPermission contact_permissions = 6;

  TimeSpan observed_time_span = 7;

  float confidence = 8;
  repeated string evidence_ids = 9;
}

message PlannerConstraint {
  string constraint_id = 1;

  ConstraintKind kind = 2;

  // Hard constraints must be satisfied.
  // Soft constraints become planner costs.
  bool hard = 3;
  double weight = 4;

  oneof constraint {
    PoseConstraint pose = 5;
    RegionConstraint region = 6;
    RelationConstraint relation = 7;
    ArticulationConstraint articulation = 8;
    ContactConstraint contact = 9;
    TrajectoryConstraint trajectory = 10;
    OrientationConstraint orientation = 11;
    VisibilityConstraint visibility = 12;
  }

  float confidence = 13;
  repeated string evidence_ids = 14;
}

enum ConstraintKind {
  CONSTRAINT_KIND_UNSPECIFIED = 0;

  OBJECT_POSE_GOAL = 1;
  OBJECT_REGION_GOAL = 2;
  OBJECT_RELATION_GOAL = 3;

  ARTICULATION_GOAL = 4;

  CONTACT_MODE_CONSTRAINT = 5;
  CONTACT_PERMISSION_CONSTRAINT = 6;

  WAYPOINT_SEQUENCE = 7;
  KEEP_ORIENTATION = 8;

  KEEP_VISIBLE = 9;
}

message PoseConstraint {
  string target_object_id = 1;

  Pose3D target_pose = 2;
  Tolerance tolerance = 3;
}

message RegionConstraint {
  string target_object_id = 1;

  // Example: cube center should end inside tray volume.
  Region3D target_region = 2;

  Tolerance tolerance = 3;
}

message RelationConstraint {
  string subject_object_id = 1;
  string object_object_id = 2;

  RelationKind desired_relation = 3;

  Tolerance tolerance = 4;
}

message ArticulationConstraint {
  string articulated_object_id = 1;

  JointKind joint_kind = 2;
  double target_joint_value = 3;
  ArticulationValueKind value_kind = 4;

  Tolerance tolerance = 5;
}

message ContactConstraint {
  EntityRef a = 1;
  EntityRef b = 2;

  ContactMode required_mode = 3;

  // During which stage should this hold?
  string stage_id = 4;

  Tolerance tolerance = 5;
}

message TrajectoryConstraint {
  string target_object_id = 1;

  // Sparse waypoints from observed object trajectory.
  // These are object waypoints, not human hand waypoints.
  repeated Pose3D object_waypoints = 2;

  Tolerance waypoint_tolerance = 3;
}

message OrientationConstraint {
  string target_object_id = 1;

  Quaternion desired_orientation_wxyz = 2;
  string frame_id = 3;

  Tolerance tolerance = 4;
}

message VisibilityConstraint {
  string target_object_id = 1;

  string camera_id = 2;
  double min_visibility = 3;
}

message ContactPermission {
  EntityRef a = 1;
  EntityRef b = 2;

  ContactPermissionKind permission = 3;

  string stage_id = 4;
}

enum ContactPermissionKind {
  CONTACT_PERMISSION_KIND_UNSPECIFIED = 0;
  CONTACT_ALLOWED = 1;
  CONTACT_FORBIDDEN = 2;
  CONTACT_REQUIRED = 3;
}

// -----------------------------------------------------------------------------
// EVIDENCE / PROVENANCE
// -----------------------------------------------------------------------------

message Evidence {
  string evidence_id = 1;

  EstimatorKind estimator = 2;

  string model_name = 3;       // Example: "Sapiens2", "SAM2", "FoundationPose".
  string model_version = 4;

  string source_video_id = 5;
  string camera_id = 6;

  int64 start_frame_index = 7;
  int64 end_frame_index = 8;

  float confidence = 9;

  // Optional URI to raw estimator output:
  // masks, pose json, tracking file, heatmaps, VLM response, etc.
  string artifact_uri = 10;
}

enum EstimatorKind {
  ESTIMATOR_KIND_UNSPECIFIED = 0;

  HUMAN_POSE = 1;
  HUMAN_PART_SEGMENTATION = 2;
  HUMAN_POINTMAP = 3;

  OBJECT_DETECTION = 4;
  OBJECT_SEGMENTATION = 5;
  OBJECT_TRACKING = 6;

  OBJECT_6D_POSE = 7;

  VLM_RELATION_LABEL = 8;
  VLM_EVENT_LABEL = 9;

  OPTICAL_FLOW = 10;
  DEPTH_OR_POINTMAP = 11;

  MANUAL_ANNOTATION = 12;

  // V0.1: provenance for facts derived by the rollout extractor from a
  // simulated state trace (csg/rollout_extract.py). A rollout/robot CSG must
  // use this estimator family only — never human-side estimators.
  SIM_STATE_EXTRACTION = 13;
}
```

---

# Planner-facing interpretation

The planner should **not** consume the whole graph. It should consume only:

```text
PlannerView
  bodies
  stages
  preconditions
  path_constraints
  goal_constraints
  contact_permissions
```

For example, a human video of “put red cube in black tray” should compile to:

```text
Stage 1:
  contact allowed: gripper ↔ red_cube
  path constraint: red_cube remains visible
  goal: red_cube lifted above table

Stage 2:
  path constraint: red_cube follows sparse object waypoints
  orientation constraint: keep cube approximately upright, if observed

Stage 3:
  goal relation: red_cube INSIDE black_tray
  contact permission: red_cube ↔ black_tray allowed
  contact requirement: gripper ↔ red_cube ends
```

The graph does **not** say:

```text
human intended to pick up cube
robot should imitate hand shape
force should be 3.2 N
friction is high
grasp is stable
```

It says:

```text
red_cube changed pose
red_cube became co-moving with right_hand
red_cube transitioned from outside tray to inside tray
contact likely began before motion and ended after placement
```

That is the right V0 quotient approximation.

---

# Minimal JSON example

This is what a tiny serialized graph might look like after protobuf JSON mapping.

```json
{
  "schemaVersion": "csg.v0",
  "graphId": "demo_put_cube_in_tray_0001",
  "taskCaption": "human puts red cube into black tray",
  "objects": [
    {
      "objectId": "obj_red_cube",
      "categoryLabel": "red cube",
      "categoryConfidence": 0.94,
      "physicalKind": "RIGID_OBJECT",
      "geometry": {
        "source": "FROM_6D_POSE_AND_CAD",
        "orientedBox": {
          "pose": {
            "frameId": "world",
            "positionM": {"x": 0.31, "y": 0.12, "z": 0.025},
            "orientationWxyz": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "confidence": 0.82
          },
          "sizeM": {"x": 0.04, "y": 0.04, "z": 0.04}
        },
        "confidence": 0.82
      },
      "visualAttributes": [
        {
          "name": "color",
          "value": "red",
          "confidence": 0.96
        }
      ]
    },
    {
      "objectId": "obj_black_tray",
      "categoryLabel": "black tray",
      "categoryConfidence": 0.91,
      "physicalKind": "STATIC_SCENE_SURFACE",
      "geometry": {
        "source": "FROM_2D_MASK_ONLY",
        "maskOnly": {
          "note": "tray detected and tracked; metric 3D volume estimated from calibrated table plane"
        },
        "confidence": 0.76
      }
    }
  ],
  "contacts": [
    {
      "contactId": "contact_right_hand_cube_01",
      "a": {
        "kind": "HUMAN_PART_ENTITY",
        "id": "human_right_hand"
      },
      "b": {
        "kind": "OBJECT_ENTITY",
        "id": "obj_red_cube"
      },
      "timeSpan": {
        "startTimeNs": "2100000000",
        "endTimeNs": "4800000000"
      },
      "mode": "GRASP_LIKELY",
      "contactEvidence": {
        "min2dDistancePx": 2.7,
        "maskOverlapAreaPx": 184.0,
        "motionCorrelation": 0.91,
        "stateChangeNearContactBoundary": true
      },
      "relativeMotion": "STICKING_LIKELY",
      "confidence": 0.81
    }
  ],
  "events": [
    {
      "eventId": "event_cube_inside_tray",
      "eventKind": "CONTAINMENT_CHANGE",
      "timeSpan": {
        "startTimeNs": "4700000000",
        "endTimeNs": "5300000000"
      },
      "involvedObjectIds": [
        "obj_red_cube",
        "obj_black_tray"
      ],
      "observedDeltas": [
        {
          "objectId": "obj_red_cube",
          "relationTransition": {
            "subjectObjectId": "obj_red_cube",
            "objectObjectId": "obj_black_tray",
            "fromRelation": "NEAR",
            "toRelation": "INSIDE"
          },
          "confidence": 0.88
        }
      ],
      "confidence": 0.88
    }
  ],
  "plannerView": {
    "worldFrameId": "world",
    "bodies": [
      {
        "objectId": "obj_red_cube",
        "physicalKind": "RIGID_OBJECT",
        "initialStateId": "state_cube_start",
        "mobility": "MOVABLE",
        "confidence": 0.82
      },
      {
        "objectId": "obj_black_tray",
        "physicalKind": "STATIC_SCENE_SURFACE",
        "initialStateId": "state_tray",
        "mobility": "STATIC",
        "confidence": 0.76
      }
    ],
    "stages": [
      {
        "stageId": "stage_place_cube_in_tray",
        "sourceEventId": "event_cube_inside_tray",
        "goalConstraints": [
          {
            "constraintId": "goal_cube_inside_tray",
            "kind": "OBJECT_RELATION_GOAL",
            "hard": true,
            "weight": 1.0,
            "relation": {
              "subjectObjectId": "obj_red_cube",
              "objectObjectId": "obj_black_tray",
              "desiredRelation": "INSIDE",
              "tolerance": {
                "positionM": 0.03
              }
            },
            "confidence": 0.88
          }
        ],
        "contactPermissions": [
          {
            "a": {
              "kind": "ROBOT_PART_ENTITY",
              "id": "robot_gripper"
            },
            "b": {
              "kind": "OBJECT_ENTITY",
              "id": "obj_red_cube"
            },
            "permission": "CONTACT_ALLOWED",
            "stageId": "stage_place_cube_in_tray"
          }
        ],
        "confidence": 0.85
      }
    ]
  }
}
```

---

# What this schema deliberately does not solve

This is not yet `Q*`. It is the **V0 observable carrier** from which approximations to `Q*` can be learned.

In math terms:

```text
raw video y_H
  → estimator outputs
  → CausalSkillGraph V0
  → approximate task quotient τ
  → robot planner / policy
```

The CSG is not the quotient itself. It is the **typed evidence graph** that makes quotient learning possible.

The V0 boundary is:

```text
Store measured world-state transitions.
Do not store hidden causes as facts.
Represent uncertain contacts as likelihoods.
Represent inferred events as evidence-backed transitions.
Expose planner constraints separately from narrative labels.
```

That gives the research program a solid foundation: every later LLM, VLA, simulator, and quotient learner can operate on the same inspectable graph instead of hallucinated task descriptions.

[1]: https://mujoco.readthedocs.io/en/stable/XMLreference.html?utm_source=chatgpt.com "XML Reference - MuJoCo Documentation"
[2]: https://arxiv.org/abs/2604.21681?utm_source=chatgpt.com "[2604.21681] Sapiens2"
