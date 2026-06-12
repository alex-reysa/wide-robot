> **RETIRED AS PROJECT SCOPE (2026-06-10).** This document is the original
> broad vision ("Universal Demonstration Compiler" / one-shot human-to-robot
> learning). It is kept as background reading and long-horizon context only.
> The project's actual scope, claims discipline, and phase plan live in
> `roadmap.md`: **a leakage-clean compiler/verifier loop for fixed-base
> robotic-arm manipulation.** Nothing below (VLA training, Sapiens2 pipelines,
> Unreal data factories, world-model verification, one-shot claims) is current
> work — see `roadmap.md` §9 "What NOT to build". The one idea from this
> document that survived intact is the abstraction: map object-state
> transitions, relations, contacts, and event order — never human joints to
> robot joints.

---

Yes. The research direction is real, timely, and not solved. The disruptive version is **not** “robot copies human joint motion.” It is:

> **A human demonstration is compiled into a causal, object-centric task program; a robot foundation model executes that program through its own embodiment.**

That is the core shift. The robot should not imitate the human body. It should infer **what changed in the world**, **why it changed**, **which contacts caused it**, and **which robot actions can reproduce the same state transition**.

# 1. The research problem

Define the objective as:

```text
Given:
  one human demonstration video V_H
  a robot R with cameras, proprioception, gripper, action space A_R
  a current scene observation O_R

Infer:
  the task T demonstrated by the human
  the causal object-state changes C
  the robot-executable policy π_R

Such that:
  executing π_R in the robot scene causes the same task-level outcome.
```

Formal version:

```text
G* = argmax_G P(G | V_H, world_prior, physics_prior)

π_R(a_t | o_t, q_t, G*, V_H) → robot actions

Success if:
  final_world_state(robot) ≈ final_world_state(human_demo)
  and constraints are satisfied:
    safety, collisions, object integrity, task order, success condition.
```

Where `G` is not a text caption. It is a **causal task graph**:

```text
objects:
  cup, drawer, cloth, handle, cube, tray

states:
  cup.location = table
  drawer.state = closed
  cloth.state = wrinkled

events:
  contact(hand, object)
  grasp(object)
  move(object, target)
  insert(object, container)
  wipe(surface)
  release(object)

constraints:
  keep cup upright
  apply force along drawer axis
  avoid collision with bowl
  maintain cloth contact with table

goal:
  object_inside_container = true
  surface_clean = true
  drawer_open = true
```

This is the “compiler target.” The human video becomes a **task program**, not merely training data.

# 2. Existing research landscape

## A. One-shot imitation from human videos

This idea has existed for years, but it has usually been limited to short-horizon manipulation, constrained tasks, or systems that still need prior robot data. Domain-Adaptive Meta-Learning, for example, explicitly framed the problem as learning from raw human video despite differences in viewpoint, environment, and embodiment, then using prior human/robot demonstration data to generalize from one human video. ([arXiv][1])

WHIRL, from CMU, attacked “in-the-wild” one-shot human-to-robot imitation by extracting intent from third-person human videos and improving the policy through real-world interaction; it demonstrated 20 real-world manipulation tasks, but still relies on robot-side exploration and optimization after observing the human. ([arXiv][2])

More recent work such as “Robot Learning from Human Videos: A Survey” frames the field around task-, observation-, and action-oriented transfer from human videos, and explicitly identifies the abundance of human video as a path to scaling robot learning beyond expensive robot demonstrations. ([arXiv][3])

## B. Human affordance learning

A major thread is not to learn full actions from human videos, but to learn **where and how humans interact with the world**. “Affordances from Human Videos as a Versatile Representation for Robotics” trains a visual affordance model from internet videos to estimate likely human interaction regions and directions, then uses those affordances across offline imitation learning, exploration, goal-conditioned learning, and RL action parameterization. ([arXiv][4])

HRP extends that idea by extracting hand-object affordance information from human video and distilling it into robot visual representations; its reported experiments involved 3,000+ robot trials and showed performance gains across multiple robot morphologies. ([arXiv][5])

This is important because affordances are closer to the right abstraction than raw human motion. A robot does not need to know that a human elbow bent 37 degrees. It needs to know: **this object can be grasped here, pushed there, rotated around this axis, or inserted through this opening.**

## C. VLM/LLM planning from human video

SeeDo is directly aligned with your idea: it uses keyframe selection, visual perception, and VLM reasoning to interpret long-horizon human demonstration videos into robot task plans. It then executes those plans through language model programs and action primitives in simulation and on a real robot arm. ([arXiv][6])

Its limitation is also instructive: the authors report that directly feeding long videos to VLMs performs poorly because current VLMs struggle with temporal order, spatial relations, and object tracking. SeeDo therefore adds keyframe selection, visual prompting, object tracking, and structured planning around the VLM. ([arXiv][6])

That is a critical lesson: **the LLM should not be the whole robot brain. It should be the symbolic/semantic reasoning layer inside a larger perception-control system.**

## D. VLA robot foundation models

The current frontier is Vision-Language-Action models. RT-2 showed that web-scale VLM knowledge can be transferred into robotic control by expressing robot actions as tokens and co-training on web vision-language tasks and robot trajectory data. ([arXiv][7])

Open X-Embodiment/RT-X showed the importance of cross-embodiment robot data at scale: the dataset contains 1M+ real robot trajectories across 22 robot embodiments. ([robotics-transformer-x.github.io][8])

OpenVLA made this more accessible: it is a 7B open-source VLA trained on 970k real-world robot demonstrations and designed for parameter-efficient fine-tuning. ([arXiv][9])

π0 introduced a flow-matching VLA architecture built on top of a pretrained VLM and trained on diverse robot platforms including single-arm robots, dual-arm robots, and mobile manipulators; its paper specifically targets general robot control and dexterous tasks. ([arXiv][10])

GR00T N1 is another relevant direction: it uses a dual-system VLA architecture, with a vision-language “System 2” for interpretation and a diffusion-transformer “System 1” for real-time motor actions, trained from a mixture of real robot trajectories, human videos, and synthetic data. ([arXiv][11])

The key trend is clear: the field is moving from “train one robot for one task” toward **pretrained robot foundation models adapted by language, video, and small amounts of embodiment-specific data**.

## E. One-shot demo-conditioned VLAs

The closest current research to your target is ViVLA: “See Once, Then Act.” It conditions a VLA on a single expert demonstration video at test time, alongside the robot’s current visual observations, and trains from nearly 893k expert-agent samples. The paper reports large gains on unseen simulated tasks, cross-embodiment videos, and real-world human-video experiments. ([arXiv][12])

This is probably the most relevant “already emerging” direction: **the demonstration video becomes an input prompt to the robot policy**, not merely data used for offline training.

## F. Generated videos and world models

RIGVid is highly disruptive: it uses generated videos as robot demonstrations. Given a command and initial scene image, a video diffusion model generates candidate videos; a VLM filters them; a 6D pose tracker extracts object trajectories; then those trajectories are retargeted to the robot. The paper reports manipulation tasks such as pouring, wiping, and mixing without physical demonstrations or robot-specific training. ([arXiv][13])

GR-2 also points in this direction: it pretrains on 38 million internet video clips and over 50 billion tokens to capture world dynamics, then fine-tunes for video generation and action prediction using robot trajectories. ([arXiv][14])

This matters because the future may not be “collect more robot demos.” It may be:

```text
human video → infer task
task → generate many possible successful rollouts
rollouts → extract object trajectories / affordances
robot → execute via embodiment-specific controller
```

## G. LLMs as planners, not motor controllers

SayCan showed a useful decomposition: use a language model for high-level task usefulness, but ground choices with value functions that estimate whether the robot can actually execute a skill in the current state. ([arXiv][15])

Code as Policies showed that LLMs can generate robot policy code that calls perception and control APIs, including waypoint-based control and feedback loops. ([arXiv][16])

VoxPoser goes further by using LLMs and VLMs to compose 3D value maps for manipulation, then using motion planning to synthesize trajectories. ([arXiv][17])

The lesson: the LLM should generate **plans, constraints, code, task graphs, value maps, and verification logic**. It should not directly output raw torque or joint commands.

# 3. The bottleneck everyone is still hitting

The unsolved problem is **not** “can a model understand a video?” The unsolved problem is:

> How do we transform a human video into a robot-executable policy despite embodiment mismatch, hidden contact forces, partial observability, and real-world physics?

The core blockers:

1. **Embodiment gap**
   Human hands have five fingers, soft skin, tactile sensing, and complex compliance. A DK1 gripper has very different geometry, force limits, and failure modes.

2. **Action ambiguity**
   In a video, you often see the result but not the force, friction, grip pressure, torque, or micro-adjustments.

3. **Causality gap**
   Current VLMs often describe visible events but do not reliably infer which contact caused which state change.

4. **Precision gap**
   Language-level plans like “put the cube in the tray” are too coarse. Manipulation needs millimeter-scale pose, approach vector, gripper width, velocity, and contact timing.

5. **Physics gap**
   Video models can generate plausible-looking motion that may violate contact mechanics, friction, mass, or robot reachability.

6. **Verification gap**
   A robot must know when it is wrong, recover, and replan. Most imitation systems are brittle because they lack a robust self-checking loop.

This is where disruption is possible.

# 4. The disruptive architecture: Human Demonstration Compiler

Build a system called, conceptually:

```text
Human Demonstration Compiler
```

Its job:

```text
human video
  → 4D scene reconstruction
  → human/object/contact parsing
  → causal task graph
  → embodiment-agnostic skill representation
  → robot-specific executable policy
  → simulated verification
  → real-world execution
  → failure-driven self-correction
```

## Layer 1: Human-centric perception

Use Sapiens2 as a human understanding module, not as the robot policy. Sapiens2 is a family of high-resolution human-centric vision transformers, pretrained on 1B high-quality human images, with model sizes from roughly 0.4B to 5B parameters and post-training heads for pose, body-part segmentation, pointmaps, normals, and albedo. ([arXiv][18])

For your use case, Sapiens2 gives:

```text
human pose
hand position
body/hand segmentation
human foreground matting
human pointmap / surface geometry
temporal hand-object interaction cues
```

But Sapiens2 is human-centric. You still need object perception, object tracking, 6D pose estimation, depth, and contact inference.

The right perception stack:

```text
video frames
  → human pose / hand segmentation / hand trajectory
  → object segmentation + tracking
  → object 6D pose / 3D bounding volume
  → hand-object distance/contact likelihood
  → object state changes
  → event boundaries
```

The output is not a caption. It is a structured event stream:

```json
[
  {"t": 1.2, "event": "approach", "actor": "right_hand", "object": "red_cube"},
  {"t": 2.0, "event": "contact", "actor": "right_hand", "object": "red_cube"},
  {"t": 2.3, "event": "grasp", "object": "red_cube"},
  {"t": 3.1, "event": "lift", "object": "red_cube", "height_delta": 0.08},
  {"t": 4.6, "event": "place", "object": "red_cube", "target": "black_tray"},
  {"t": 4.9, "event": "release", "object": "red_cube"}
]
```

## Layer 2: Object-centric task graph

The next representation should ignore human body motion unless it matters. Use an object-centric graph:

```text
G = (O, S, E, C)

O = objects
S = object states and relations
E = events/actions
C = constraints and causal dependencies
```

Example:

```yaml
task: insert_battery_into_slot

objects:
  battery:
    affordances:
      grasp_regions: [long_sides]
      forbidden_regions: [terminal_end]
    constraints:
      keep_orientation: true

  slot:
    affordances:
      insertion_axis: +x
      tolerance_mm: 2

events:
  - locate battery
  - grasp battery on long sides
  - align battery with slot axis
  - insert along slot axis
  - release after seated

success:
  battery.pose inside slot tolerance
  battery motion stopped
```

This is the abstraction that transfers across humans and robots.

## Layer 3: Embodiment lifting

This is the hard part. You need to map:

```text
human contact and object motion
  → robot grasp/action/trajectory candidates
```

Do not retarget human hand motion directly. Instead infer:

```text
object trajectory
contact mode
force direction
grasp affordance
task constraints
```

Then solve:

```text
Find robot action sequence a_R such that:
  object_state_trajectory_R ≈ object_state_trajectory_H
  contact constraints are satisfied
  robot kinematics are feasible
  collision constraints are satisfied
  gripper can maintain stable grasp
```

This becomes optimization:

```text
a_R* = argmin_a [
    D_object(rollout(a), human_object_trace)
  + λ1 collision_penalty
  + λ2 reachability_penalty
  + λ3 grasp_stability_penalty
  + λ4 task_constraint_penalty
]
```

This is where a combination of VLA policy, motion planner, and simulator beats pure imitation learning.

## Layer 4: LLM as task-program synthesizer

Use an LLM to synthesize an executable robot program from the task graph.

Not:

```text
LLM → raw robot actions
```

Instead:

```text
LLM → task program / constraints / calls to skills
```

Example output:

```python
def execute_task(scene, robot):
    cube = scene.object("red_cube")
    tray = scene.object("black_tray")

    grasp_pose = choose_grasp(
        object=cube,
        affordance="side_grasp",
        avoid_occlusion=True
    )

    robot.move_to(pregrasp(grasp_pose))
    robot.close_gripper_until_contact(max_force=F_SAFE)
    robot.lift(height=0.08)

    target_pose = pose_above(tray.center, z=0.10)
    robot.move_to(target_pose)
    robot.lower_until(object_inside=tray)
    robot.open_gripper()
    robot.retreat()
```

This can be generated by an LLM, but each function must be grounded by perception, motion planning, and robot-specific controllers. This is consistent with the Code as Policies and VoxPoser direction, where LLMs generate structured robot programs or value maps rather than directly controlling motors. ([arXiv][16])

## Layer 5: Demo-conditioned VLA executor

For flexible skills, add a VLA policy conditioned on the demonstration video:

```text
π_R(a_t | robot_images, robot_state, language, task_graph, human_demo_video)
```

This is the ViVLA-style direction. The policy sees:

```text
the human demonstration
the robot's current view
the task graph
the language instruction
the robot state
```

It outputs:

```text
short action chunks
waypoints
gripper commands
or continuous end-effector deltas
```

This is where “one-shot” becomes real: the model is trained over many prior expert-agent pairs, so at test time the new human video is just a prompt. ViVLA explicitly trains this kind of one-shot demo-conditioned VLA using large expert-agent paired data. ([arXiv][12])

## Layer 6: Physics/video/world-model verification

Before the robot moves, simulate or predict the outcome:

```text
candidate robot program
  → simulated rollout
  → video/world model predicted rollout
  → compare against human demonstration outcome
  → accept / repair / reject
```

Use three critics:

```text
semantic critic:
  Did the task happen?

geometric critic:
  Did the object reach the right pose/state?

physics critic:
  Was the contact plausible and robot-safe?
```

This is where generated video and digital twins become powerful. RIGVid suggests that generated videos can provide usable manipulation supervision when filtered and tracked properly; RialTo shows the value of constructing digital-twin simulations from small amounts of real-world data to robustify policies via simulation before deployment. ([arXiv][13])

# 5. Where Unreal Engine fits

Unreal can help, but not as the sole robotics simulator.

Use Unreal for:

```text
photorealistic human demonstration generation
synthetic camera views
lighting/domain randomization
scene diversity
human animation variation
object appearance variation
training data for perception and task parsing
```

Do not make Unreal your primary manipulation physics engine unless you are prepared to invest heavily in robotics-grade contact modeling, robot kinematics, control latency, gripper physics, and sim-to-real validation.

A stronger stack is:

```text
Unreal:
  photorealistic human videos + visual domain randomization

Isaac Lab / MuJoCo / ManiSkill:
  robot physics, control, manipulation, RL, sim-to-real

LeRobot:
  real robot datasets, policies, recording, training, evaluation

Sapiens2 / VLMs:
  human and scene understanding

LLM:
  task-graph construction, code generation, replanning, verification
```

NVIDIA Isaac Lab is designed as a GPU-accelerated, open-source robot learning framework for training policies at scale with physics, rendering, imitation learning, reinforcement learning, and motion planning workflows. ([NVIDIA Developer][19])

MuJoCo Playground/MJX is also relevant because it targets fast robot learning and sim-to-real with GPU/JAX-based simulation. ([arXiv][20])

The disruptive Unreal strategy is therefore:

```text
Unreal creates synthetic humans doing tasks.
Physics sim creates robot attempts at the same tasks.
The compiler aligns both into object-centric causal traces.
The VLA learns expert-video → robot-action transfer.
```

# 6. The actual research contribution

The publishable/disruptive contribution should not be “we used an LLM with a robot.”

It should be:

> **A unified object-centric compiler that converts one human demonstration video into a robot-executable causal skill program, using foundation models for perception, LLMs for program synthesis, world models for counterfactual rollout, and a VLA/motion-planning backend for execution.**

I would name the core representation something like:

```text
Embodiment-Invariant Causal Skill Graph
```

Its fields:

```yaml
objects:
  id
  category
  geometry
  pose
  affordances
  articulation
  physical_properties_estimate

states:
  symbolic_state
  metric_pose
  relation_graph
  containment
  support
  open_closed
  attached_detached
  clean_dirty

contacts:
  actor
  object
  contact_region
  contact_normal
  force_direction_estimate
  duration
  effect

events:
  preconditions
  action_type
  trajectory_constraint
  postconditions
  failure_modes

robot_binding:
  candidate_grasps
  candidate_waypoints
  required_skills
  fallback_skills
  verification_checks
```

This makes the system inspectable, debuggable, and compositional.

# 7. How to make it one-shot

“One-shot” only works if the system already has massive priors.

The priors should be:

```text
human prior:
  hands, pose, gaze, contact, intent

object prior:
  affordances, articulation, geometry, material

physics prior:
  support, containment, friction, gravity, force closure

robot prior:
  reachability, grasping, pushing, placing, insertion, recovery

task prior:
  common subgoals, temporal structure, preconditions/effects

language prior:
  semantic decomposition, procedural knowledge, commonsense
```

Then the single video only specifies:

```text
which task
which objects
which order
which constraints
which success condition
```

This is analogous to how humans learn. When a person watches someone open a bottle once, they are not learning physics, hands, objects, or motor control from scratch. They are binding a new task onto a huge existing prior.

# 8. Training strategy

## Stage 1: Pretrain the human-video parser

Input:

```text
internet human videos
synthetic Unreal human videos
egocentric videos
multi-view lab videos
```

Targets:

```text
hands
objects
contacts
object state changes
event boundaries
task graph
```

Supervision can come from:

```text
synthetic labels from Unreal
automatic hand/object/contact heuristics
VLM-generated weak labels
human correction for a small subset
```

## Stage 2: Build expert-agent pairs

This is the most important data asset.

An expert-agent pair is:

```text
expert demonstration:
  human video or generated video

agent execution:
  robot trajectory that accomplishes same task

shared representation:
  causal skill graph
```

The training sample:

```text
(V_human, O_robot_t, q_robot_t, G_task) → a_robot_t:t+k
```

This is exactly the direction that ViVLA points toward: train on many expert-agent pairs so that a single test-time video can condition the policy. ([arXiv][12])

## Stage 3: Synthetic pair factory

Use Unreal + simulation to scale:

```text
1. Generate human performing task in Unreal.
2. Export perfect ground-truth object states, contacts, and event labels.
3. Convert human trace to causal task graph.
4. Generate robot solution in Isaac/MuJoCo using planner/RL.
5. Store pair:
     human video ↔ robot execution ↔ task graph
6. Randomize:
     camera, lighting, object shape, object texture, clutter, start states.
```

This is the bridge between “videos are abundant” and “robots need actions.”

## Stage 4: Train the robot executor

Train several executors:

```text
low-level motor skill model:
  grasp, place, push, pull, insert, wipe, fold, pour

demo-conditioned VLA:
  human video + robot observation → action chunk

planner-conditioned policy:
  task graph + robot observation → action chunk

recovery policy:
  failure state + intended subgoal → corrective action
```

Use ACT/Diffusion/VLA variants depending on the task. ACT showed strong real-world bimanual manipulation from small demonstrations, while Diffusion Policy showed strong visuomotor policy learning with diffusion-based action generation. ([arXiv][21])

## Stage 5: Closed-loop verification and repair

Execution should look like this:

```text
observe
infer current task state
choose next subgoal
propose action
simulate/predict outcome
execute small chunk
re-observe
compare to expected state
repair if mismatch
```

This is what most “imitation” systems lack. A revolutionary system must not just imitate. It must **notice divergence and recover**.

# 9. What you should do with only 12 hours of robot access

Use the robot time to learn the **embodiment adapter**, not to learn every task.

You need data that answers:

```text
What can this robot physically do?
How does its gripper interact with objects?
How accurate is its camera-to-action mapping?
What are its failure modes?
How does it recover?
```

Collect robot data in categories, not tasks.

## Category A: calibration data

```text
camera intrinsics/extrinsics
workspace frame
robot base frame
gripper frame
table plane
reachability map
safe joint/action limits
latency
camera synchronization
```

## Category B: primitive skill data

Collect many short clips:

```text
reach to point
approach object
side grasp
top grasp
lift
place
push
pull
slide
rotate
open gripper near object
close gripper until contact
recover from missed grasp
recover from object slip
```

This teaches the executor what the DK1 embodiment can do.

## Category C: object-affordance probes

For each object type:

```text
cube
cup
bottle
cloth
drawer handle
tool handle
container
```

Record:

```text
successful grasps
failed grasps
push directions
stable placements
slips
collisions
occlusions
```

## Category D: paired human-robot demonstrations

For a small number of tasks, record:

```text
human video of task
robot teleop execution of same task
same camera view if possible
same object start/end state
```

These are precious because they directly supervise the human-to-robot bridge.

## Category E: failure/recovery

Most people under-collect this. You need:

```text
missed grasp → retry
object moved unexpectedly → re-localize
object dropped → regrasp
blocked path → replan
wrong orientation → reorient
```

A general robot is mostly a recovery machine.

# 10. What to build before robot access

Before touching the DK1, have these modules running:

```text
1. Human video ingestion
2. Sapiens2 feature extraction
3. Object segmentation/tracking
4. Event boundary detection
5. Task graph generation
6. LLM task-program synthesis
7. Simulation execution
8. Policy training pipeline
9. Video-based verification
10. LeRobot dataset export/import
```

For the DK1 specifically, the public repo uses LeRobot plugin conventions and is detected by LeRobot when installed in the same Python environment; LeRobot’s workflow is already built around teleoperation, recording trajectories, training policies, and deploying them. ([GitHub][22])

The robot session should not be exploratory chaos. It should be a planned data-acquisition campaign.

# 11. The research roadmap

## Work Package 1: Human Demonstration Compiler

Goal:

```text
V_H → causal skill graph G
```

Deliverables:

```text
video parser
object/contact/event detector
task graph schema
uncertainty estimates
LLM-based graph repair
```

Metric:

```text
Does the graph correctly predict:
  objects
  order of events
  contact events
  final state
  constraints
```

## Work Package 2: Embodiment-invariant representation

Goal:

```text
human action → object-centric action → robot action
```

Core idea:

```text
Do not align human joints to robot joints.
Align object state transitions and contact modes.
```

Metric:

```text
Can the same graph execute on:
  simulated robot A
  simulated robot B
  DK1
  another arm
```

## Work Package 3: Demo-conditioned VLA

Goal:

```text
π(a | robot_obs, robot_state, human_demo, task_graph)
```

Training data:

```text
synthetic expert-agent pairs
public robot datasets
your DK1 calibration/primitive data
generated videos
human videos
```

Metric:

```text
single unseen demo → successful robot execution
without task-specific robot teleop
```

## Work Package 4: World-model verification

Goal:

```text
predict whether candidate action will reproduce demo outcome
```

Use:

```text
video prediction
physics simulation
VLM critic
geometric state comparison
```

Metric:

```text
Can the system reject bad plans before robot execution?
Can it repair after failure?
```

## Work Package 5: Real-to-sim-to-real adaptation

Goal:

```text
build a digital twin from the real robot scene
train/verify variations in sim
deploy back to robot
```

This follows the real-to-sim-to-real direction seen in RialTo, which uses digital-twin simulations from small real-world data to improve robustness without extensive real-world collection. ([arXiv][23])

# 12. What would genuinely disrupt the field

The strongest disruptive bet is:

> **Create the first open “human video → robot skill” compiler with an inspectable causal representation and a standard dataset format.**

Not just another model. A full data and execution interface.

The open standard could be:

```text
HumanDemoDataset
  raw videos
  camera calibration
  Sapiens2 features
  object tracks
  contact events
  task graph
  language description
  generated robot programs
  robot execution traces
  success/failure labels
```

Then anyone can add:

```text
new human videos
new robot embodiments
new simulators
new VLA policies
new task graphs
```

This becomes a data flywheel:

```text
More humans demonstrate tasks
  → compiler extracts graphs
  → simulator generates robot attempts
  → robot executes
  → failures become new training data
  → model improves
```

That is how you disrupt.

# 13. The clean thesis statement

Use this as the research thesis:

> **We propose a one-shot human-to-robot learning system that treats human demonstration videos as executable task specifications. The system converts a single video into an embodiment-invariant causal skill graph using human-centric vision, object tracking, contact inference, and VLM reasoning. A robot-specific executor then grounds this graph into feasible actions through a combination of VLA policies, motion planning, affordance fields, and simulation-based verification. The robot does not imitate the human body; it reproduces the demonstrated causal transformation of the world.**

That is the right north star.

[1]: https://arxiv.org/abs/1802.01557?utm_source=chatgpt.com "One-Shot Imitation from Observing Humans via Domain-Adaptive Meta-Learning"
[2]: https://arxiv.org/abs/2207.09450?utm_source=chatgpt.com "Human-to-Robot Imitation in the Wild"
[3]: https://arxiv.org/abs/2604.27621 "[2604.27621] Robot Learning from Human Videos: A Survey"
[4]: https://arxiv.org/abs/2304.08488 "[2304.08488] Affordances from Human Videos as a Versatile Representation for Robotics"
[5]: https://arxiv.org/abs/2407.18911?utm_source=chatgpt.com "HRP: Human Affordances for Robotic Pre-Training"
[6]: https://arxiv.org/html/2410.08792v2 "VLM See, Robot Do: Human Demo Video to Robot Action Plan via Vision Language Model"
[7]: https://arxiv.org/abs/2307.15818?utm_source=chatgpt.com "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"
[8]: https://robotics-transformer-x.github.io/?utm_source=chatgpt.com "Open X-Embodiment: Robotic Learning Datasets and RT-X ..."
[9]: https://arxiv.org/abs/2406.09246?utm_source=chatgpt.com "OpenVLA: An Open-Source Vision-Language-Action Model"
[10]: https://arxiv.org/abs/2410.24164 "[2410.24164] $π_0$: A Vision-Language-Action Flow Model for General Robot Control"
[11]: https://arxiv.org/abs/2503.14734?utm_source=chatgpt.com "GR00T N1: An Open Foundation Model for Generalist Humanoid Robots"
[12]: https://arxiv.org/abs/2512.07582?utm_source=chatgpt.com "See Once, Then Act: Vision-Language-Action Model with Task Learning from One-Shot Video Demonstrations"
[13]: https://arxiv.org/abs/2507.00990?utm_source=chatgpt.com "Robotic Manipulation by Imitating Generated Videos Without Physical Demonstrations"
[14]: https://arxiv.org/abs/2410.06158?utm_source=chatgpt.com "GR-2: A Generative Video-Language-Action Model with Web-Scale Knowledge for Robot Manipulation"
[15]: https://arxiv.org/abs/2204.01691?utm_source=chatgpt.com "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances"
[16]: https://arxiv.org/abs/2209.07753?utm_source=chatgpt.com "Code as Policies: Language Model Programs for Embodied Control"
[17]: https://arxiv.org/abs/2307.05973?utm_source=chatgpt.com "VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models"
[18]: https://arxiv.org/html/2604.21681v1 "Sapiens2"
[19]: https://developer.nvidia.com/isaac/lab?utm_source=chatgpt.com "NVIDIA Isaac Lab"
[20]: https://arxiv.org/html/2502.08844v1?utm_source=chatgpt.com "MuJoCo Playground"
[21]: https://arxiv.org/abs/2304.13705?utm_source=chatgpt.com "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware"
[22]: https://github.com/robot-learning-co/trlc-dk1?utm_source=chatgpt.com "robot-learning-co/trlc-dk1: TRLC's Developer Kit 1"
[23]: https://arxiv.org/abs/2403.03949?utm_source=chatgpt.com "Reconciling Reality through Simulation: A Real-to-Sim-to-Real Approach for Robust Manipulation"
