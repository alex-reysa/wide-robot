"""Physical-validity checks on synthetic SimTraces (mujoco-free).

These pin the six checks of ``csg/validity.md`` against hand-built traces so the
verdict logic is tested without a simulator. The end-to-end physics run is in
``test_mujoco_backend.py`` (gated by importorskip).
"""
import copy

from csg.backends.mujoco.trace import SimTrace, SimStep, ContactRecord
from csg.backends.mujoco import validity as V

CUBE = (0.04, 0.04, 0.04)
TRAY = (0.24, 0.18, 0.03)
TRAY_C = (0.45, 0.0, 0.015)
CUBE_IN = (0.45, 0.0, 0.028)  # resting on the 8mm cavity floor, center below rim
Q0 = (1.0, 0.0, 0.0, 0.0)


def _step(t, cube_xyz, *, closed=False, aperture=0.08, contacts=None,
          fingers=(), joints=None):
    return SimStep(
        time_s=t,
        effector_xyz=(cube_xyz[0], cube_xyz[1], cube_xyz[2] + 0.10),
        effector_quat=Q0,
        gripper_aperture_m=aperture,
        gripper_closed_cmd=closed,
        object_poses={"cube": (cube_xyz, Q0), "tray": (TRAY_C, Q0)},
        joint_values=joints or {"j0": 0.0},
        joint_limits={"j0": (-2.0, 2.0)},
        contacts=contacts or [],
        finger_contacts=fingers,
        gripper_force=5.0,
    )


def _good_trace():
    """Smooth approach -> grasp -> transport -> release into the tray, cube
    rests supported INSIDE through the settle window."""
    floor_contact = [ContactRecord("cube", "tray", 0.0, 1.0)]
    grip = [ContactRecord("left_finger", "cube", 0.0, 0.0),
            ContactRecord("right_finger", "cube", 0.0, 0.0)]
    steps = []
    t = 0.0
    # approach (gripper open, no contact)
    for x in (0.27, 0.27):
        steps.append(_step(t, (x, 0.0, 0.02))); t += 0.1
    # grasp + transport (gripper closed around cube, bilateral finger contact).
    # Steps stay well under the 50mm/frame continuity limit.
    grasp_start = len(steps)
    path = [(0.27, 0.0, 0.02), (0.28, 0.0, 0.06), (0.30, 0.0, 0.10),
            (0.34, 0.0, 0.12), (0.38, 0.0, 0.12), (0.42, 0.0, 0.11),
            (0.45, 0.0, 0.08), (0.45, 0.0, 0.05), CUBE_IN]
    for x in path:
        steps.append(_step(t, x, closed=True, aperture=0.04, contacts=grip, fingers=("left_finger", "right_finger"))); t += 0.1
    grasp_end = len(steps) - 1
    release = len(steps)
    # release + settle: cube resting in tray, supported, INSIDE persists
    for _ in range(12):
        steps.append(_step(t, CUBE_IN, closed=False, aperture=0.08, contacts=floor_contact)); t += 0.1
    return SimTrace(
        steps=steps, frame_dt_s=0.1, release_indices=[release],
        ik_failures=[], grasped_object="cube", grasp_interval=(grasp_start, grasp_end),
        object_min_width_m=0.04, object_max_width_m=0.04,
        figure_id="cube", ground_id="tray",
        body_sizes={"cube": CUBE, "tray": TRAY}, static_bodies=("tray", "table"),
        gripper_force_limit_n=20.0, articulation_limits={},
    )


# ---- happy path -------------------------------------------------------------


def test_good_trace_passes():
    rep = V.check_validity(_good_trace())
    assert rep.passed, rep.reason
    assert rep.checks["articulation_limits"]["applicable"] is False
    assert rep.checks["gripper_feasibility"]["applicable"] is True
    assert rep.checks["quasi_static_support_at_release"]["passed"]


# ---- check 1: non-penetration ----------------------------------------------


def test_penetration_4mm_passes():
    tr = _good_trace()
    tr.steps[5].contacts.append(ContactRecord("cube", "tray", 0.004, 0.0))
    assert V.check_non_penetration(tr, V.DEFAULT)["passed"]


def test_penetration_8mm_fails():
    tr = _good_trace()
    tr.steps[5].contacts.append(ContactRecord("cube", "tray", 0.008, 0.0))
    res = V.check_non_penetration(tr, V.DEFAULT)
    assert not res["passed"]
    rep = V.check_validity(tr)
    assert not rep.passed and rep.reason.startswith("non_penetration")


# ---- check 2: pose continuity ----------------------------------------------


def test_continuity_smooth_passes():
    assert V.check_pose_continuity(_good_trace(), V.DEFAULT)["passed"]


def test_continuity_teleport_fails():
    tr = _good_trace()
    bad = tr.steps[8]
    tr.steps[8].object_poses["cube"] = ((bad.object_poses["cube"][0][0] + 0.20,) +
                                        bad.object_poses["cube"][0][1:], Q0)
    res = V.check_pose_continuity(tr, V.DEFAULT)
    assert not res["passed"]
    rep = V.check_validity(tr)
    assert not rep.passed and rep.reason.startswith("pose_continuity")


# ---- check 3: quasi-static support at release ------------------------------


def test_release_drop_fails():
    tr = _good_trace()
    # cube falls out after release
    for i in range(tr.release_indices[0] + 1, len(tr.steps)):
        tr.steps[i].object_poses["cube"] = ((0.45, 0.0, 0.028 - 0.05 * (i - tr.release_indices[0])), Q0)
        tr.steps[i].contacts = []  # nothing supports it
    res = V.check_quasi_static_support(tr, V.DEFAULT)
    assert not res["passed"]


def test_release_lost_inside_fails():
    tr = _good_trace()
    # INSIDE at release, but during settle the cube ends up ON_TOP_OF the rim.
    rim = (0.45, 0.0, 0.045)  # bottom at 0.025 ~ tray top 0.03 -> ON_TOP_OF, not INSIDE
    for i in range(tr.release_indices[0] + 1, len(tr.steps)):
        tr.steps[i].object_poses["cube"] = (rim, Q0)
        tr.steps[i].contacts = [ContactRecord("cube", "tray", 0.0, 1.0)]
    res = V.check_quasi_static_support(tr, V.DEFAULT)
    assert not res["passed"]
    assert "lost" in res["detail"].lower() or "inside" in res["detail"].lower()


def test_release_unsupported_fails():
    tr = _good_trace()
    for i in range(tr.release_indices[0], len(tr.steps)):
        tr.steps[i].contacts = []  # floating, no support contact
    assert not V.check_quasi_static_support(tr, V.DEFAULT)["passed"]


# ---- check 4: gripper feasibility ------------------------------------------


def test_gripper_aperture_too_small_fails():
    tr = _good_trace()
    for i in range(*tr.grasp_interval):
        tr.steps[i].gripper_aperture_m = 0.02  # narrower than the 40mm cube
    res = V.check_gripper_feasibility(tr, V.DEFAULT)
    assert not res["passed"] and "aperture" in res["detail"]


def test_gripper_no_bilateral_contact_fails():
    tr = _good_trace()
    for i in range(tr.grasp_interval[0], tr.grasp_interval[1] + 1):
        tr.steps[i].finger_contacts = ("left_finger",)  # only one finger
    res = V.check_gripper_feasibility(tr, V.DEFAULT)
    assert not res["passed"] and "bilateral" in res["detail"]


def test_gripper_force_over_limit_fails():
    tr = _good_trace()
    for i in range(tr.grasp_interval[0], tr.grasp_interval[1] + 1):
        tr.steps[i].gripper_force = 50.0  # over the 20N limit
    assert not V.check_gripper_feasibility(tr, V.DEFAULT)["passed"]


# ---- check 5: workspace reachability ---------------------------------------


def test_ik_failure_fails_reachability():
    tr = _good_trace()
    tr.ik_failures = ["transport@t=0.4"]
    res = V.check_workspace_reachability(tr, V.DEFAULT)
    assert not res["passed"] and "IK" in res["detail"]


def test_joint_out_of_limit_fails_reachability():
    tr = _good_trace()
    tr.steps[4].joint_values["j0"] = 5.0  # outside [-2, 2]
    assert not V.check_workspace_reachability(tr, V.DEFAULT)["passed"]


# ---- check 6: articulation limits ------------------------------------------


def test_articulation_non_applicable_for_pick_place():
    res = V.check_articulation_limits(_good_trace(), V.DEFAULT)
    assert res["applicable"] is False
    assert res["passed"] is True


def test_articulation_over_travel_fails():
    tr = _good_trace()
    tr.articulation_limits = {"drawer": (0.0, 0.20)}
    for s in tr.steps:
        s.articulation = {"drawer": 0.0}
    tr.steps[6].articulation = {"drawer": 0.35}  # over-travel
    res = V.check_articulation_limits(tr, V.DEFAULT)
    assert not res["passed"]


# ---- verdict aggregation ----------------------------------------------------


def test_verdict_ands_only_applicable_checks():
    rep = V.check_validity(_good_trace())
    # articulation is non-applicable and must not drag the verdict either way
    assert rep.passed
    assert rep.checks["articulation_limits"]["applicable"] is False


def test_independent_traces_are_not_mutated():
    base = _good_trace()
    snap = copy.deepcopy(base)
    V.check_validity(base)
    assert [s.time_s for s in base.steps] == [s.time_s for s in snap.steps]
