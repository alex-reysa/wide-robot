"""Real-camera object_inside_container — the matcher seam (Phase 3A).

Mirrors tests/test_rlbench_articulation_event.py: synthetic ``real_camera.tracks.v0``
episodes are converted by the real pilot bridge (:func:`tracks_to_rollout`) into
``csg.rollout.v0`` and judged by the FROZEN verifier against the two camera targets.

The two targets form a strictly-stronger pair (the camera analogue of the RLBench
value-only → articulation-event progression):
  * terminal_only  — only the cube's TERMINAL relation to the tray is INSIDE
    (hard OBJECT_RELATION_GOAL → goal_satisfaction);
  * relation_event — additionally the cube STARTED NEAR (initial_state), ENDED INSIDE
    (terminal_state + relation_transitions), and a CONTAINMENT_CHANGE event is present
    (event_presence). event_order stays support 0 (one event, no pair).

The load-bearing subtlety this suite pins (verified against the frozen extractor):
``csg/rollout_extract.py`` seeds ``prev_rel="NEAR"`` UNCONDITIONALLY, so a "born-inside"
cube (inside the whole time, but moving) STILL emits a NEAR→INSIDE CONTAINMENT_CHANGE
delta. So event_presence/relation_transitions do NOT reject born-inside — only
``initial_state`` (the robot's first relation is INSIDE, the target demands NEAR) does.
A born-inside episode therefore PASSes terminal_only but FAILs relation_event.

Runs with NO opencv/numpy installed; ``csg/`` is never touched.
"""
import copy
from pathlib import Path

import pytest

import csg.predicates as P
from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg

from pilots.external_rollout import ExternalTraceLeakage, assert_rollout_leakage_clean
from pilots.external_verify import external_confusion_report, load_gold_targets, verify_external_rollout
from pilots.real_camera.tracks_to_rollout import tracks_to_rollout

_REPO = Path(__file__).resolve().parents[1]
_GOLD_DIR = _REPO / "gold_tests"
_TARGETS_DIR = _REPO / "pilots" / "real_camera" / "targets"
_TERMINAL_ONLY_TARGET = _TARGETS_DIR / "object_inside_container_terminal_only.json"
_RELATION_EVENT_TARGET = _TARGETS_DIR / "object_inside_container_relation_event.json"

# Tray resting on the table: center (TX,TY,TZ), sizeM (0.24,0.18,0.03) -> half
# (0.12,0.09,0.015) so tray.bottom=0.0, tray.top=0.03. Cube is 0.04^3 (half 0.02).
TX, TY, TZ = 0.30, 0.0, 0.015
_TRAY_SIZE = [0.24, 0.18, 0.03]
_CUBE_SIZE = [0.04, 0.04, 0.04]

# Terminal cube centers per scenario (verified by the geometry tripwire below):
_INSIDE = (TX, TY, 0.03)            # within shrunk footprint, center z<=top+slack, bottom>=floor
_NEAR_NOT_INSIDE = (TX + 0.13, TY, 0.05)   # just outside footprint, box_gap<=near_gap
_ON_RIM = (TX, TY, 0.05)            # cube.bottom==tray.top -> ON_TOP_OF, not INSIDE
_FAR = (TX + 0.35, TY, 0.02)        # box_gap>near_gap -> FAR_FROM
_START_NEAR = (TX + 0.16, TY, 0.05)  # outside footprint, NEAR


def _tracks(cube_seq, tray_xyz=(TX, TY, TZ), fps=30.0, cube_conf=0.95, tray_conf=0.99):
    objs = [
        {"sourceRole": "cube", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": False, "sizeM": list(_CUBE_SIZE), "markerIds": [7]},
        {"sourceRole": "tray", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": True, "sizeM": list(_TRAY_SIZE), "markerIds": [10, 11, 12, 13]},
    ]
    frames = []
    for i, (cx, cy, cz) in enumerate(cube_seq):
        frames.append({"frameIndex": i, "timeS": i * (1.0 / fps), "poses": {
            "cube": {"positionM": {"x": cx, "y": cy, "z": cz}, "confidence": cube_conf},
            "tray": {"positionM": {"x": tray_xyz[0], "y": tray_xyz[1], "z": tray_xyz[2]}, "confidence": tray_conf},
        }})
    return {"schemaVersion": "real_camera.tracks.v0", "episodeId": "ep_test",
            "videoSha256": None, "calibrationHash": None, "fps": fps,
            "objects": objs, "frames": frames}


def _approach_then(end_xyz):
    """A cube that starts NEAR (outside the tray) and moves to ``end_xyz`` over >= 6 frames
    (so the terminal relation persists; cube displacement >> MOTION_EPS_M)."""
    sx, sy, sz = _START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _rollout(cube_seq, **kw):
    r = tracks_to_rollout(_tracks(cube_seq, **kw))
    assert_rollout_leakage_clean(r)
    return r


def _term(target_path, rollout):
    return verify_external_rollout(load_json(target_path), rollout, case_name="real_camera_oic")


# ---------------------------------------------------------------------------
# Geometry tripwire — pin the exact fixture coordinates against the frozen predicates
# ---------------------------------------------------------------------------


def _box(center, size):
    return P.box_from(center, tuple(size))


def test_fixture_geometry_classifies_as_intended():
    tray = _box((TX, TY, TZ), _TRAY_SIZE)
    assert P.is_inside(_box(_INSIDE, _CUBE_SIZE), tray) is True
    assert P.primary_topo_relation(_box(_INSIDE, _CUBE_SIZE), tray) == "INSIDE"
    # start NEAR but not inside
    assert P.is_inside(_box(_START_NEAR, _CUBE_SIZE), tray) is False
    assert P.is_near(_box(_START_NEAR, _CUBE_SIZE), tray) is True
    # rim placement -> ON_TOP_OF, not INSIDE
    assert P.is_inside(_box(_ON_RIM, _CUBE_SIZE), tray) is False
    assert P.is_on_top_of(_box(_ON_RIM, _CUBE_SIZE), tray) is True
    # near-not-inside -> NEAR and not inside
    assert P.is_inside(_box(_NEAR_NOT_INSIDE, _CUBE_SIZE), tray) is False
    assert P.is_near(_box(_NEAR_NOT_INSIDE, _CUBE_SIZE), tray) is True
    # dropped outside -> FAR_FROM
    assert P.is_near(_box(_FAR, _CUBE_SIZE), tray) is False
    assert P.primary_topo_relation(_box(_FAR, _CUBE_SIZE), tray) is None


# ---------------------------------------------------------------------------
# Targets structure / not-a-gold-task
# ---------------------------------------------------------------------------


def test_targets_structure_and_deferrals():
    term = load_json(_TERMINAL_ONLY_TARGET)
    rele = load_json(_RELATION_EVENT_TARGET)
    for t in (term, rele):
        goals = t["plannerView"]["stages"][0]["goalConstraints"]
        assert [g["kind"] for g in goals] == ["OBJECT_RELATION_GOAL"]
        assert goals[0]["hard"] is True
        assert goals[0]["relation"]["desiredRelation"] == "INSIDE"
        # marker-only: no honest hand/effector, and no contact/order machinery
        assert t["agentParts"] == []
        assert "contacts" not in t
        assert "temporalEdges" not in t
    # terminal-only asserts ONLY the goal (no initial state / transition / event)
    assert "relations" not in term and "events" not in term
    # relation-event adds the initial NEAR + terminal INSIDE relations and ONE event
    assert [r["relation"] for r in rele["relations"]] == ["NEAR", "INSIDE"]
    assert [e["eventKind"] for e in rele["events"]] == ["CONTAINMENT_CHANGE"]
    trans = rele["events"][0]["observedDeltas"][0]["relationTransition"]
    assert (trans["fromRelation"], trans["toRelation"]) == ("NEAR", "INSIDE")


def test_targets_are_not_gold_tasks():
    assert not (_GOLD_DIR / "object_inside_container_terminal_only").exists()
    assert not (_GOLD_DIR / "object_inside_container_relation_event").exists()
    assert load_json(_TERMINAL_ONLY_TARGET)["pilotMetadata"]["diagnostic"] == "object-inside-container-terminal-only"
    assert load_json(_RELATION_EVENT_TARGET)["pilotMetadata"]["diagnostic"] == "object-inside-container-relation-event"


# ---------------------------------------------------------------------------
# Positive — a real put-in PASSes both targets, non-vacuously, leakage-clean
# ---------------------------------------------------------------------------


def test_success_passes_both_targets_non_vacuously():
    rollout = _rollout(_approach_then(_INSIDE))
    for path in (_TERMINAL_ONLY_TARGET, _RELATION_EVENT_TARGET):
        case = _term(path, rollout)
        assert case["passed"] is True, (path.name, case["hardMismatches"])
        assert case["leakageClean"] is True
        assert case["physicalValidity"] is None
        assert case["hardMismatches"] == []

    res = match(load_json(_RELATION_EVENT_TARGET), extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    for probe in ("goal_satisfaction", "initial_state", "terminal_state",
                  "relation_transitions", "event_presence"):
        assert res.probe_support[probe] == 1, probe
        assert res.probe_agreement[probe] is True, probe
    assert res.probe_support["event_order"] == 0  # single event, no pair to order against


# ---------------------------------------------------------------------------
# Strictly stronger — born-inside PASSes terminal-only but FAILs relation-event
# (on initial_state, NOT on the event/transition — the load-bearing subtlety)
# ---------------------------------------------------------------------------


def test_born_inside_passes_terminal_only_fails_relation_event_on_initial_state():
    # Cube inside the whole time but moving within the tray (so it IS a figure and the
    # extractor reports a terminal INSIDE). It never started NEAR.
    born = [(TX - 0.03, TY, 0.03), (TX - 0.01, TY, 0.03), (TX + 0.01, TY, 0.03),
            (TX + 0.03, TY, 0.03), (TX + 0.01, TY, 0.03), (TX - 0.01, TY, 0.03)]
    rollout = _rollout(born)

    term = _term(_TERMINAL_ONLY_TARGET, rollout)
    rele = _term(_RELATION_EVENT_TARGET, rollout)
    assert term["passed"] is True, term["hardMismatches"]          # ended inside -> terminal-only ok
    assert rele["passed"] is False                                 # never started near
    assert rele["hardMismatches"] == ["initial_state"], rele["hardMismatches"]
    # the event/transition are NOT what rejects it (extractor seeds prev_rel=NEAR):
    assert "event_presence" not in rele["hardMismatches"]
    assert "relation_transitions" not in rele["hardMismatches"]
    assert "goal_satisfaction" not in rele["hardMismatches"]


# ---------------------------------------------------------------------------
# Failure modes — each FAILs leakage-clean, naming the probe(s) it should trip
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,end_xyz,term_passes,rele_must_mismatch", [
    ("near_not_inside", _NEAR_NOT_INSIDE, False, {"goal_satisfaction", "terminal_state", "relation_transitions", "event_presence"}),
    ("rim_placement", _ON_RIM, False, {"goal_satisfaction", "terminal_state", "relation_transitions", "event_presence"}),
    ("dropped_outside", _FAR, False, {"goal_satisfaction", "terminal_state", "relation_transitions", "event_presence"}),
])
def test_failure_modes(name, end_xyz, term_passes, rele_must_mismatch):
    rollout = _rollout(_approach_then(end_xyz))

    term = _term(_TERMINAL_ONLY_TARGET, rollout)
    assert term["passed"] is term_passes, (name, term["hardMismatches"])
    assert term["leakageClean"] is True
    if not term_passes:
        assert "goal_satisfaction" in term["hardMismatches"], (name, term["hardMismatches"])

    rele = _term(_RELATION_EVENT_TARGET, rollout)
    assert rele["passed"] is False, name
    assert rele["leakageClean"] is True
    assert rele_must_mismatch <= set(rele["hardMismatches"]), (name, rele["hardMismatches"])


# ---------------------------------------------------------------------------
# Leakage — a leaky trace is rejected at the door, before the matcher can PASS
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,mutate,match_re", [
    ("targetCsg", lambda r: r.__setitem__("targetCsg", {"leaked": True}), "forbidden"),
    ("plannerView", lambda r: r.__setitem__("plannerView", {"leaked": True}), "forbidden"),
    ("objectIdMap", lambda r: r.__setitem__("objectIdMap", {"h_cube": "body_000"}), "objectIdMap"),
    ("body_field", lambda r: r["sceneBodies"][0].__setitem__("categoryLabel", "cube"), "non-whitelisted"),
    ("non_neutral_body_id", lambda r: r["sceneBodies"][0].__setitem__("objectId", "the_cube"), "neutral"),
])
def test_leaky_trace_is_rejected_before_matcher_success(name, mutate, match_re):
    base = _rollout(_approach_then(_INSIDE))
    assert _term(_RELATION_EVENT_TARGET, base)["passed"] is True
    bad = copy.deepcopy(base)
    mutate(bad)
    with pytest.raises(ExternalTraceLeakage, match=match_re):
        assert_rollout_leakage_clean(bad)
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(load_json(_RELATION_EVENT_TARGET), bad)


def test_source_identity_never_enters_the_rollout():
    # tag ids and source role names ("cube"/"tray") are quarantined: they appear in the
    # tracks but NEVER in the rollout's bodies, ids, objectIdMap, or per-frame pose keys.
    rollout = _rollout(_approach_then(_INSIDE))
    assert rollout["backend"] == "real_camera_external"
    assert rollout["objectIdMap"] == {}
    assert rollout["skillProgram"]["source"] == "real_camera"
    import json as _json
    bodies_blob = _json.dumps(rollout["sceneBodies"])
    for forbidden in ("cube", "tray", "marker", "7", "sourceRole"):
        assert forbidden not in bodies_blob, f"{forbidden!r} leaked into sceneBodies"
    for body in rollout["sceneBodies"]:
        assert str(body["objectId"]).startswith("body_")
    for frame in rollout["frames"]:
        assert all(k.startswith("body_") for k in frame["objectPoses"])


# ---------------------------------------------------------------------------
# Cross-task confusion — a camera success matches NO off-task gold target
# ---------------------------------------------------------------------------


def test_success_matches_no_off_task_gold_target():
    conf = external_confusion_report(
        _rollout(_approach_then(_INSIDE)), load_gold_targets(_GOLD_DIR),
        expected_case="put_cube_in_tray")
    # The camera relation/event subset has containment but NO contact/co-motion/release
    # evidence, so it matches no FULL gold task (every gold containment task additionally
    # demands those events). The honest result is: matches nothing.
    assert conf["passes"] == [], conf["passes"]
