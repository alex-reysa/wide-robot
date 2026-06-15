"""RH20T external-source object_inside_container — the matcher seam (Phase 3A.5).

Mirrors tests/test_real_camera_rollout.py: synthetic ``rh20t.tracks.v0`` episodes are
converted by the RH20T rollout door (:func:`tracks_to_rollout`) into ``csg.rollout.v0``
and judged by the FROZEN verifier against the two RH20T targets. RH20T is treated as a
SEPARATE external source — these fixtures prove source-adapter discipline and the leakage
quarantine without touching RH20T raw media, numpy/cv2, or ``csg/``.

The two targets form a strictly-stronger pair (the RH20T analogue of the RLBench
value-only → articulation-event and real-camera terminal-only → relation-event
progressions):
  * terminal_only  — only the mover's TERMINAL relation to the container is INSIDE
    (hard OBJECT_RELATION_GOAL → goal_satisfaction);
  * relation_event — additionally the mover STARTED NEAR (initial_state), ENDED INSIDE
    (terminal_state + relation_transitions), and a CONTAINMENT_CHANGE event is present
    (event_presence). event_order stays support 0 (one event, no pair).

Load-bearing subtlety pinned here (verified against the frozen extractor): a "born-inside"
mover (inside the whole time, but moving) STILL emits a NEAR→INSIDE CONTAINMENT_CHANGE
delta because ``csg/rollout_extract.py`` seeds ``prev_rel="NEAR"`` UNCONDITIONALLY — so
event_presence/relation_transitions do NOT reject born-inside; only ``initial_state`` does.

QUARANTINE: an RH20T ``episodeId`` *is* the source identity. The rollout is fully
source-blind — ``task_0017`` / ``RH20T_cfg3`` / source role names appear NOWHERE in the
rollout blob, not even in diagnostics (only a one-way ``episodeRef`` hash).
"""
import copy
import json
import re
from pathlib import Path

import pytest

import csg.predicates as P
from csg.common import load_json
from csg.matcher import MatcherConfig, match
from csg.rollout_extract import extract_robot_csg

from pilots.external_rollout import ExternalTraceLeakage, assert_rollout_leakage_clean
from pilots.external_verify import external_confusion_report, load_gold_targets, verify_external_rollout
from pilots.rh20t.tracks_to_rollout import RH20TTracksError, tracks_to_rollout
from pilots.rh20t.verify_episode import verify_episode_both

_REPO = Path(__file__).resolve().parents[1]
_GOLD_DIR = _REPO / "gold_tests"
_TARGETS = _REPO / "pilots" / "rh20t" / "targets"
_TERMINAL = _TARGETS / "object_inside_container_terminal_only.json"
_REL_EVENT = _TARGETS / "object_inside_container_relation_event.json"

# Same proven tabletop geometry as the real-camera fixtures: tray center (TX,TY,TZ),
# size (0.24,0.18,0.03) -> footprint x in [0.18,0.42]; mover is a 0.04^3 cube.
TX, TY, TZ = 0.30, 0.0, 0.015
_TRAY = [0.24, 0.18, 0.03]
_CUBE = [0.04, 0.04, 0.04]
_INSIDE = (TX, TY, 0.03)
_NEAR_NOT_INSIDE = (TX + 0.13, TY, 0.05)
_ON_RIM = (TX, TY, 0.05)
_FAR = (TX + 0.35, TY, 0.02)
_START_NEAR = (TX + 0.16, TY, 0.05)


def _tracks(mover_seq, container_xyz=(TX, TY, TZ), fps=10.0, mover_conf=0.95, container_conf=0.99):
    """An ``rh20t.tracks.v0`` episode whose ``source`` block carries the FULL RH20T
    identity (task id, description, scene path) so the quarantine test proves the rollout
    door drops all of it."""
    objects = [
        {"sourceRole": "mover", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
         "isContainer": False, "sizeM": list(_CUBE)},
        {"sourceRole": "container", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
         "isContainer": True, "sizeM": list(_TRAY)},
    ]
    frames = []
    for i, (x, y, z) in enumerate(mover_seq):
        frames.append({"frameIndex": i, "timeS": i / fps, "poses": {
            "mover": {"positionM": {"x": x, "y": y, "z": z}, "confidence": mover_conf},
            "container": {"positionM": {"x": container_xyz[0], "y": container_xyz[1],
                                        "z": container_xyz[2]}, "confidence": container_conf},
        }})
    return {
        "schemaVersion": "rh20t.tracks.v0",
        "episodeId": "task_0017_user_0001_scene_0001_cfg_0003",
        "source": {
            "dataset": "RH20T",
            "taskId": "task_0017",
            "taskDescription": "Put the pen into the pen holder",
            "scenePath": "RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003",
            "archiveSha256": "0" * 64,
        },
        "fps": fps,
        "objects": objects,
        "frames": frames,
    }


def _approach_then(end_xyz):
    """Mover starts NEAR (outside the container) and moves to ``end_xyz`` over 6 frames so
    the terminal relation persists and mover displacement >> MOTION_EPS_M."""
    sx, sy, sz = _START_NEAR
    ex, ey, ez = end_xyz
    return [(sx, sy, sz), (sx, sy, sz),
            (0.5 * (sx + ex), 0.5 * (sy + ey), 0.5 * (sz + ez)),
            (ex, ey, ez), (ex, ey, ez), (ex, ey, ez)]


def _rollout(mover_seq, **kw):
    r = tracks_to_rollout(_tracks(mover_seq, **kw))
    assert_rollout_leakage_clean(r)
    return r


def _verify(target_path, rollout):
    return verify_external_rollout(load_json(target_path), rollout, case_name="rh20t_oic")


# ---------------------------------------------------------------------------
# Geometry tripwire — pin the fixture coordinates against the frozen predicates
# ---------------------------------------------------------------------------


def _box(center, size):
    return P.box_from(center, tuple(size))


def test_fixture_geometry_classifies_as_intended():
    tray = _box((TX, TY, TZ), _TRAY)
    assert P.is_inside(_box(_INSIDE, _CUBE), tray) is True
    assert P.primary_topo_relation(_box(_INSIDE, _CUBE), tray) == "INSIDE"
    assert P.is_inside(_box(_START_NEAR, _CUBE), tray) is False
    assert P.is_near(_box(_START_NEAR, _CUBE), tray) is True
    assert P.is_inside(_box(_ON_RIM, _CUBE), tray) is False
    assert P.is_on_top_of(_box(_ON_RIM, _CUBE), tray) is True
    assert P.is_inside(_box(_NEAR_NOT_INSIDE, _CUBE), tray) is False
    assert P.is_near(_box(_NEAR_NOT_INSIDE, _CUBE), tray) is True
    assert P.is_near(_box(_FAR, _CUBE), tray) is False


# ---------------------------------------------------------------------------
# Targets structure / not-a-gold-task
# ---------------------------------------------------------------------------


def test_targets_structure_and_deferrals():
    term = load_json(_TERMINAL)
    rele = load_json(_REL_EVENT)
    for t in (term, rele):
        goals = t["plannerView"]["stages"][0]["goalConstraints"]
        assert [g["kind"] for g in goals] == ["OBJECT_RELATION_GOAL"]
        assert goals[0]["hard"] is True
        assert goals[0]["relation"]["desiredRelation"] == "INSIDE"
        assert t["agentParts"] == []
        assert "contacts" not in t and "temporalEdges" not in t
    assert "relations" not in term and "events" not in term
    assert [r["relation"] for r in rele["relations"]] == ["NEAR", "INSIDE"]
    assert [e["eventKind"] for e in rele["events"]] == ["CONTAINMENT_CHANGE"]
    trans = rele["events"][0]["observedDeltas"][0]["relationTransition"]
    assert (trans["fromRelation"], trans["toRelation"]) == ("NEAR", "INSIDE")


def test_targets_are_not_gold_tasks():
    assert not (_GOLD_DIR / "object_inside_container_terminal_only").exists()
    assert not (_GOLD_DIR / "object_inside_container_relation_event").exists()
    assert load_json(_TERMINAL)["pilotMetadata"]["diagnostic"] == "rh20t-object-inside-container-terminal-only"
    assert load_json(_REL_EVENT)["pilotMetadata"]["diagnostic"] == "rh20t-object-inside-container-relation-event"


# ---------------------------------------------------------------------------
# Positive — a real put-in PASSes both targets, non-vacuously, leakage-clean
# ---------------------------------------------------------------------------


def test_rh20t_success_passes_both_targets_leakage_clean():
    rollout = tracks_to_rollout(_tracks(_approach_then(_INSIDE)))
    assert rollout["backend"] == "rh20t_external"
    assert rollout["skillProgram"]["source"] == "rh20t"
    assert rollout["diagnostics"]["physicalValidity"] is None
    assert rollout["objectIdMap"] == {}
    assert_rollout_leakage_clean(rollout)

    for path in (_TERMINAL, _REL_EVENT):
        case = _verify(path, rollout)
        assert case["passed"] is True, (path.name, case["hardMismatches"])
        assert case["leakageClean"] is True
        assert case["physicalValidity"] is None
        assert case["hardMismatches"] == []

    res = match(load_json(_REL_EVENT), extract_robot_csg(rollout), MatcherConfig())
    assert res.vacuous is False
    for probe in ("goal_satisfaction", "initial_state", "terminal_state",
                  "relation_transitions", "event_presence"):
        assert res.probe_support[probe] == 1, probe
        assert res.probe_agreement[probe] is True, probe
    assert res.probe_support["event_order"] == 0


# ---------------------------------------------------------------------------
# Strictly stronger — born-inside PASSes terminal-only but FAILs relation-event
# on initial_state (NOT on the event/transition — the load-bearing subtlety)
# ---------------------------------------------------------------------------


def test_born_inside_passes_terminal_only_fails_relation_event_on_initial_state():
    born = [(TX - 0.03, TY, 0.03), (TX - 0.01, TY, 0.03), (TX + 0.01, TY, 0.03),
            (TX + 0.03, TY, 0.03), (TX + 0.01, TY, 0.03), (TX - 0.01, TY, 0.03)]
    rollout = _rollout(born)
    term = _verify(_TERMINAL, rollout)
    rele = _verify(_REL_EVENT, rollout)
    assert term["passed"] is True, term["hardMismatches"]
    assert rele["passed"] is False
    assert rele["hardMismatches"] == ["initial_state"], rele["hardMismatches"]
    assert "event_presence" not in rele["hardMismatches"]
    assert "relation_transitions" not in rele["hardMismatches"]
    assert "goal_satisfaction" not in rele["hardMismatches"]


# ---------------------------------------------------------------------------
# Failure modes — each FAILs leakage-clean, naming the probe(s) it should trip
# ---------------------------------------------------------------------------


def test_rh20t_near_not_inside_fails_both_targets():
    rollout = tracks_to_rollout(_tracks(_approach_then(_NEAR_NOT_INSIDE)))
    assert _verify(_TERMINAL, rollout)["passed"] is False
    rel = _verify(_REL_EVENT, rollout)
    assert rel["passed"] is False
    assert "goal_satisfaction" in rel["hardMismatches"]


@pytest.mark.parametrize("name,end_xyz", [
    ("near_not_inside", _NEAR_NOT_INSIDE),
    ("rim_placement", _ON_RIM),
    ("dropped_outside", _FAR),
])
def test_failure_modes(name, end_xyz):
    rollout = _rollout(_approach_then(end_xyz))
    term = _verify(_TERMINAL, rollout)
    assert term["passed"] is False, (name, term["hardMismatches"])
    assert term["leakageClean"] is True
    assert "goal_satisfaction" in term["hardMismatches"], (name, term["hardMismatches"])

    rele = _verify(_REL_EVENT, rollout)
    assert rele["passed"] is False, name
    assert rele["leakageClean"] is True
    assert {"goal_satisfaction", "terminal_state", "relation_transitions",
            "event_presence"} <= set(rele["hardMismatches"]), (name, rele["hardMismatches"])


# ---------------------------------------------------------------------------
# Leakage — a leaky trace is rejected at the door, before the matcher can PASS
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,mutate,match_re", [
    ("targetCsg", lambda r: r.__setitem__("targetCsg", {"leaked": True}), "forbidden"),
    ("plannerView", lambda r: r.__setitem__("plannerView", {"leaked": True}), "forbidden"),
    ("objectIdMap", lambda r: r.__setitem__("objectIdMap", {"h_cube": "body_000"}), "objectIdMap"),
    ("body_field", lambda r: r["sceneBodies"][0].__setitem__("categoryLabel", "cube"), "non-whitelisted"),
    ("non_neutral_body_id", lambda r: r["sceneBodies"][0].__setitem__("objectId", "the_pen"), "neutral"),
])
def test_leaky_trace_is_rejected_before_matcher_success(name, mutate, match_re):
    base = _rollout(_approach_then(_INSIDE))
    assert _verify(_REL_EVENT, base)["passed"] is True
    bad = copy.deepcopy(base)
    mutate(bad)
    with pytest.raises(ExternalTraceLeakage, match=match_re):
        assert_rollout_leakage_clean(bad)
    with pytest.raises(ExternalTraceLeakage):
        verify_external_rollout(load_json(_REL_EVENT), bad)


# ---------------------------------------------------------------------------
# Source identity quarantine — the rollout is fully source-blind
# ---------------------------------------------------------------------------


def test_rh20t_source_identity_is_quarantined():
    rollout = tracks_to_rollout(_tracks(_approach_then(_INSIDE)))
    blob = json.dumps(rollout)
    # (a) fixture-specific identity absent
    for forbidden in ("task_0017", "pen", "holder", "mover", "container", "RH20T_cfg3"):
        assert forbidden not in blob, f"{forbidden!r} leaked into the rollout"
    # (b) STRUCTURAL + task-agnostic: no RH20T scene-path identity token appears ANYWHERE,
    # so the guarantee is not coupled to this fixture's task id (catches a leak from any
    # task, e.g. task_0091/RH20T_cfg5). None of these tokens occur in a legitimate rollout
    # string (backend "rh20t_external", physicalValidityReason prose, body ids, etc.).
    for token in ("task_", "scene_", "user_", "_cfg_", "RH20T_cfg"):
        assert token not in blob, f"RH20T identity token {token!r} leaked into the rollout"
    # (c) diagnostics carry only a source-blind hash + a SHAPE-VALIDATED archive sha
    diag = rollout["diagnostics"]
    assert diag["sourceDataset"] == "RH20T"
    assert diag["source"] == "rh20t"
    assert diag["archiveSha256"] is None or re.fullmatch(r"[0-9a-f]{64}", diag["archiveSha256"])
    assert len(diag["episodeRef"]) == 16 and "task" not in diag["episodeRef"]
    assert "episodeId" not in diag and "sourceTaskId" not in diag
    for body in rollout["sceneBodies"]:
        assert str(body["objectId"]).startswith("body_")
    for frame in rollout["frames"]:
        assert all(k.startswith("body_") for k in frame["objectPoses"])


def test_poisoned_archive_sha_is_rejected_not_leaked():
    """A human paste-error (a scene path / task id pasted where the 64-hex sha belongs)
    must be rejected at the door — NEVER minted into a 'source-blind' rollout, and via
    verify_episode it surfaces as UNCERTAIN source_evidence_invalid, never PASS."""
    poison = "RH20T_cfg5/task_0091_user_0003_scene_0007_cfg_0005"  # a scene path, not a hash
    bad = _tracks(_approach_then(_INSIDE))
    bad["source"]["archiveSha256"] = poison
    with pytest.raises(RH20TTracksError, match="archiveSha256"):
        tracks_to_rollout(bad)
    res = verify_episode_both(tracks=bad)
    for name, rec in res.items():
        assert rec["status"] == "UNCERTAIN", (name, rec)
        assert rec["failureClass"] == "source_evidence_invalid"
    # also a too-short / uppercase / non-hex digest is rejected
    for bad_sha in ("0" * 63, "ABC" + "0" * 61, "g" * 64):
        b2 = _tracks(_approach_then(_INSIDE))
        b2["source"]["archiveSha256"] = bad_sha
        with pytest.raises(RH20TTracksError):
            tracks_to_rollout(b2)


# ---------------------------------------------------------------------------
# Cross-task confusion — an RH20T success matches NO off-task gold target
# ---------------------------------------------------------------------------


def test_success_matches_no_off_task_gold_target():
    conf = external_confusion_report(
        _rollout(_approach_then(_INSIDE)), load_gold_targets(_GOLD_DIR),
        expected_case="put_cube_in_tray")
    # The RH20T relation/event subset has containment but NO contact/co-motion/release
    # evidence, so it matches no FULL gold task (every gold containment task additionally
    # demands those events). The honest result is: matches nothing.
    assert conf["passes"] == [], conf["passes"]
