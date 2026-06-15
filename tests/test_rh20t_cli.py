"""RH20T pilot CLI + annotation-sidecar smoke tests (Phase 3A.5).

Exercises the full committed pipeline end-to-end on synthetic evidence, with NO RH20T raw
media, numpy/cv2, or RLBench: a reviewed ``rh20t.annotation.v0`` sidecar → tracks →
rollout → frozen-verifier verdict, both in-process and through the three module ``main()``
CLIs writing to ``tmp_path``.
"""
import json
from pathlib import Path

import pytest

from csg.common import load_json
from pilots.rh20t.annotations_to_tracks import RH20TAnnotationError, annotations_to_tracks, main as ann_main
from pilots.rh20t.tracks_to_rollout import RH20TTracksError, main as roll_main
from pilots.rh20t.verify_episode import main as verify_main, verify_episode_both


def _sidecar(mover_seq=None):
    """A reviewed sidecar: mover NEAR(0.46) → mid → INSIDE(0.30, persists), container static."""
    if mover_seq is None:
        mover_seq = [(0.46, 0.0, 0.05), (0.38, 0.0, 0.04), (0.30, 0.0, 0.03), (0.30, 0.0, 0.03)]
    frames = []
    for i, (x, y, z) in enumerate(mover_seq):
        frames.append({"frameIndex": i, "timeS": i * 0.1, "poses": {
            "mover": {"positionM": {"x": x, "y": y, "z": z}, "confidence": 0.9},
            "container": {"positionM": {"x": 0.30, "y": 0.0, "z": 0.015}, "confidence": 0.99}}})
    return {
        "schemaVersion": "rh20t.annotation.v0",
        "episodeId": "task_0017_user_0001_scene_0001_cfg_0003",
        "source": {
            "dataset": "RH20T", "taskId": "task_0017",
            "taskDescription": "Put the pen into the pen holder",
            "scenePath": "RH20T_cfg3/task_0017_user_0001_scene_0001_cfg_0003",
            "archiveSha256": "0" * 64,
        },
        "fps": 10.0,
        "objects": [
            {"sourceRole": "mover", "physicalKind": "RIGID_OBJECT", "mobility": "MOVABLE",
             "isContainer": False, "sizeM": [0.04, 0.04, 0.04]},
            {"sourceRole": "container", "physicalKind": "RIGID_OBJECT", "mobility": "STATIC",
             "isContainer": True, "sizeM": [0.24, 0.18, 0.03]},
        ],
        "frames": frames,
        "review": {"method": "unit-test synthetic sidecar"},
    }


def test_annotations_to_tracks_then_verify_both():
    tracks = annotations_to_tracks(_sidecar())
    assert tracks["schemaVersion"] == "rh20t.tracks.v0"
    # the tracks envelope KEEPS provenance (it is committed evidence the verifier never reads)
    assert tracks["source"]["taskId"] == "task_0017"
    result = verify_episode_both(tracks=tracks)
    assert result["object_inside_container_terminal_only"]["status"] == "PASS"
    assert result["object_inside_container_relation_event"]["status"] == "PASS"


def test_negative_sidecar_fails_both():
    # terminal mover left NEAR-not-inside (x=0.43) -> both targets FAIL, leakage clean
    neg = [(0.46, 0.0, 0.05), (0.45, 0.0, 0.05), (0.43, 0.0, 0.05), (0.43, 0.0, 0.05)]
    result = verify_episode_both(tracks=annotations_to_tracks(_sidecar(neg)))
    for name, rec in result.items():
        assert rec["status"] == "FAIL", (name, rec)
        assert rec["leakageClean"] is True
        assert rec["physicalValidity"] is None


def test_malformed_sidecar_is_rejected():
    bad_schema = {**_sidecar(), "schemaVersion": "wrong"}
    with pytest.raises(RH20TAnnotationError):
        annotations_to_tracks(bad_schema)
    too_few = _sidecar([(0.46, 0.0, 0.05), (0.30, 0.0, 0.03)])  # 2 frames < MIN_TRACK_FRAMES
    with pytest.raises(RH20TTracksError):
        annotations_to_tracks(too_few)


def test_cli_pipeline_in_tmp_path(tmp_path, capsys):
    ann = tmp_path / "ep.annotation.json"
    tracks = tmp_path / "ep.tracks.json"
    rollout = tmp_path / "ep.rollout.json"
    ann.write_text(json.dumps(_sidecar()))

    assert ann_main(["--annotation", str(ann), "--out", str(tracks)]) == 0
    assert tracks.exists()
    assert load_json(tracks)["schemaVersion"] == "rh20t.tracks.v0"

    assert roll_main(["--tracks", str(tracks), "--out", str(rollout)]) == 0
    assert rollout.exists()
    r = load_json(rollout)
    assert r["backend"] == "rh20t_external"
    assert r["diagnostics"]["physicalValidity"] is None
    # the written rollout artifact is fully source-blind
    assert "task_0017" not in rollout.read_text()

    capsys.readouterr()
    rc = verify_main(["--tracks", str(tracks), "--json"])
    out = capsys.readouterr().out
    assert rc == 0
    parsed = json.loads(out)
    assert parsed["object_inside_container_terminal_only"]["status"] == "PASS"
    assert parsed["object_inside_container_relation_event"]["status"] == "PASS"


def test_cli_verify_rollout_path_in_tmp(tmp_path, capsys):
    # verify directly from a pre-built rollout (no tracks): single-target form
    ann = tmp_path / "ep.annotation.json"
    tracks = tmp_path / "ep.tracks.json"
    rollout = tmp_path / "ep.rollout.json"
    ann.write_text(json.dumps(_sidecar()))
    ann_main(["--annotation", str(ann), "--out", str(tracks)])
    roll_main(["--tracks", str(tracks), "--out", str(rollout)])

    target = Path(__file__).resolve().parents[1] / "pilots" / "rh20t" / "targets" / \
        "object_inside_container_relation_event.json"
    capsys.readouterr()
    rc = verify_main(["--rollout", str(rollout), "--target", str(target), "--json"])
    out = capsys.readouterr().out
    assert rc == 0
    assert json.loads(out)["object_inside_container_relation_event"]["status"] == "PASS"
