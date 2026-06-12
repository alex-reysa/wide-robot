"""Benchmark confusion matrix: cross-task separation + documented equivalences.

Every target must PASS its own rollout (diagonal) and FAIL every other task's
rollout, with one documented exception: insert_object and put_cube_in_tray are
the same observable quotient class (move a rigid object INSIDE a container),
so they mutually PASS. That mutual PASS is asserted, not tolerated — it is the
quotient semantics working as specified. Any other off-diagonal PASS would
mean an under-constrained target or a hardcoded solver trajectory.
"""
from csg.benchmark import discover_targets, run_benchmark


def test_confusion_matrix(tmp_path):
    targets = discover_targets(["gold_tests"], None)
    report = run_benchmark(targets, tmp_path, confusion=True)
    conf = report["confusion"]
    assert report["summary"]["failed"] == 0
    assert conf["missedDiagonal"] == []
    assert conf["unexpectedOffDiagonalPasses"] == []
    # The documented quotient equivalence must hold in BOTH directions.
    assert conf["matrix"]["insert_object"]["put_cube_in_tray"] is True
    assert conf["matrix"]["put_cube_in_tray"]["insert_object"] is True
    assert sorted(conf["offDiagonalPasses"]) == [
        ["insert_object", "put_cube_in_tray"],
        ["put_cube_in_tray", "insert_object"],
    ]


def test_confusion_separates_manner_and_relation(tmp_path):
    """Spot-check the strongest separations: push (manner-constrained) and
    open_drawer (articulation) accept no other task's rollout."""
    targets = discover_targets(["gold_tests"], None)
    report = run_benchmark(targets, tmp_path, confusion=True)
    m = report["confusion"]["matrix"]
    for other in ("insert_object", "place_on_top", "put_cube_in_tray", "open_drawer"):
        assert m["push_object"][other] is False
    for other in ("insert_object", "place_on_top", "put_cube_in_tray", "push_object"):
        assert m["open_drawer"][other] is False
