"""Gold tests: hand-authored fixtures with frozen expected matcher behavior.

Each task dir has target.json, an expected.json manifest, and one robot_*.json
per expected outcome. robot_success.json is hand-authored (cube case) or
loop-generated (drawer case) and proves PASS is achievable without copying.
"""
import json

import pytest

from csg.matcher import match
from conftest import GOLD, load_json


def _gold_cases():
    cases = []
    for task_dir in sorted(GOLD.iterdir()):
        man = task_dir / "expected.json"
        if not man.is_file():
            continue
        expected = load_json(man)
        for name, exp in expected.items():
            cases.append((task_dir.name, name, task_dir / f"{name}.json", exp))
    return cases


@pytest.mark.parametrize("task,name,path,exp", _gold_cases(), ids=lambda v: v if isinstance(v, str) else "")
def test_gold_case(task, name, path, exp):
    target = load_json(GOLD / task / "target.json")
    robot = load_json(path)
    result = match(target, robot)
    assert result.passed == exp["passed"], f"{task}/{name}: passed={result.passed} expected={exp['passed']} mismatches={[p for p in result.hard_probes if not result.probe_agreement[p]]}"
    for probe in exp.get("expect_mismatch", []):
        assert not result.probe_agreement[probe], f"{task}/{name}: expected {probe} to mismatch"
