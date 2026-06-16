# Phase 3A Next Steps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the current conservative Sony/iPhone real-camera result into a defensible Phase 3A report with clear claim boundaries and a next capture/calibration decision.

**Architecture:** Keep `csg/` byte-frozen. Work only in the real-camera pilot, dataset reports, target metadata, and roadmap/docs. Treat current generated tracks/rollouts as evidence to audit, not as unquestioned ground truth.

**Tech Stack:** Python 3.12 bundled runtime, OpenCV AprilTag detection, `pilots.real_camera`, JSON verdict artifacts, pytest.

---

### Task 1: Freeze The Current Evidence Summary

**Files:**
- Read: `datasets/sony_object_inside_container_v0/verdicts_all.json`
- Modify: `datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md`
- Test: command-line JSON summary

- [ ] **Step 1: Recompute headline metrics from JSON**

Run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 - <<'PY'
import json
from collections import Counter
rows=json.load(open("datasets/sony_object_inside_container_v0/verdicts_all.json"))["rows"]
failure={"near_not_inside","left_on_rim","dropped_or_left_outside","inside_to_outside","static_no_task"}
false_pass=[r for r in rows if r["expectedClass"] in failure and ("PASS" in (r.get("actualTerminal"), r.get("actualRelation")))]
success=[r for r in rows if r["expectedClass"]=="success"]
print("rows", len(rows), "errors", sum("error" in r for r in rows), "false_failure_passes", len(false_pass))
for cam in ("sony_front","iphone_top"):
    rs=[r for r in success if r["camera"]==cam]
    print(cam, "terminal", Counter(r["actualTerminal"] for r in rs), "relation", Counter(r["actualRelation"] for r in rs))
PY
```

Expected: `rows 78`, `errors 0`, `false_failure_passes 0`, Sony terminal successes `10/16`, iPhone terminal successes `8/16`.

- [ ] **Step 2: Edit the results report wording**

In `datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md`, make the top claim:

```text
Successful conservative Phase 3A pilot: real-camera failures are not falsely accepted, while success recall remains limited by calibration and target strictness.
```

Avoid claiming “real-camera ingestion complete” without the qualifier “conservative pilot.”

- [ ] **Step 3: Verify report and artifacts still parse**

Run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m json.tool datasets/sony_object_inside_container_v0/verdicts_all.json >/tmp/verdicts_all.checked.json
```

Expected: exit `0`.

### Task 2: Decide The Relation-Event Target Semantics

**Files:**
- Read: `pilots/real_camera/targets/object_inside_container_relation_event.json`
- Modify only if approved: a new target file, not the existing one
- Test: `tests/test_real_camera_rollout.py`

- [ ] **Step 1: Document the decision point**

Add a short section to `INGESTION_RESULTS.md`:

```text
Decision needed: the current relation-event target requires initial NEAR. Many real put-ins begin FAR, so terminal success can pass while relation-event fails initial_state. Options: keep strict NEAR for a narrow task, or author a separate real-camera target that means "not initially INSIDE -> terminal INSIDE."
```

- [ ] **Step 2: Do not mutate the existing target**

If changing semantics, create a new file:

```text
pilots/real_camera/targets/object_inside_container_not_initially_inside_event.json
```

Do not modify `object_inside_container_relation_event.json` in place.

- [ ] **Step 3: Add tests before creating the new target**

If this task is executed, first add tests showing:

```text
FAR -> INSIDE success passes the new target.
born-inside fails the new target.
near-not-inside fails the new target.
```

Run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_rollout.py -q
```

Expected before implementation: new tests fail. Expected after implementation: all pass.

### Task 3: Audit False Negatives Before More Tuning

**Files:**
- Read: `datasets/sony_object_inside_container_v0/verdicts_all.json`
- Read: selected files in `datasets/sony_object_inside_container_v0/tracks/`
- Optional scratch only: `output/frames/`

- [ ] **Step 1: Select five false negatives**

Use:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 - <<'PY'
import json
rows=json.load(open("datasets/sony_object_inside_container_v0/verdicts_all.json"))["rows"]
for r in rows:
    if r["expectedClass"]=="success" and r["actualTerminal"]!="PASS":
        print(r["episodeId"], r["camera"], r["actualTerminal"], r.get("terminalClass"))
PY
```

Pick at least: one Sony `FAIL`, one Sony `UNCERTAIN`, one iPhone `FAIL`, one iPhone `UNCERTAIN`, and one obstructed success.

- [ ] **Step 2: For each selected clip, inspect why it failed**

Record one reason per clip:

```text
calibration footprint miss
cube tag dropout
tray tag occlusion
rim/height ambiguity
true borderline placement
```

- [ ] **Step 3: Stop tuning if the failure is truly borderline**

If the cube is near a wall/rim or tag evidence is weak, keep `FAIL`/`UNCERTAIN`. Do not shift geometry to make it pass.

### Task 4: Choose Between Salvage And Second Capture

**Files:**
- Modify: `roadmap.md`
- Modify: `datasets/sony_object_inside_container_v0/INGESTION_RESULTS.md`

- [ ] **Step 1: Set the salvage bar**

Accept the current capture as a Phase 3A pilot if these remain true:

```text
0 false PASSes on genuine failures
>= 50% terminal success recall on at least one camera
all ambiguous/occluded clips fail closed
csg/ diff remains empty
```

- [ ] **Step 2: Set the second-capture trigger**

Do a cleaner second capture if public-facing success recall is needed:

```text
larger rigid tray, 22-25 cm inner footprint
2-3 cm rim
two tray markers on the floor/table plane
one tray marker guaranteed visible after cube placement
cube placed well inside for success clips
calibration clip with all markers visible and cube outside
```

- [ ] **Step 3: Update roadmap status**

In `roadmap.md`, Phase 3A should say:

```text
Conservative real-camera pilot complete: genuine failures are not falsely accepted; success recall is partial due to calibration/capture limits. Next: decide target semantics and/or run a cleaner second capture.
```

- [ ] **Step 4: Verify real-camera tests**

Run:

```bash
/Users/alejandro/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest tests/test_real_camera_*.py -q
```

Expected: all real-camera tests pass.

