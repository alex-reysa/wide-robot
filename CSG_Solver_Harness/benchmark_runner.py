#!/usr/bin/env python3
"""CSG Solver Harness V0 benchmark runner.

Runs the frozen pipeline:
  target_csg.json -> csg_to_sim -> skill_skeleton -> csg_solver -> rollout
  -> rollout_to_csg -> unchanged CSG_Matcher -> report/logs/visual trace.
"""
from __future__ import annotations

import argparse
import csv
import json
import traceback
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence

from csg_common import Json, as_list, get_any, load_json, pose_xyz, write_json
from csg_matcher import MatcherConfig, match_csg_json
from csg_solver import SolverConfig, solve_target_csg


def discover_cases(positional: Sequence[str] = (), cases: Sequence[str] = (), case_dir: Optional[str] = None, target: Optional[str] = None) -> List[Path]:
    out: List[Path] = []
    if target:
        out.append(Path(target))
    out.extend(Path(p) for p in positional)
    out.extend(Path(p) for p in cases)
    if case_dir:
        root = Path(case_dir)
        skip = {
            "report.json", "report.md", "summary.json", "benchmark_report.json", "matcher_report.json",
            "benchmark_matcher_report.json", "solve_report.json", "rollout.json", "robot_csg.json",
            "target_csg.json", "skill_skeletons.json", "scene.compiled.json", "scene.isaac.json",
            "video_manifest.json",
        }
        for p in sorted(root.rglob("*.json")):
            if p.name not in skip:
                out.append(p)
    seen: set[str] = set()
    unique: List[Path] = []
    for p in out:
        r = str(p.resolve())
        if r not in seen:
            seen.add(r)
            unique.append(p)
    return unique


def _case_name(path: Path, idx: int) -> str:
    s = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in path.stem).strip("_")
    return s or f"case_{idx:03d}"


def _timeline_svg(rollout: Mapping[str, Any], path: Path) -> None:
    frames = as_list(get_any(rollout, "frames", default=[]))
    width, height = 1000, 340
    subj = ""
    # Find the first object with movement frames.
    if frames:
        poses = get_any(frames[0], "objectPoses", default={}) or {}
        if isinstance(poses, Mapping) and poses:
            subj = next(iter(poses.keys()))
    pts: List[tuple[float, float]] = []
    for fr in frames:
        poses = get_any(fr, "objectPoses", default={}) or {}
        if isinstance(poses, Mapping) and subj in poses:
            x, y, _ = pose_xyz(poses[subj])
            pts.append((80 + 820 * x, 260 - 820 * y))
    if not pts:
        pts = [(60, 260), (300, 170), (700, 230)]
    poly = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
    circles = "\n".join(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="black"/>' for x, y in pts)
    labels = []
    for i, fr in enumerate(frames):
        phase = str(get_any(fr, "phase", default=f"f{i}"))
        labels.append(f'<text x="20" y="{25 + 18*i}" font-size="12" font-family="monospace">{i}: {phase}</text>')
    path.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">'
        f'<rect width="100%" height="100%" fill="white"/>'
        f'<text x="20" y="18" font-size="16" font-family="sans-serif">CSG symbolic rollout trace: {subj}</text>'
        f'<polyline points="{poly}" fill="none" stroke="black" stroke-width="2"/>{circles}'
        f'{"".join(labels)}</svg>\n',
        encoding="utf-8",
    )


def _frames_jsonl(rollout: Mapping[str, Any], path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        for fr in as_list(get_any(rollout, "frames", default=[])):
            f.write(json.dumps(fr, sort_keys=True) + "\n")




def _rollout_mp4(rollout: Mapping[str, Any], path: Path) -> Optional[str]:
    """Write a lightweight MP4 visual trace for the symbolic rollout."""
    try:
        from PIL import Image, ImageDraw
        import imageio.v2 as imageio
    except Exception:
        return None
    frames = as_list(get_any(rollout, "frames", default=[]))
    if not frames:
        return None
    W, H = 720, 432
    images = []
    for i, fr in enumerate(frames):
        img = Image.new("RGB", (W, H), "white")
        d = ImageDraw.Draw(img)
        d.rectangle((40, 70, W - 40, H - 45), outline="black", width=2)
        d.text((24, 24), f"CSG symbolic rollout frame {i}: {get_any(fr, 'phase', default='')}", fill="black")
        poses = get_any(fr, "objectPoses", default={}) or {}
        if isinstance(poses, Mapping):
            for oid, pose in sorted(poses.items()):
                if not isinstance(pose, Mapping):
                    continue
                x, y, _ = pose_xyz(pose)
                px, py = int(W / 2 + 760 * x), int(H / 2 - 760 * y)
                r = 11
                d.rectangle((px - r, py - r, px + r, py + r), outline="black", width=2)
                d.text((px + 14, py - 8), str(oid), fill="black")
        eff = get_any(fr, "effectorPose", "gripperPose", default=None)
        if isinstance(eff, Mapping):
            x, y, _ = pose_xyz(eff)
            px, py = int(W / 2 + 760 * x), int(H / 2 - 760 * y)
            d.ellipse((px - 8, py - 8, px + 8, py + 8), outline="black", width=2)
            d.text((px + 11, py + 8), "gripper", fill="black")
        images.extend([img] * 8)
    imageio.mimsave(path, images, fps=8)
    return str(path)

def run_one_case(path: Path, out_root: Path, idx: int, solver_cfg: SolverConfig, matcher_cfg: MatcherConfig) -> Json:
    cname = _case_name(path, idx)
    cdir = out_root / cname
    cdir.mkdir(parents=True, exist_ok=True)
    log: List[str] = [f"case={cname}", f"target={path}"]
    try:
        result = solve_target_csg(path, cdir, solver_cfg)
        target = load_json(path)
        robot_csg = load_json(result.best_robot_csg_path)
        rollout = load_json(result.best_rollout_path)
        # Re-run matcher outside the solver so the benchmark is independent.
        match = match_csg_json(target, robot_csg, matcher_cfg)
        bench_match_path = cdir / "benchmark_matcher_report.json"
        write_json(bench_match_path, match.to_json())
        timeline_path = cdir / "rollout_timeline.svg"
        frames_path = cdir / "rollout_frames.jsonl"
        _timeline_svg(rollout, timeline_path)
        _frames_jsonl(rollout, frames_path)
        mp4_path = _rollout_mp4(rollout, cdir / "rollout_video.mp4")
        video_manifest_path = cdir / "video_manifest.json"
        write_json(video_manifest_path, {
            "schemaVersion": "csg.video_manifest.v0",
            "timelineSvg": str(timeline_path),
            "framesJsonl": str(frames_path),
            "mp4": str(mp4_path) if mp4_path else None,
            "note": "V0 symbolic backend emits MP4/SVG/JSONL traces; MuJoCo/Isaac renderer can replace this with photorealistic video.",
        })
        solve_report = load_json(result.report_path)
        selected = get_any(solve_report, "selectedProgram", default={}) or {}
        status = "PASS" if match.distance == 0.0 and match.same_quotient_class else "FAIL"
        log += [
            f"distance={match.distance}",
            f"same_quotient_class={match.same_quotient_class}",
            f"selected_program={get_any(selected, 'programId', default='')}",
            f"status={status}",
        ]
        (cdir / "run.log").write_text("\n".join(log) + "\n", encoding="utf-8")
        return {
            "case": cname,
            "target": str(path),
            "status": status,
            "distance": match.distance,
            "sameQuotientClass": match.same_quotient_class,
            "selectedProgram": get_any(selected, "programId", default=None),
            "solverFailures": as_list(get_any(rollout, "failures", default=[])),
            "componentDistances": match.component_distances,
            "objectMapping": match.object_mapping,
            "outDir": str(cdir),
            "artifacts": {
                "solveReport": result.report_path,
                "rollout": result.best_rollout_path,
                "robotCsg": result.best_robot_csg_path,
                "solverMatcherReport": result.best_match_path,
                "benchmarkMatcherReport": str(bench_match_path),
                "timelineSvg": str(timeline_path),
                "framesJsonl": str(frames_path),
                "video": str(cdir / "rollout_video.mp4"),
                "videoManifest": str(video_manifest_path),
                "log": str(cdir / "run.log"),
            },
        }
    except Exception as exc:
        log += ["ERROR", traceback.format_exc()]
        (cdir / "run.log").write_text("\n".join(log) + "\n", encoding="utf-8")
        return {"case": cname, "target": str(path), "status": "ERROR", "distance": None, "sameQuotientClass": False, "error": repr(exc), "outDir": str(cdir), "artifacts": {"log": str(cdir / "run.log")}}


def run_benchmark(cases: Sequence[Path], out_dir: str | Path, solver_cfg: Optional[SolverConfig] = None, matcher_cfg: Optional[MatcherConfig] = None) -> Json:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    solver_cfg = solver_cfg or SolverConfig()
    matcher_cfg = matcher_cfg or MatcherConfig(same_class_threshold=solver_cfg.same_class_threshold)
    reports = [run_one_case(case, out, i, solver_cfg, matcher_cfg) for i, case in enumerate(cases)]
    passed = sum(1 for r in reports if r.get("status") == "PASS")
    report: Json = {
        "schemaVersion": "csg.benchmark_report.v0",
        "summary": {"total": len(reports), "passed": passed, "failed": len(reports) - passed, "successCriterion": "distance == 0 under unchanged CSG_Matcher"},
        "cases": reports,
    }
    write_json(out / "report.json", report)
    write_json(out / "summary.json", report)
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["case", "status", "distance", "sameQuotientClass", "selectedProgram", "target"])
        writer.writeheader()
        for r in reports:
            writer.writerow({k: r.get(k) for k in writer.fieldnames})
    lines = [
        "# CSG Solver Harness Benchmark Report",
        "",
        f"total: {len(reports)}",
        f"passed: {passed}",
        f"failed: {len(reports) - passed}",
        "",
        "| case | status | distance | selected program |",
        "|---|---:|---:|---|",
    ]
    for r in reports:
        lines.append(f"| {r['case']} | {r['status']} | {r.get('distance')} | {r.get('selectedProgram')} |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run the full CSG Solver Harness benchmark pipeline.")
    p.add_argument("targets", nargs="*", help="Target CSG JSON files.")
    p.add_argument("--target", default=None, help="Single target CSG JSON file.")
    p.add_argument("--cases", nargs="*", default=[], help="Additional target CSG JSON files.")
    p.add_argument("--tests-dir", "--case-dir", dest="case_dir", default=None, help="Directory containing frozen CSG JSON cases.")
    p.add_argument("--out-dir", "--out", default="csg_benchmark_out")
    p.add_argument("--json", action="store_true")
    p.add_argument("--engine", default="symbolic_kinematic")
    p.add_argument("--max-candidates", type=int, default=8)
    p.add_argument("--threshold", type=float, default=1e-9)
    p.add_argument("--preserve-object-ids", action="store_true")
    p.add_argument("--emit-extra-observations", action="store_true")
    p.add_argument("--require-zero", "--strict", action="store_true")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    cases = discover_cases(args.targets, args.cases, args.case_dir, args.target)
    if not cases:
        raise SystemExit("No cases found. Use positional targets, --target, --cases, or --tests-dir.")
    solver_cfg = SolverConfig(
        engine=args.engine,
        max_candidates=args.max_candidates,
        same_class_threshold=args.threshold,
        preserve_object_ids=args.preserve_object_ids,
        emit_extra_observations=args.emit_extra_observations,
    )
    matcher_cfg = MatcherConfig(same_class_threshold=args.threshold)
    report = run_benchmark(cases, args.out_dir, solver_cfg, matcher_cfg)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"report={Path(args.out_dir) / 'report.json'}")
        print(f"passed={report['summary']['passed']} failed={report['summary']['failed']}")
        for case in report["cases"]:
            print(f"{case['case']}: {case['status']} distance={case['distance']}")
    if args.require_zero and any(r.get("status") != "PASS" for r in report["cases"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
