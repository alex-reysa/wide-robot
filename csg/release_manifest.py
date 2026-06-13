#!/usr/bin/env python3
"""Generate a release manifest (and optionally ``RELEASE_SHA256SUMS``).

``release_manifest.json`` pins, for one published release:
  * the version, tag, and the 40-hex commit the tag points at,
  * every distributable asset with its SHA-256 and byte size,
  * the expected benchmark summaries (read from the regenerated reports so they
    cannot drift from reality), and
  * the exact canonical reproduction commands.

It is generated at release time and shipped as a release asset alongside
``RELEASE_SHA256SUMS`` (committing it would be a self-reference paradox: it pins
the commit and asset SHAs, but committing it changes the commit). Hashing reuses
:func:`csg.verify_release.sha256_file` — a single :mod:`hashlib` implementation,
never ``sha256sum``.
"""
from __future__ import annotations

import argparse
import json
import tomllib
from pathlib import Path
from typing import Dict, List

from .common import Json, get_any, load_json, write_json
from .verify_release import (
    VerifyReleaseError,
    locate_report_dirs,
    resolve_tag_commit,
    sha256_file,
)

SUMS_NAME = "RELEASE_SHA256SUMS"
MANIFEST_NAME = "release_manifest.json"


def _pyproject_version(pyproject: str | Path | None = None) -> str | None:
    path = Path(pyproject) if pyproject else Path(__file__).resolve().parents[1] / "pyproject.toml"
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    return get_any(data.get("project", {}) or {}, "version", default=None)


def _asset_files(asset_dir: str | Path, *, exclude: set[str]) -> List[Path]:
    return sorted(p for p in Path(asset_dir).iterdir() if p.is_file() and p.name not in exclude)


def write_sha256sums(asset_dir: str | Path, *, exclude: tuple[str, ...] = (SUMS_NAME, MANIFEST_NAME)) -> Path:
    """Write ``RELEASE_SHA256SUMS`` in the standard two-space format (excludes itself)."""
    files = _asset_files(asset_dir, exclude=set(exclude))
    text = "".join(f"{sha256_file(p)}  {p.name}\n" for p in files)
    out = Path(asset_dir) / SUMS_NAME
    out.write_text(text, encoding="utf-8")
    return out


def collect_assets(asset_dir: str | Path, *, exclude: tuple[str, ...] = (MANIFEST_NAME,)) -> List[Json]:
    """Every distributable asset with its SHA-256 and byte size."""
    return [
        {"name": p.name, "sha256": sha256_file(p), "bytes": p.stat().st_size}
        for p in _asset_files(asset_dir, exclude=set(exclude))
    ]


def _summary(report: Json | None) -> Json:
    summary = (report or {}).get("summary", {}) or {}
    return {
        "total": summary.get("total"),
        "passed": summary.get("passed"),
        "failed": summary.get("failed"),
        "physicalValidity": summary.get("physicalValidity"),
        "leakage": summary.get("leakage"),
    }


def _maybe_load(path: Path) -> Json | None:
    try:
        return load_json(path)
    except (OSError, json.JSONDecodeError):
        return None


def expected_benchmark_summaries(reports_root: str | Path, *, seeds: int = 30) -> Json:
    """Read the per-directory summaries from the regenerated report artifacts."""
    dirs = locate_report_dirs(reports_root, seeds=seeds)
    out: Json = {"seeds": dirs["seeds"]}
    for key, dir_key in (("symbolic", "symbolic_dir"), ("mujoco", "mujoco_dir"), ("randomized", "randomized_dir")):
        out[key] = _summary(_maybe_load(Path(dirs[dir_key]) / "report.json"))
    comparison = _maybe_load(Path(dirs["comparison_dir"]) / "comparison_report.json") or {}
    baselines = comparison.get("baselines", {}) or {}
    out["comparison"] = {
        "baselineOrder": comparison.get("baselineOrder"),
        "baselines": {name: (baseline.get("summary", {}) or {}) for name, baseline in baselines.items()},
    }
    invalid = _maybe_load(Path(dirs["invalid_fixtures_dir"]) / "invalid_fixtures_report.json") or {}
    out["invalid"] = invalid.get("summary")
    return out


def exact_commands(tag: str, *, seeds: int = 30) -> Dict[str, str]:
    """Canonical ``python3 -m csg.X`` reproduction commands for the release."""
    randomized = f"mujoco_randomized_{seeds}"
    return {
        "core_tests": "python3 -m pytest tests/ -q",
        "symbolic_gold": "python3 -m csg.benchmark gold_tests --confusion --require-pass --out <out>/symbolic",
        "mujoco_gold": ".venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco "
                       "--confusion --require-pass --out <out>/mujoco",
        "mujoco_randomized": f".venv-sim/bin/python -m csg.benchmark gold_tests --backend mujoco "
                             f"--confusion --randomized --seeds {seeds} --require-pass --out <out>/{randomized}",
        "backend_comparison": ".venv-sim/bin/python -m csg.benchmark gold_tests "
                              "--compare-backends symbolic,noop,mujoco --confusion --require-pass --out <out>/comparison",
        "invalid_fixtures": ".venv-sim/bin/python -m csg.benchmark --invalid-fixtures gold_invalid "
                            "--require-pass --out <out>/invalid_fixtures",
        "release_rehearsal": f"python3 -m csg.release_rehearsal --out <out> --sim-python .venv-sim/bin/python "
                             f"--seeds {seeds} --require-final-metadata --project-root .",
        "release_audit": f"python3 -m csg.release_audit --symbolic <out>/symbolic --mujoco <out>/mujoco "
                         f"--randomized <out>/{randomized} --comparison <out>/comparison "
                         f"--invalid-fixtures <out>/invalid_fixtures --require-final-metadata --project-root .",
        "clean_clone_rehearsal": f"bash scripts/clean_clone_rehearsal.sh {tag}",
        "verify_release": f"python3 -m csg.verify_release --tag {tag}",
    }


def build_manifest(
    *,
    tag: str,
    commit: str,
    asset_dir: str | Path,
    reports_root: str | Path,
    version: str | None = None,
    seeds: int = 30,
    generated_from: str = "clean-checkout",
) -> Json:
    return {
        "schemaVersion": "csg.release_manifest.v1",
        "version": version or _pyproject_version(),
        "tag": tag,
        "commit": commit,
        "generatedFrom": generated_from,
        "checksumsFile": SUMS_NAME,
        "assets": collect_assets(asset_dir),
        "expectedBenchmarkSummaries": expected_benchmark_summaries(reports_root, seeds=seeds),
        "exactCommands": exact_commands(tag, seeds=seeds),
    }


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate a CSG release manifest.")
    parser.add_argument("--asset-dir", required=True, help="directory holding the release assets")
    parser.add_argument("--reports-root", required=True, help="unpacked report directories (symbolic/, mujoco/, ...)")
    parser.add_argument("--tag", required=True, help="release tag, e.g. v0.3.1")
    parser.add_argument("--version", default=None, help="package version (default: read from pyproject.toml)")
    parser.add_argument("--commit", default=None, help="release commit (default: resolve from tag)")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--out", default=None, help="manifest output path (default: <asset-dir>/release_manifest.json)")
    parser.add_argument("--write-checksums", action="store_true", help="also (re)write RELEASE_SHA256SUMS first")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if args.write_checksums:
        write_sha256sums(args.asset_dir)

    try:
        commit = args.commit or resolve_tag_commit(args.tag)
    except VerifyReleaseError as exc:
        print(f"release-manifest ERROR: {exc}")
        return 3

    manifest = build_manifest(
        tag=args.tag,
        commit=commit,
        asset_dir=args.asset_dir,
        reports_root=args.reports_root,
        version=args.version,
        seeds=args.seeds,
    )
    out = Path(args.out) if args.out else Path(args.asset_dir) / MANIFEST_NAME
    write_json(out, manifest)
    if args.json:
        print(json.dumps(manifest, indent=2, sort_keys=True))
    else:
        print(
            f"release-manifest wrote {out} "
            f"(tag={manifest['tag']} commit={manifest['commit']} assets={len(manifest['assets'])})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
