#!/usr/bin/env python3
"""Verify a published CSG GitHub release end to end.

Downloads the release assets for a tag, verifies ``RELEASE_SHA256SUMS`` by
recomputing SHA-256 with :mod:`hashlib` (never shelling out to ``sha256sum`` —
macOS only ships ``shasum``), unpacks the report-artifacts tarball, reuses
:func:`csg.release_audit.audit_release_artifacts` for content checks, and asserts
that *every* embedded report's ``sourceProvenance.git.commit`` equals the tag's
commit (with ``kind == "git"`` and ``dirty is False``).

The release tag may be annotated (v0.3.0 is): commit resolution peels the tag via
``git rev-list -n 1 <tag>`` so the embedded commit matches.

Exit codes (``main``): 0 ok, 2 verification failed (the release is bad), 3
operational error (tag unresolved, ``gh``/``git`` missing, download/extract
failure). All network/git access is isolated behind small functions so the unit
tests run fully offline.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import re
import subprocess
import tarfile
import tempfile
from pathlib import Path
from typing import Dict, List, Mapping

from .common import Json, get_any
from .release_audit import audit_release_artifacts

REPORT_FILENAMES = (
    "report.json",
    "comparison_report.json",
    "invalid_fixtures_report.json",
    "failure_classification.json",
)

_SUMS_LINE = re.compile(r"^([0-9a-fA-F]{64})[ \t]+\*?(.+)$")
_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


class VerifyReleaseError(Exception):
    """Operational failure (environment/tooling) — distinct from a bad release."""


def _check(checks: List[Json], name: str, ok: bool, message: str) -> None:
    checks.append({"name": name, "ok": bool(ok), "message": message})


# -----------------------------------------------------------------------------
# Hashing / checksum parsing (pure, offline)
# -----------------------------------------------------------------------------


def sha256_file(path: str | Path, *, chunk_size: int = 1 << 20) -> str:
    """Stream a file through SHA-256 and return the lowercase hex digest."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_sha256sums(text: str) -> Dict[str, str]:
    """Parse standard ``sha256sum`` output: ``<64 hex>  <filename>`` per line.

    Tolerates the binary ``*`` marker and a ``./`` prefix; ignores blank lines.
    The checksums file does not list itself. Raises ``ValueError`` on a
    malformed line.
    """
    sums: Dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        match = _SUMS_LINE.match(line)
        if not match:
            raise ValueError(f"malformed sha256sums line: {line!r}")
        name = match.group(2).strip()
        if name.startswith("./"):
            name = name[2:]
        sums[name] = match.group(1).lower()
    return sums


def verify_checksums(asset_dir: str | Path, sums: Mapping[str, str]) -> List[Json]:
    """Recompute and compare SHA-256 for every asset listed in ``sums``."""
    checks: List[Json] = []
    asset_dir = Path(asset_dir)
    if not sums:
        _check(checks, "checksums:entries", False, "RELEASE_SHA256SUMS contained no entries")
        return checks
    for name, expected in sorted(sums.items()):
        path = asset_dir / name
        if not path.is_file():
            _check(checks, f"checksum:{name}", False, f"asset {name} listed in RELEASE_SHA256SUMS is missing")
            continue
        actual = sha256_file(path)
        _check(checks, f"checksum:{name}", actual == expected,
               f"{name} sha256 expected {expected} got {actual}")
    return checks


# -----------------------------------------------------------------------------
# git / gh wrappers (isolated so tests can monkeypatch them)
# -----------------------------------------------------------------------------


def resolve_tag_commit(tag: str, *, cwd: str | Path = ".") -> str:
    """Return the 40-hex commit a tag points at, peeling annotated tags."""
    try:
        out = subprocess.run(
            ["git", "rev-list", "-n", "1", tag],
            cwd=cwd, check=True, text=True, capture_output=True,
        ).stdout.strip()
    except FileNotFoundError as exc:
        raise VerifyReleaseError("git executable not found") from exc
    except subprocess.CalledProcessError as exc:
        raise VerifyReleaseError(
            f"could not resolve commit for tag {tag!r}: {exc.stderr.strip()}"
        ) from exc
    if not _COMMIT_RE.match(out):
        raise VerifyReleaseError(f"unexpected commit {out!r} for tag {tag!r}")
    return out


def default_repo(cwd: str | Path = ".") -> str | None:
    """Derive ``owner/repo`` from the ``origin`` remote, or ``None``."""
    try:
        url = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            cwd=cwd, check=True, text=True, capture_output=True,
        ).stdout.strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    match = re.search(r"[:/]([^/]+/[^/]+?)(?:\.git)?$", url)
    return match.group(1) if match else None


def download_release_assets(tag: str, dest: str | Path, *, repo: str | None = None) -> Dict[str, Path]:
    """Download every asset of a release via ``gh`` into ``dest``."""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    argv = ["gh", "release", "download", tag, "--dir", str(dest), "--clobber"]
    if repo:
        argv += ["--repo", repo]
    try:
        subprocess.run(argv, check=True, text=True, capture_output=True)
    except FileNotFoundError as exc:
        raise VerifyReleaseError("gh executable not found") from exc
    except subprocess.CalledProcessError as exc:
        raise VerifyReleaseError(
            f"gh release download failed for {tag!r}: {exc.stderr.strip()}"
        ) from exc
    return {p.name: p for p in sorted(dest.iterdir()) if p.is_file()}


# -----------------------------------------------------------------------------
# Tarball handling + report discovery
# -----------------------------------------------------------------------------


def _safe_extract(tar: tarfile.TarFile, dest: Path) -> None:
    dest = dest.resolve()
    for member in tar.getmembers():
        target = (dest / member.name).resolve()
        if target != dest and not str(target).startswith(str(dest) + os.sep):
            raise VerifyReleaseError(f"unsafe path in tarball: {member.name!r}")
    try:
        tar.extractall(dest, filter="data")  # Python 3.12+
    except TypeError:
        tar.extractall(dest)  # 3.10/3.11 — members already validated above


def unpack_report_tarball(tarball: str | Path, out: str | Path) -> Path:
    """Safely extract the report-artifacts tarball into ``out``."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with tarfile.open(tarball, "r:*") as tar:
        _safe_extract(tar, out)
    return out


def find_report_tarball(assets: Mapping[str, Path]) -> Path | None:
    """Pick the report-artifacts tarball out of the downloaded assets."""
    tgz = [name for name in assets if name.endswith(".tar.gz")]
    for predicate in (
        lambda n: "report" in n.lower() and "artifact" in n.lower(),
        lambda n: "report" in n.lower(),
    ):
        matches = sorted(name for name in tgz if predicate(name))
        if matches:
            return assets[matches[0]]
    return None


def locate_report_dirs(root: str | Path, *, seeds: int = 30) -> Json:
    """Map the fixed top-level report dirs produced by ``release_rehearsal``.

    Names are stable (``symbolic``, ``mujoco``, ``mujoco_randomized_<N>``,
    ``comparison``, ``invalid_fixtures``); the randomized seed count is read from
    the directory name when it differs from ``seeds``.
    """
    root = Path(root)
    randomized = root / f"mujoco_randomized_{seeds}"
    if not randomized.is_dir():
        candidates = sorted(p for p in root.glob("mujoco_randomized_*") if p.is_dir())
        if candidates:
            randomized = candidates[0]
            match = re.search(r"mujoco_randomized_(\d+)$", randomized.name)
            if match:
                seeds = int(match.group(1))
    return {
        "symbolic_dir": root / "symbolic",
        "mujoco_dir": root / "mujoco",
        "randomized_dir": randomized,
        "comparison_dir": root / "comparison",
        "invalid_fixtures_dir": root / "invalid_fixtures",
        "seeds": seeds,
    }


def iter_report_files(root: str | Path) -> List[Path]:
    """Every provenance-bearing report JSON under ``root`` (incl. nested ones)."""
    root = Path(root)
    found: List[Path] = []
    for name in REPORT_FILENAMES:
        found.extend(sorted(root.rglob(name)))
    return found


def verify_report_commits(root: str | Path, expected_commit: str) -> List[Json]:
    """Assert every report's Git provenance pins ``expected_commit`` cleanly."""
    checks: List[Json] = []
    root = Path(root)
    files = iter_report_files(root)
    if not files:
        _check(checks, "provenance:files", False, "no report JSON files found to verify provenance")
        return checks
    for path in files:
        rel = path.relative_to(root).as_posix()
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            _check(checks, f"provenance:{rel}", False, f"could not read {rel}: {exc}")
            continue
        provenance = get_any(data, "sourceProvenance", default={}) or {}
        kind = get_any(provenance, "kind", default="")
        git = get_any(provenance, "git", default={}) or {}
        commit = get_any(git, "commit", default=None)
        dirty = get_any(git, "dirty", default=None)
        ok = kind == "git" and commit == expected_commit and dirty is False
        _check(checks, f"provenance:{rel}", ok,
               f"{rel} kind={kind} commit={commit} dirty={dirty} "
               f"(expected kind=git commit={expected_commit} dirty=False)")
    return checks


# -----------------------------------------------------------------------------
# Orchestration
# -----------------------------------------------------------------------------


def verify_release(
    tag: str,
    *,
    repo: str | None = None,
    seeds: int = 30,
    work_dir: str | Path | None = None,
    asset_dir: str | Path | None = None,
    download: bool = True,
    expected_commit: str | None = None,
    cwd: str | Path = ".",
) -> Json:
    """Download (or read) release assets and verify checksums, audit, provenance."""
    expected_commit = expected_commit or resolve_tag_commit(tag, cwd=cwd)

    with contextlib.ExitStack() as stack:
        if work_dir is None:
            base = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="csg-verify-release-")))
        else:
            base = Path(work_dir)
            base.mkdir(parents=True, exist_ok=True)

        if asset_dir is not None:
            asset_path = Path(asset_dir)
        else:
            asset_path = base / "assets"
            if download:
                download_release_assets(tag, asset_path, repo=repo)
            else:
                asset_path.mkdir(parents=True, exist_ok=True)
        assets = (
            {p.name: p for p in sorted(asset_path.iterdir()) if p.is_file()}
            if asset_path.is_dir()
            else {}
        )

        checks: List[Json] = []

        sums_path = asset_path / "RELEASE_SHA256SUMS"
        if sums_path.is_file():
            _check(checks, "checksums:RELEASE_SHA256SUMS", True, "RELEASE_SHA256SUMS present")
            sums = parse_sha256sums(sums_path.read_text(encoding="utf-8"))
            checks.extend(verify_checksums(asset_path, sums))
        else:
            _check(checks, "checksums:RELEASE_SHA256SUMS", False, "RELEASE_SHA256SUMS asset is missing")

        audit: Json | None = None
        tarball = find_report_tarball(assets)
        if tarball is None:
            _check(checks, "reports:tarball", False, "report artifacts tarball not found among release assets")
        else:
            _check(checks, "reports:tarball", True, f"report artifacts tarball {Path(tarball).name}")
            reports_root = unpack_report_tarball(tarball, base / "reports")
            dirs = locate_report_dirs(reports_root, seeds=seeds)
            audit = audit_release_artifacts(
                dirs["symbolic_dir"],
                dirs["mujoco_dir"],
                dirs["randomized_dir"],
                dirs["comparison_dir"],
                dirs["invalid_fixtures_dir"],
                seeds=dirs["seeds"],
                require_final_metadata=False,
            )
            checks.extend(audit.get("checks", []))
            checks.extend(verify_report_commits(reports_root, expected_commit))

        failed = [check for check in checks if not check["ok"]]
        return {
            "schemaVersion": "csg.verify_release.v1",
            "tag": tag,
            "repo": repo,
            "expectedCommit": expected_commit,
            "ok": not failed,
            "summary": {
                "checksTotal": len(checks),
                "checksPassed": len(checks) - len(failed),
                "checksFailed": len(failed),
            },
            "checks": checks,
            "audit": audit,
        }


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify a published CSG GitHub release end to end.")
    parser.add_argument("tag", nargs="?", help="release tag, e.g. v0.3.0")
    parser.add_argument("--tag", dest="tag_opt", help="release tag (alternative to positional)")
    parser.add_argument("--repo", default=None, help="GitHub owner/repo (default: derived from origin)")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--work-dir", default=None, help="working directory for downloads/extraction")
    parser.add_argument("--asset-dir", default=None, help="verify a pre-downloaded asset directory (skips gh)")
    parser.add_argument("--no-download", action="store_true", help="do not call gh; read assets from --work-dir/assets")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    tag = args.tag_opt or args.tag
    if not tag:
        parser.error("a release tag is required (positional or --tag)")
    repo = args.repo or default_repo()

    try:
        report = verify_release(
            tag,
            repo=repo,
            seeds=args.seeds,
            work_dir=args.work_dir,
            asset_dir=args.asset_dir,
            download=not args.no_download,
        )
    except VerifyReleaseError as exc:
        print(f"verify-release ERROR: {exc}")
        return 3

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        summary = report["summary"]
        print(
            f"verify-release ok={report['ok']} tag={report['tag']} "
            f"commit={report['expectedCommit']} checks={summary['checksPassed']}/{summary['checksTotal']}"
        )
        for check in report["checks"]:
            if not check["ok"]:
                print(f"  FAIL {check['name']}: {check['message']}")
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
