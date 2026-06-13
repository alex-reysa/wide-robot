#!/usr/bin/env python3
"""Verify a published CSG GitHub release against the tagged source.

Trust model
-----------
The verifier's only roots of trust are baked into this committed source file —
they are never read from the release being checked:

  * ``CANONICAL_REPO`` and ``KNOWN_TAG_COMMITS`` pin *who* published and *which*
    40-hex commit each release tag must point at. The download repo defaults to
    the canonical one; the ``origin`` remote is never trusted to decide identity
    (a re-pointed fork cannot redirect verification), and a forged local tag is
    caught because the expected commit comes from the pinned map, not the tag.

  * ``git archive <pinned-commit>`` reconstructs the exact committed source tree
    from git's content-addressed object store. Two independent facts are then
    *bound* to it, neither of which the publisher can mint:
      - every report's ``sourceProvenance.snapshot`` must equal the snapshot
        recomputed from that tree (defeats fabricated reports), and
      - the distributed ``csg/`` Python source inside every wheel/sdist must be
        byte-identical to that tree (defeats a trojan wheel).

``RELEASE_SHA256SUMS`` and ``release_manifest.json`` are publisher-supplied, so
they are treated as *claims to reconcile* — not anchors. Every checksum and
manifest field is cross-checked against the git-anchored facts above and against
the asset bytes recomputed here with :mod:`hashlib` (never shelling out to
``sha256sum`` — macOS only ships ``shasum``).

What this does NOT prove: that a wheel's *compiled metadata / non-source bytes*
were built from the commit. Those are pinned only for transit tamper-evidence
(recomputed SHA-256), not re-derived from source. See README "Reproducibility".

Exit codes (``main``): 0 ok, 2 the release fails verification (bad or forged
content), 3 operational error (``gh``/``git`` missing, tag/commit unresolved,
download or ``git archive`` failure). Hostile or corrupt *release* bytes are
classified as 2 and never escape as a traceback.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import re
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from pathlib import Path
from typing import Dict, List, Mapping

from .common import Json, get_any
from .release_audit import audit_release_artifacts

# ---------------------------------------------------------------------------
# In-source trust anchors (NOT fetched from the release).
# ---------------------------------------------------------------------------
CANONICAL_REPO = "alex-reysa/wide-robot"

# The known-good commit each published tag must resolve to. This is the
# out-of-band anchor: it ships in the committed source you cloned, so a forged
# release/tag cannot change it. Add a line here when you publish a new tag.
KNOWN_TAG_COMMITS: Dict[str, str] = {
    "v0.3.0": "74e893bcb73992bb4ac7b7760516fafc17e48231",
    "v0.3.1": "8e0c6af4ddc09ca18c1d1a9c8dfdac5134fc2dac",
}

REPORT_FILENAMES = (
    "report.json",
    "comparison_report.json",
    "invalid_fixtures_report.json",
    "failure_classification.json",
)
SUMS_NAME = "RELEASE_SHA256SUMS"
MANIFEST_NAME = "release_manifest.json"

# A checksums line is exactly ``<64 hex><space><space|*><name>`` — the only two
# forms GNU ``sha256sum`` / ``shasum -a 256`` emit (text vs binary). Anything
# else (tabs, single space, CRLF) is rejected as malformed.
_SUMS_LINE = re.compile(r"^([0-9a-fA-F]{64}) ([ *])(.+)$")
_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
# Match a ``csg/...py`` package path inside any distribution, ignoring an
# arbitrary leading top-level directory (sdists nest under ``<name>-<ver>/``).
_CSG_PY_RE = re.compile(r"(?:^|/)(csg/(?:[^/]+/)*[^/]+\.py)$")


class VerifyReleaseError(Exception):
    """Operational failure (environment/tooling) — maps to exit 3.

    Distinct from a *bad release*, which is reported as failed checks (exit 2).
    """


class ReleaseContentError(Exception):
    """Malformed / hostile *release* content — caught and turned into a failed
    check (exit 2). Never reaches the CLI as a traceback."""


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

    Tolerates the binary ``*`` marker and a single ``./`` prefix; ignores blank
    lines. The checksums file does not list itself. Raises ``ValueError`` on a
    malformed line, a duplicate logical filename (so a decoy line cannot shadow
    the enforced one), or a filename that is not a plain relative name (so a
    listing cannot escape the asset directory).
    """
    sums: Dict[str, str] = {}
    for raw in text.split("\n"):
        if raw == "":
            continue
        if "\r" in raw or "\t" in raw:
            raise ValueError(f"malformed sha256sums line (stray whitespace): {raw!r}")
        match = _SUMS_LINE.fullmatch(raw)
        if not match:
            raise ValueError(f"malformed sha256sums line: {raw!r}")
        name = match.group(3)
        if name.startswith("./"):
            name = name[2:]
        if "/" in name or "\\" in name or name in (".", "..") or os.path.isabs(name):
            raise ValueError(f"unsafe filename in {SUMS_NAME}: {name!r}")
        if name in sums:
            raise ValueError(f"duplicate filename in {SUMS_NAME}: {name!r}")
        sums[name] = match.group(1).lower()
    return sums


def verify_checksums(asset_dir: str | Path, sums: Mapping[str, str]) -> List[Json]:
    """Recompute and compare SHA-256 for every asset listed in ``sums``."""
    checks: List[Json] = []
    asset_dir = Path(asset_dir)
    if not sums:
        _check(checks, "checksums:entries", False, f"{SUMS_NAME} contained no entries")
        return checks
    for name, expected in sorted(sums.items()):
        # ``name`` is already a validated plain relative name (see parse), so it
        # cannot escape ``asset_dir``; re-assert defensively.
        path = asset_dir / name
        if not path.is_file() or os.path.dirname(name):
            _check(checks, f"checksum:{name}", False, f"asset {name} listed in {SUMS_NAME} is missing")
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


def resolve_remote_tag_commit(repo: str, tag: str) -> str | None:
    """Resolve a tag's commit on the *remote* via ``gh api`` (peeling annotated
    tags). Returns ``None`` if ``gh`` is unavailable/unauthenticated — the
    pinned :data:`KNOWN_TAG_COMMITS` map remains the authoritative anchor."""
    ref = f"repos/{repo}/git/refs/tags/{tag}"
    try:
        out = subprocess.run(
            ["gh", "api", ref], check=True, text=True, capture_output=True
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    try:
        obj = json.loads(out).get("object", {}) or {}
    except json.JSONDecodeError:
        return None
    sha, kind = obj.get("sha"), obj.get("type")
    if kind == "commit" and isinstance(sha, str) and _COMMIT_RE.match(sha):
        return sha
    if kind == "tag" and isinstance(sha, str):  # annotated tag → peel one level
        try:
            tag_obj = subprocess.run(
                ["gh", "api", f"repos/{repo}/git/tags/{sha}"],
                check=True, text=True, capture_output=True,
            ).stdout
            peeled = (json.loads(tag_obj).get("object", {}) or {}).get("sha")
        except (subprocess.CalledProcessError, json.JSONDecodeError):
            return None
        return peeled if isinstance(peeled, str) and _COMMIT_RE.match(peeled) else None
    return None


def origin_repo(cwd: str | Path = ".") -> str | None:
    """Derive ``owner/repo`` from the ``origin`` remote (diagnostic only — this
    is *not* trusted to decide which release to verify).

    Handles SSH (``git@host:owner/repo.git``), HTTPS (``https://host/owner/repo``)
    and nested GHE/GitLab group paths, dropping a trailing ``.git`` and ``/``
    case-insensitively.
    """
    try:
        url = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            cwd=cwd, check=True, text=True, capture_output=True,
        ).stdout.strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    if not url:
        return None
    # Strip a trailing ``.git`` (any case) and trailing slash.
    path = re.sub(r"\.git/?$", "", url, flags=re.IGNORECASE).rstrip("/")
    # Drop the scheme + host: everything up to and including the first ``:`` of
    # an SSH spec or the host segment of a URL.
    scheme = re.match(r"^[A-Za-z][A-Za-z0-9+.\-]*://[^/]+/", path)  # https://host/
    if scheme:
        path = path[scheme.end():]
    else:
        ssh = re.match(r"^[^/@]+@[^/:]+:", path)  # git@host:
        if ssh:
            path = path[ssh.end():]
    path = path.strip("/")
    return path if "/" in path else None


# Backwards-compatible alias (origin is no longer the default trust source).
default_repo = origin_repo


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


def resolve_source_tree(commit: str, dest: str | Path, *, cwd: str | Path = ".") -> Path:
    """Reconstruct the committed source tree at ``commit`` into ``dest``.

    Uses ``git archive``, which reads git's content-addressed object store, so
    the result is anchored to the commit's tree hash — not to anything the
    release publisher controls. This is the keystone of the trust model.
    """
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    try:
        proc = subprocess.run(
            ["git", "archive", "--format=tar", commit],
            cwd=cwd, check=True, capture_output=True,
        )
    except FileNotFoundError as exc:
        raise VerifyReleaseError("git executable not found") from exc
    except subprocess.CalledProcessError as exc:
        raise VerifyReleaseError(
            f"git archive failed for {commit!r}: {exc.stderr.decode('utf-8', 'replace').strip()}"
        ) from exc
    try:
        with tarfile.open(fileobj=io.BytesIO(proc.stdout), mode="r:") as tar:
            # Trusted content (our own commit); extract regular files/dirs only.
            members = [m for m in tar.getmembers() if m.isfile() or m.isdir()]
            try:
                tar.extractall(dest, members=members, filter="data")  # py3.12+
            except TypeError:
                tar.extractall(dest, members=members)
    except (tarfile.TarError, OSError) as exc:
        raise VerifyReleaseError(f"could not unpack git archive for {commit!r}: {exc}") from exc
    return dest


# -----------------------------------------------------------------------------
# Source snapshot (single source of truth: csg.benchmark._source_snapshot)
# -----------------------------------------------------------------------------


def compute_source_snapshot(tree: str | Path) -> Json:
    """The canonical source snapshot over ``tree`` (same globs/algorithm the
    benchmark stamps into every report's ``sourceProvenance``)."""
    from .benchmark import _source_snapshot  # local import: keeps CLI lean / avoids cycles
    return _source_snapshot(Path(tree))


def _aggregate_snapshot_digest(files: object) -> str | None:
    """Recompute the aggregate digest from a report's own ``files`` table.

    Mirrors ``csg.benchmark._source_snapshot`` exactly so a report whose stored
    ``digest`` does not match its own ``files`` list (a tampered table) is
    caught. Returns ``None`` if the table is not the expected shape.
    """
    if not isinstance(files, list):
        return None
    agg = hashlib.sha256()
    for entry in files:
        if not isinstance(entry, Mapping):
            return None
        rel = get_any(entry, "path", default=None)
        sha = get_any(entry, "sha256", default=None)
        if not isinstance(rel, str) or not isinstance(sha, str):
            return None
        agg.update(rel.encode("utf-8"))
        agg.update(b"\0")
        agg.update(sha.encode("ascii", "replace"))
        agg.update(b"\n")
    return agg.hexdigest()


# -----------------------------------------------------------------------------
# Tarball handling + report discovery
# -----------------------------------------------------------------------------


def _safe_extract(tar: tarfile.TarFile, dest: Path) -> None:
    """Extract a *hostile-input* tarball, rejecting anything but regular files
    and directories that stay under ``dest``.

    Symlinks, hardlinks, devices and FIFOs are refused outright (so a symlink
    member plus a write-through file cannot escape the root on any Python — the
    pre-3.12 fallback no longer honours link members), as is any path that
    resolves outside ``dest``. A malicious member is :class:`ReleaseContentError`
    (release-bad → exit 2), not an operational error.
    """
    dest = dest.resolve()
    members = tar.getmembers()
    for member in members:
        if member.issym() or member.islnk():
            raise ReleaseContentError(f"link member not allowed in tarball: {member.name!r}")
        if member.ischr() or member.isblk() or member.isfifo() or member.isdev():
            raise ReleaseContentError(f"special member not allowed in tarball: {member.name!r}")
        if not (member.isfile() or member.isdir()):
            raise ReleaseContentError(f"unexpected member type in tarball: {member.name!r}")
        target = (dest / member.name).resolve()
        if target != dest and not str(target).startswith(str(dest) + os.sep):
            raise ReleaseContentError(f"unsafe path in tarball: {member.name!r}")
    for member in members:
        try:
            tar.extract(member, dest, filter="data")  # py3.12+
        except TypeError:
            tar.extract(member, dest)  # <3.12: members already validated above


def unpack_report_tarball(tarball: str | Path, out: str | Path) -> Path:
    """Safely extract the report-artifacts tarball into ``out``."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with tarfile.open(tarball, "r:*") as tar:
        _safe_extract(tar, out)
    return out


def find_report_tarballs(assets: Mapping[str, Path]) -> List[str]:
    """All assets that look like the report-artifacts tarball (sorted)."""
    return sorted(
        name for name in assets
        if name.endswith((".tar.gz", ".tgz"))
        and "report" in name.lower() and "artifact" in name.lower()
    )


def find_report_tarball(assets: Mapping[str, Path]) -> Path | None:
    """Back-compat single-tarball accessor (returns the first match, if any)."""
    matches = find_report_tarballs(assets)
    return assets[matches[0]] if matches else None


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


def verify_report_commits(
    root: str | Path,
    expected_commit: str,
    *,
    expected_snapshot: Json | None = None,
) -> List[Json]:
    """Assert every report pins ``expected_commit`` cleanly *and*, when an
    ``expected_snapshot`` (recomputed from ``git archive``) is supplied, that the
    report's embedded source snapshot equals it and is internally consistent."""
    checks: List[Json] = []
    root = Path(root)
    files = iter_report_files(root)
    if not files:
        _check(checks, "provenance:files", False, "no report JSON files found to verify provenance")
        return checks
    exp_digest = get_any(expected_snapshot or {}, "digest", default=None)
    exp_count = get_any(expected_snapshot or {}, "fileCount", default=None)
    for path in files:
        rel = path.relative_to(root).as_posix()
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError, ValueError) as exc:
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

        if expected_snapshot is not None:
            snap = get_any(provenance, "snapshot", default={}) or {}
            digest = get_any(snap, "digest", default=None)
            count = get_any(snap, "fileCount", default=None)
            snap_files = get_any(snap, "files", default=None)
            internal = _aggregate_snapshot_digest(snap_files)
            snap_ok = (
                digest == exp_digest
                and count == exp_count
                and internal is not None
                and internal == digest
            )
            _check(checks, f"snapshot:{rel}", snap_ok,
                   f"{rel} snapshot digest={digest} fileCount={count} "
                   f"selfConsistent={internal == digest if internal is not None else False} "
                   f"(expected digest={exp_digest} fileCount={exp_count} bound to git archive)")
    return checks


# -----------------------------------------------------------------------------
# Source-distribution binding (defeats a trojan wheel/sdist)
# -----------------------------------------------------------------------------


def _tree_csg_sources(tree: str | Path) -> Dict[str, str]:
    """``{csg/...py: sha256}`` for the committed source tree."""
    root = Path(tree)
    out: Dict[str, str] = {}
    pkg = root / "csg"
    if not pkg.is_dir():
        return out
    for p in pkg.rglob("*.py"):
        if "__pycache__" in p.parts or not p.is_file():
            continue
        out[p.relative_to(root).as_posix()] = sha256_file(p)
    return out


def _wheel_top_level_violations(path: str | Path) -> List[str]:
    """Top-level wheel entries that are neither the ``csg`` package nor build
    metadata. The canonical wheel ships *only* the ``csg`` package (see
    ``[tool.setuptools] packages`` in pyproject), so a smuggled sibling module —
    a backdoor placed *outside* ``csg/`` that the source-snapshot binding would
    not see — shows up here. Update the allowlist if packaging ever changes."""
    violations: set[str] = set()
    with zipfile.ZipFile(path) as zf:
        for info in zf.infolist():
            name = info.filename
            if not name:
                continue
            top = name.split("/", 1)[0]
            if top == "csg" or top.endswith(".dist-info") or top.endswith(".data"):
                continue
            violations.add(top)
    return sorted(violations)


def _dist_csg_sources(path: str | Path) -> Dict[str, str]:
    """``{csg/...py: sha256}`` for the ``csg`` package inside a wheel or sdist,
    read in memory (no extraction → no symlink/traversal exposure)."""
    path = Path(path)
    out: Dict[str, str] = {}
    if path.name.endswith((".whl", ".zip")):
        with zipfile.ZipFile(path) as zf:
            for info in zf.infolist():
                if info.is_dir():
                    continue
                name = info.filename
                if "__pycache__" in name.split("/"):
                    continue
                m = _CSG_PY_RE.search(name)
                if m:
                    out[m.group(1)] = hashlib.sha256(zf.read(info)).hexdigest()
    else:
        with tarfile.open(path, "r:*") as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    continue
                name = member.name
                if "__pycache__" in name.split("/"):
                    continue
                m = _CSG_PY_RE.search(name)
                if m:
                    handle = tar.extractfile(member)
                    if handle is None:
                        continue
                    out[m.group(1)] = hashlib.sha256(handle.read()).hexdigest()
    return out


def verify_source_distributions(
    assets: Mapping[str, Path],
    expected_sources: Mapping[str, str],
    *,
    skip: frozenset[str] = frozenset(),
) -> List[Json]:
    """Require every distribution's ``csg/`` Python source to be byte-identical
    to ``expected_sources`` (recomputed from ``git archive``). This is what
    makes a trojan wheel fail rather than pass on a matching checksum."""
    checks: List[Json] = []
    expected = dict(expected_sources)
    if not expected:
        _check(checks, "source_dist:expected", False,
               "could not enumerate csg/*.py in the tagged source tree")
        return checks
    candidates = sorted(
        name for name in assets
        if name not in skip and name.endswith((".whl", ".zip", ".tar.gz", ".tgz", ".tar"))
    )
    if not candidates:
        _check(checks, "source_dist:present", False, "no wheel/sdist distribution found among assets")
        return checks
    for name in candidates:
        try:
            got = _dist_csg_sources(assets[name])
        except (zipfile.BadZipFile, tarfile.TarError, EOFError, OSError, ValueError) as exc:
            _check(checks, f"source_dist:{name}", False, f"could not read distribution {name}: {exc}")
            continue
        if not got:
            _check(checks, f"source_dist:{name}", False,
                   f"{name} contains no csg/*.py source (not built from the tagged package?)")
            continue
        missing = sorted(set(expected) - set(got))
        extra = sorted(set(got) - set(expected))
        mismatch = sorted(k for k in expected if k in got and expected[k] != got[k])
        # For wheels (the install artifact), also reject any top-level package
        # smuggled alongside csg/ — a backdoor outside csg/ that the csg-source
        # binding alone would not catch.
        rogue: List[str] = []
        if name.endswith(".whl"):
            try:
                rogue = _wheel_top_level_violations(assets[name])
            except (zipfile.BadZipFile, OSError) as exc:
                _check(checks, f"source_dist:{name}", False, f"could not inspect wheel layout {name}: {exc}")
                continue
        ok = not missing and not extra and not mismatch and not rogue
        _check(checks, f"source_dist:{name}", ok,
               f"{name} csg source vs git archive: "
               f"missing={missing} extra={extra} mismatch={mismatch} rogueTopLevel={rogue}")
    return checks


# -----------------------------------------------------------------------------
# Manifest reconciliation (F12 — the manifest can no longer lie freely)
# -----------------------------------------------------------------------------


def verify_manifest(
    manifest: Json,
    sums: Mapping[str, str],
    present_names: frozenset[str],
    *,
    expected_commit: str,
    observed_summaries: Json | None,
) -> List[Json]:
    """Reconcile the (publisher-supplied) manifest with anchored facts."""
    checks: List[Json] = []
    _check(checks, "manifest:schema",
           get_any(manifest, "schemaVersion", default=None) == "csg.release_manifest.v1",
           f"manifest schemaVersion {get_any(manifest, 'schemaVersion', default=None)}")
    _check(checks, "manifest:commit",
           get_any(manifest, "commit", default=None) == expected_commit,
           f"manifest commit {get_any(manifest, 'commit', default=None)} (expected {expected_commit})")

    manifest_assets = get_any(manifest, "assets", default=[]) or []
    declared = {get_any(a, "name", default=""): get_any(a, "sha256", default=None) for a in manifest_assets}

    # Every checksum line must be backed by an identical manifest sha256.
    for name, digest in sorted(sums.items()):
        _check(checks, f"manifest:checksum:{name}", declared.get(name) == digest,
               f"manifest sha256 for {name} = {declared.get(name)} (sums say {digest})")

    # The set the sums enforce must equal the manifest's asset set minus the
    # self-excluded checksums file (which the sums file never lists itself).
    expected_sums_keys = set(declared) - {SUMS_NAME}
    _check(checks, "manifest:checksum_set", set(sums) == expected_sums_keys,
           f"manifest assets (minus {SUMS_NAME}) = {sorted(expected_sums_keys)}; sums list {sorted(sums)}")

    # The files actually present must be exactly the manifest's assets plus the
    # manifest itself (which excludes itself). No unlisted extras, none missing.
    expected_present = set(declared) | {MANIFEST_NAME}
    extra = sorted(present_names - expected_present)
    missing = sorted(expected_present - present_names)
    _check(checks, "manifest:asset_set", not extra and not missing,
           f"present-vs-manifest asset set: unlisted={extra} missing={missing}")

    if observed_summaries is not None:
        declared_summaries = get_any(manifest, "expectedBenchmarkSummaries", default=None)
        _check(checks, "manifest:summaries", declared_summaries == observed_summaries,
               "manifest expectedBenchmarkSummaries match the regenerated report summaries"
               if declared_summaries == observed_summaries
               else "manifest expectedBenchmarkSummaries disagree with the observed report summaries")
    return checks


# -----------------------------------------------------------------------------
# Orchestration
# -----------------------------------------------------------------------------


def verify_release(
    tag: str,
    *,
    repo: str | None = None,
    allow_untrusted_repo: bool = False,
    seeds: int = 30,
    work_dir: str | Path | None = None,
    asset_dir: str | Path | None = None,
    download: bool = True,
    expected_commit: str | None = None,
    remote_commit: str | None = None,
    source_tree: str | Path | None = None,
    verify_distributions: bool = True,
    require_manifest: bool = True,
    cwd: str | Path = ".",
) -> Json:
    """Download (or read) release assets and verify them against the tagged
    source. Release-content problems are recorded as failed checks; only
    environment/tooling problems raise :class:`VerifyReleaseError`."""
    checks: List[Json] = []

    # --- Identity: which repo, which commit (anchored, origin not trusted) ---
    effective_repo = repo or CANONICAL_REPO
    if repo:  # explicit override
        ok_repo = (effective_repo == CANONICAL_REPO) or allow_untrusted_repo
        _check(checks, "identity:repo", ok_repo,
               f"verifying repo {effective_repo} (canonical {CANONICAL_REPO}; "
               f"override {'allowed' if allow_untrusted_repo else 'requires --allow-untrusted-repo'})")
    else:
        _check(checks, "identity:repo", True, f"verifying canonical repo {CANONICAL_REPO} (origin remote not trusted)")

    if expected_commit is None:
        pinned = KNOWN_TAG_COMMITS.get(tag)
        local_commit = resolve_tag_commit(tag, cwd=cwd)
        if pinned is not None:
            expected_commit = pinned
            _check(checks, "identity:tag_commit", local_commit == pinned,
                   f"local tag {tag} resolves to {local_commit} (pinned known-good {pinned})")
        else:
            expected_commit = local_commit
            _check(checks, "identity:tag_commit", True,
                   f"tag {tag} not in known-good map; trusting local resolution {local_commit} "
                   f"(reports + source are still bound to this commit's git archive)")

    # Best-effort cross-check against the remote tag (when gh resolved it).
    if remote_commit is not None:
        _check(checks, "identity:remote_commit", remote_commit == expected_commit,
               f"gh-resolved remote tag commit {remote_commit} (expected {expected_commit})")

    with contextlib.ExitStack() as stack:
        if work_dir is None:
            base = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="csg-verify-release-")))
        else:
            base = Path(work_dir)
            base.mkdir(parents=True, exist_ok=True)

        # --- Acquire assets ---
        if asset_dir is not None:
            asset_path = Path(asset_dir)
        else:
            asset_path = base / "assets"
            if download:
                downloaded = download_release_assets(tag, asset_path, repo=effective_repo)
                if not downloaded:
                    raise VerifyReleaseError(f"gh download for {tag!r} produced no assets")
            else:
                asset_path.mkdir(parents=True, exist_ok=True)
        assets = (
            {p.name: p for p in sorted(asset_path.iterdir()) if p.is_file()}
            if asset_path.is_dir()
            else {}
        )
        present_names = frozenset(assets)

        # --- Resolve the anchored source tree (git archive of the commit) ---
        expected_snapshot: Json | None = None
        expected_sources: Dict[str, str] = {}
        if source_tree is None:
            src = resolve_source_tree(expected_commit, base / "source", cwd=cwd)
        else:
            src = Path(source_tree)
        try:
            expected_snapshot = compute_source_snapshot(src)
            expected_sources = _tree_csg_sources(src)
            _check(checks, "source:snapshot", bool(expected_snapshot.get("digest")),
                   f"recomputed source snapshot from git archive {expected_commit}: "
                   f"{expected_snapshot.get('algorithm')}:{expected_snapshot.get('digest')} "
                   f"({expected_snapshot.get('fileCount')} files)")
        except (OSError, ValueError) as exc:  # source tree unreadable
            raise VerifyReleaseError(f"could not compute source snapshot: {exc}") from exc

        # --- Assets fetched? (positive assertion; F10) ---
        report_tarballs = find_report_tarballs(assets)
        sums_present = (asset_path / SUMS_NAME).is_file()
        _check(checks, "assets:fetched", sums_present and bool(report_tarballs),
               f"fetched {len(assets)} assets (sums={'yes' if sums_present else 'no'}, "
               f"report tarballs={len(report_tarballs)})")

        # --- Checksums ---
        sums: Dict[str, str] = {}
        sums_path = asset_path / SUMS_NAME
        if sums_path.is_file():
            _check(checks, "checksums:RELEASE_SHA256SUMS", True, f"{SUMS_NAME} present")
            try:
                sums = parse_sha256sums(sums_path.read_text(encoding="utf-8"))
                checks.extend(verify_checksums(asset_path, sums))
            except (ValueError, OSError) as exc:
                _check(checks, "checksums:parse", False, f"{SUMS_NAME} could not be parsed: {exc}")
        else:
            _check(checks, "checksums:RELEASE_SHA256SUMS", False, f"{SUMS_NAME} asset is missing")

        # --- Report tarball (exactly one) → audit + provenance + snapshot ---
        audit: Json | None = None
        observed_summaries: Json | None = None
        if not report_tarballs:
            _check(checks, "reports:tarball", False, "report artifacts tarball not found among release assets")
        elif len(report_tarballs) > 1:
            _check(checks, "reports:tarball", False,
                   f"multiple report tarballs present (decoy risk): {report_tarballs}")
        else:
            tarball_name = report_tarballs[0]
            _check(checks, "reports:tarball", True, f"report artifacts tarball {tarball_name}")
            try:
                reports_root = unpack_report_tarball(assets[tarball_name], base / "reports")
            except (ReleaseContentError, tarfile.TarError, EOFError, OSError) as exc:
                _check(checks, "reports:extract", False, f"could not extract {tarball_name}: {exc}")
                reports_root = None
            if reports_root is not None:
                dirs = locate_report_dirs(reports_root, seeds=seeds)
                try:
                    audit = audit_release_artifacts(
                        dirs["symbolic_dir"], dirs["mujoco_dir"], dirs["randomized_dir"],
                        dirs["comparison_dir"], dirs["invalid_fixtures_dir"],
                        seeds=dirs["seeds"], require_final_metadata=False,
                    )
                    checks.extend(audit.get("checks", []))
                except (json.JSONDecodeError, ValueError, OSError) as exc:
                    _check(checks, "reports:audit", False, f"release audit could not run on the reports: {exc}")
                checks.extend(verify_report_commits(reports_root, expected_commit, expected_snapshot=expected_snapshot))
                try:
                    from . import release_manifest as _rm  # local import: avoids import cycle
                    observed_summaries = _rm.expected_benchmark_summaries(reports_root, seeds=dirs["seeds"])
                except (json.JSONDecodeError, ValueError, OSError):
                    observed_summaries = None

        # --- Source-distribution binding (trojan wheel defeat) ---
        if verify_distributions:
            skip = frozenset(report_tarballs) | {SUMS_NAME, MANIFEST_NAME}
            checks.extend(verify_source_distributions(assets, expected_sources, skip=skip))

        # --- Manifest reconciliation ---
        manifest_path = asset_path / MANIFEST_NAME
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, ValueError, OSError) as exc:
                _check(checks, "manifest:parse", False, f"{MANIFEST_NAME} could not be parsed: {exc}")
            else:
                checks.extend(verify_manifest(
                    manifest, sums, present_names,
                    expected_commit=expected_commit, observed_summaries=observed_summaries,
                ))
        elif require_manifest:
            _check(checks, "manifest:present", False, f"{MANIFEST_NAME} asset is missing")

        failed = [check for check in checks if not check["ok"]]
        return {
            "schemaVersion": "csg.verify_release.v1",
            "tag": tag,
            "repo": effective_repo,
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


# NB: release_manifest imports this module, so verify_release imports it lazily
# (inside the manifest-reconciliation block) to avoid an import cycle.


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify a published CSG GitHub release against the tagged source.")
    parser.add_argument("tag", nargs="?", help="release tag, e.g. v0.3.1")
    parser.add_argument("--tag", dest="tag_opt", help="release tag (alternative to positional)")
    parser.add_argument("--repo", default=None,
                        help=f"GitHub owner/repo to verify (default: pinned canonical {CANONICAL_REPO})")
    parser.add_argument("--allow-untrusted-repo", action="store_true",
                        help="permit --repo to name a non-canonical repo (forks); trust anchors will not apply")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--work-dir", default=None, help="working directory for downloads/extraction")
    parser.add_argument("--asset-dir", default=None, help="verify a pre-downloaded asset directory (skips gh)")
    parser.add_argument("--no-download", action="store_true", help="do not call gh; read assets from --work-dir/assets")
    parser.add_argument("--no-verify-distributions", action="store_true",
                        help="skip the wheel/sdist source-vs-archive binding check")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    tag = args.tag_opt or args.tag
    if not tag:
        parser.error("a release tag is required (positional or --tag)")

    download = not args.no_download and args.asset_dir is None
    # When we have gh in hand anyway (a real download), cross-check the tag
    # commit against the remote as defense-in-depth (best-effort; offline-safe).
    remote_commit = resolve_remote_tag_commit(args.repo or CANONICAL_REPO, tag) if download else None

    try:
        report = verify_release(
            tag,
            repo=args.repo,
            allow_untrusted_repo=args.allow_untrusted_repo,
            seeds=args.seeds,
            work_dir=args.work_dir,
            asset_dir=args.asset_dir,
            download=download,
            remote_commit=remote_commit,
            verify_distributions=not args.no_verify_distributions,
        )
    except VerifyReleaseError as exc:
        print(f"verify-release ERROR: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:  # pragma: no cover
        raise
    except Exception as exc:  # backstop: never a traceback on hostile release bytes
        print(f"verify-release ERROR (release content): {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        summary = report["summary"]
        print(
            f"verify-release ok={report['ok']} tag={report['tag']} repo={report['repo']} "
            f"commit={report['expectedCommit']} checks={summary['checksPassed']}/{summary['checksTotal']}"
        )
        for check in report["checks"]:
            if not check["ok"]:
                print(f"  FAIL {check['name']}: {check['message']}")
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
