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
    from git's content-addressed object store. Facts are then *bound* to it:
      - the distributed ``csg/`` Python source inside every wheel/sdist must be
        byte-identical to that tree (defeats a trojan wheel); a wheel additionally
        may carry no top-level package other than ``csg``;
      - for tarball distributions (the sdist *and* the full ``*-source.tar.gz``)
        EVERY file is bound, not just ``csg/``: a path in the tree must match it
        byte-for-byte and a path absent from the tree is allowed only if it is
        setuptools-generated sdist metadata, so a backdoor placed outside ``csg/``
        (a sibling package, a tampered ``scripts/``/``gold_tests/`` file, a planted
        ``setup.py``) cannot ride along unbound; and
      - every report's ``sourceProvenance.snapshot`` must equal the snapshot
        recomputed from that tree. NB: the snapshot is computable from the public
        source, so it binds *source identity*, not the benchmark *numbers* — a
        report citing the genuine commit with fabricated results would pass this
        check alone. The numbers are bound separately:

  * Deterministic evidence is **re-derived**: the symbolic / noop / invalid-fixture
    benchmarks are re-run from the git-archive tree and their results diffed against
    the published reports (``rederive_evidence``). A fabricated number diverges. This
    is what actually defeats a fabricated report. MuJoCo numbers are machine-dependent
    floats that cannot be re-derived cross-machine; they are instead covered by a CI
    **build-provenance attestation** (``verify_attestation`` + ``ATTESTED_TAGS``)
    binding the assets to the pinned release workflow's OIDC identity. Tags predating
    attestation report their MuJoCo evidence as self-attested, never silently blessed.

``RELEASE_SHA256SUMS`` and ``release_manifest.json`` are publisher-supplied, so
they are treated as *claims to reconcile* — not anchors. Every checksum and
manifest field is cross-checked against the git-anchored facts above and against
the asset bytes recomputed here with :mod:`hashlib` (never shelling out to
``sha256sum`` — macOS only ships ``shasum``).

What this does NOT prove: that a wheel's *compiled metadata / non-source bytes*
were built from the commit. Those are pinned only for transit tamper-evidence
(recomputed SHA-256), not re-derived from source. See README "Reproducibility".

Evidence coverage: a verdict reports which guarantees it actually established —
``evidence.deterministicReDerived`` (symbolic/noop/invalid re-run and matched) and
``evidence.mujocoCoverage`` (``attested`` vs ``self-attested``). NB
``deterministicReDerived`` covers ONLY the deterministic subset; it is *not* a
"numbers verified" stamp over the physics. The MuJoCo/randomized floats — the thing
this benchmark exists to demonstrate — are bound only by a CI attestation
(``ATTESTED_TAGS``); a tag not listed there has its physics numbers *self-attested*,
i.e. taken on the publisher's word. Such a release sets ``evidence.complete=False``,
says so loudly, and — by default — exits 1, not 0, so it can never read as a clean
"all verified" pass. ``--strict`` escalates that incompleteness to a hard failure
(exit 2) for consumers who must gate on fully-bound evidence.

Exit codes (``main``): 0 the release verified AND its evidence is fully bound
(deterministic re-derived *and* MuJoCo CI-attested); 1 every check passed but the
evidence coverage is incomplete (e.g. a self-attested tag whose MuJoCo physics is
not independently verified, or a binding layer was skipped) — a deliberate non-zero
so a partial verification is never mistaken for a full one; 2 the release fails
verification (bad or forged content — a diverging re-derived number, a *refuted*
attestation, or — under ``--strict`` — self-attested/skipped evidence); 3 operational
error (``gh``/``git`` missing, tag/commit unresolved, download / ``git archive`` /
re-derivation failure, an attestation that cannot be *reached* (offline/unauthenticated
``gh``), or a filesystem/environment failure such as an unwritable work dir or
out-of-disk). Hostile or corrupt *release* bytes are classified as 2 and never escape
as a traceback; environment failures (incl. an unreachable attestation) are 3 — being
unable to *complete* a check must never read as "the release is bad".
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
    "v0.3.2": "15094fd156eca7801e109258698c62f223f1d20e",
}

# Tags whose assets were cut + signed *inside GitHub Actions* with a build-provenance
# attestation (``.github/workflows/release.yml``). For these, ``gh attestation verify``
# binds every asset to that CI run's OIDC identity — the trust root for the machine-
# dependent MuJoCo evidence that cannot be re-derived locally. Tags NOT listed here
# predate CI attestation (laptop cut): their MuJoCo numbers are self-attested, and only
# the git-archive source binding + checksums + symbolic re-derivation apply. Add a tag
# here in the same post-release commit that adds it to KNOWN_TAG_COMMITS.
ATTESTED_TAGS: frozenset = frozenset()
# The workflow identity (owner/repo/path) that must have produced an attestation. Pinned
# in committed source so a forged attestation from any other workflow is rejected. The
# trigger ref is per-tag (refs/tags/<tag>); the per-asset digest binding inherent in the
# attestation prevents replaying one release's attestation onto another's assets.
EXPECTED_SIGNER_WORKFLOW = f"{CANONICAL_REPO}/.github/workflows/release.yml"

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


def _tree_all_files(tree: str | Path) -> Dict[str, str]:
    """``{relpath: sha256}`` for *every* file in the git-archive tree.

    This is the trust anchor for binding an sdist / full source tarball in its
    entirety — not just its ``csg/`` package. The source-snapshot binding only
    covers ``SOURCE_PROVENANCE_GLOBS`` (e.g. it misses ``scripts/``, ``.github/``),
    so it cannot, by itself, catch a tampered build script or a planted file
    outside those globs. The git archive *is* the whole committed tree, so hashing
    all of it gives a complete reference. Uses :func:`os.walk` (not ``rglob``) so
    dotfiles/dot-dirs such as ``.github/`` and ``.gitignore`` are always included
    regardless of the Python version's glob semantics. ``__pycache__`` and a stray
    ``.git`` (when a working tree is passed in tests) are skipped."""
    root = Path(tree)
    out: Dict[str, str] = {}
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in ("__pycache__", ".git")]
        for fname in filenames:
            p = Path(dirpath) / fname
            if not p.is_file():  # skip symlinks/specials defensively
                continue
            out[p.relative_to(root).as_posix()] = sha256_file(p)
    return out


# Files that legitimately appear in a setuptools-built *sdist* but are NOT in the
# committed git tree: build-backend-generated metadata. Everything else that is
# absent from the git-archive tree is an unbound smuggled file (a backdoor module,
# a build-time ``setup.py``/``.pth``, a planted script). The allowlist is exact —
# arbitrary ``*.py`` inside an ``.egg-info/`` directory is NOT permitted, so a
# package cannot hide importable code behind a metadata-looking path.
_SDIST_GENERATED_OK = re.compile(
    r"(?:^|/)(?:PKG-INFO|setup\.cfg|MANIFEST\.in)$"
    r"|(?:^|/)[^/]+\.egg-info/(?:PKG-INFO|SOURCES\.txt|dependency_links\.txt|"
    r"entry_points\.txt|requires\.txt|top_level\.txt|not-zip-safe|zip-safe|"
    r"namespace_packages\.txt)$"
)


# The single ``<distribution>-<version>.dist-info`` directory of the canonical wheel.
# Anchored to the ``csg`` distribution name + an exact ``.dist-info`` suffix so a *decoy*
# top-level dir whose name merely *ends* ``.dist-info`` (pip would install it into
# site-packages like any other package) is NOT mistaken for trusted metadata.
_WHEEL_DIST_INFO_RE = re.compile(r"csg-[^/]+\.dist-info")


def _wheel_entry_point_violations(blob: bytes, label: str) -> List[str]:
    """Entry-point declarations that pip would turn into an executable on ``PATH``
    targeting code *outside* the byte-bound ``csg`` package.

    pip materialises every ``[console_scripts]`` / ``[gui_scripts]`` line of a wheel's
    ``entry_points.txt`` into a launcher in ``$PREFIX/bin`` at install time. The
    canonical wheel's targets are all ``csg.*:main`` (pyproject ``[project.scripts]``),
    so any target whose module is not the ``csg`` package can only invoke non-csg code
    and is rejected. A csg-targeted entry can at worst alias genuine, byte-bound code."""
    import configparser
    cp = configparser.ConfigParser(delimiters=("=",), strict=False)
    cp.optionxform = str  # preserve entry-point names verbatim
    try:
        cp.read_string(blob.decode("utf-8", "replace"))
    except configparser.Error:
        return [f"{label}:unparseable-entry-points"]
    bad: List[str] = []
    for section in ("console_scripts", "gui_scripts"):
        if not cp.has_section(section):
            continue
        for ep_name, value in cp.items(section):
            module = value.split(":", 1)[0].strip()
            if module != "csg" and not module.startswith("csg."):
                bad.append(f"{label}[{section}] {ep_name} = {value.strip()}")
    return bad


def _wheel_binding_violations(
    path: str | Path, tree_all: Mapping[str, str] | None,
) -> tuple[List[str], List[str], List[str]]:
    """Bind a wheel's *entire* contents to the ``git archive`` tree, mirroring the
    sdist/source-tarball whole-tree binding. Returns ``(rogue, tree_mismatch, unbound)``.

    The canonical wheel ships only the pure-Python ``csg`` package (byte-bound to the
    archive) plus a single ``csg-<ver>.dist-info`` metadata dir. So every wheel member
    is checked against the archive: a path present in the tree must byte-match (else
    ``tree_mismatch``); a path *absent* from the tree is allowed ONLY if it lives under
    that exact ``.dist-info`` dir (compiled wheel metadata is out of scope — see README
    "Reproducibility" — and is not reconstructable from source). Everything else is
    ``unbound``: a native ``*.so`` / ``*.pyc`` (or ``__pycache__``) dropped into the
    installed ``csg`` package; ANY ``*.data`` install payload (``scripts`` onto PATH,
    ``purelib``/``platlib``/``data`` into site-packages or the prefix); or a decoy
    ``*.data`` / ``*.dist-info`` dir whose stem is not ``csg``. The one metadata file we
    DO bind is ``entry_points.txt`` (see :func:`_wheel_entry_point_violations`): a
    ``[console_scripts]`` target outside ``csg`` becomes a PATH executable → ``rogue``.

    With ``tree_all=None`` (legacy, no archive) the whole-tree layer is unavailable, so
    only ``csg/*.py`` (covered by the separate csg-source byte check) and ``.dist-info``
    are tolerated; any other member is still ``unbound``."""
    rogue: List[str] = []
    tree_mismatch: List[str] = []
    unbound: List[str] = []
    with zipfile.ZipFile(path) as zf:
        for info in zf.infolist():
            if info.is_dir():
                continue
            rel = info.filename
            if not rel:
                continue
            top = rel.split("/", 1)[0]
            if _WHEEL_DIST_INFO_RE.fullmatch(top):
                if rel.rsplit("/", 1)[-1] == "entry_points.txt":
                    rogue.extend(_wheel_entry_point_violations(zf.read(info), rel))
                continue  # other dist-info metadata: out of scope (not in the archive)
            if tree_all is not None and rel in tree_all:
                if hashlib.sha256(zf.read(info)).hexdigest() != tree_all[rel]:
                    tree_mismatch.append(rel)
            elif (tree_all is None and top == "csg" and rel.endswith(".py")
                  and "__pycache__" not in rel.split("/")):
                continue  # legacy: csg/*.py is byte-bound by the csg-source check
            else:
                unbound.append(rel)
    return sorted(rogue), sorted(tree_mismatch), sorted(unbound)


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


def _dist_all_files(path: str | Path) -> Dict[str, str]:
    """``{relpath: sha256}`` for *every* file in a tarball/zip distribution,
    read in memory (no extraction → no symlink/traversal exposure).

    A single common leading directory (the sdist / source-tarball
    ``<name>-<ver>/`` prefix) is stripped so paths line up with the git-archive
    tree. A distribution whose members do not share one top dir (a bare-root
    archive, or one with a planted ``../`` member) is left unstripped, so such a
    member simply fails to match the tree and is reported as unbound."""
    path = Path(path)
    raw: Dict[str, str] = {}
    if path.name.endswith((".whl", ".zip")):
        with zipfile.ZipFile(path) as zf:
            for info in zf.infolist():
                if info.is_dir():
                    continue
                name = info.filename
                if "__pycache__" in name.split("/"):
                    continue
                raw[name] = hashlib.sha256(zf.read(info)).hexdigest()
    else:
        with tarfile.open(path, "r:*") as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    continue
                name = member.name
                if "__pycache__" in name.split("/"):
                    continue
                handle = tar.extractfile(member)
                if handle is None:
                    continue
                raw[name] = hashlib.sha256(handle.read()).hexdigest()
    names = list(raw)
    tops = {n.split("/", 1)[0] for n in names}
    if len(tops) == 1 and all("/" in n for n in names):
        prefix = next(iter(tops)) + "/"
        return {n[len(prefix):]: sha for n, sha in raw.items()}
    return raw


def verify_source_distributions(
    assets: Mapping[str, Path],
    expected_sources: Mapping[str, str],
    *,
    tree_all: Mapping[str, str] | None = None,
    skip: frozenset[str] = frozenset(),
) -> List[Json]:
    """Bind every distribution to the tagged ``git archive`` tree.

    Two layers, so neither a trojan wheel nor a backdoor smuggled *outside*
    ``csg/`` in an sdist / source tarball can pass on a matching checksum:

      * **csg source** — every distribution's ``csg/`` Python source must be
        byte-identical to ``expected_sources`` (recomputed from ``git archive``).
      * **whole-tree** — *every* file is also bound: a path present in the git
        tree must match it byte-for-byte; a path absent from the tree is permitted
        only if it is build-backend metadata. For tarball distributions (sdist +
        full source tarball) the allowlist is setuptools-generated sdist metadata
        (:data:`_SDIST_GENERATED_OK`); for wheels it is the single
        ``csg-<ver>.dist-info`` dir (:func:`_wheel_binding_violations`), whose
        ``entry_points.txt`` is additionally checked so a ``[console_scripts]``
        target outside ``csg`` cannot smuggle a PATH executable. Anything else — a
        sibling package, a planted ``setup.py``/``.pth``, a tampered
        ``scripts/``/``gold_tests/`` file, a native ``*.so`` or ``*.data`` install
        payload in a wheel — is reported as ``unbound`` or ``treeMismatch``.

    ``tree_all`` is the full ``{relpath: sha256}`` of the archive tree
    (:func:`_tree_all_files`); when omitted, the whole-tree layer is skipped
    (legacy csg-only behaviour)."""
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
        is_wheel = name.endswith(".whl")
        # Bind the WHOLE distribution to the archive tree, not just csg/. A backdoor
        # placed outside csg/ (a sibling package, a tampered script, a native *.so /
        # *.pyc dropped into the installed package, an install-time .data/scripts PATH
        # payload, or a rogue console-script entry point) is invisible to the csg-only
        # binding above. ``rogue`` = structurally illegitimate wheel content (bad entry
        # points); ``tree_mismatch`` / ``unbound`` = a tracked path that diverges, or a
        # path absent from the archive that is not allowlisted metadata.
        rogue: List[str] = []
        tree_mismatch: List[str] = []
        unbound: List[str] = []
        try:
            if is_wheel:
                rogue, tree_mismatch, unbound = _wheel_binding_violations(assets[name], tree_all)
            elif tree_all is not None:
                for rel, sha in sorted(_dist_all_files(assets[name]).items()):
                    ref = tree_all.get(rel)
                    if ref is not None:
                        if ref != sha:
                            tree_mismatch.append(rel)
                    elif not _SDIST_GENERATED_OK.search(rel):
                        unbound.append(rel)
        except (zipfile.BadZipFile, tarfile.TarError, EOFError, OSError, ValueError) as exc:
            _check(checks, f"source_dist:{name}", False, f"could not inspect distribution layout {name}: {exc}")
            continue
        ok = not (missing or extra or mismatch or rogue or tree_mismatch or unbound)
        _check(checks, f"source_dist:{name}", ok,
               f"{name} vs git archive: "
               f"missing={missing} extra={extra} mismatch={mismatch} rogueTopLevel={rogue} "
               f"treeMismatch={tree_mismatch} unbound={unbound}")
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
# Evidence re-derivation (F1 residual: bind the *numbers*, not just the source)
# -----------------------------------------------------------------------------
# The snapshot/source bindings prove a report cites the right *commit*; they do
# NOT prove the published benchmark numbers were produced by running that source
# (the snapshot is computable from public bytes). So we re-run the *deterministic*
# benchmarks (symbolic / noop / invalid fixtures — pure stdlib, no sim) from the
# git-archive tree and diff their results against the published reports. A
# fabricated number diverges. MuJoCo evidence is machine-dependent floats and is
# NOT re-derived here — it is covered by CI attestation (see verify_attestation).

# Subdirs whose evidence is deterministic and re-derivable from a base (no-sim)
# checkout. mujoco/ and mujoco_randomized_* are intentionally excluded.
_REDERIVE_STAGES = (
    ("symbolic", ["gold_tests", "--confusion"], "report.json"),
    ("comparison", ["gold_tests", "--compare-backends", "symbolic,noop", "--confusion"], "comparison_report.json"),
    ("invalid_fixtures", ["--invalid-fixtures", "gold_invalid"], "invalid_fixtures_report.json"),
)
# Re-derivable comparison baselines (mujoco is machine-dependent → excluded).
_REDERIVABLE_BASELINES = ("symbolic", "noop")
# Per-case fields that are discrete and stable (no floats / abs paths / config).
_CASE_FIELDS = ("case", "baseCase", "seed", "status", "passed", "leakageClean",
                "matcherPassed", "physicalValidity", "objectOrbitAmbiguous", "vacuous")
_SUMMARY_FIELDS = ("total", "passed", "failed", "failureClassification",
                   "physicalValidity", "leakage")
_CONFUSION_FIELDS = ("knownEquivalentTasks", "matrix", "missedDiagonal",
                     "offDiagonalPasses", "unexpectedOffDiagonalPasses")


def _read_json_or_none(path: Path) -> object:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError):
        return None


def _project_cases(cases: object) -> List[Json]:
    """Whitelist the discrete per-case fields, dropping floats (``distance``),
    absolute paths (``outDir``/``target``) and config so the diff is robust to
    legitimate variance but still catches a flipped PASS/FAIL or category."""
    out: List[Json] = []
    if not isinstance(cases, list):
        return out
    for case in cases:
        if not isinstance(case, Mapping):
            continue
        row = {k: case.get(k) for k in _CASE_FIELDS}
        fc = case.get("failureClassification")
        row["failureCategory"] = fc.get("category") if isinstance(fc, Mapping) else None
        out.append(row)
    return sorted(out, key=lambda r: (str(r.get("case")), str(r.get("seed"))))


def _project_summary(summary: object) -> Json:
    summary = summary if isinstance(summary, Mapping) else {}
    return {k: summary.get(k) for k in _SUMMARY_FIELDS}


def _project_confusion(confusion: object) -> Json:
    confusion = confusion if isinstance(confusion, Mapping) else {}
    return {k: confusion.get(k) for k in _CONFUSION_FIELDS}


def _project_benchmark_report(report: object) -> Json:
    report = report if isinstance(report, Mapping) else {}
    randomized = report.get("randomized")
    randomized = randomized if isinstance(randomized, Mapping) else {}
    return {
        "summary": _project_summary(report.get("summary")),
        "confusion": _project_confusion(report.get("confusion")),
        "randomizedEnabled": bool(randomized.get("enabled")),
        "cases": _project_cases(report.get("cases")),
    }


def _project_comparison_report(report: object) -> Json:
    report = report if isinstance(report, Mapping) else {}
    baselines = report.get("baselines")
    baselines = baselines if isinstance(baselines, Mapping) else {}
    projected: Dict[str, Json] = {}
    for name in _REDERIVABLE_BASELINES:  # mujoco baseline is not re-derived
        b = baselines.get(name)
        b = b if isinstance(b, Mapping) else {}
        projected[name] = {
            "backend": b.get("backend"),
            "baseline": b.get("baseline"),
            "expectedFailure": b.get("expectedFailure"),
            "summary": _project_summary(b.get("summary")),
            "confusion": _project_confusion(b.get("confusion")),
            "cases": _project_cases(b.get("cases")),
        }
    order = report.get("baselineOrder")
    # The published order includes mujoco; compare only the re-derivable prefix.
    if isinstance(order, list):
        order = [b for b in order if b in _REDERIVABLE_BASELINES]
    return {"baselineOrder": order, "baselines": projected}


def _project_invalid_report(report: object) -> Json:
    report = report if isinstance(report, Mapping) else {}
    fixtures = report.get("fixtures")
    fixtures = fixtures if isinstance(fixtures, list) else []
    rows: List[Json] = []
    categories: Dict[str, int] = {}
    for fx in fixtures:
        if not isinstance(fx, Mapping):
            continue
        result = fx.get("result")
        result = result if isinstance(result, Mapping) else {}
        fc = result.get("failureClassification")
        cat = fc.get("category") if isinstance(fc, Mapping) else None
        if isinstance(cat, str):
            categories[cat] = categories.get(cat, 0) + 1
        rows.append({
            "fixtureId": fx.get("fixtureId"),
            "task": fx.get("task"),
            "expectedFailureMatched": fx.get("expectedFailureMatched"),
            "status": result.get("status"),
            "failureCategory": cat,
        })
    rows.sort(key=lambda r: str(r.get("fixtureId")))
    return {
        "summary": report.get("summary"),
        "categories": dict(sorted(categories.items())),
        "fixtures": rows,
    }


def _evidence_projection(root: str | Path) -> Json:
    """Comparison-stable projection of the re-derivable evidence under ``root``."""
    root = Path(root)
    return {
        "symbolic": _project_benchmark_report(_read_json_or_none(root / "symbolic" / "report.json")),
        "comparison": _project_comparison_report(
            _read_json_or_none(root / "comparison" / "comparison_report.json")),
        "invalid_fixtures": _project_invalid_report(
            _read_json_or_none(root / "invalid_fixtures" / "invalid_fixtures_report.json")),
    }


def _run_rederive_stages(interpreter: str, source_tree: Path, out_root: Path) -> None:
    """Run each deterministic benchmark stage from ``source_tree`` into
    ``out_root/<stage>``. Raises :class:`VerifyReleaseError` (operational) if a
    stage cannot run. ``--require-pass`` is deliberately omitted so a non-green run
    still exits 0 (a divergence is judged by the caller's diff, not the exit code)."""
    try:
        out_root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise VerifyReleaseError(f"could not create re-derivation dir: {exc}") from exc
    # Run the tagged code: cwd + PYTHONPATH = the archive tree, so verifying an old
    # tag from a newer checkout re-derives with the *tag's* code, and a stray
    # editable install cannot shadow it. Strip sim-only env (irrelevant for stdlib).
    env = {k: v for k, v in os.environ.items() if k not in ("MUJOCO_GL", "PYTHONPATH")}
    env["PYTHONPATH"] = str(source_tree)
    for name, args, _report in _REDERIVE_STAGES:
        argv = [interpreter, "-m", "csg.benchmark", *args, "--out", str(out_root / name)]
        try:
            subprocess.run(argv, cwd=str(source_tree), env=env, check=True,
                           capture_output=True, text=True)
        except FileNotFoundError as exc:
            raise VerifyReleaseError(f"interpreter {interpreter!r} not found for re-derivation") from exc
        except subprocess.CalledProcessError as exc:
            raise VerifyReleaseError(
                f"re-derivation stage {name} failed (rc={exc.returncode}): "
                f"{(exc.stderr or '').strip()[:500]}"
            ) from exc


def rederive_evidence(
    source_tree: str | Path,
    reports_root: str | Path,
    work_dir: str | Path,
    expected_snapshot: Json,
    *,
    interpreter: str | None = None,
) -> List[Json]:
    """Re-run the deterministic benchmarks from the *tagged* source and diff the
    produced numbers against the published reports.

    A *divergence* is a failed check (release-bad → exit 2). An inability to *run*
    the re-derivation, or proof the wrong source ran, is a :class:`VerifyReleaseError`
    (operational → exit 3). We never pass ``--require-pass`` so a legitimately
    non-green re-derivation still exits 0 and is reported as a divergence, keeping
    the 2-vs-3 split clean.
    """
    checks: List[Json] = []
    interpreter = interpreter or sys.executable
    rd = Path(work_dir) / "rederive"
    _run_rederive_stages(interpreter, Path(source_tree), rd)

    # Prove the subprocess ran the *archived* code: every re-derived report must
    # carry the snapshot digest we recomputed from git archive. A mismatch means a
    # different csg shadowed it — an environment failure, not a release verdict.
    exp_digest = get_any(expected_snapshot or {}, "digest", default=None)
    for name, _args, report_name in _REDERIVE_STAGES:
        data = _read_json_or_none(rd / name / report_name)
        prov = get_any(data or {}, "sourceProvenance", default={}) or {}
        snap = get_any(prov, "snapshot", default={}) or {}
        rd_digest = get_any(snap, "digest", default=None)
        if rd_digest != exp_digest:
            raise VerifyReleaseError(
                f"re-derived {name} ran unexpected source (snapshot {rd_digest} != "
                f"git archive {exp_digest}); cannot bind evidence in this environment")

    published = _evidence_projection(reports_root)
    rederived = _evidence_projection(rd)
    for key in ("symbolic", "comparison", "invalid_fixtures"):
        pub = json.dumps(published.get(key), sort_keys=True)
        red = json.dumps(rederived.get(key), sort_keys=True)
        ok = pub == red
        _check(checks, f"rederive:{key}", ok,
               f"published {key} numbers match re-derivation from git archive" if ok else
               f"published {key} numbers DIVERGE from re-derivation of the tagged source "
               f"(published={pub[:300]} rederived={red[:300]})")
    return checks


# -----------------------------------------------------------------------------
# CI attestation (trust root for the non-re-derivable MuJoCo evidence)
# -----------------------------------------------------------------------------
# Symbolic/noop/invalid evidence is re-derived above. MuJoCo numbers are machine-
# dependent floats that cannot be re-derived cross-machine; instead, releases cut by
# ``.github/workflows/release.yml`` carry a GitHub build-provenance attestation binding
# every asset's bytes to that CI run's OIDC identity (Sigstore-signed). ``gh
# attestation verify`` checks it against the in-source-pinned signer workflow, so a
# publisher cannot mint a passing attestation off a laptop. Tags predating attestation
# are reported (loudly) as self-attested, never silently blessed.


# Classifying a NON-zero ``gh attestation verify`` exit as "the release is bad"
# (refuted → 2) vs "we could not complete the check" (operational → 3) must key on
# structured signals — HTTP status codes and Go transport-error substrings — NOT on
# loose English. The earlier marker list keyed on words like ``signature``/``identity``
# which also appear in TLS handshake errors (a network blip → wrongly "bad"), while the
# real gh "no attestation" message (``HTTP 404: Not Found``) contained none of them (a
# missing attestation → wrongly "operational"). Both inversions broke the fail-closed
# contract the moment ATTESTED_TAGS was populated.
#
# Substrings that mark gh being UNABLE TO REACH/COMPLETE the attestations API — a
# transport/auth/server failure (operational, exit 3). These are stable Go ``net``/
# ``net/http`` error fragments plus GitHub-API auth wording; none of them can appear in
# a Sigstore *verification* verdict, so matching one here never masks a real refutation.
_GH_ATTESTATION_OPERATIONAL_MARKERS = (
    "dial tcp",                            # Go dialer: DNS/connect failure
    "no such host",                        # DNS resolution failure
    "could not resolve host",
    "temporary failure in name resolution",
    "connection refused",
    "connection reset",
    "network is unreachable",
    "no route to host",
    "i/o timeout",
    "operation timed out",
    "context deadline exceeded",
    "client.timeout exceeded",
    "request canceled",
    "tls handshake",                       # TLS *transport*, not a signer-identity verdict
    "handshake timeout",
    "server misbehaving",
    "unexpected eof",
    "requires authentication",             # GitHub API 401 body
    "bad credentials",                     # GitHub API 401 body
    "must be authenticated",
    "gh auth login",                       # gh's own "not logged in" hint
    "no github token",
)
# HTTP status codes that are operational (gh reached the API but it could not serve the
# request: auth, rate-limit, request timeout, or a server-side error). 404 is pointedly
# NOT here: "no attestation exists for these assets" is a *refutation*, not a transport
# failure, and is matched as such ahead of this set in _classify_gh_attestation_failure.
_GH_ATTESTATION_OPERATIONAL_HTTP = frozenset({401, 403, 408, 429})
_HTTP_STATUS_RE = re.compile(r"http (\d{3})\b")


def _classify_gh_attestation_failure(blob: str) -> str:
    """Classify a NON-zero ``gh attestation verify`` result as ``"operational"``
    (gh could not complete the check → exit 3) or ``"refuted"`` (the attestation is
    missing / invalid / identity-mismatched → release-bad, exit 2).

    Operational is the *reserved, enumerated* category: a recognised transport/auth/
    server failure. Everything else — including ``HTTP 404`` (no attestation exists),
    a signer/SAN mismatch, or a Sigstore verification failure — defaults to
    ``"refuted"``. That default is what makes the check **fail-closed**: a non-zero gh
    result that is not a known transport error is treated as a failed verification,
    never silently downgraded to "couldn't check"."""
    low = blob.lower()
    # A definitive "the attestation does not exist" verdict wins outright, so a
    # transient marker that happens to share the same blob cannot mask it. These are the
    # server's verdict (404 / nothing found), NOT a mere "unable to fetch" — a transport
    # error that only references a URL does not trip them.
    if ("http 404" in low or "no attestation" in low or "no attestations" in low
            or "no matching attestation" in low):
        return "refuted"
    if any(m in low for m in _GH_ATTESTATION_OPERATIONAL_MARKERS):
        return "operational"
    for code in _HTTP_STATUS_RE.findall(low):
        n = int(code)
        if n in _GH_ATTESTATION_OPERATIONAL_HTTP or 500 <= n <= 599:
            return "operational"
        # Any other client code (e.g. 422) is a verdict about the attestation, not a
        # transport failure → fall through to fail-closed "refuted".
    return "refuted"


def _gh_attestation_verify(asset: Path, repo: str, signer_workflow: str) -> tuple[str, str]:
    """Run ``gh attestation verify`` for one asset. Returns ``(status, message)`` where
    status is ``"verified"`` (rc 0), ``"refuted"`` (a real verification failure incl. a
    missing/404 attestation → release-bad, exit 2), or ``"operational"`` (could not
    complete the check, e.g. offline/unauthenticated/5xx → exit 3). Raises
    :class:`VerifyReleaseError` only when ``gh`` is entirely absent."""
    argv = ["gh", "attestation", "verify", str(asset),
            "--repo", repo, "--signer-workflow", signer_workflow]
    try:
        proc = subprocess.run(argv, check=False, text=True, capture_output=True)
    except FileNotFoundError as exc:
        raise VerifyReleaseError("gh executable not found (required to verify attestations)") from exc
    blob = (proc.stderr or "") + (proc.stdout or "")
    lines = blob.strip().splitlines()
    msg = lines[-1].strip() if lines else f"rc={proc.returncode}"
    if proc.returncode == 0:
        return "verified", msg
    return _classify_gh_attestation_failure(blob), msg


def verify_attestation(
    asset_dir: str | Path,
    tag: str,
    present_names: frozenset,
    *,
    repo: str,
) -> List[Json]:
    """Verify the CI build-provenance attestation over every distributable asset.

    For a tag in :data:`ATTESTED_TAGS`, every asset must verify against the pinned
    :data:`EXPECTED_SIGNER_WORKFLOW`. Fail-closed: a *missing* (HTTP 404), invalid, or
    identity-mismatched attestation is a failed check → exit 2. Only an attestation we
    cannot *reach* (offline / unauthenticated / rate-limited / 5xx) is operational →
    exit 3, so a network blip never brands a genuine release "bad" (see
    :func:`_classify_gh_attestation_failure`). For any other tag, attestation is not
    expected: a single loud ``attestation:skipped`` check records that the MuJoCo
    evidence is self-attested for that (grandfathered) tag rather than silently passing
    it."""
    checks: List[Json] = []
    if tag not in ATTESTED_TAGS:
        _check(checks, "attestation:skipped", True,
               f"tag {tag} predates CI attestation: MuJoCo/randomized evidence is "
               f"self-attested (laptop cut); source identity + checksums + symbolic "
               f"re-derivation still apply")
        return checks
    asset_dir = Path(asset_dir)
    targets = sorted(present_names)
    if not targets:
        _check(checks, "attestation:assets", False, "no assets present to verify attestation for")
        return checks
    for name in targets:
        status, msg = _gh_attestation_verify(asset_dir / name, repo, EXPECTED_SIGNER_WORKFLOW)
        if status == "operational":
            # Could not complete the check (offline / unauthenticated gh). Operational
            # (exit 3) — do not brand a genuine release "bad" because we are offline.
            raise VerifyReleaseError(
                f"could not verify attestation for {name} (operational, e.g. offline or "
                f"unauthenticated gh): {msg}")
        _check(checks, f"attestation:{name}", status == "verified",
               f"gh attestation verify {name} via {EXPECTED_SIGNER_WORKFLOW}: {msg}")
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
    rederive: bool = True,
    verify_attestations: bool = True,
    strict: bool = False,
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
        # Creating the working directory is an *operational* step: a failure here
        # (unwritable path, out of disk) is an environment problem (exit 3), never a
        # "bad release" (exit 2). Wrap it so the distinction is explicit rather than
        # leaking a bare OSError into main's content backstop.
        try:
            if work_dir is None:
                base = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="csg-verify-release-")))
            else:
                base = Path(work_dir)
                base.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise VerifyReleaseError(f"could not create working directory: {exc}") from exc

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
                try:
                    asset_path.mkdir(parents=True, exist_ok=True)
                except OSError as exc:
                    raise VerifyReleaseError(f"could not create asset directory: {exc}") from exc
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
        expected_tree: Dict[str, str] = {}
        try:
            expected_snapshot = compute_source_snapshot(src)
            expected_sources = _tree_csg_sources(src)
            expected_tree = _tree_all_files(src)
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
                # Re-derive the deterministic evidence from the tagged source and
                # diff the numbers (binds the *results*, not just the source identity).
                if rederive:
                    checks.extend(rederive_evidence(src, reports_root, base, expected_snapshot))

        # --- Source-distribution binding (trojan wheel / sdist / source tarball) ---
        if verify_distributions:
            skip = frozenset(report_tarballs) | {SUMS_NAME, MANIFEST_NAME}
            checks.extend(verify_source_distributions(
                assets, expected_sources, tree_all=expected_tree, skip=skip))

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

        # --- CI attestation (trust root for non-re-derivable MuJoCo evidence) ---
        if verify_attestations:
            checks.extend(verify_attestation(asset_path, tag, present_names, repo=effective_repo))

        # --- Evidence coverage (surface what was *bound* vs self-attested/skipped) ---
        # A self-attested or partially-skipped run must NOT read identically to a
        # fully-verified one. These reflect what actually ran AND passed (so a skipped
        # layer, absent reports, or a diverging re-derivation all read as "not bound"):
        # the deterministic evidence is bound only if re-derivation ran and matched; the
        # machine-dependent MuJoCo evidence is bound only by a passing CI attestation
        # (tag in ATTESTED_TAGS). Anything else leaves the MuJoCo numbers self-attested.
        rederive_checks = [c for c in checks if c["name"].startswith("rederive:")]
        attest_checks = [c for c in checks
                         if c["name"].startswith("attestation:") and c["name"] != "attestation:skipped"]
        deterministic_bound = bool(rederive) and bool(rederive_checks) and all(c["ok"] for c in rederive_checks)
        mujoco_attested = (bool(verify_attestations) and tag in ATTESTED_TAGS
                           and bool(attest_checks) and all(c["ok"] for c in attest_checks))
        evidence_complete = deterministic_bound and mujoco_attested
        # In strict mode an incomplete binding is a hard failure (exit 2) — for
        # consumers who must gate on fully-verified evidence, not self-attestation.
        if strict:
            _check(checks, "evidence:complete", evidence_complete,
                   f"strict: evidence fully bound (deterministic re-derived={deterministic_bound}, "
                   f"mujoco attested={mujoco_attested}); self-attested/skipped evidence fails --strict")

        failed = [check for check in checks if not check["ok"]]
        return {
            "schemaVersion": "csg.verify_release.v1",
            "tag": tag,
            "repo": effective_repo,
            "expectedCommit": expected_commit,
            "ok": not failed,
            "evidence": {
                "deterministicReDerived": deterministic_bound,
                "mujocoCoverage": "attested" if mujoco_attested else "self-attested",
                "complete": evidence_complete,
            },
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
    parser.add_argument("--no-rederive", action="store_true",
                        help="skip re-running the deterministic benchmarks from the tagged "
                             "source to diff the published numbers (evidence binding)")
    parser.add_argument("--no-attestation", action="store_true",
                        help="skip verifying the CI build-provenance attestation over the assets")
    parser.add_argument("--strict", action="store_true",
                        help="fail (exit 2) unless the evidence is fully bound: deterministic "
                             "numbers re-derived AND MuJoCo numbers CI-attested. Without this, a "
                             "self-attested release passes but its incomplete coverage is reported.")
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
            rederive=not args.no_rederive,
            verify_attestations=not args.no_attestation,
            strict=args.strict,
        )
    except VerifyReleaseError as exc:
        print(f"verify-release ERROR: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:  # pragma: no cover
        raise
    except (OSError, MemoryError) as exc:
        # Operational/environment failure (unwritable dir, out of disk, OOM), NOT a
        # bad release. Every release-*bytes* read path inside verify_release is
        # individually wrapped and converted to a failed check, so an OSError that
        # escapes to here is genuinely filesystem/environment — classify it as 3,
        # not 2. Defense in depth behind the explicit wrapping in verify_release.
        print(f"verify-release ERROR (environment): {type(exc).__name__}: {exc}", file=sys.stderr)
        return 3
    except Exception as exc:  # backstop: never a traceback on hostile release bytes
        print(f"verify-release ERROR (release content): {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        summary = report["summary"]
        evidence = report.get("evidence", {})
        print(
            f"verify-release ok={report['ok']} tag={report['tag']} repo={report['repo']} "
            f"commit={report['expectedCommit']} checks={summary['checksPassed']}/{summary['checksTotal']}"
        )
        # Always surface coverage so a self-attested pass never reads like a fully
        # verified one (a green ok=True with MuJoCo unverified must say so, loudly).
        print(
            f"  evidence: deterministic={'re-derived' if evidence.get('deterministicReDerived') else 'NOT re-derived'}, "
            f"mujoco={evidence.get('mujocoCoverage', 'unknown')}, complete={evidence.get('complete')}"
        )
        if not evidence.get("complete"):
            detail = ("MuJoCo/randomized PHYSICS numbers are self-attested (NOT independently "
                      "verified: not re-derived, not CI-attested) — the benchmark's central claim "
                      "is unbound for this tag"
                      if evidence.get("mujocoCoverage") != "attested"
                      else "deterministic re-derivation did not bind the numbers")
            if not evidence.get("deterministicReDerived"):
                detail += "; deterministic re-derivation did not run/pass"
            print(f"  WARNING: evidence coverage INCOMPLETE — {detail}. Exiting 1 (NOT a full "
                  f"verification); --strict escalates to exit 2.")
        for check in report["checks"]:
            if not check["ok"]:
                print(f"  FAIL {check['name']}: {check['message']}")
    # Exit-code contract: 2 if any check failed (bad/forged release, or --strict on
    # incomplete evidence); 1 if every check passed but evidence coverage is incomplete
    # (e.g. MuJoCo self-attested) so the result must NOT read as a full verification; 0
    # only when the release verified AND its evidence is fully bound.
    if not report["ok"]:
        return 2
    if not report.get("evidence", {}).get("complete", False):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
