#!/usr/bin/env python3
"""Convert a reviewed ``rh20t.annotation.v0`` sidecar into ``rh20t.tracks.v0``.

RH20T does not ship ready-made task-object pose tracks, so the first RH20T smoke test
uses a **human-reviewed annotation sidecar** (estimated/depth-backed world poses for the
mover + container in selected frames; see ``pilots/rh20t/annotations_schema.md``). This
module is a thin, fail-closed adapter: it lifts the sidecar's evidence fields into the
``rh20t.tracks.v0`` envelope and validates them with the SAME
:func:`validate_tracks_v0` the rollout door uses. It is source-evidence bookkeeping, not
target authoring — it never reads or touches a target CSG.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Mapping, Optional

from csg.common import Json, load_json, write_json
from pilots.rh20t.tracks_to_rollout import TRACKS_SCHEMA_VERSION, validate_tracks_v0

ANNOTATION_SCHEMA_VERSION = "rh20t.annotation.v0"


class RH20TAnnotationError(ValueError):
    """An ``rh20t.annotation.v0`` sidecar is malformed (wrong schema, missing fields)."""


def annotations_to_tracks(annotation: Mapping[str, Any]) -> Json:
    """Lift a reviewed sidecar into a validated ``rh20t.tracks.v0`` envelope.

    Raises :class:`RH20TAnnotationError` for a wrong/missing schema version, and
    :class:`pilots.rh20t.tracks_to_rollout.RH20TTracksError` (via ``validate_tracks_v0``)
    for structurally-broken evidence (too few frames, missing endpoint poses, non-numeric
    confidence, non-monotonic timestamps, missing object sizes).
    """
    if not isinstance(annotation, Mapping):
        raise RH20TAnnotationError("annotation must be an object")
    if annotation.get("schemaVersion") != ANNOTATION_SCHEMA_VERSION:
        raise RH20TAnnotationError(f"schemaVersion must be {ANNOTATION_SCHEMA_VERSION!r}")
    for key in ("episodeId", "fps", "objects", "frames"):
        if key not in annotation:
            raise RH20TAnnotationError(f"annotation missing {key!r}")
    tracks: Json = {
        "schemaVersion": TRACKS_SCHEMA_VERSION,
        "episodeId": str(annotation["episodeId"]),
        "source": dict(annotation.get("source") or {}),
        "fps": float(annotation["fps"]),
        "objects": list(annotation["objects"]),
        "frames": list(annotation["frames"]),
        "review": dict(annotation.get("review") or {}),
    }
    validate_tracks_v0(tracks)
    return tracks


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Convert rh20t.annotation.v0 into rh20t.tracks.v0")
    parser.add_argument("--annotation", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    tracks = annotations_to_tracks(load_json(Path(args.annotation)))
    write_json(Path(args.out), tracks)
    print(f"rh20t annotations_to_tracks: wrote {args.out} frames={len(tracks['frames'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
