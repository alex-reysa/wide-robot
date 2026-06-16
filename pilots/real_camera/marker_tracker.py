#!/usr/bin/env python3
"""Detect fiducial markers in a single frame — **marker observations only**.

Strict scope (per the Phase 3A contract): this module turns one image frame into a list
of :class:`MarkerObservation` (a detected tag id + its pixel corners). It does NOT
estimate world poses, build tracks, decide containment, or make any PASS/FAIL judgement —
those belong to ``video_to_tracks`` and ``verify_episode``. Keeping it this thin means a
test can drive the whole pipeline with a :class:`FakeDetector` and never import OpenCV.

OpenCV is an OPTIONAL dependency (``pip install -e ".[camera]"``): ``cv2`` is imported
lazily *inside* :meth:`ArucoDetector.detect`, and all ``cv2.aruco`` calls are isolated
here behind the :class:`MarkerDetector` protocol so the rest of the pilot (and the test
suite) never touches OpenCV. The ``cv2.aruco`` API moved across 4.6/4.7/4.x, so the one
place that calls it tolerates both the old (``Dictionary_get``/``detectMarkers``) and new
(``getPredefinedDictionary``/``ArucoDetector``) spellings.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Protocol, Sequence


def camera_available() -> bool:
    """True iff the optional camera stack (OpenCV + numpy) is importable. Never raises."""
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401
        return True
    except Exception:
        return False


@dataclass(frozen=True)
class MarkerObservation:
    """One fiducial marker detected in one frame: its id and 4 pixel corners.

    ``corners`` are the 4 image-space [x, y] corners (OpenCV ArUco order). No pose, no
    object role, no world coordinates — that mapping happens later, in ``video_to_tracks``
    using the calibration. ``confidence`` is a detector-side quality in [0, 1].
    """
    marker_id: int
    corners: List[List[float]]
    confidence: float = 1.0

    def to_dict(self) -> Dict[str, Any]:
        return {"markerId": int(self.marker_id),
                "corners": [[float(x), float(y)] for x, y in self.corners],
                "confidence": float(self.confidence)}


class MarkerDetector(Protocol):
    """Detect markers in a single frame. The injection seam the tests use."""

    def detect(self, frame: Any) -> List[MarkerObservation]:
        ...


class ArucoDetector:
    """Real OpenCV AprilTag/Aruco detector. Lazy-imports cv2; smoke-tested only."""

    def __init__(self, dictionary_name: str = "DICT_APRILTAG_36h11", min_confidence: float = 1.0) -> None:
        self.dictionary_name = dictionary_name
        self.min_confidence = float(min_confidence)

    def detect(self, frame: Any) -> List[MarkerObservation]:
        import cv2  # lazy: only the real detector needs OpenCV

        aruco = cv2.aruco
        dict_id = getattr(aruco, self.dictionary_name)
        # Tolerate both the pre-4.7 and 4.7+ ArUco APIs.
        if hasattr(aruco, "getPredefinedDictionary"):
            ar_dict = aruco.getPredefinedDictionary(dict_id)
        else:  # pragma: no cover - old OpenCV
            ar_dict = aruco.Dictionary_get(dict_id)
        if hasattr(aruco, "ArucoDetector"):
            params = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(ar_dict, params)
            corners, ids, _ = detector.detectMarkers(frame)
        else:  # pragma: no cover - old OpenCV
            params = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(frame, ar_dict, parameters=params)

        out: List[MarkerObservation] = []
        if ids is None:
            return out
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = [[float(x), float(y)] for x, y in marker_corners.reshape(-1, 2)]
            out.append(MarkerObservation(int(marker_id), pts, self.min_confidence))
        return out


@dataclass
class FakeDetector:
    """Scripted detector for tests — returns observations per frame WITHOUT OpenCV.

    ``script`` is either a list indexed by frame position, or a callable ``frame ->
    [MarkerObservation]``. This lets the whole video→tracks pipeline run deterministically
    in CI with no cv2 and no real video.
    """
    script: Any
    _calls: int = field(default=0, init=False)

    def detect(self, frame: Any) -> List[MarkerObservation]:
        if callable(self.script):
            return list(self.script(frame))
        i = self._calls
        self._calls += 1
        if isinstance(frame, int) and 0 <= frame < len(self.script):
            return list(self.script[frame])
        return list(self.script[i]) if i < len(self.script) else []
