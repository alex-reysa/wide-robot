"""DEPRECATED shim — the canonical matcher now lives in ``csg/matcher.py``.

The old 1416-line weighted-distance matcher had an empty honest-zero set and
several confirmed bugs (see the audit). It is replaced by the probe-based
checker in the ``csg`` package. This shim re-exports the new API so existing
imports / CLI invocations keep working. New code should ``import csg.matcher``.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from csg.matcher import (  # noqa: E402,F401
    MatchResult,
    MatcherConfig,
    main,
    match,
    match_csg_files,
    match_csg_json,
)

if __name__ == "__main__":
    main()
