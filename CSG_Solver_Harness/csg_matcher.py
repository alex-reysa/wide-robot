"""DEPRECATED shim — see ``csg/matcher.py``. Re-exports the new probe-based API."""
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
