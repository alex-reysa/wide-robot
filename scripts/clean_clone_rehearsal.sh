#!/usr/bin/env bash
# Clean-clone rehearsal — prove a fresh checkout reproduces the symbolic and
# MuJoCo benchmark evidence from scratch.
#
# This is the ONLY shell deliverable: it does the bootstrap that genuinely needs
# a shell (git clone + venv creation + pip install) and then delegates ALL
# benchmark logic to `python -m csg.release_rehearsal`, so there is no command
# drift with the rest of the toolchain.
#
# It clones the local repository at a ref into a throwaway temp dir, installs the
# package editably (so report `sourceProvenance` roots at the clone's .git with
# kind=git and dirty=false), runs the core tests + symbolic benchmark, then — if
# Python 3.12 is available — builds a `.venv-sim`, installs the `sim` extra, and
# runs the full MuJoCo rehearsal + release audit.
#
# Claim boundary: verification discipline on a fixed-base arm, not robot capability.
#
# Usage:
#   scripts/clean_clone_rehearsal.sh [REF] [--symbolic-only] [--seeds N]
#
#   REF              git ref to rehearse (tag, branch, or SHA). Default: HEAD.
#   --symbolic-only  stop after the dependency-free symbolic path (no MuJoCo).
#   --seeds N        randomized seeds for the MuJoCo sweep. Default: 30.
#
# Environment overrides: PYTHON (base interpreter, default python3),
#   SIM_PYTHON (sim interpreter, default python3.12), SEEDS, TMPDIR.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

REF="HEAD"
SYMBOLIC_ONLY=0
SEEDS="${SEEDS:-30}"
BASE_PYTHON="${PYTHON:-python3}"
SIM_PYTHON="${SIM_PYTHON:-python3.12}"

while [ $# -gt 0 ]; do
  case "$1" in
    --symbolic-only) SYMBOLIC_ONLY=1 ;;
    --seeds) SEEDS="$2"; shift ;;
    --seeds=*) SEEDS="${1#*=}" ;;
    -h|--help)
      echo "usage: clean_clone_rehearsal.sh [REF] [--symbolic-only] [--seeds N]"
      exit 0 ;;
    --*) echo "unknown option: $1" >&2; exit 2 ;;
    *) REF="$1" ;;
  esac
  shift
done

WORK="$(mktemp -d "${TMPDIR:-/tmp}/wide-robot-cleanclone.XXXXXX")"
CLONE="$WORK/wide-robot"
OUT="$WORK/out"

echo "[clean-clone] work dir : $WORK"
echo "[clean-clone] source   : $SRC_REPO"
echo "[clean-clone] ref      : $REF"

# 1. Offline clone of the local repo at REF. A clone preserves .git, so reports
#    get Git-backed provenance; a fresh detached checkout is clean (dirty=false).
git clone --quiet "$SRC_REPO" "$CLONE"
git -C "$CLONE" checkout --quiet --detach "$REF"
RESOLVED_COMMIT="$(git -C "$CLONE" rev-parse HEAD)"
echo "[clean-clone] commit   : $RESOLVED_COMMIT"

# 2. Base environment (dependency-free package + pytest). Editable install keeps
#    csg.__file__ rooted at the clone so provenance is Git-backed.
"$BASE_PYTHON" -m venv "$CLONE/.venv"
BASE_PY="$CLONE/.venv/bin/python"
"$BASE_PY" -m pip install --quiet --upgrade pip
( cd "$CLONE" && "$BASE_PY" -m pip install --quiet -e ".[dev]" )

# 3. Core tests + symbolic benchmark (run from the clone).
( cd "$CLONE" && "$BASE_PY" -m pytest tests/ -q )
( cd "$CLONE" && "$BASE_PY" -m csg.benchmark gold_tests --confusion --require-pass --out "$OUT/symbolic" )

if [ "$SYMBOLIC_ONLY" -eq 1 ]; then
  echo "[clean-clone] symbolic-only OK. Artifacts: $OUT/symbolic"
  echo "[clean-clone] commit=$RESOLVED_COMMIT  work dir kept at $WORK"
  exit 0
fi

# 4. Sim environment (Python 3.12 per README; mujoco wheels target CPython <= 3.13).
if ! command -v "$SIM_PYTHON" >/dev/null 2>&1; then
  echo "[clean-clone] '$SIM_PYTHON' not found — symbolic path validated; skipping MuJoCo."
  echo "[clean-clone] install Python 3.12 (or set SIM_PYTHON) to run the full sim rehearsal."
  echo "[clean-clone] commit=$RESOLVED_COMMIT  work dir kept at $WORK"
  exit 0
fi
"$SIM_PYTHON" -m venv "$CLONE/.venv-sim"
SIM_PY="$CLONE/.venv-sim/bin/python"
"$SIM_PY" -m pip install --quiet --upgrade pip
( cd "$CLONE" && "$SIM_PY" -m pip install --quiet -e ".[dev,sim]" )

# 5. Full rehearsal: MuJoCo gold + randomized + comparison + invalid + release audit,
#    driven through csg.release_rehearsal (strict, Git-backed provenance required).
( cd "$CLONE" && "$BASE_PY" -m csg.release_rehearsal \
    --out "$OUT" \
    --python "$BASE_PY" \
    --sim-python "$SIM_PY" \
    --seeds "$SEEDS" \
    --require-final-metadata \
    --project-root "$CLONE" )

echo "[clean-clone] DONE. commit=$RESOLVED_COMMIT"
echo "[clean-clone] result: $OUT/release_rehearsal_result.json"
echo "[clean-clone] work dir kept at $WORK"
