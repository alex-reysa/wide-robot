#!/usr/bin/env bash
# Clean-clone rehearsal — prove a checkout at a ref reproduces the symbolic and
# MuJoCo benchmark evidence from scratch.
#
# This is the ONLY shell deliverable: it does the bootstrap that genuinely needs
# a shell (git clone + venv creation + pip install) and then delegates ALL
# benchmark logic to `python -m csg.release_rehearsal`, so there is no command
# drift with the rest of the toolchain.
#
# It clones a repository at a ref into a throwaway temp dir, installs the package
# editably (so report `sourceProvenance` roots at the clone's .git with kind=git
# and dirty=false), runs the core tests + symbolic benchmark, then — if a sim
# interpreter is available — builds a `.venv-sim`, installs the `sim` extra, and
# runs the full MuJoCo rehearsal + release audit.
#
# Source: by default this clones the *local* repository at REF (a detached
# checkout at a tag is clean, so dirty=false). It does NOT fetch from the public
# GitHub remote unless you pass `--remote <url>`. So it proves the committed
# source reproduces; remote-fresh reproduction is the `--remote` path.
#
# Claim boundary: verification discipline on a fixed-base arm, not robot capability.
#
# Usage:
#   scripts/clean_clone_rehearsal.sh [REF] [options]
#
#   REF                git ref to rehearse (tag, branch, or SHA). Default: HEAD.
#   --symbolic-only    stop after the dependency-free symbolic path (no MuJoCo).
#   --allow-skip-sim   if the sim interpreter is absent, skip MuJoCo (degraded)
#                      instead of failing. Without it, a missing sim is an error.
#   --seeds N          randomized seeds for the MuJoCo sweep. Default: 30.
#   --remote URL       clone from URL instead of the local repository.
#   --keep-work        do not delete the temp work dir on exit.
#   --result-json PATH machine-readable outcome marker. Default:
#                      ./clean_clone_rehearsal_result.json
#
# Exit codes:
#   0   full pass (symbolic + MuJoCo + release audit)
#   10  degraded pass (symbolic-only, or sim skipped via --allow-skip-sim)
#   2   usage error (bad option, missing value, unknown ref)
#   3   sim interpreter required but missing (no --allow-skip-sim)
#   *   any other non-zero: an underlying stage failed (see the result marker)
#
# Environment overrides: PYTHON (base interpreter, default python3),
#   SIM_PYTHON (sim interpreter, default python3.12), SEEDS, TMPDIR.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

REF="HEAD"
SYMBOLIC_ONLY=0
ALLOW_SKIP_SIM=0
KEEP_WORK=0
REMOTE=""
SEEDS="${SEEDS:-30}"
BASE_PYTHON="${PYTHON:-python3}"
SIM_PYTHON="${SIM_PYTHON:-python3.12}"
RESULT_JSON="$PWD/clean_clone_rehearsal_result.json"

# Outcome state (also written to the result marker by the EXIT trap).
WORK=""
RESOLVED_COMMIT=""
MODE="full"
STAGES=""

print_help() {
  sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

die_usage() {
  echo "clean-clone: error: $1" >&2
  echo "run 'scripts/clean_clone_rehearsal.sh --help' for usage" >&2
  exit 2
}

add_stage() { STAGES="${STAGES:+$STAGES }$1"; }

write_result() {
  local rc="$1" arr="" s degraded
  for s in $STAGES; do arr="${arr:+$arr, }\"$s\""; done
  case "$MODE" in full) degraded=false ;; *) degraded=true ;; esac
  local kept; if [ "$KEEP_WORK" -eq 1 ]; then kept=true; else kept=false; fi
  mkdir -p "$(dirname "$RESULT_JSON")" 2>/dev/null || true
  cat > "$RESULT_JSON" <<EOF
{
  "schemaVersion": "csg.clean_clone_rehearsal.v1",
  "ref": "$REF",
  "commit": "$RESOLVED_COMMIT",
  "source": "${REMOTE:-$SRC_REPO}",
  "mode": "$MODE",
  "degraded": $degraded,
  "stagesRun": [$arr],
  "seeds": $SEEDS,
  "exitCode": $rc,
  "workDir": "$WORK",
  "keptWork": $kept
}
EOF
}

on_exit() {
  local rc=$?
  write_result "$rc"
  if [ "$KEEP_WORK" -eq 0 ] && [ -n "$WORK" ] && [ -d "$WORK" ]; then
    rm -rf "$WORK" || true
  fi
  echo "[clean-clone] outcome: exit=$rc mode=$MODE marker=$RESULT_JSON"
  if [ "$KEEP_WORK" -eq 1 ] && [ -n "$WORK" ]; then
    echo "[clean-clone] work dir kept at $WORK"
  fi
}

# ---- Argument parsing (validates values before doing any work) -------------
while [ $# -gt 0 ]; do
  case "$1" in
    --symbolic-only) SYMBOLIC_ONLY=1 ;;
    --allow-skip-sim) ALLOW_SKIP_SIM=1 ;;
    --keep-work) KEEP_WORK=1 ;;
    --remote)
      [ $# -ge 2 ] || die_usage "--remote requires a URL"
      REMOTE="$2"; shift ;;
    --remote=*) REMOTE="${1#*=}" ;;
    --seeds)
      [ $# -ge 2 ] || die_usage "--seeds requires a value"
      case "$2" in ''|*[!0-9]*) die_usage "--seeds must be a non-negative integer, got '$2'" ;; esac
      SEEDS="$2"; shift ;;
    --seeds=*)
      SEEDS="${1#*=}"
      case "$SEEDS" in ''|*[!0-9]*) die_usage "--seeds must be a non-negative integer, got '$SEEDS'" ;; esac ;;
    --result-json)
      [ $# -ge 2 ] || die_usage "--result-json requires a path"
      RESULT_JSON="$2"; shift ;;
    --result-json=*) RESULT_JSON="${1#*=}" ;;
    -h|--help) print_help; exit 0 ;;
    --*) die_usage "unknown option: $1" ;;
    *) REF="$1" ;;
  esac
  shift
done

# Validate REF up front against the clone source so an unknown ref fails clean
# (exit 2) with a clear message — not a confusing git checkout error later.
if [ -z "$REMOTE" ]; then
  if ! git -C "$SRC_REPO" rev-parse --verify --quiet "${REF}^{commit}" >/dev/null 2>&1; then
    die_usage "ref not found in $SRC_REPO: '$REF'"
  fi
fi

CLONE_SRC="${REMOTE:-$SRC_REPO}"

# Everything below is a rehearsal attempt → always leave a result marker.
trap on_exit EXIT

WORK="$(mktemp -d "${TMPDIR:-/tmp}/wide-robot-cleanclone.XXXXXX")"
CLONE="$WORK/wide-robot"
OUT="$WORK/out"

echo "[clean-clone] work dir : $WORK"
echo "[clean-clone] source   : $CLONE_SRC"
echo "[clean-clone] ref      : $REF"

# 1. Clone at REF. A clone preserves .git, so reports get Git-backed provenance;
#    a fresh detached checkout is clean (dirty=false).
git clone --quiet "$CLONE_SRC" "$CLONE"
add_stage clone
if ! git -C "$CLONE" rev-parse --verify --quiet "${REF}^{commit}" >/dev/null 2>&1; then
  die_usage "ref not found in clone of $CLONE_SRC: '$REF'"
fi
git -C "$CLONE" checkout --quiet --detach "$REF"
RESOLVED_COMMIT="$(git -C "$CLONE" rev-parse HEAD)"
echo "[clean-clone] commit   : $RESOLVED_COMMIT"

# 2. Base environment (dependency-free package + pytest). Editable install keeps
#    csg.__file__ rooted at the clone so provenance is Git-backed.
"$BASE_PYTHON" -m venv "$CLONE/.venv"
BASE_PY="$CLONE/.venv/bin/python"
"$BASE_PY" -m pip install --quiet --upgrade pip
( cd "$CLONE" && "$BASE_PY" -m pip install --quiet -e ".[dev]" )
add_stage base_env

# 3. Core tests + symbolic benchmark (run from the clone).
( cd "$CLONE" && "$BASE_PY" -m pytest tests/ -q )
add_stage core_tests
( cd "$CLONE" && "$BASE_PY" -m csg.benchmark gold_tests --confusion --require-pass --out "$OUT/symbolic" )
add_stage symbolic_gold

if [ "$SYMBOLIC_ONLY" -eq 1 ]; then
  echo "[clean-clone] symbolic-only OK (degraded: no MuJoCo). Artifacts: $OUT/symbolic"
  MODE="symbolic-only"
  exit 10
fi

# 4. Sim environment (Python 3.12 per README; mujoco wheels target CPython <= 3.13).
if ! command -v "$SIM_PYTHON" >/dev/null 2>&1; then
  if [ "$ALLOW_SKIP_SIM" -eq 1 ]; then
    echo "[clean-clone] '$SIM_PYTHON' not found — skipping MuJoCo (--allow-skip-sim; degraded)."
    MODE="sim-skipped"
    exit 10
  fi
  echo "[clean-clone] ERROR: sim interpreter '$SIM_PYTHON' not found and --allow-skip-sim not given." >&2
  echo "[clean-clone] install Python 3.12 (or set SIM_PYTHON), or pass --allow-skip-sim / --symbolic-only." >&2
  MODE="sim-missing"
  exit 3
fi
"$SIM_PYTHON" -m venv "$CLONE/.venv-sim"
SIM_PY="$CLONE/.venv-sim/bin/python"
"$SIM_PY" -m pip install --quiet --upgrade pip
( cd "$CLONE" && "$SIM_PY" -m pip install --quiet -e ".[dev,sim]" )
add_stage sim_env

# 5. Full rehearsal: MuJoCo gold + randomized + comparison + invalid + release audit,
#    driven through csg.release_rehearsal (strict, Git-backed provenance required).
( cd "$CLONE" && "$BASE_PY" -m csg.release_rehearsal \
    --out "$OUT" \
    --python "$BASE_PY" \
    --sim-python "$SIM_PY" \
    --seeds "$SEEDS" \
    --require-final-metadata \
    --project-root "$CLONE" )
add_stage full_rehearsal

echo "[clean-clone] DONE (full pass). commit=$RESOLVED_COMMIT"
echo "[clean-clone] rehearsal result: $OUT/release_rehearsal_result.json"
MODE="full"
exit 0
