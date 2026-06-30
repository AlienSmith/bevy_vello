#!/bin/bash
# set_angular_compliance.sh — Batch-update the last float value in all
# "Angular" constraint arrays inside a character JSON's "joints" section.
#
# Usage:
#   ./set_angular_compliance.sh <new_value>            # only v6.character.json
#   ./set_angular_compliance.sh <new_value> --backup   # both .json and _backup.json
#   ./set_angular_compliance.sh <new_value> --dry-run  # show diff without changing
#
# Examples:
#   ./set_angular_compliance.sh 0.1
#   ./set_angular_compliance.sh 0.005 --backup
#   ./set_angular_compliance.sh 0.002 --dry-run
#
# Requires: jq  (install with: sudo apt install jq)

set -euo pipefail

# ── helpers ──────────────────────────────────────────────────────────────
die() { echo "ERROR: $*" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
JSON_FILE="${SCRIPT_DIR}/v6.character.json"
JSON_BACKUP="${SCRIPT_DIR}/v6.character_backup.json"

NEW_VAL="${1:-}"
[[ -z "$NEW_VAL" ]] && die "Usage: $0 <new_number> [--backup] [--dry-run]"

DRY_RUN=false
DO_BACKUP=false
for arg in "${@:2}"; do
  case "$arg" in
    --backup) DO_BACKUP=true ;;
    --dry-run) DRY_RUN=true ;;
    *) die "Unknown flag: $arg" ;;
  esac
done

# Confirm jq is available
command -v jq >/dev/null 2>&1 || die "jq is not installed. Run: sudo apt install jq"

# Validate new value is a number
[[ "$NEW_VAL" =~ ^[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?$ ]] \
  || die "'$NEW_VAL' does not look like a number"

# ── jq filter ────────────────────────────────────────────────────────────
JQ_FILTER='.joints |= map(
  if .config.Angular then
    .config.Angular[-1] = $val
  else . end
)'

process_file() {
  local f="$1"
  [[ ! -f "$f" ]] && { echo "SKIP (not found): $f"; return; }

  if $DRY_RUN; then
    echo "── DRY-RUN: $f ──"
    diff -u "$f" <(jq --argjson val "$NEW_VAL" "$JQ_FILTER" "$f") || true
  else
    jq --argjson val "$NEW_VAL" "$JQ_FILTER" "$f" > "${f}.tmp" \
      && mv "${f}.tmp" "$f"
    echo "UPDATED: $f  (Angular[-1] → $NEW_VAL)"
  fi
}

# ── main ─────────────────────────────────────────────────────────────────
process_file "$JSON_FILE"
$DO_BACKUP && process_file "$JSON_BACKUP"
