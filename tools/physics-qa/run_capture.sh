#!/usr/bin/env bash
# Run the game on a private Xvfb display and capture a burst of screenshots.
#
#   run_capture.sh --out DIR [--frames N] [--interval S] [--settle S] [--binary PATH]
#
# The game resolves assets relative to the executable directory
# (target/release/assets), so this script syncs the example asset folder there
# first. Prints a JSON summary on stdout.
#
# The display is private (Xvfb), so no window appears on the developer's screen
# and the capture does not depend on a physical display being attached.
set -uo pipefail

PIPELINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tools/physics-qa -> repository root
REPO="${REPO:-$(cd "$PIPELINE_DIR/../.." && pwd)}"
EXAMPLE="$REPO/examples/collision_detection"
BINARY="$REPO/target/release/collision_detection"
DISPLAY_NUM="${DISPLAY_NUM:-:99}"
SCREEN="${SCREEN:-1280x720x24}"

# Set to 1 to ask the game to suppress particle/VFX layers while capturing.
# The vision judge must see the character silhouette, not explosion effects:
# a burst of red particles covers the body and reads as a physics failure.
CAPTURE_MODE="${CAPTURE_MODE:-1}"

OUT=""
FRAMES=3
INTERVAL=0.6
SETTLE=3.0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    --settle) SETTLE="$2"; shift 2 ;;
    --binary) BINARY="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

[[ -n "$OUT" ]] || { echo "usage: run_capture.sh --out DIR" >&2; exit 2; }
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

if [[ ! -x "$BINARY" ]]; then
  echo "game binary not found: $BINARY" >&2
  echo "build it with: cargo build --release -p collision_detection" >&2
  exit 3
fi

# --- assets: the game loads from <exe_dir>/assets -------------------------
ASSET_DST="$(dirname "$BINARY")/assets"
ASSET_SRC="$EXAMPLE/assets"
if [[ ! -d "$ASSET_SRC" ]]; then
  echo "missing source assets: $ASSET_SRC" >&2; exit 3
fi
mkdir -p "$ASSET_DST"
# Sync only when the source is newer than the stamp, to keep runs cheap.
STAMP="$ASSET_DST/.synced"
if [[ ! -f "$STAMP" || -n "$(find "$ASSET_SRC" -newer "$STAMP" -print -quit 2>/dev/null)" ]]; then
  cp -a "$ASSET_SRC/." "$ASSET_DST/"
  touch "$STAMP"
fi

# --- display --------------------------------------------------------------
XVFB_PID=""
if ! DISPLAY="$DISPLAY_NUM" xdpyinfo >/dev/null 2>&1; then
  XVFB_BIN="$(command -v Xvfb || true)"
  # Bundled fallback so the pipeline does not depend on a system package.
  for cand in "$PIPELINE_DIR/tools/Xvfb" /tmp/xvfb_pkg/usr/bin/Xvfb; do
    [[ -n "$XVFB_BIN" ]] && break
    [[ -x "$cand" ]] && XVFB_BIN="$cand"
  done
  if [[ -z "$XVFB_BIN" ]]; then
    echo "Xvfb not found (install it, or place a binary at $PIPELINE_DIR/tools/Xvfb)" >&2; exit 4
  fi
  "$XVFB_BIN" "$DISPLAY_NUM" -screen 0 "$SCREEN" >"$OUT/xvfb.log" 2>&1 &
  XVFB_PID=$!
  for _ in $(seq 1 40); do
    DISPLAY="$DISPLAY_NUM" xdpyinfo >/dev/null 2>&1 && break
    sleep 0.25
  done
fi

# --- run ------------------------------------------------------------------
# A stale instance on the same display would own the window we capture, so
# refuse to start unless the display is clear of game windows.
STALE="$(DISPLAY="$DISPLAY_NUM" xwininfo -root -tree 2>/dev/null | grep -c 'Vello Study')"
if [[ "$STALE" -gt 0 ]]; then
  echo "{\"ok\":false,\"stage\":\"precheck\",\"error\":\"$STALE stale game window(s) already on $DISPLAY_NUM\"}"
  exit 5
fi

LOG="$OUT/game.log"
# PHYSICS_QA_CAPTURE is the contract the game reads to suppress VFX layers.
DISPLAY="$DISPLAY_NUM" PHYSICS_QA_CAPTURE="$CAPTURE_MODE" "$BINARY" >"$LOG" 2>&1 &
GAME_PID=$!

cleanup() {
  kill "$GAME_PID" 2>/dev/null
  wait "$GAME_PID" 2>/dev/null
  [[ -n "$XVFB_PID" ]] && kill "$XVFB_PID" 2>/dev/null
}
trap cleanup EXIT

# Wait for the game window to appear.
WIN=""
for _ in $(seq 1 120); do
  if ! kill -0 "$GAME_PID" 2>/dev/null; then
    echo "{\"ok\":false,\"stage\":\"startup\",\"error\":\"game exited during startup\",\"log\":\"$LOG\"}"
    exit 1
  fi
  WIN="$(DISPLAY="$DISPLAY_NUM" xwininfo -root -tree 2>/dev/null \
        | grep -o '0x[0-9a-f]* "Vello Study[^"]*"' | head -1 | awk '{print $1}')"
  [[ -n "$WIN" ]] && break
  sleep 0.25
done
if [[ -z "$WIN" ]]; then
  echo "{\"ok\":false,\"stage\":\"window\",\"error\":\"no game window appeared\",\"log\":\"$LOG\"}"
  exit 1
fi

# Let assets load and the simulation settle.
sleep "$SETTLE"

SHOTS=()
for i in $(seq 1 "$FRAMES"); do
  SHOT="$OUT/frame_$(printf '%02d' "$i").png"
  DISPLAY="$DISPLAY_NUM" import -window "$WIN" "$SHOT" 2>/dev/null
  if [[ -s "$SHOT" ]]; then SHOTS+=("$SHOT"); fi
  sleep "$INTERVAL"
done

ALIVE="true"
kill -0 "$GAME_PID" 2>/dev/null || ALIVE="false"

python3 - "$OUT" "$ALIVE" "$LOG" "${SHOTS[@]}" <<'PY'
import json, sys
out, alive, log, *shots = sys.argv[1:]
print(json.dumps({"ok": len(shots) > 0, "alive_after_capture": alive == "true",
                  "frames": shots, "log": log}, indent=2))
PY
