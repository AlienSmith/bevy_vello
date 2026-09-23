#!/usr/bin/env bash
# Run the game on a private Xvfb display and capture a burst of screenshots.
#
#   run_capture.sh --out DIR [--frames N] [--interval S] [--settle S] [--binary PATH]
#                  [--input "hold d 3; tap f"] [--hold]
#   run_capture.sh --check
#
# --input injects synthetic X11 input (XTEST, via x11_input.sh) after the settle
# delay and before the capture burst, so a QA run can exercise the character -
# running, firing, aiming - while frames are taken. The game binary itself keeps
# no test hooks: it remains an ordinary, human-playable game.
#
# --hold leaves the game (and the Xvfb, if this script started it) running after
# the burst, reporting their pids in the JSON, so an agent can keep driving the
# game with x11_input.sh. A held game must be killed before the next run.
#
# The game resolves assets relative to the executable directory
# (target/release/assets), so this script syncs the example asset folder there
# first. Prints a JSON summary on stdout.
#
# --check runs the environment checks only and exits without launching anything;
# use it to validate a new machine (see SETUP.md).
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
CHECK_ONLY=0
INPUT_SPEC=""
HOLD=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    --settle) SETTLE="$2"; shift 2 ;;
    --binary) BINARY="$2"; shift 2 ;;
    --input) INPUT_SPEC="$2"; shift 2 ;;
    --hold) HOLD=1; shift ;;
    --check) CHECK_ONLY=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ "$CHECK_ONLY" != "1" ]]; then
  [[ -n "$OUT" ]] || { echo "usage: run_capture.sh --out DIR" >&2; exit 2; }
  mkdir -p "$OUT"
  OUT="$(cd "$OUT" && pwd)"
fi

# --- preflight ------------------------------------------------------------
# Fail early and readably. The expensive mistakes on a fresh machine are silent
# ones: cloning bevy_vello's default branch (upstream v0.4, which has no game in
# it) or study_vello's default branch (which lacks the crates Cargo.toml needs).
# Both produce confusing errors much later, so they are checked up front.
PROBLEMS=()

if [[ ! -d "$EXAMPLE" ]]; then
  hint=""
  if git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
    hint="
      bevy_vello's default branch is 'main' (upstream v0.4) and does not contain
      examples/collision_detection. Check out the right branch:
          git -C $REPO checkout ray_trace"
  fi
  PROBLEMS+=("no game source at $EXAMPLE$hint")
fi

# bevy_vello/Cargo.toml depends on ../study_vello by path, so the sibling has to
# be present and on a branch that actually has these crates.
STUDY="$(cd "$REPO/.." 2>/dev/null && pwd)/study_vello"
for crate in integrations/velato integrations/vello_physics; do
  if [[ ! -f "$STUDY/$crate/Cargo.toml" ]]; then
    PROBLEMS+=("missing $STUDY/$crate/Cargo.toml
      bevy_vello depends on study_vello as a sibling directory (path deps in
      Cargo.toml), and on a branch that contains velato and vello_physics.
      study_vello's default branch is 'study', which does not. Clone it beside
      this repo:
          git clone -b transmission <study_vello-url> $STUDY")
    break
  fi
done

if [[ ! -x "$BINARY" ]]; then
  PROBLEMS+=("game binary not found: $BINARY
      build it with: cargo build --release -p collision_detection")
fi

for tool in xwininfo xdpyinfo import python3; do
  command -v "$tool" >/dev/null 2>&1 || PROBLEMS+=("missing command: $tool
      sudo apt install x11-utils imagemagick python3")
done

if ! python3 -c "import numpy, PIL" >/dev/null 2>&1; then
  PROBLEMS+=("python3 is missing numpy and/or Pillow (needed by metrics.py)
      python3 -m pip install numpy pillow")
fi

# Resolve Xvfb now so the check and the launch below agree on the same binary.
XVFB_BIN="$(command -v Xvfb || true)"
for cand in "$PIPELINE_DIR/tools/Xvfb" "$PIPELINE_DIR/.x11/root/usr/bin/Xvfb" \
            /tmp/xvfb_pkg/usr/bin/Xvfb /tmp/x11get/root/usr/bin/Xvfb; do
  [[ -n "$XVFB_BIN" ]] && break
  [[ -x "$cand" ]] && XVFB_BIN="$cand"
done
# An Xvfb is only needed when the target display is not already up.
if [[ -z "$XVFB_BIN" ]] && ! DISPLAY="$DISPLAY_NUM" xdpyinfo >/dev/null 2>&1; then
  PROBLEMS+=("Xvfb not found, and display $DISPLAY_NUM is not running
      sudo apt install xvfb
      (or place a binary at $PIPELINE_DIR/tools/Xvfb)")
fi

if [[ ${#PROBLEMS[@]} -gt 0 ]]; then
  echo "preflight failed:" >&2
  for p in "${PROBLEMS[@]}"; do echo "  - $p" >&2; done
  echo >&2
  echo "see tools/physics-qa/SETUP.md for the full setup" >&2
  exit 6
fi

if [[ "$CHECK_ONLY" == "1" ]]; then
  echo "preflight ok"
  echo "  repo:    $REPO"
  echo "  example: $EXAMPLE"
  echo "  binary:  $BINARY"
  echo "  study:   $STUDY"
  echo "  xvfb:    ${XVFB_BIN:-<display $DISPLAY_NUM already running>}"
  exit 0
fi

# --- assets: the game loads from <exe_dir>/assets -------------------------
ASSET_DST="$(dirname "$BINARY")/assets"
ASSET_SRC="$EXAMPLE/assets"
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
  if [[ "$HOLD" == "1" ]]; then
    echo "holding: game pid $GAME_PID stays up on $DISPLAY_NUM (kill it when done)" >&2
    return
  fi
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

# Synthetic input goes through X11 (XTEST), so the game keeps no test hooks. On
# a bare Xvfb there is no window manager, so the window must be focused
# explicitly or the keystrokes never reach the game.
if [[ -n "$INPUT_SPEC" ]]; then
  if ! "$PIPELINE_DIR/x11_input.sh" --display "$DISPLAY_NUM" focus; then
    echo "{\"ok\":false,\"stage\":\"input\",\"error\":\"could not focus the game window; is xdotool available? (see SETUP.md)\"}"
    exit 1
  fi
  IFS=';' read -ra INPUT_CMDS <<< "$INPUT_SPEC"
  for cmd in "${INPUT_CMDS[@]}"; do
    # shellcheck disable=SC2086
    "$PIPELINE_DIR/x11_input.sh" --display "$DISPLAY_NUM" $cmd || true
  done
else
  "$PIPELINE_DIR/x11_input.sh" --display "$DISPLAY_NUM" focus >/dev/null 2>&1 || true
fi

SHOTS=()
for i in $(seq 1 "$FRAMES"); do
  SHOT="$OUT/frame_$(printf '%02d' "$i").png"
  DISPLAY="$DISPLAY_NUM" import -window "$WIN" "$SHOT" 2>/dev/null
  if [[ -s "$SHOT" ]]; then SHOTS+=("$SHOT"); fi
  sleep "$INTERVAL"
done

ALIVE="true"
kill -0 "$GAME_PID" 2>/dev/null || ALIVE="false"

python3 - "$OUT" "$ALIVE" "$LOG" "$HOLD" "$GAME_PID" "$DISPLAY_NUM" "${SHOTS[@]}" <<'PY'
import json, sys
out, alive, log, hold, game_pid, display, *shots = sys.argv[1:]
result = {"ok": len(shots) > 0, "alive_after_capture": alive == "true",
          "frames": shots, "log": log}
if hold == "1":
    result["held"] = {"game_pid": int(game_pid), "display": display,
                      "note": "game left running; drive it with x11_input.sh --display "
                              + display + ", then kill pid " + game_pid}
print(json.dumps(result, indent=2))
PY
