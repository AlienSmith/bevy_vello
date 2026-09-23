#!/usr/bin/env bash
# x11_input.sh - inject synthetic keyboard/mouse input into the game via X11 (XTEST).
#
# Why X11 input instead of scripted input inside the Rust code:
#   It is purely additive. The game binary stays a normal, human-playable game -
#   no test hooks, no feature flags, no second input path to maintain. The same
#   mechanism works against a private Xvfb display for automated capture, or
#   against a real desktop if a human and an agent ever play side by side.
#
# Requirements:
#   - an X server with the XTEST extension (Xvfb qualifies)
#   - xdotool. It is often NOT installed system-wide; set XDOTOOL=/path/to/xdotool
#     or run x11_setup.sh, which unpacks it into tools/physics-qa/.x11/root.
#
# Usage:
#   x11_input.sh [--display :99] [--window NAME] [--force] <command> [args]
#
# Commands:
#   focus                 find and focus the game window
#   geometry              print the game window geometry
#   tap <key>             press and release, e.g. tap f
#   hold <key> <seconds>  press, wait, release, e.g. hold d 3
#   click <x> <y>         move the pointer and click button 1
#   move <x> <y>          move the pointer only (the game aims at the cursor)
#
# Keys use xdotool syntax: w a s d f v, or names like Return, space, Up.
#
# Safety: a private Xvfb has one or two windows. A developer's real desktop has
# many. If the target display looks like a real desktop, the injecting commands
# refuse unless --force is given, because synthetic keystrokes would land in
# whatever window the human currently has focused.
set -uo pipefail

PIPELINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPLAY_ARG=""
WINDOW_NAME="Vello"
FORCE=0

usage() { sed -n '2,40p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --display) DISPLAY_ARG="$2"; shift 2 ;;
    --display=*) DISPLAY_ARG="${1#*=}"; shift ;;
    --window) WINDOW_NAME="$2"; shift 2 ;;
    --window=*) WINDOW_NAME="${1#*=}"; shift ;;
    --force) FORCE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) break ;;
  esac
done

COMMAND="${1:-}"
shift || true

case "$COMMAND" in
  ""|help|-h|--help) usage; exit 0 ;;
esac

# --- locate xdotool -------------------------------------------------------
XDOTOOL="${XDOTOOL:-}"
if [[ -z "$XDOTOOL" || ! -x "$XDOTOOL" ]]; then
  XDOTOOL=""
  for cand in "$(command -v xdotool || true)" \
              "$PIPELINE_DIR/.x11/root/usr/bin/xdotool" \
              /tmp/x11get/root/usr/bin/xdotool; do
    [[ -n "$cand" && -x "$cand" ]] && { XDOTOOL="$cand"; break; }
  done
fi
if [[ -z "$XDOTOOL" ]]; then
  echo "xdotool not found. run tools/physics-qa/x11_setup.sh, or set XDOTOOL." >&2
  exit 6
fi

# libxdo sits beside xdotool when unpacked from a .deb; teach the loader where.
XDO_LIB="$(dirname "$(dirname "$XDOTOOL")")/lib/x86_64-linux-gnu"
[[ -d "$XDO_LIB" ]] && export LD_LIBRARY_PATH="$XDO_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

[[ -n "$DISPLAY_ARG" ]] && export DISPLAY="$DISPLAY_ARG"
if ! xdpyinfo >/dev/null 2>&1; then
  echo "display ${DISPLAY:-<unset>} is unreachable" >&2
  exit 5
fi
if ! xdpyinfo 2>/dev/null | grep -qi XTEST; then
  echo "display ${DISPLAY:-} has no XTEST extension; synthetic input is unavailable" >&2
  exit 6
fi

find_window() { "$XDOTOOL" search --name "$WINDOW_NAME" 2>/dev/null | head -1 || true; }
WID="$(find_window)"
if [[ -z "$WID" ]]; then
  echo "no window matching '$WINDOW_NAME' on ${DISPLAY:-}" >&2
  exit 1
fi

case "$COMMAND" in
  focus)
    exec "$XDOTOOL" windowfocus "$WID" ;;
  geometry)
    exec "$XDOTOOL" getwindowgeometry "$WID" ;;
esac

# Refuse to type into what looks like a real desktop.
NWIN="$(xwininfo -root -children 2>/dev/null | grep -c ' 0x' || true)"
if [[ "$NWIN" -gt 12 && "$FORCE" != "1" ]]; then
  echo "display ${DISPLAY:-} has $NWIN root children - it does not look like a private" >&2
  echo "Xvfb. Refusing to inject input without --force, because synthetic keys would" >&2
  echo "land in whatever the human has focused right now." >&2
  exit 6
fi

"$XDOTOOL" windowfocus "$WID"

case "$COMMAND" in
  tap)
    [[ $# -ge 1 ]] || { echo "tap <key>" >&2; exit 2; }
    exec "$XDOTOOL" key "$1" ;;
  hold)
    [[ $# -ge 1 ]] || { echo "hold <key> <seconds>" >&2; exit 2; }
    "$XDOTOOL" keydown "$1"
    sleep "${2:-1}"
    exec "$XDOTOOL" keyup "$1" ;;
  click)
    [[ $# -ge 2 ]] || { echo "click <x> <y>" >&2; exit 2; }
    "$XDOTOOL" mousemove --sync "$1" "$2"
    exec "$XDOTOOL" click 1 ;;
  move)
    [[ $# -ge 2 ]] || { echo "move <x> <y>" >&2; exit 2; }
    exec "$XDOTOOL" mousemove --sync "$1" "$2" ;;
  *)
    echo "unknown command: $COMMAND (see --help)" >&2
    exit 2 ;;
esac
