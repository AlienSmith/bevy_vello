#!/usr/bin/env bash
# x11_setup.sh - fetch Xvfb and xdotool without root.
#
# The QA pipeline needs an X server with the XTEST extension (Xvfb) and xdotool
# in order to run the game off-screen and inject synthetic input. Neither may be
# installed, and passwordless sudo is not always available, so this downloads
# the .deb packages and unpacks them into a local prefix. No root required.
#
#   x11_setup.sh [--prefix DIR]
#
# Default prefix: tools/physics-qa/.x11/root (gitignored).
# run_capture.sh and x11_input.sh both look there automatically.
set -uo pipefail

PIPELINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="$PIPELINE_DIR/.x11/root"
if [[ "${1:-}" == "--prefix" ]]; then PREFIX="$2"; fi
mkdir -p "$PREFIX"

# xserver-common is a hard dependency of the xvfb package; libxdo3 is xdotool's.
PACKAGES=(xvfb xserver-common xdotool libxdo3)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
cd "$TMP"

echo "downloading: ${PACKAGES[*]}"
if ! apt-get download "${PACKAGES[@]}"; then
  echo "apt-get download failed. try 'sudo apt update' first, or install system-wide:" >&2
  echo "  sudo apt install xvfb xdotool" >&2
  exit 6
fi

for d in ./*.deb; do dpkg-deb -x "$d" "$PREFIX"; done

MISSING=()
for b in usr/bin/Xvfb usr/bin/xdotool; do
  [[ -x "$PREFIX/$b" ]] || MISSING+=("$b")
done
if [[ ${#MISSING[@]} -gt 0 ]]; then
  echo "incomplete unpack: missing ${MISSING[*]} under $PREFIX" >&2
  exit 6
fi

echo "installed into $PREFIX"
echo "  Xvfb    $PREFIX/usr/bin/Xvfb"
echo "  xdotool $PREFIX/usr/bin/xdotool"
echo
echo "verify:"
echo "  $PREFIX/usr/bin/Xvfb :99 -screen 0 1280x720x24 &"
echo "  DISPLAY=:99 $PREFIX/usr/bin/xdotool getdisplaygeometry"
