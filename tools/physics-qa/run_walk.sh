#!/usr/bin/env bash
# run_walk.sh — one command: windowless VISUAL walk + hip/feet plot + gates.
#
#   tools/physics-qa/run_walk.sh [TAG] [KEY=VALUE ...]
#
#   TAG                 name for the artifacts (default: "walk")
#   KEY=VALUE           env overrides forwarded to the harness. Known keys that
#                       the harness itself reads: LEG_COMPLIANCE, STEP_HEIGHT,
#                       STANCE, SWING, GAIT_AUTHORITY, ATT_LEAN_KP, ATT_LEAN_KD,
#                       SUBSTEPS, TARGET_SPEED, WIN_HIDDEN, COLLISION, FACING,
#                       MPC_TRIGGER, TOE_LEN, V_BOOT, DCM_TC, ...
#                       (T_CONV was removed: it is the derived beta coefficient
#                       of the analytic deadbeat law — tune DCM_TC instead.)
#
# The binary is `visual_arm_pd` — the rendered twin of the old headless bin:
# same script driver, same 90 Hz XPBD physics, same telemetry CSV, but booted
# with the FULL VelloPlugin + VelloCollisionResponsePlugin GPU collision chain
# (the PostUpdate event redistribution the manual headless chain omitted).
# WIN_HIDDEN=1 (default) runs the whole visual pipeline with no OS window;
# pass WIN_HIDDEN=0 to watch the character walk (the run self-exits at script
# end either way). This is the pipeline going forward: the QA walk IS the
# visual app, so the graph we plot is the graph on screen.
#
# Defaults: LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1,
# plans/walk_script.json (mirrors run_visual.ps1; the older headless defaults
# were LEG_COMPLIANCE=5e-9 STEP_HEIGHT=50).
# Artifacts land in tmp/headless/walks/:
#   <TAG>.csv            telemetry (104+ columns)
#   <TAG>.hips_feet.png  hip + feet x/y plot (plot_p3.py)
#   stdout summary       rig/leg verdicts + swing + foot-slide gates
#
# The character asset (examples/collision_detection/assets/character/
# v10.character.json) is read at startup, so collider edits need no rebuild.
# A Rust change does: rebuild first with
#   cargo build --release -p collision_detection --example visual_arm_pd

set -euo pipefail
cd "$(dirname "$0")/../.."
REPO=$PWD

BIN=$REPO/target/release/examples/visual_arm_pd
[ -x "$BIN" ] || { echo "missing $BIN — run: cargo build --release -p collision_detection --example visual_arm_pd" >&2; exit 1; }

TAG="walk"
ENVARGS=()
for a in "$@"; do
  if [[ "$a" == *"="* && "$a" != */* ]]; then
    ENVARGS+=("$a")
  else
    TAG="$a"
  fi
done

# defaults (a KEY=VALUE passed above wins: last assignment to a var in env
# takes effect when ordered after the defaults)
OUTDIR=$REPO/tmp/headless/walks
mkdir -p "$OUTDIR"
CSV=$OUTDIR/$TAG.csv

# Snapshot the asset WITH the tag so each artifact is self-identifying. The
# asset is live-edited between runs, and the tag does not pin it: two runs of
# the same tag with edited colliders in between are different experiments.
cp "$REPO/examples/collision_detection/assets/character/v10.character.json" \
   "$OUTDIR/$TAG.character.json"

echo "== run $TAG: ${ENVARGS[*]:-<defaults: LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1>} (asset snapshot: $OUTDIR/$TAG.character.json)"
env ASSET_ROOT="$REPO/examples/collision_detection/assets" \
    SCRIPT="$REPO/plans/walk_script.json" \
    OUT_CSV="$CSV" \
    LEG_COMPLIANCE="${LEG_COMPLIANCE:-1e-8}" \
    STEP_HEIGHT="${STEP_HEIGHT:-70}" \
    COLLISION="${COLLISION:-1}" \
    WIN_HIDDEN="${WIN_HIDDEN:-1}" \
    "${ENVARGS[@]}" \
    "$BIN" 2>&1 | grep -E "body: rows|rigidity:|leg:|walk params|visual harness" || true

echo "== plot"
python3 tools/physics-qa/plot_p3.py "$CSV" "$OUTDIR/$TAG.hips_feet.png" 2>/dev/null | head -1

echo "== gates"
python3 tools/physics-qa/analyze_swing.py "$CSV" --per-swing 2>/dev/null | tail -4 || true
python3 tools/physics-qa/analyze_foot_slide.py "$CSV" --skip-start 1.6 --skip-end 10.5 2>/dev/null | tail -2 || true

echo "== artifacts: $CSV  $OUTDIR/$TAG.hips_feet.png"
command -v code >/dev/null && code "$OUTDIR/$TAG.hips_feet.png" || xdg-open "$OUTDIR/$TAG.hips_feet.png" 2>/dev/null || true
