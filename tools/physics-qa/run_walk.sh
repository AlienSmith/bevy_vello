#!/usr/bin/env bash
# run_walk.sh — Linux twin of tools/physics-qa/run_visual.ps1: one command,
# windowless VISUAL walk + hip/feet plot + gates.
#
#   tools/physics-qa/run_walk.sh [TAG] [KEY=VALUE ...]
#
#   TAG                 name for the artifacts (default: "walk")
#   KEY=VALUE           env overrides forwarded to the harness. Known keys the
#                       harness reads: LEG_COMPLIANCE, STEP_HEIGHT, STANCE,
#                       SWING, GAIT_AUTHORITY, ATT_LEAN_KP, ATT_LEAN_KD,
#                       SUBSTEPS, TARGET_SPEED, WIN_HIDDEN, COLLISION, FACING,
#                       MPC_TRIGGER, TOE_LEN, DCM_TC, SCRIPT, ...
#                       (T_CONV and DCM_KP were removed: they are single-dof
#                       truncations of the analytic deadbeat law — tune DCM_TC
#                       instead. V_BOOT and PUSH_OFF (startup gates) were
#                       removed: they never worked and were never enabled.)
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
# DEFAULT ENV (identical to run_visual.ps1 — keep the two in sync):
#   LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1 DCM_TC=0
#   BOOT_LUNGE=1 (the hip-kick launch is the QA baseline; BOOT_LUNGE=0 runs the
#   fiction-only startup). DCM_TC=0 = analytic deadbeat placement, the ONLY
#   runtime placement law, t_c auto-picks the stance duration. SCRIPT defaults
#   to plans/walk_script.json. Pass DCM_TC=<s> to tune the deadline (or
#   DCM_TC=-1 to disable the law, falling back to the Raibert neutral point).
#
# ENV PRECEDENCE (mirrors run_visual.ps1): the defaults above are set
# UNCONDITIONALLY — an inherited shell env var does NOT change the baseline.
# Only a KEY=VALUE argument overrides a default. This is what makes the .sh and
# .ps1 produce the same run from the same arguments.
#
# PLOT BEHAVIOR (mirrors run_visual.ps1): the PNG is written but NOT opened by
# default. Pass SHOW_PLOT=1 to open it after the run.
#
# Artifacts land in tmp/headless/walks/ (same names as run_visual.ps1):
#   <TAG>.csv            telemetry (104+ columns)
#   <TAG>.hips_feet.png  hip + feet x/y plot (plot_p3.py)
#   <TAG>.character.json asset snapshot (self-identifying artifacts)
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

# --- parse args: first non KEY=VALUE token is the tag, rest are env vars -----
# KEY=VALUE if it starts with an identifier followed by '=' — the VALUE may
# contain '/' (e.g. SCRIPT=/abs/path); anything else is the TAG.
TAG="walk"
ENVARGS=()
for a in "$@"; do
  if [[ "$a" =~ ^[A-Za-z_][A-Za-z0-9_]*= ]]; then
    ENVARGS+=("$a")
  else
    TAG="$a"
  fi
done

# --- output paths ------------------------------------------------------------
OUTDIR=$REPO/tmp/headless/walks
mkdir -p "$OUTDIR"
CSV=$OUTDIR/$TAG.csv
PNG=$OUTDIR/$TAG.hips_feet.png

# Snapshot the asset WITH the tag so each artifact is self-identifying. The
# asset is live-edited between runs, and the tag does not pin it: two runs of
# the same tag with edited colliders in between are different experiments.
cp "$REPO/examples/collision_detection/assets/character/v10.character.json" \
   "$OUTDIR/$TAG.character.json"

# --- env model (mirrors run_visual.ps1's $fixed + $EnvAll + KEY=VALUE) --------
# FIXED: paths + output, always set by this script (a KEY=VALUE can override).
# DEFS:  the baseline defaults, set UNCONDITIONALLY (no ${VAR:-} fallback, so an
#        inherited shell env can't shift the baseline — same as the .ps1).
declare -A FIXED=(
  [ASSET_ROOT]="$REPO/examples/collision_detection/assets"
  [SCRIPT]="$REPO/plans/walk_script.json"
  [OUT_CSV]="$CSV"
)
declare -A DEFS=(
  [LEG_COMPLIANCE]='1e-8'
  [STEP_HEIGHT]='70'
  [COLLISION]='1'
  [WIN_HIDDEN]='1'
  [DCM_TC]='0'
  [BOOT_LUNGE]='1'
)

# Build the assignment list: FIXED + DEFS first, then any KEY=VALUE overrides
# LAST so `env` last-wins gives the override precedence (matching the .ps1).
ENVTOKENS=()
for k in "${!FIXED[@]}"; do ENVTOKENS+=("$k=${FIXED[$k]}"); done
for k in "${!DEFS[@]}";    do ENVTOKENS+=("$k=${DEFS[$k]}");    done
for a in ${ENVARGS[@]+"${ENVARGS[@]}"}; do ENVTOKENS+=("$a"); done

if [ ${#ENVARGS[@]} -gt 0 ]; then
  SHOWN="${ENVARGS[*]}"
else
  SHOWN='<defaults: LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1 DCM_TC=0 BOOT_LUNGE=1>'
fi
echo "== run $TAG: $SHOWN (asset snapshot: $OUTDIR/$TAG.character.json)"

env "${ENVTOKENS[@]}" "$BIN" 2>&1 | grep -E "body: rows|rigidity:|leg:|walk params|visual harness" || true

[ -f "$CSV" ] || { echo "no telemetry written: $CSV (the harness exited before finalize?)" >&2; exit 1; }

# --- gates use SHOW_PLOT-gated plot, like run_visual.ps1 --------------------
echo "== plot"
python3 tools/physics-qa/plot_p3.py "$CSV" "$PNG" 2>/dev/null | head -1

echo "== gates"
python3 tools/physics-qa/analyze_swing.py "$CSV" --per-swing 2>/dev/null | tail -4 || true
python3 tools/physics-qa/analyze_foot_slide.py "$CSV" --skip-start 1.6 --skip-end 10.5 2>/dev/null | tail -2 || true

# --- artifacts (plot NOT opened unless SHOW_PLOT=1, like run_visual.ps1) ----
# SHOW_PLOT is never a default; it only takes effect as a KEY=VALUE argument.
echo "== artifacts: $CSV  $PNG"
for a in ${ENVARGS[@]+"${ENVARGS[@]}"}; do
  if [ "$a" = "SHOW_PLOT=1" ] && [ -f "$PNG" ]; then
    command -v code >/dev/null && code "$PNG" || xdg-open "$PNG" 2>/dev/null || true
  fi
done
