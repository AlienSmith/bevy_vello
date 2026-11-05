#!/usr/bin/env bash
# run_walker_qa.sh — build + run the walker_qa QA scene headless, then draw the
# hip/feet plot from its telemetry CSV (the same legacy format as every other
# harness, so plot_p3.py reads it unchanged).
#
#   tools/physics-qa/run_walker_qa.sh [TAG] [KEY=VALUE ...]
#
#   TAG                 artifact name under tmp/walker_qa/ (default: "run")
#   KEY=VALUE           env overrides forwarded to walker_qa. Known levers:
#                       WIN_HIDDEN (default 1 — no OS window; the whole
#                       pipeline still runs), ASSET_ROOT,
#                       SCRIPT=path.json — a command track that drives the
#                       walker and self-exits the app when the track ends.
#                       CSV_PATH is fixed by the binary.
#   RUN_SECS            hard backstop kill, seconds (default 600). A scripted
#                       run exits on its own; RUN_SECS only guards a hung
#                       keyboard run.
#
# Artifacts:
#   tmp/walker_qa/<TAG>.csv            telemetry (legacy header: scalars + P-channels)
#   tmp/walker_qa/<TAG>.hips_feet.png  plot_p3.py output (hip + feet, gait bands)
#   stdout summary                     grounded fraction + hip/feet settle table
set -euo pipefail
cd "$(dirname "$0")/../.."
REPO=$PWD

TAG="run"
ENVARGS=()
RUN_SECS="${RUN_SECS:-8}"
for a in "$@"; do
  if [[ "$a" =~ ^[A-Za-z_][A-Za-z0-9_]*= ]]; then
    ENVARGS+=("$a")
  elif [[ "$a" =~ ^RUN_SECS= ]]; then
    RUN_SECS="${a#RUN_SECS=}"
  else
    TAG="$a"
  fi
done

OUTDIR=$REPO/tmp/walker_qa
mkdir -p "$OUTDIR"
CSV=$OUTDIR/$TAG.csv
PNG=$OUTDIR/$TAG.hips_feet.png

BIN=$REPO/target/debug/walker_qa
echo "== build walker_qa"
cargo build -p walker_qa 2>&1 | tail -1

echo "== run: $TAG (${ENVARGS[*]:-<defaults>}) — backstop ${RUN_SECS}s"
# walker_qa writes its telemetry to tmp/walker_qa/walk.csv; move it to the tag.
rm -f tmp/walker_qa/walk.csv
env WIN_HIDDEN=1 ${ENVARGS[@]+"${ENVARGS[@]}"} timeout "$RUN_SECS" "$BIN" \
  > "$OUTDIR/$TAG.log" 2>&1 || true
if [ ! -f tmp/walker_qa/walk.csv ]; then
  echo "no telemetry written: tmp/walker_qa/walk.csv (see $OUTDIR/$TAG.log)" >&2
  tail -5 "$OUTDIR/$TAG.log" >&2
  exit 1
fi
mv tmp/walker_qa/walk.csv "$CSV"
grep -E 'attached|floor|wrote' "$OUTDIR/$TAG.log" | tail -3

echo "== plot"
python3 tools/physics-qa/plot_p3.py "$CSV" "$PNG"

echo "== standing summary"
python3 - "$CSV" <<'EOF'
import csv, statistics, sys
rows = list(csv.DictReader(open(sys.argv[1])))
def num(r, k): return float(r[k])
if not rows:
    sys.exit("empty csv")
tail = rows[-90:]
print(f"t={num(rows[-1],'t'):.1f}s mode={rows[-1]['mode']} "
      f"grounded={sum(int(r['grounded']) for r in rows)/len(rows):.2f}")
print(f"hip y last-1s mean {statistics.mean(num(r,'P3y') for r in tail):.1f} "
      f"(physics y-down; floor surface 312, feet at ~305-310 when standing)")
print(f"feet y last-1s mean {statistics.mean((num(r,'PLLLy')+num(r,'PRLLy'))/2 for r in tail):.1f}")
print(f"hip x last-1s drift: {min(num(r,'P3x') for r in tail):.1f} .. {max(num(r,'P3x') for r in tail):.1f}")
print(f"v_filt last-1s mean {statistics.mean(abs(num(r,'v_filt')) for r in tail):.1f} px/s")
EOF

echo "== artifacts: $CSV  $PNG"
