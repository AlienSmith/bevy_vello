#!/usr/bin/env python3
"""Sweep the foot colliders' `friction_compliance_scaler` on the headless walk.

This is the friction half of the walking foot-slide loop documented in
`physics_tuning_progress.md`. For each candidate value it

  1. patches `friction_compliance_scaler` **only** inside the named colliders
     (`LLL` / `RLL`, the colliders that contain the foot particles `PLLL` /
     `PRLL`) in the runtime character asset, textually, so the diff is exactly
     the lines it changed;
  2. runs the headless walk (`headless_arm_pd` + a walk script);
  3. re-reads the telemetry CSV with `analyze_foot_slide.py` and reports the
     **steady-state** stance drift (startup/stop transients trimmed);
  4. restores the asset byte-exactly, including on failure or interrupt.

No rebuild is needed: the parameters live in the runtime asset, which the game
reads at startup.

Friction semantics (see `physics_tuning_progress.md` §3): floor contact uses
`StaticCollisionConstraint`, where `friction_strength = 1 - friction_compliance`,
so **lower `friction_compliance_scaler` = more friction**. `0.0` means no
friction at all in `FineCollisionConstraint`; on the static floor it is the
maximum grip, which the asset itself warns can jitter — hence the sweep.

Usage
-----
  sweep_foot_friction.py [--frictions 0.1,0.05,0.02,0.0] [--colliders LLL,RLL]
                         [--repeat N] [--gate 2.0]
                         [--skip-start 1.1] [--skip-end 10.5]
                         [--asset PATH] [--binary PATH] [--script PATH]
                         [--out-root DIR] [--keep]

  --dry-run   show the patch for the first value, run nothing
  --keep      leave the last candidate applied (no restore)

Exit codes: 0 = sweep completed (read the report for the verdict),
            2 = bad input / missing binary or asset, 3 = a run failed.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analyze_foot_slide as afs  # noqa: E402  (same directory)

# The two colliders that own the realized foot particles PLLL / PRLL.
DEFAULT_COLLIDERS = "LLL,RLL"
DEFAULT_FRICTIONS = "0.1,0.05,0.02,0.0"
# Startup / stop transients in the 11.5 s walk (see physics_tuning_progress.md).
# 1.6 s, not 1.1 s: at 1.1 s the window still catches the tail of the startup
# transient (a stance at t=1.13-1.72 s was the "worst" stance for several
# candidates), which showed up as a fake cliff in the sweep curve.
DEFAULT_SKIP_START = 1.6
DEFAULT_SKIP_END = 10.5
DEFAULT_GATE = 2.0

PATH_ID_RE = re.compile(r'"path_id"\s*:\s*"([^"]+)"')
FRICTION_RE = re.compile(
    r'("friction_compliance_scaler"\s*:\s*)(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)'
)
BARRIER = "=" * 72


# ---------------------------------------------------------------------------
# paths / defaults
# ---------------------------------------------------------------------------

def repo_root():
    """tools/physics-qa -> repository root (bevy_vello)."""
    return HERE.parents[1]


def default_binary(root):
    exe = "headless_arm_pd.exe" if os.name == "nt" else "headless_arm_pd"
    return root / "target" / "release" / "examples" / exe


def default_asset(root):
    return (root / "examples" / "collision_detection" / "assets" / "character"
            / "v10.character.json")


def default_script(root):
    return root / "plans" / "walk_script.json"


# ---------------------------------------------------------------------------
# asset patching (textual, formatting-preserving)
# ---------------------------------------------------------------------------

def format_value(v):
    """JSON-friendly number: 0.05 -> '0.05', 0.0 -> '0.0', 0.1 -> '0.1'."""
    s = f"{v:g}"
    if "." in s or "e" in s or "E" in s:
        return s
    return s + ".0"


def patch_friction(text, path_ids, value):
    """Set `friction_compliance_scaler` inside the named collider blocks.

    The asset is walked line by line; `"path_id"` opens a collider block, so the
    next `friction_compliance_scaler` belongs to it. Only those lines change.
    Returns (new_text, hits_by_collider, missing_colliders).
    """
    wanted = set(path_ids)
    out = []
    cur = None
    hits = {}
    for line in text.splitlines(keepends=True):
        m = PATH_ID_RE.search(line)
        if m:
            cur = m.group(1)
        if cur in wanted and "friction_compliance_scaler" in line:
            new_line, n = FRICTION_RE.subn(
                lambda mm: mm.group(1) + format_value(value), line)
            if n:
                hits[cur] = hits.get(cur, 0) + n
                line = new_line
        out.append(line)
    return "".join(out), hits, wanted - set(hits)


def write_text_exact(path, text):
    """Write utf-8 without newline translation (keeps CRLF byte-exact)."""
    path.write_bytes(text.encode("utf-8"))


# ---------------------------------------------------------------------------
# run + measure
# ---------------------------------------------------------------------------

def run_walk(binary, root, asset_root, script, out_csv):
    env = os.environ.copy()
    env.update({
        "ASSET_ROOT": str(asset_root),
        "SCRIPT": str(script),
        "OUT_CSV": str(out_csv),
        "COLLISION": "1",
    })
    env.setdefault("RUST_LOG", "info")
    started = time.time()
    proc = subprocess.run(
        [str(binary)], cwd=str(root), env=env,
        capture_output=True, text=True, encoding="utf-8", errors="replace",
    )
    return proc, time.time() - started


def measure(csv_path, gate, skip_start, skip_end):
    """Foot-slide summary for one telemetry CSV (steady window + full run)."""
    rows = afs.load(str(csv_path))
    if not rows:
        raise RuntimeError(f"no telemetry rows in {csv_path}")
    if all(math.isnan(r["lry"]) and math.isnan(r["rry"]) for r in rows):
        raise RuntimeError(f"{csv_path}: missing PLLL/PRLL columns "
                           "(not a whole-body walk CSV)")

    floor_l = afs.floor_height(rows, "lry")
    floor_r = afs.floor_height(rows, "rry")
    segs_l = afs.stance_segments(rows, floor_l, "lry", "lrx", "t", "gl",
                                 afs.DEFAULT_FLOOR_TOL, afs.DEFAULT_MIN_FRAMES)
    segs_r = afs.stance_segments(rows, floor_r, "rry", "rrx", "t", "gr",
                                 afs.DEFAULT_FLOOR_TOL, afs.DEFAULT_MIN_FRAMES)

    steady_l = afs.summarize(afs.filter_window(segs_l, skip_start, skip_end), "left ")
    steady_r = afs.summarize(afs.filter_window(segs_r, skip_start, skip_end), "right")
    full_l = afs.summarize(segs_l, "left ")
    full_r = afs.summarize(segs_r, "right")

    steady_worst = max(steady_l["max_drift"], steady_r["max_drift"])
    return {
        "ticks": len(rows),
        "dur_s": round(rows[-1]["t"] - rows[0]["t"], 2) if rows else 0.0,
        "steady": {"left": steady_l, "right": steady_r,
                   "worst_px": round(steady_worst, 3),
                   "gate_px": gate,
                   "verdict": "SLIDING" if steady_worst > gate else "OK"},
        "full": {"left": full_l, "right": full_r,
                 "worst_px": round(max(full_l["max_drift"],
                                       full_r["max_drift"]), 3)},
    }


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------

def render_markdown(results, meta):
    lines = [
        "# Foot friction sweep",
        "",
        f"- asset: `{meta['asset']}`",
        f"- colliders: `{', '.join(meta['colliders'])}` (feet, contain PLLL/PRLL)",
        f"- run: `{meta['binary']}` + `{meta['script']}`",
        f"- steady window: t >= {meta['skip_start']:.2f}s, t <= {meta['skip_end']:.2f}s",
        f"- gate: {meta['gate']:.2f}px worst stance drift",
        f"- repeats per candidate: {meta['repeat']}",
        "",
        "| friction | steady worst px | steady L max | steady R max | "
        "steady mean L/R | full-run worst px | verdict |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in results:
        s = r["steady"]
        lines.append(
            f"| {r['friction']:g} | {s['worst_px']:.3f} | "
            f"{s['left']['max_drift']:.3f} | {s['right']['max_drift']:.3f} | "
            f"{s['left']['mean_drift']:.2f} / {s['right']['mean_drift']:.2f} | "
            f"{r['full']['worst_px']:.3f} | {s['verdict']} |")
    lines.append("")
    return "\n".join(lines)


def main():
    root = repo_root()
    ap = argparse.ArgumentParser(
        description="Sweep foot friction_compliance_scaler on the headless walk")
    ap.add_argument("--frictions", default=DEFAULT_FRICTIONS,
                    help="comma-separated friction_compliance_scaler values "
                         "(lower = more friction on the static floor)")
    ap.add_argument("--colliders", default=DEFAULT_COLLIDERS,
                    help="comma-separated collider path_ids to patch")
    ap.add_argument("--repeat", type=int, default=1,
                    help="runs per candidate (the walk has run-to-run variance)")
    ap.add_argument("--gate", type=float, default=DEFAULT_GATE,
                    help="px; steady stance drift beyond this is SLIDING")
    ap.add_argument("--skip-start", type=float, default=DEFAULT_SKIP_START,
                    help="s; trim stance segments starting earlier (startup)")
    ap.add_argument("--skip-end", type=float, default=DEFAULT_SKIP_END,
                    help="s; trim stance segments ending later (stop)")
    ap.add_argument("--asset", default=str(default_asset(root)))
    ap.add_argument("--binary", default=str(default_binary(root)))
    ap.add_argument("--script", default=str(default_script(root)))
    ap.add_argument("--out-root", default=str(root / "tmp" / "headless"
                                              / "friction_sweep"))
    ap.add_argument("--keep", action="store_true",
                    help="leave the last candidate applied")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the patch for the first value and exit")
    args = ap.parse_args()

    # Resolve everything: the walk runs with cwd = repo root, so a relative
    # OUT_CSV would otherwise be resolved against a different directory than the
    # one this process reports.
    asset = Path(args.asset).resolve()
    binary = Path(args.binary).resolve()
    script = Path(args.script).resolve()
    out_root = Path(args.out_root).resolve()
    for p, label in ((asset, "asset"), (binary, "binary"), (script, "script")):
        if not p.exists():
            print(f"{label} not found: {p}", file=sys.stderr)
            sys.exit(2)

    frictions = []
    for tok in args.frictions.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            frictions.append(float(tok))
        except ValueError:
            print(f"bad friction value: {tok}", file=sys.stderr)
            sys.exit(2)
    if not frictions:
        print("no friction values given", file=sys.stderr)
        sys.exit(2)
    colliders = [c.strip() for c in args.colliders.split(",") if c.strip()]
    if not colliders:
        print("no colliders given", file=sys.stderr)
        sys.exit(2)

    original = asset.read_bytes().decode("utf-8")

    if args.dry_run:
        patched, hits, missing = patch_friction(original, colliders, frictions[0])
        print(f"would patch friction_compliance_scaler -> "
              f"{format_value(frictions[0])} in {hits}")
        if missing:
            print(f"WARNING: colliders not found: {sorted(missing)}",
                  file=sys.stderr)
        changed = [(i + 1, b) for i, (a, b) in
                   enumerate(zip(original.splitlines(), patched.splitlines()))
                   if a != b]
        for lineno, new in changed:
            print(f"  line {lineno}: {new.strip()}")
        return

    out_root.mkdir(parents=True, exist_ok=True)
    asset_root = asset.parent.parent  # .../assets (ASSET_ROOT points here)

    meta = {
        "asset": str(asset), "binary": str(binary), "script": str(script),
        "colliders": colliders, "gate": args.gate, "repeat": args.repeat,
        "skip_start": args.skip_start, "skip_end": args.skip_end,
    }
    results = []
    failures = []
    try:
        for fv in frictions:
            runs = []
            for rep in range(1, args.repeat + 1):
                patched, hits, missing = patch_friction(original, colliders, fv)
                if missing:
                    print(f"ERROR: colliders not found: {sorted(missing)}",
                          file=sys.stderr)
                    sys.exit(2)
                write_text_exact(asset, patched)

                tag = f"fric_{fv:g}" + (f"_r{rep}" if args.repeat > 1 else "")
                csv_path = out_root / f"{tag}.csv"
                proc, secs = run_walk(binary, root, asset_root, script, csv_path)
                if proc.returncode != 0 or not csv_path.exists():
                    reason = (f"rc={proc.returncode}" if proc.returncode
                              else f"no CSV at {csv_path}")
                    failures.append({"friction": fv, "rep": rep,
                                     "returncode": proc.returncode,
                                     "tail": (proc.stderr or "")[-800:]})
                    print(f"[{tag}] RUN FAILED {reason} ({secs:.1f}s)",
                          file=sys.stderr)
                    continue
                try:
                    m = measure(csv_path, args.gate,
                                args.skip_start, args.skip_end)
                except RuntimeError as exc:
                    failures.append({"friction": fv, "rep": rep,
                                     "error": str(exc)})
                    print(f"[{tag}] {exc}", file=sys.stderr)
                    continue
                m.update({"friction": fv, "rep": rep, "secs": round(secs, 1),
                          "csv": str(csv_path)})
                runs.append(m)
                print(f"[{tag}] {secs:5.1f}s  steady worst "
                      f"{m['steady']['worst_px']:6.3f}px  "
                      f"(L {m['steady']['left']['max_drift']:.3f} / "
                      f"R {m['steady']['right']['max_drift']:.3f})  "
                      f"full {m['full']['worst_px']:.3f}px  "
                      f"{m['steady']['verdict']}")
            if not runs:
                continue
            # Aggregate repeats: worst-case is the honest comparison (the loop
            # must not accept a config that slides in one run out of N).
            worst = max(runs, key=lambda r: r["steady"]["worst_px"])
            merged = dict(worst)
            merged["runs"] = len(runs)
            merged["steady_worst_mean_px"] = round(
                sum(r["steady"]["worst_px"] for r in runs) / len(runs), 3)
            results.append(merged)
    finally:
        if not args.keep:
            asset.write_bytes(original.encode("utf-8"))
        else:
            print("--keep: leaving the last candidate applied")

    if not results:
        print("no successful runs", file=sys.stderr)
        sys.exit(3)

    print()
    print(BARRIER)
    table = render_markdown(results, meta)
    print(table)
    (out_root / "friction_sweep.md").write_text(table, encoding="utf-8")
    (out_root / "friction_sweep.json").write_text(
        json.dumps({"meta": meta, "results": results,
                    "failures": failures}, indent=2), encoding="utf-8")
    print(f"wrote {out_root / 'friction_sweep.md'}")
    print(f"wrote {out_root / 'friction_sweep.json'}")

    best = min(results, key=lambda r: r["steady"]["worst_px"])
    print(f"best candidate: friction={best['friction']:g}  steady worst "
          f"{best['steady']['worst_px']:.3f}px  "
          f"({best['steady']['verdict']}, gate {args.gate:.2f}px)")
    if any(f.get("returncode") for f in failures):
        sys.exit(3)


if __name__ == "__main__":
    main()
