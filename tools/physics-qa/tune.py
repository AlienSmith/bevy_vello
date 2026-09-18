#!/usr/bin/env python3
"""Parameter search loop for the soft-body character.

Mechanical half of the pipeline: apply a candidate edit, capture, measure, keep or
revert. The *choice* of which parameter to move and which way is left to the
caller (an agent, using the vision judge's `suspect_params` / `suggested_direction`
plus these metrics). This script never guesses physics.

Because the parameters are a runtime asset, a trial costs a process restart and a
capture (~10s), not a rebuild.

Usage
-----
List the tunable knobs and their current values:

    tune.py knobs

Score the current parameters (one baseline run):

    tune.py run --out /tmp/t0

Try one edit, keeping it only if it scores better:

    tune.py try --set softbody.substeps=3 --out /tmp/t1

Drive a search from a list of candidates, best-first, reverting losers:

    tune.py sweep --candidates candidates.json --out-root /tmp/sweep --rounds 2

Restore the original parameters (also happens automatically on interrupt):

    tune.py restore

Scoring
-------
Lower is better. `score = mean_rmse + 100 * motion_frac`, with a large penalty for
a run that trips any objective gate. RMSE catches jitter; motion_frac catches a
body (or an effect) that disturbs far more of the arena than a settled character
should.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = Path(os.environ.get("REPO", HERE.parent.parent))
ASSET = REPO / "examples/collision_detection/assets/character/v8.character.json"

# Knobs the loop is allowed to touch. Each is a path into the character JSON plus
# a range to stay inside. The ranges are deliberately conservative: the goal is to
# find a stable region, not to explore the whole space.
KNOBS = {
    "softbody.substeps": {
        "apply": "per_collider", "field": "substeps", "type": "int",
        "range": [1, 8],
        "why": "more substeps = stiffer constraint solving, less divergence",
    },
    "softbody.inner_constraints_scaler": {
        "apply": "per_collider", "field": "inner_constraints_scaler", "type": "float",
        "range": [10.0, 1000.0],
        "why": "higher pulls inner constraints together harder",
    },
    "softbody.bilinear_constraints_scaler": {
        "apply": "per_collider", "field": "bilinear_constraints_scaler", "type": "float",
        "range": [0.1, 20.0],
        "why": "scales the bilinear (frame) constraints on each collider",
    },
    "softbody.frame_constraints_scaler": {
        "apply": "per_collider", "field": "frame_constraints_scaler", "type": "float",
        "range": [0.1, 20.0],
        "why": "scales how strongly colliders follow their frame",
    },
    "softbody.shape_matching_damping": {
        "apply": "per_collider", "field": "shape_matching_damping", "type": "float",
        "range": [0.0, 0.99],
        "why": "higher damps the shape-matching correction, reducing ringing",
    },
    "softbody.self_collision_complaince": {
        "apply": "per_collider", "field": "self_collision_complaince", "type": "float",
        "range": [0.0, 1.0],
        "why": "self-collision stiffness; too low lets the body pass through itself",
    },
    "collision.push_compliance_penetration_scaler": {
        "apply": "per_collider", "field": "push_compliance_penetration_scaler",
        "type": "float", "range": [0.0, 2.0], "group": "collision",
        "why": "how hard contacts push apart",
    },
    "collision.pull_compliance_scaler": {
        "apply": "per_collider", "field": "pull_compliance_scaler",
        "type": "float", "range": [0.0, 2.0], "group": "collision",
        "why": "how hard contacts pull together",
    },
    "frame.frame_particles_damping": {
        # Lives under frame.init_config, not directly on frame.
        "apply": "frame", "field": "frame_particles_damping", "type": "float",
        "range": [0.0, 0.99],
        "why": "damping on the frame particles",
    },
    "particles.frame_conn.compliance": {
        "apply": "particles", "field": "compliance", "type": "float",
        "range": [0.0, 1.0],
        "why": "per-particle frame connection compliance",
    },
}

GATE_PENALTY = 1000.0


def load_asset():
    with open(ASSET) as fh:
        return json.load(fh)


def save_asset(data):
    """Write the asset, preserving the file's existing formatting where possible.

    A plain json.dump round-trip reformats every float (0.000001 -> 1e-06) and
    turns a one-line parameter change into a 68-line diff. Instead the values are
    substituted textually in the original text, so a trial edit shows up as
    exactly the lines it changed.
    """
    text = ASSET.read_text()
    for path, spec in KNOBS.items():
        field = spec["field"]
        value = read_field_value(data, path)
        text = _subst_field(text, field, value, spec["type"])
    _atomic_write(text)


def read_field_value(data, path):
    """The single value a knob currently holds (all its sites are set together)."""
    vals = get_knob(data, path)
    if not vals:
        raise SystemExit(f"knob {path} has no sites")
    uniq = set(vals)
    if len(uniq) > 1:
        raise SystemExit(f"knob {path} is not uniform across sites: {sorted(uniq)}")
    return vals[0]


def _fmt(value, kind):
    if kind == "int":
        return str(int(value))
    # Keep floats plain (0.1, not 1e-01) and trim a trailing .0.
    s = repr(float(value))
    if s.endswith(".0"):
        s = s[:-2]
    return s


def _subst_field(text, field, value, kind):
    """Replace `"field": <number>` for every occurrence, keeping layout intact."""
    pattern = re.compile(r'("' + re.escape(field) + r'"\s*:\s*)(-?[0-9][0-9eE.+-]*)')
    new, n = pattern.subn(lambda m: m.group(1) + _fmt(value, kind), text)
    if n == 0:
        raise SystemExit(f"field {field!r} not found in {ASSET}")
    return new


def _atomic_write(text):
    tmp = ASSET.with_suffix(".json.tmp")
    tmp.write_text(text)
    os.replace(tmp, ASSET)


def apply_knob(path, value):
    """Set a knob textually. Returns the exact prior file text for restoration."""
    spec = KNOBS[path]
    lo, hi = spec["range"]
    if not (lo <= value <= hi):
        raise SystemExit(f"{path}={value} outside allowed range [{lo}, {hi}]")
    original_text = ASSET.read_text()
    new_text = _subst_field(original_text, spec["field"], value, spec["type"])
    _atomic_write(new_text)
    return original_text


def restore_text(text):
    _atomic_write(text)


def frame_container(data, spec):
    """Where a `frame`-scoped field lives (currently frame.init_config)."""
    return data["frame"]["init_config"] if spec.get("nested") is not False else data["frame"]


def get_knob(data, path):
    spec = KNOBS[path]
    where = spec["apply"]
    field = spec["field"]
    if where == "per_collider":
        group = spec.get("group", "softbody")
        return [c[group][field] for c in data["colliders"]]
    if where == "frame":
        return [frame_container(data, spec)[field]]
    if where == "particles":
        return [p["frame_conn"][field] for p in data["particles"]]
    raise KeyError(where)


def set_knob(data, path, value):
    """Set a knob on every site. Returns the list of previous values."""
    spec = KNOBS[path]
    where = spec["apply"]
    field = spec["field"]
    lo, hi = spec["range"]
    if not (lo <= value <= hi):
        raise SystemExit(f"{path}={value} outside allowed range [{lo}, {hi}]")
    if spec["type"] == "int":
        value = int(value)

    prev = []
    if where == "per_collider":
        group = spec.get("group", "softbody")
        for c in data["colliders"]:
            prev.append(c[group][field])
            c[group][field] = value
    elif where == "frame":
        box = frame_container(data, spec)
        prev.append(box[field])
        box[field] = value
    elif where == "particles":
        for p in data["particles"]:
            prev.append(p["frame_conn"][field])
            p["frame_conn"][field] = value
    else:
        raise KeyError(where)
    return prev


def restore_knob(data, path, prev):
    spec = KNOBS[path]
    where = spec["apply"]
    field = spec["field"]
    if where == "per_collider":
        group = spec.get("group", "softbody")
        for c, v in zip(data["colliders"], prev):
            c[group][field] = v
    elif where == "frame":
        frame_container(data, spec)[field] = prev[0]
    elif where == "particles":
        for p, v in zip(data["particles"], prev):
            p["frame_conn"][field] = v


def capture_and_score(out_dir, frames, interval):
    """Run one capture and return (score, metrics). Raises on capture failure."""
    out_dir = Path(out_dir)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    cmd = [str(HERE / "run_capture.sh"), "--out", str(out_dir),
           "--frames", str(frames), "--interval", str(interval)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    try:
        summary = json.loads(proc.stdout)
    except json.JSONDecodeError:
        raise SystemExit(f"capture produced no JSON (exit {proc.returncode}):\n"
                         f"{proc.stdout}\n{proc.stderr}")
    if not summary.get("ok"):
        raise SystemExit(f"capture failed: {json.dumps(summary)}")

    shots = sorted(str(p) for p in out_dir.glob("frame_*.png"))
    mproc = subprocess.run([sys.executable, str(HERE / "metrics.py"), "--json", *shots],
                           capture_output=True, text=True)
    if mproc.returncode != 0:
        raise SystemExit(f"metrics failed:\n{mproc.stdout}\n{mproc.stderr}")
    metrics = json.loads(mproc.stdout)

    score = metrics["mean_rmse"] + 100.0 * metrics["motion_frac"]
    if not metrics["ok"]:
        score += GATE_PENALTY
    return score, metrics


def cmd_knobs(_args):
    data = load_asset()
    print(f"asset: {ASSET}\n")
    for path, spec in KNOBS.items():
        vals = get_knob(data, path)
        uniq = sorted(set(vals))
        shown = uniq if len(uniq) <= 4 else f"{len(uniq)} distinct"
        print(f"{path}\n    current: {shown}\n    range:   {spec['range']}  ({spec['type']})")
        print(f"    {spec['why']}\n")


def cmd_run(args):
    score, metrics = capture_and_score(args.out, args.frames, args.interval)
    print(json.dumps({"score": round(score, 3), "metrics": metrics}, indent=2))


def parse_set(s):
    path, _, raw = s.partition("=")
    if path not in KNOBS:
        raise SystemExit(f"unknown knob {path!r}; see `tune.py knobs`")
    if not raw:
        raise SystemExit(f"missing value in {s!r}; expected knob=value")
    value = float(raw) if KNOBS[path]["type"] == "float" else int(raw)
    return path, value


def cmd_try(args):
    path, value = parse_set(args.set)

    # Snapshot the exact file text so a revert restores formatting byte-for-byte.
    original_text = apply_knob(path, value)
    print(f"applied {path}={value}", file=sys.stderr)

    try:
        score, metrics = capture_and_score(args.out, args.frames, args.interval)
    except BaseException:
        restore_text(original_text)
        print(f"reverted {path} after capture failure", file=sys.stderr)
        raise

    verdict = "kept"
    if args.baseline is not None:
        verdict = "improved" if score < args.baseline else "worse"
    if verdict == "worse":
        restore_text(original_text)
        print(f"reverted {path}: {score:.3f} >= baseline {args.baseline:.3f}",
              file=sys.stderr)

    print(json.dumps({
        "set": args.set, "score": round(score, 3),
        "baseline": args.baseline, "verdict": verdict, "metrics": metrics,
    }, indent=2))


def cmd_sweep(args):
    with open(args.candidates) as fh:
        candidates = json.load(fh)
    if not isinstance(candidates, list):
        raise SystemExit("candidates file must hold a JSON list of 'knob=value' strings")

    root = Path(args.out_root)
    root.mkdir(parents=True, exist_ok=True)
    original_text = ASSET.read_text()
    (root / "original.character.json").write_text(original_text)

    results = []
    try:
        best_score, best_metrics = capture_and_score(root / "baseline", args.frames, args.interval)
        print(f"baseline score {best_score:.3f}", file=sys.stderr)
        results.append({"set": None, "score": round(best_score, 3), "verdict": "baseline"})

        for rnd in range(args.rounds):
            improved = False
            for cand in candidates:
                try:
                    path, value = parse_set(cand)
                except SystemExit as exc:
                    print(f"  skipping {cand!r}: {exc}", file=sys.stderr)
                    continue

                before = ASSET.read_text()
                apply_knob(path, value)
                try:
                    score, metrics = capture_and_score(
                        root / f"r{rnd}_{cand.replace('=', '_')}", args.frames, args.interval)
                except SystemExit as exc:
                    print(f"  {cand}: capture failed ({exc})", file=sys.stderr)
                    restore_text(before)
                    results.append({"set": cand, "score": None, "verdict": "capture_failed"})
                    continue

                if score < best_score:
                    best_score, best_metrics = score, metrics
                    improved = True
                    verdict = "improved"
                    print(f"  {cand}: {score:.3f} IMPROVED", file=sys.stderr)
                else:
                    restore_text(before)
                    verdict = "reverted"
                    print(f"  {cand}: {score:.3f} (reverted)", file=sys.stderr)
                results.append({"set": cand, "score": round(score, 3), "verdict": verdict})
            if not improved:
                print("no improvement this round; stopping", file=sys.stderr)
                break
    finally:
        # Leave the asset in a state the caller can inspect deliberately.
        if not args.keep_best:
            restore_text(original_text)
            print("restored original parameters", file=sys.stderr)

    print(json.dumps({"best_score": round(best_score, 3),
                      "best_metrics": best_metrics,
                      "results": results}, indent=2))


def cmd_restore(args):
    src = Path(args.from_file)
    if not src.exists():
        raise SystemExit(f"no backup at {src}")
    shutil.copyfile(src, ASSET)
    print(f"restored {ASSET} from {src}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("knobs", help="list tunable knobs and current values")
    p.set_defaults(func=cmd_knobs)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--out", default=None)
    common.add_argument("--frames", type=int, default=3)
    common.add_argument("--interval", type=float, default=0.6)

    p = sub.add_parser("run", parents=[common], help="score current parameters")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("try", parents=[common], help="try one edit")
    p.add_argument("--set", required=True, help="knob=value, e.g. softbody.substeps=3")
    p.add_argument("--baseline", type=float, default=None,
                   help="keep the edit only if the score beats this")
    p.set_defaults(func=cmd_try)

    p = sub.add_parser("sweep", help="try candidates best-first, reverting losers")
    p.add_argument("--candidates", required=True)
    p.add_argument("--out-root", required=True)
    p.add_argument("--rounds", type=int, default=1)
    p.add_argument("--frames", type=int, default=3)
    p.add_argument("--interval", type=float, default=0.6)
    p.add_argument("--keep-best", action="store_true",
                   help="leave the best parameters applied instead of restoring")
    p.set_defaults(func=cmd_sweep)

    p = sub.add_parser("restore", help="restore the character asset from a backup")
    p.add_argument("--from-file", required=True)
    p.set_defaults(func=cmd_restore)

    args = ap.parse_args()
    if getattr(args, "out", None) is None and args.func is cmd_run:
        args.out = tempfile.mkdtemp(prefix="physics-qa-run-")
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
