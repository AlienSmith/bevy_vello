#!/usr/bin/env python3
"""Set the XPBD compliance of named joints in the runtime character asset.

Stiffness lives in the asset:
  Distance joints  ["P3", "P41", 1e-06]          third element = compliance
  Angular joints   ["P2", "P3", "P41", 5e-07, null]  fourth element = compliance
Lower = stiffer. The legs are four Distance + four Angular joints:

  right: P3_P41 (thigh), P41_PRLL (calf), P2_P3_P41 (hip), P3_P41_PRLL (knee)
  left:  P3_P40 (thigh), P40_PLLL (calf), P2_P3_P40 (hip), P3_P40_PLLL (knee)

Edits are textual (the file is patched in place, so the diff is exactly the
compliance lines it changed) and `--restore` puts the backup back byte-exactly.

Usage:
  set_joint_compliance.py [--compliance 1e-9] [--joints ID,ID,...]
                          [--asset PATH] [--restore] [--dry-run]

With no `--joints` the eight leg joints above are patched.
Exit codes: 0 ok, 2 bad input.
"""
import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULT_ASSET = (REPO / "examples" / "collision_detection" / "assets"
                 / "character" / "v10.character.json")
BACKUP = HERE / ".joint_compliance_backup.json"

LEG_JOINTS = "P3_P41,P41_PRLL,P3_P40,P40_PLLL,P2_P3_P41,P3_P41_PRLL,P2_P3_P40,P3_P40_PLLL"

PATH_ID_RE = re.compile(r'"path_id"\s*:\s*"([^"]+)"')
# A standalone numeric element line inside a joint's `config` array.
NUMBER_LINE_RE = re.compile(r'^(\s*)(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)(,?)(\s*)$')


def format_compliance(v):
    """Keep the asset's exponential style: 1e-9, 5e-07, 0.000001."""
    if v == 0:
        return "0.0"
    mantissa, exponent = f"{v:.10e}".split("e")
    mantissa = mantissa.rstrip("0").rstrip(".")
    return f"{mantissa}e{int(exponent):+03d}"


def patch(text, joints, value):
    wanted = set(joints)
    out, cur, hits, missing = [], None, {}, []
    for line in text.splitlines(keepends=True):
        m = PATH_ID_RE.search(line)
        if m:
            cur = m.group(1)
            if cur in wanted:
                missing.append(cur)
        if cur in wanted:
            nm = NUMBER_LINE_RE.match(line)
            # The first bare number in the block is the compliance.
            if nm and hits.get(cur) is None:
                hits[cur] = 1
                line = (f"{nm.group(1)}{format_compliance(value)}"
                        f"{nm.group(3)}{nm.group(4)}")
                missing = [j for j in missing if j != cur]
        out.append(line)
    return "".join(out), hits, missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--compliance", type=float, default=1e-9,
                    help="XPBD compliance to set (lower = stiffer)")
    ap.add_argument("--joints", default=LEG_JOINTS,
                    help="comma-separated joint path_ids")
    ap.add_argument("--asset", default=str(DEFAULT_ASSET))
    ap.add_argument("--restore", action="store_true",
                    help="restore the backup written by the previous edit")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the changed lines, do not write")
    args = ap.parse_args()

    asset = Path(args.asset).resolve()
    if not asset.exists():
        print(f"asset not found: {asset}", file=sys.stderr)
        sys.exit(2)

    if args.restore:
        if not BACKUP.exists():
            print(f"no backup at {BACKUP}", file=sys.stderr)
            sys.exit(2)
        asset.write_bytes(BACKUP.read_bytes())
        print(f"restored {asset} from {BACKUP}")
        return

    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    original = asset.read_bytes().decode("utf-8")
    patched, hits, missing = patch(original, joints, args.compliance)
    if missing:
        print(f"joints not found: {sorted(missing)}", file=sys.stderr)
        sys.exit(2)
    if not hits:
        print("nothing patched", file=sys.stderr)
        sys.exit(2)

    changed = [(i + 1, b) for i, (a, b) in
               enumerate(zip(original.splitlines(), patched.splitlines())) if a != b]
    print(f"{len(changed)} line(s) -> compliance {format_compliance(args.compliance)}")
    for lineno, new in changed:
        print(f"  line {lineno}: {new.strip()}")
    if args.dry_run:
        return

    if not BACKUP.exists():
        BACKUP.write_bytes(original.encode("utf-8"))
    asset.write_bytes(patched.encode("utf-8"))
    print(f"wrote {asset} (backup: {BACKUP})")


if __name__ == "__main__":
    main()
