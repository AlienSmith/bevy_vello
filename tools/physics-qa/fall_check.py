#!/usr/bin/env python3
"""Did the locked body actually fall? Hip vs planted-feet support polygon."""
import csv, sys
for tag in sys.argv[1:]:
    rows = list(csv.DictReader(open(f"tmp/headless/walks/{tag}.csv")))
    hip0 = float(rows[0]["tP3x"])
    lf0, rf0 = float(rows[0]["tPLLLx"]), float(rows[0]["tPRLLx"])
    # support edges from the seed stance; feet may slide but not step (locked)
    rear, front = min(lf0, rf0), max(lf0, rf0)
    hip = [float(r["tP3x"]) for r in rows]
    ts = [float(r["t"]) for r in rows]
    dx = [float(r["P1x"]) - float(r["P3x"]) for r in rows]  # shoulder−hip
    over_f = max(h - front for h in hip)
    over_r = max(rear - h for h in hip)
    peak_dx = max(abs(d) for d in dx)
    fall = over_f > 25 or over_r > 25 or peak_dx > 60
    end_hip = hip[-1] - hip0
    print(f"{tag:8s} end_hip={end_hip:+7.1f} over_front={over_f:+6.1f} "
          f"over_rear={over_r:+6.1f} peak|dx|={peak_dx:5.1f} "
          f"{'FELL' if fall else 'upright'}")
