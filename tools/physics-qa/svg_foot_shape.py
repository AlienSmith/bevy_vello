"""Extract the foot collider capsule geometry from v10.character.svg.

Finds the <g id="Collider/LLL"> (and RLL) groups and reports their path data +
bounding boxes, so we can derive the capsule half-thickness (radius) from the
SVG shape — the value the particle-centre-vs-floor comparisons were missing.
"""
import re
import sys

svg_path = sys.argv[1] if len(sys.argv) > 1 else (
    "examples/collision_detection/assets/character/v10.character.svg")
text = open(svg_path, encoding="utf-8").read()

# All element ids
ids = re.findall(r'id="([^"]+)"', text)
print("ids containing LL:", [i for i in ids if "LL" in i])

# The character root transform (scale!) — the SVG coordinates get scaled by it.
m = re.search(r'transform="[^"]*scale\(([\d.]+)\)', text)
if m:
    print(f"first scale() transform in file: {m.group(1)}")

# The foot colliders are elements with those ids directly (path or g).
def extent(d):
    nums = [float(x) for x in re.findall(r'-?\d+\.?\d*', d)]
    xs, ys = nums[0::2], nums[1::2]
    return (min(xs), max(xs), min(ys), max(ys))

for cid in ("LLL", "RLL", "PLLL", "PRLL"):
    el = re.search(rf'<(\w+)[^>]*id="{re.escape(cid)}"[^>]*?(?:/>|>(.*?)</\1>)', text, re.S)
    if not el:
        print(f"{cid}: element not found")
        continue
    tag, inner = el.group(1), el.group(2) or ""
    tf = re.search(r'transform="([^"]+)"', el.group(0))
    print(f"\n== {cid}  <{tag}>  transform: {tf.group(1) if tf else 'none'}")
    ds = re.findall(r'\sd="([^"]+)"', el.group(0) + inner)
    for d in ds:
        x0, x1, y0, y1 = extent(d)
        print(f"  d x∈[{x0:.1f},{x1:.1f}] y∈[{y0:.1f},{y1:.1f}]  w={x1-x0:.1f} h={y1-y0:.1f}")
        print(f"  raw: {d[:220]}")
    # <use href> indirection
    href = re.findall(r'href="#([^"]+)"', el.group(0) + inner)
    for h in href:
        p = re.search(rf'<path[^>]*id="{re.escape(h)}"[^>]*\sd="([^"]+)"', text)
        if p:
            x0, x1, y0, y1 = extent(p.group(1))
            print(f"  uses <path id={h}>: x∈[{x0:.1f},{x1:.1f}] y∈[{y0:.1f},{y1:.1f}] "
                  f"w={x1-x0:.1f} h={y1-y0:.1f}")
            print(f"  raw: {p.group(1)[:220]}")
