# Reproducing the stance-delivery analysis

Everything needed to regenerate every number in
[`plans/stance-delivery-is-the-brake.md`](../plans/stance-delivery-is-the-brake.md:1)
is in the repository. Two paths: **offline** (no build, no GPU, seconds) and
**live** (rebuild the telemetry from scratch).

## A. Offline — use the committed fixture (recommended on another machine)

```bash
python3 tools/physics-qa/analyze_stance_delivery.py \
    tools/physics-qa/fixtures/walk20_mix07_s90.csv
```

Expected output (this is the canonical result; any deviation means the code or
the tool changed):

```
== tools/physics-qa/fixtures/walk20_mix07_s90.csv: 1936 rows
  [L] swings=36 stances=35
    clock    stance_T 0.289 s   swing_T 0.278 s
    swing    target   98.8px @ 355.5px/s   actual   74.2px @ 267.2px/s   ratio 0.80   peak 739px/s
    shortfall at touchdown 55.3px
    lag by swing decile 32.9 4.7 -15.1 -8.4 10.2 24.0 47.2 59.3 65.3 61.1
    stance   drag 24.5px (peak 248px/s)   hip_adv 45.7px   corr(drag,shortfall) +0.66
    pendulum rel_sweep -18.5px (demand -1.9px)  dv -12.9px/s   lipm +47.8px/s
    corr(rel_sweep,dv) +0.061   corr(hip_adv,dv) -0.028   corr(T,dv) +0.222   delivered_fraction +0.01
  [R] swings=35 stances=35
    clock    stance_T 0.289 s   swing_T 0.278 s
    swing    target   91.4px @ 328.9px/s   actual   66.4px @ 239.0px/s   ratio 0.74   peak 762px/s
    shortfall at touchdown 58.2px
    lag by swing decile 31.6 -0.1 -14.4 -5.9 13.6 27.1 50.9 62.6 64.7 60.0
    stance   drag 26.5px (peak 255px/s)   hip_adv 47.9px   corr(drag,shortfall) +0.69
    pendulum rel_sweep -11.6px (demand +2.3px)  dv +6.7px/s   lipm +29.5px/s
    corr(rel_sweep,dv) -0.129   corr(hip_adv,dv) +0.019   corr(T,dv) +0.146   delivered_fraction -0.01
  [L] SLIDING: the stance foot leaks and the pendulum cannot brake
  [R] SLIDING: the stance foot leaks and the pendulum cannot brake
```

## B. Live — regenerate the CSV (needs release build + GPU/X11)

```bash
cargo build --release -p collision_detection --example visual_arm_pd

tools/physics-qa/run_walk.sh walk20_mix07_s90 \
    SCRIPT="$PWD/plans/long_walk20_s90_script.json" \
    HIP_V_MIX=0.7 PLAN_MEAS_V=1 VEL_FEEDBACK=1 BOOT_LUNGE=1 \
    TARGET_SPEED=120 FLOOR_HALF_LEN=3500 \
    GAIT_AUTHORITY=timer DCM_TC=0
```

Artifacts land in `tmp/headless/walks/walk20_mix07_s90.{csv,hips_feet.png,character.json}`.

### Why each switch is there (all default-off, none committed behaviour)

| switch | value | why this run needs it |
|---|---|---|
| `SCRIPT` | `plans/long_walk20_s90_script.json` | the 20 s forward move at **90 px/s** — the deceleration test |
| `HIP_V_MIX` | `0.7` | the single-knob `v = 0.3·v_meas_filt + 0.7·v_ref` blend being tested |
| `PLAN_MEAS_V` | `1` | the placement law brakes against the *measured* CoM speed, not the fiction |
| `VEL_FEEDBACK` | `1` | locked baseline (2026-10-30): the fiction hip velocity is overwritten by reality |
| `BOOT_LUNGE` | `1` | locked baseline: the hip kick at startup |
| `TARGET_SPEED` | `120` | the *fiction* cruise command; the script asks 90, which is how deceleration authority is probed |
| `FLOOR_HALF_LEN` | `3500` | a ~20 s walk at ~200 px/s covers ~4000 px; the default 2400 free-falls off the edge |
| `GAIT_AUTHORITY` | `timer` | **the upright regime.** `full` collapses the hip onto the floor from t≈12 (see doc §7) — do not use it for these measurements |
| `DCM_TC` | `0` | analytic deadbeat, `t_c` auto-picks the stance duration (the only runtime placement law) |

Also implicitly fixed by `run_walk.sh`: `LEG_COMPLIANCE=1e-8`, `STEP_HEIGHT=70`,
`COLLISION=1`, `WIN_HIDDEN=1`.

### Determinism note

The run is **deterministic**: re-running the exact command above reproduced the
committed fixture byte-for-byte (`worst stance drift: 73.560px`, 1936 rows,
identical `sha256`). The fixture's `sha256` is
`3e8885d64e3b59101f53168b0f4c3bd7ea46e070f7b002cec963bd813d0e1f34`.

If a future run differs, it is because an input changed — check, in order:
`git log` on `examples/game_lib/src/lower_body/`, the asset snapshot
(`fixtures/walk20_mix07_s90.character.json` vs
`examples/collision_detection/assets/character/v10.character.json`), and the
`physical_*` constants in `examples/collision_detection/examples/headless_arm_pd/`.

## C. Fixtures committed here

| file | what |
|---|---|
| `fixtures/walk20_mix07_s90.csv` | the 1936-row telemetry (1.4 MB, 106 columns) |
| `fixtures/walk20_mix07_s90.character.json` | the asset snapshot taken at run start, so collider edits since then cannot be mistaken for the cause |

The asset snapshot matters: `run_walk.sh` copies the live
`v10.character.json` into the artifact directory on every run precisely because
the asset is hand-edited between experiments. Diff the snapshot before blaming
code for a change in the numbers.

## D. Related analyzers on the same CSV

```bash
python3 tools/physics-qa/analyze_foot_slide.py  <csv> --skip-start 1.6 --skip-end 10.5
python3 tools/physics-qa/analyze_swing.py       <csv> --per-swing
python3 tools/physics-qa/plot_p3.py             <csv> out.png
```

`analyze_foot_slide.py` must still report `SLIDING` (73.560 px worst stance).
That gate and `analyze_stance_delivery.py`'s `SLIDING` verdict are two views of
the same defect and should always agree.

## E. What is *not* reproducible from the committed data

The contact-fraction question (doc §6a/§6b) **cannot** be answered from this
fixture: the telemetry has no collision-event columns. That is exactly why the
next experiment is to emit `contact_l`/`contact_r` from the pin's debounced
`grounded` flag (doc §6b). Until that lands, no contact claim should be made
from these numbers — the particle is a capsule centre, not a contact point.
