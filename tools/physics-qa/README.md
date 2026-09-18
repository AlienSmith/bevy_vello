# physics-qa — soft-body parameter tuning pipeline

Tools for tuning the XPBD soft-body parameters of the `collision_detection`
example by running it, screenshotting it, and judging whether the physics is
stable.

The loop:

```
   edit params  ->  run headless  ->  capture frames  ->  judge  ->  edit params
   (runtime asset)   (Xvfb)          (PNG burst)         (metrics + vision)
```

## Why params can be edited without a rebuild

The character's physics parameters live in the **runtime asset**

```
examples/collision_detection/assets/character/v8.character.json
```

which the game loads at startup. Changing a compliance, a substep count or a
damping value needs no `cargo build` — only a restart. A Rust change costs an
incremental rebuild (~15s), so the loop is fast either way.

Editable per collider: `softbody` (`max_complexity`, `resititution`,
`velocity_against_nromal_damping_threhold`, `total_inv_mass`,
`inner_constraints_scaler`, `bilinear_constraints_scaler`,
`frame_constraints_scaler`, `substeps`, `self_collision_complaince`,
`self_collision_distance_threhold`, `shape_matching_damping`), `collision`
(`push_compliance_penetration_scaler`, `friction_compliance_scaler`,
`pull_compliance_scaler`), `frame.frame_particles_damping`, and per-particle
`frame_conn.compliance`.

## Components

| file | role |
| --- | --- |
| `run_capture.sh` | builds nothing, runs the game on a private Xvfb display, captures a burst of PNGs, prints a JSON summary |
| `metrics.py` | objective stability metrics and pass/fail gates for a frame burst |
| `tune.py` | the search loop: apply an edit, capture, score, keep or revert |
| `qa_prompt.txt` | the vision judge's prompt: what EXPLOSION / WOBBLE / COLLAPSE mean |
| `vision.py` | optional direct-API path for calling a vision model on frames |

`run_capture.sh` needs `Xvfb`, `xwininfo`, `xdpyinfo` and ImageMagick's `import`
on `PATH`. `Xvfb` comes from the `xvfb` package (`apt install xvfb`); the script
also accepts a binary at `tools/physics-qa/tools/Xvfb` if one is dropped there,
which is deliberately not committed — a vendored 2MB third-party binary is not
worth the permanent history.

## Build and run

Setting this up on a new machine — including the two branch traps and the sibling
layout — is covered in **[SETUP.md](SETUP.md)**. Verify a machine with:

```bash
tools/physics-qa/run_capture.sh --check
```

Then:

```bash
cargo build --release -p collision_detection

tools/physics-qa/run_capture.sh --out /tmp/shots --frames 3
python3 tools/physics-qa/metrics.py /tmp/shots/frame_*.png
```

`run_capture.sh` derives the repository root from its own location, so it can be
run from anywhere; set `REPO=/path/to/bevy_vello` to override. It syncs
`examples/collision_detection/assets` to `target/release/assets` first, because
the game resolves assets relative to the executable.

The game needs a GPU: if `/dev/dri` is not accessible to the process it falls
back to llvmpipe, where the renderer panics on a buffer binding limit. Run the
capture with access to the GPU device nodes.

## Judging: metrics decide, vision explains

**Objective metrics are the gate.** `metrics.py` measures the arena (excluding
the static debug UI panel on the left) and fails a run when:

- `mean_rmse > 12.0` — consecutive frames differ too much (jitter / thrash)
- `motion_frac > 0.15` — too much of the arena changes across the burst
- `motion_growth > 2.5` — the changed region keeps expanding pair over pair
- `no_motion` — nothing changed at all, i.e. the capture is blank

Thresholds are overridable with `--gate-*`.

The gates are **temporal** on purpose. The camera is fixed and the arena is drawn
with a lot of static detail, so a "differs from the background" mask is ~98%
unchanging scenery rather than character. Foreground area looks like a natural
stability measure and is not one — it mostly measures the map. What moves between
frames is the signal.

Measured separation:

| run | `mean_rmse` | `motion_frac` | verdict |
| --- | --- | --- | --- |
| stable baseline | 6.56 | 0.030 | ok |
| stable, no collision in window | 2.54 | 0.008 | ok |
| particle VFX over the body | 23.61 | 0.332 | fail |
| blank capture | 0.00 | 0.000 | fail (`no_motion`) |

**The vision judge is advisory.** It is good at describing what it sees and at
naming which parameter looks responsible; it is not reliable as a gate. On
near-identical frames the same model returned `OK` and then `EXPLOSION`. Use its
`suspect_params` / `suggested_direction` to pick the next edit, and the metrics
to decide whether the edit helped.

## The tuning loop

`tune.py` is the mechanical half: it applies an edit, captures, scores, and keeps
or reverts. It never guesses physics — the choice of which knob to move is yours
or the agent's, informed by the judge's `suspect_params` / `suggested_direction`.

```bash
python3 tools/physics-qa/tune.py knobs                      # what can be tuned, and where it is now
python3 tools/physics-qa/tune.py run --out /tmp/t0          # baseline score
python3 tools/physics-qa/tune.py try --set softbody.substeps=3 \
        --out /tmp/t1 --baseline 6.6                        # keep only if it beats 6.6
python3 tools/physics-qa/tune.py sweep --candidates cands.json --out-root /tmp/sweep
python3 tools/physics-qa/tune.py restore --from-file /tmp/sweep/original.character.json
```

Score is lower-is-better: `mean_rmse + 100 * motion_frac`, plus a large penalty
when any gate trips.

Two properties worth knowing:

- **Edits are textual.** The asset is patched in place rather than re-serialised,
  so a one-parameter trial shows up as exactly the lines it changed. A naive JSON
  round-trip rewrites every float (`0.000001` → `1e-06`) and turns a one-line
  change into a 68-line diff.
- **Reverts are byte-exact.** The exact prior file text is snapshotted and put
  back, including on capture failure or interrupt, so a failed trial leaves no
  trace. `sweep` additionally writes `original.character.json` next to its output.

`knobs` only allows the parameters listed in `KNOBS`, each with a range it will
not leave. Widen the table to explore further.

### Muting VFX for capture

`run_capture.sh` exports `PHYSICS_QA_CAPTURE=1` when it launches the game. The
game should read that variable and suppress particle/VFX layers.

This matters because a collision spawns a burst of red particles plus an
expanding red circle. The effect covers the character silhouette, and a judge
looking at the frame reads the red burst as a physics explosion — the single
largest source of false positives seen so far (one such frame had ~10k saturated
red pixels against a ~200-pixel baseline). Mute it and the judge sees the body.

Until the game honours the flag, expect a collision during the capture window to
inflate `motion_frac` and fail an otherwise stable configuration.

## Driving the judge from an agent

If the tuning loop is driven by an agent that can read images itself, let the
agent do it: the credential stays with the agent's model provider and no script
in this repository ever handles a key. `vision.py` exists for the cases where
that is not possible.

Two things to know when running a vision model over these frames:

1. **The model must declare image input.** A harness will refuse a `read_image`
   call on a text-only model before the request is made.
2. **Give a reasoning model room.** With a tight `max_tokens` it can spend the
   whole budget on reasoning and return empty content with
   `finish_reason: length`.

## Results so far

| run | frames | metrics | vision | note |
| --- | --- | --- | --- | --- |
| run1 | blank | fail (`no_motion`) | — | capture raced the window; frames were 250-byte stubs |
| run2 | valid | ok, `mean_rmse` 6.6 | OK | stable baseline |
| run3 | valid | fail, `mean_rmse` 23.6 | EXPLOSION | collision VFX covered the body; needs re-running once the game honours the capture flag |
| verify | valid | ok, `mean_rmse` 2.5 | — | quiet window, no collision VFX |

Run-to-run variance is high because the scene contains two characters (the player
and an AI-driven enemy), so frames are not reproducible. Compare runs using the
metrics, and re-run a suspect configuration rather than trusting one capture.
