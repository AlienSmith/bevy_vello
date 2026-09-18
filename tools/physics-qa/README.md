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
- `edge_frac > 0.02` — the disturbance reaches the arena border, i.e. a body is
  leaving the arena
- `no_motion` — nothing changed at all, i.e. the capture is blank

The first three thresholds are overridable with `--gate-rmse`,
`--gate-motion-frac` and `--gate-motion-growth`. `edge_frac > 0.02` is currently
hard-coded.

**The verdict is the `ok` field, not the exit code.** `metrics.py` exits 0 even
when the run fails its gates — a failing run is a successful measurement. Read
`ok` / `suspect` from the JSON; do not write `metrics.py ... && echo pass`.

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

Three things to know when running a vision model over these frames:

1. **The model must declare image input.** A harness will refuse a `read_image`
   call on a text-only model before the request is made. This is a property of
   the *model*, not of the file or the tool, so the same call succeeds or fails
   depending only on which model the reading agent runs on.
2. **Give a reasoning model room.** With a tight `max_tokens` it can spend the
   whole budget on reasoning and return empty content with
   `finish_reason: length`.
3. **A delegated child may inherit the parent's model.** See below; this is the
   one that silently defeats the whole setup.

### Routing a delegated judge to a vision model

The natural design — delegate the judging to a child agent and let it call
`read_image` — fails by default on a harness whose *reasoning* model is
text-only, because a delegated child inherits the parent's model. The failure is
loud and clear once it happens (`model "..." does not declare image input`), but
the cause is not where it looks: it is not the image, the path, or the tool.

Whether a delegation tool can select a child model at all is decided by two
separate things, and both must hold:

- the delegation tool is configured with model selection enabled, **and**
- a Host setting actually supplies the allowed routes.

The second is what fails in practice. It is a Host setting, off by default:

```yaml
subagent-model-selection:
  enabled: true
  allowedModels:
    - provider: <provider>
      model: <vision-capable-model>
```

With it off, the delegation tool's schema has no `provider` / `model` /
`reasoning_effort` fields at all, so the capability is effectively invisible and
no error mentions the setting. Enabling it also adds a `list_subagent_models`
tool for discovering the advertised routes; the exact provider and model ids are
whatever the harness's own provider block declares.

Two caveats when enabling it:

- **It is sampled per session, for a fresh session only.** It applies to the
  next session, not the current one, and children inherit their parent's policy.
- **A fork-style tool may omit model selection on purpose**, to keep the
  inherited conversation prefix eligible for cache reuse. A tool that inherits
  the whole conversation can therefore be unable to see images even when a plain
  delegation tool can.

### Continuity without a resident child

A judge does not have to stay resident to be re-consulted. Because the frames are
local files, a *fresh* child given the same paths and the earlier answer as text
can answer follow-up questions about a specific detail — re-reading a local file
costs nothing, so continuity can be synthesised by re-briefing rather than
inherited.

This matters because the two properties do not always arrive together: some ways
of delegating to a chosen model run to completion and return only text, with no
durable child to message afterwards. Re-briefing is then the only route, and it
works.

This is the cheaper default anyway. A judge that returns a verdict and exits
avoids holding a child open for the length of a sweep, and it keeps each verdict
independent of the last one's framing.

### The judge describes better than it adjudicates

Two observations from running a vision model over these frames, both arguing for
the metrics-first design above:

- **Perception is decent; interpretation is not.** Asked to locate the red
  collision VFX, the model placed it within ~20% of its true bounding box
  (claimed ~160x190 at x550-715; actual 131x157 at x562-692). Asked what it
  *was*, it called the particle burst a "soft-body blob" and reported a
  character overlapping it. It measured the pixels well and then drew the wrong
  conclusion from them — which is exactly the failure mode the metrics exist to
  catch.
- **A follow-up question can confirm a description but cannot settle a
  classification.** Whether a red region is one solid mass or many discrete
  particles is answerable, but not reliably from a natural-language answer;
  measure it instead (the frame above is ~12k red pixels at ~59% fill inside its
  bounding box).

## Interfaces

Everything here is scriptable, which is the point: the loop is meant to be driven
by a program, not by hand.

### `run_capture.sh`

```
run_capture.sh --out DIR [--frames N] [--interval S] [--settle S] [--binary PATH]
run_capture.sh --check
```

Prints a JSON summary on stdout: `{ok, alive_after_capture, frames: [...], log}`.
`ok` is true when at least one frame was captured. `alive_after_capture` reports
whether the game was still running at the end — useful for telling "the physics
exploded" apart from "the process died".

Exit codes:

| code | meaning |
| --- | --- |
| 0 | capture completed (check the JSON, not the code) |
| 1 | the game exited during startup, or no window ever appeared |
| 2 | bad arguments |
| 5 | a stale game window was already on the display |
| 6 | preflight failed (see `--check`) |

Exit 5 exists because a leftover instance owns the window name and would be the
thing captured, producing black or identical frames that look like a physics
result.

### `metrics.py`

```
metrics.py [--json] [--gate-rmse F] [--gate-motion-frac F] [--gate-motion-growth F] FRAME...
```

Emits one JSON object: `ok`, `suspect`, `frames`, `panel_cols`, `arena_cols`,
`mean_rmse`, `frame_rmse`, `motion_px`, `motion_frac`, `motion_bbox`,
`motion_growth`, `edge_frac`, `bg_color`.

`bg_color` is reported for sanity-checking a capture and is deliberately not
gated: a black or blank frame is obvious there. `panel_cols` is the detected
left UI panel, so a wrong detection is visible rather than silent.

Exits 0 whether or not the gates pass — see the note above.

### `tune.py sweep` candidates file

A JSON list of `"knob=value"` strings:

```json
["softbody.substeps=3", "softbody.substeps=4", "collision.pull_compliance_scaler=1.2"]
```

`sweep` tries them best-first and reverts the losers. It also writes
`original.character.json` beside its output root, so a sweep is recoverable even
if the process is interrupted.

### `vision.py`

```
vision.py [--base-url URL] [--model ID] [--key-env NAME] [--max-tokens N] \
          --prompt-file qa_prompt.txt FRAME...
```

Reads the key from the environment only — never from a credential file — and
exits non-zero if the variable is unset. `--key-env` names the variable;
`--max-tokens` defaults to 4000, which is not generous for a reasoning model.

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

