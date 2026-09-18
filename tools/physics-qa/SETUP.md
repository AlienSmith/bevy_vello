# Setting up on a new machine

Two repositories are involved, and **both have a default branch that does not
work**. Cloning either one normally succeeds and gives you a tree that cannot
build the game, so follow the branch names below exactly.

## 1. Clone both repositories as siblings

`bevy_vello/Cargo.toml` depends on `study_vello` through path dependencies
(`path = "../study_vello"`), so the two must sit next to each other:

```
parent-directory/
├── bevy_vello/      # the game: examples/collision_detection
└── study_vello/     # the engine: vello, velato, vello_physics
```

```bash
git clone -b ray_trace    <bevy_vello-url>  bevy_vello
git clone -b transmission <study_vello-url> study_vello
```

Why the branch matters:

| repo | default branch | problem |
| --- | --- | --- |
| `bevy_vello` | `main` | upstream v0.4 — contains no `examples/collision_detection` and no `tools/` |
| `study_vello` | `study` | `integrations/` has only `vello_svg`; `velato` and `vello_physics` are missing, and `bevy_vello` depends on both |

The layout is deliberate, not incidental: it is the same shape used across all
workspaces here, and the path dependencies are relative by design.

## 2. System packages

```bash
sudo apt install xvfb x11-utils imagemagick
python3 -m pip install numpy pillow
```

| package | needed for |
| --- | --- |
| `xvfb` | the off-screen display the game renders into |
| `x11-utils` | `xwininfo` / `xdpyinfo`, to find and target the game window |
| `imagemagick` | `import`, to capture the window |
| `numpy`, `pillow` | `metrics.py` |

## 3. Build

```bash
cd bevy_vello
cargo build --release -p collision_detection
```

`Cargo.lock` is not committed, so the first build resolves dependencies from
scratch and may pick up newer patch versions than another machine has.

## 4. GPU access

The renderer needs a real GPU. Without access to `/dev/dri` (and `/dev/nvidia*`
on NVIDIA) Bevy selects the llvmpipe software adapter and then panics:

```
Buffer binding 4 range 201326592 exceeds `max_*_buffer_binding_size` limit 134217728
```

This is a hard requirement, so a container or VM without GPU passthrough cannot
run the capture. Confirm which adapter was chosen by checking the game log for
`AdapterInfo`; a working run reports `device_type: DiscreteGpu`, a broken one
`device_type: Cpu`.

## 5. Verify the environment

```bash
tools/physics-qa/run_capture.sh --check
```

This validates the branch, the sibling layout, the binary, the command-line
tools, the Python packages and Xvfb, and exits non-zero with the specific fix for
anything missing. It launches nothing.

Then a real run:

```bash
tools/physics-qa/run_capture.sh --out /tmp/shots --frames 3
python3 tools/physics-qa/metrics.py /tmp/shots/frame_*.png
```

## 6. Vision judge (optional)

The pipeline's metrics need no credentials and no network. Only the vision judge
does, and the intended path is an agent that can read images itself — that way
the credential stays with the agent's model provider and no script in this
repository ever handles a key.

If the loop runs under a harness, the harness needs a model that declares image
input, otherwise a `read_image` call is refused before it is made. Configure that
in the harness's own settings, not in this repository: the key is supplied to the
harness through the environment variable it already expects, and neither the
settings file nor the credential store belongs in a git repository.

`vision.py` is the fallback for batch runs. It reads the key from the environment
**only** and never from a credential file:

```bash
export VOLC_API_KEY=...            # name is configurable with --key-env
python3 tools/physics-qa/vision.py --prompt-file tools/physics-qa/qa_prompt.txt shot*.png
```

## Troubleshooting

**`failed to load manifest ... study_vello/integrations/velato/Cargo.toml`** —
`study_vello` is missing, is not a sibling of `bevy_vello`, or is on the wrong
branch. See step 1.

**`no game source at .../examples/collision_detection`** — `bevy_vello` is on
`main`. Check out `ray_trace`.

**`Path not found: .../target/release/assets/...`** — the game resolves assets
relative to the executable. `run_capture.sh` syncs them automatically; if you run
the binary by hand, copy `examples/collision_detection/assets` to
`target/release/assets` first, or set `BEVY_ASSET_ROOT`.

**`preflight failed` with exit code 6** — read the listed items; each carries the
command that fixes it.

**Black or identical frames** — a stale game instance is probably already on the
display and owns the window being captured. `run_capture.sh` refuses to start in
that case (exit 5); kill the old process.
