---
title: Build from source
description: "Requirements, build commands, command-line options and how to run the tests."
sidebar:
  order: 2
---

There are no packaged binaries yet, so Kerfwright is built from source with Cargo.

## Requirements

| Requirement | Detail |
|---|---|
| Rust | A stable toolchain. `rust-toolchain.toml` pins the `stable` channel and adds `clippy` and `rustfmt`. Install it with [rustup](https://rustup.rs). |
| GPU | A Vulkan, Metal or DX12 device. The viewer and renderer use `wgpu`, and the desktop shell uses `eframe` with the wgpu backend on X11 and Wayland. |
| Display | A desktop session. The GPU-dependent tests and the screenshot steps skip themselves or fail without an adapter. |
| Platform | Development and testing were on Linux (a Vulkan driver, Mesa RADV on an AMD integrated GPU). Windows and macOS have not been run. |

## Build and run

```sh
git clone https://github.com/DwayneCrous/Kerfwright
cd Kerfwright

cargo run --release -p kw-app              # the application, starting at the welcome page
```

Always use `--release`. The kernel does a lot of numerical work, and a debug build is far slower than a release build.

The application's binary is named `kerfwright`. After `cargo build --release -p kw-app` it is at `target/release/kerfwright`.

## Command-line options

```text
kerfwright [PROJECT.kwf] [--demo] [--workspace NAME]
           [--screenshot FILE.png] [--exit-after-frames N]
```

| Option | Effect |
|---|---|
| `PROJECT.kwf` | Open this project and skip the welcome page. |
| `--demo` | Open the demo part. |
| `--workspace NAME` | Start in `design`, `cam`, `fea`, `ecad`, `render` or `drawing`. |
| `--screenshot FILE.png` | Write a PNG of the window after a few frames, then quit. |
| `--exit-after-frames N` | Render N frames, then quit. Useful as a smoke test. |

Through Cargo, put the application's options after `--`:

```sh
cargo run --release -p kw-app -- --demo --workspace cam
cargo run --release -p kw-app -- path/to/bracket.kwf
```

The [command-line reference](/docs/reference/command-line/) also lists the per-workspace demo flags used for screenshots.

## The standalone tools

Two earlier tools are still built from their own crates. They are useful for checking one subsystem in isolation.

```sh
cargo run --release -p kw-sketcher -- --demo            # the 2-D sketcher on its own
cargo run --release -p kw-viewer                        # the viewer: drag orbits, right-drag pans, wheel zooms, E toggles edges, F frames
cargo run --release -p kw-viewer -- --scene million     # a one-million-triangle scene
cargo run -p kw-io --example export_primitives -- out/  # STL, OBJ, glTF and GLB of every primitive
```

## Run the tests

Each phase has an acceptance script that runs formatting, `clippy -D warnings`, the tests and the measured benchmarks for that phase.

| Script | Covers | Time |
|---|---|---|
| `scripts/phase1_acceptance.sh` | Maths, NURBS, B-rep validation, tessellation, exporters, viewer | short |
| `scripts/phase2_acceptance.sh` | Sketch solver, DXF, sketcher UI smoke run | about 3 min |
| `scripts/phase3_acceptance.sh` | Booleans, fillets, STEP and IGES, including both 100,000-pair fuzzes | about 2.5 h |
| `scripts/phase4_acceptance.sh` | Parametric engine, naming corpus, `.kwf` compatibility | about 10 min |
| `scripts/phase5_acceptance.sh` | CAM, FEA, ECAD, rendering (needs a GPU for the rendering steps) | about 25 min |
| `scripts/app_acceptance.sh` | The application's tests and a screenshot of every screen | needs a display |

To run the engine's tests for one crate, use Cargo directly:

```sh
cargo test --release -p kw-feature
cargo test --release -p kw-sketch
```

:::note
Some `kw-app` shell tests currently fail on a clean checkout (an egui `TexturesDelta` panic), and the performance tests are slow, so run them with `--release` and one at a time. The performance tests exclude each other on purpose: running two at once makes the timing meaningless.
:::

## Where project data goes

The welcome page keeps your ten most recent projects in `~/.config/kerfwright/recent.json`. Missing files stay in the list with a **Forget** button. Projects themselves are ordinary `.kwf` files wherever you save them.

## Troubleshooting

**The window opens and stays black, or the program exits at start-up.** The GPU path could not be created. Check that a Vulkan, Metal or DX12 driver is installed and that `vulkaninfo` (Linux) lists a device.

**A heavy model makes the window pause.** The feature document is evaluated on the UI thread. CAM, FEA, routing, stock simulation and denoising run on background threads and do not block. See [Threading and jobs](/docs/architecture/threading/).

**An older `.kwf` does not open.** Files written by schema versions 1, 2 and 3 all open. A file that needs a newer reader than yours is refused with a message and is not changed.
