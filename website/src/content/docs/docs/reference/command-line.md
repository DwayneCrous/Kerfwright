---
title: Command line
description: "Every option of the kerfwright binary, including the demo and screenshot flags used by the acceptance scripts."
sidebar:
  order: 1
---

```text
kerfwright [PROJECT.kwf] [--demo] [--workspace NAME]
           [--screenshot FILE.png] [--exit-after-frames N]
```

Through Cargo, put the application's options after `--`: `cargo run --release -p kw-app -- --demo`.

## Opening things

| Option | Effect |
|---|---|
| `PROJECT.kwf` | Open this project and skip the welcome page. If it cannot be opened, the reason is printed and the exit code is 2. |
| `--demo` | Open the demo part. |
| `--workspace NAME` | Start in a workspace: `design` (also `sketch` or `model`), `cam`, `fea`, `ecad`, `render` or `drawing`. Use it with a project or `--demo`. |
| `--edit-sketch` | Start inside the first sketch of the project. |

## Screenshots and smoke tests

These are used by `scripts/app_acceptance.sh` to photograph every screen.

| Option | Effect |
|---|---|
| `--screenshot FILE.png` | Write a PNG of the window after a few frames, then quit. |
| `--screenshot-after N` | Wait N frames before the screenshot (default 30). |
| `--exit-after-frames N` | Render N frames, then quit. A smoke test. |

## Demo flags

Each opens a prepared state of one workspace.

| Option | State |
|---|---|
| `--design-demo WHAT` | One of `idle`, `pick`, `sketch`, `extrude`, `face`, `faceextrude`, `hole`, `fillet`, `params` |
| `--open-menu TITLE` | Force a ribbon group's dropdown open, for example `CREATE` |
| `--cam-demo` | The Manufacture workspace with a prepared setup |
| `--fea-demo` | The Simulation workspace with a prepared study |
| `--ecad-demo` | The Electronics workspace with a sample board |
| `--drawing-demo` | The Drawing workspace with a sheet |
| `--render-demo` | The Render workspace on the demo part |
| `--render-pt` | Render in path-trace mode at 128 samples per pixel |

## Help and errors

`-h` or `--help` prints the usage. An unknown option or a missing value prints a message with the usage and exits with code 2.

## Related environment

`KERFWRIGHT_CONFIG` overrides the CAM library directory, which is otherwise `~/.config/kerfwright/cam/`. The recent-projects list is `~/.config/kerfwright/recent.json`.

## Other binaries

| Binary | Command |
|---|---|
| Viewer | `cargo run --release -p kw-viewer` with `--scene million`, `--scene primitives`, `--headless`, `--pick-test` |
| Sketcher | `cargo run --release -p kw-sketcher -- --demo`, with `--open FILE.dxf` and `--exit-after-frames N` |
| Primitive exporter | `cargo run -p kw-io --example export_primitives -- out/` |
