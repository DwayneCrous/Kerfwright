---
title: Build from source
description: Run the Kerfwright desktop app and the standalone tools.
sidebar:
  order: 2
---

There are no packaged binaries yet. You need a stable Rust toolchain (`rust-toolchain.toml` pins the channel and adds `clippy` and `rustfmt`).

```sh
git clone https://github.com/DwayneCrous/Kerfwright
cd Kerfwright

cargo run --release -p kw-app                 # the application
cargo run --release -p kw-app -- --demo       # skip the welcome page, open the demo part
cargo run --release -p kw-viewer -- --scene million   # 1M-triangle viewer scene
cargo run --release -p kw-sketcher -- --demo  # 2D sketcher
```

`kerfwright FILE.kwf` opens a project directly, and `--workspace NAME` picks the starting workspace.

:::note
Some `kw-app` shell tests currently fail on a clean checkout (an egui `TexturesDelta` panic), and the performance tests are slow. Run them with `--release`.
:::

## Workspaces

The ribbon's workspace menu has **Design**, **Manufacture**, **Simulation**, **Electronics** and **Render**. Sketch and Model are one workspace (Design). See [`docs/app-shell.md`](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/app-shell.md) for every command.
