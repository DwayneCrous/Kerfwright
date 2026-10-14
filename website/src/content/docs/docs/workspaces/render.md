---
title: Render
description: "The real-time viewport and the progressive path tracer with a denoiser."
sidebar:
  order: 5
---

Render makes pictures of the model. Start it with `--workspace render`, or `--render-demo` for the demo part.

## Two modes

**Realtime** is the viewport you use while modelling: physically based shading, ambient occlusion, topological edge lines and weighted-blended transparency, with a picking buffer for hover and selection.

**Path Trace** accumulates samples progressively on the GPU, then a denoiser cleans the image. Save a PNG when it has converged. `--render-pt` starts in this mode with 128 samples per pixel.

## The real-time pipeline

```text
depth + ID pass → opaque PBR → transparency (WBOIT) → ambient occlusion (GTAO) → edge lines → composite
```

- **Picking:** the ID pass writes a face or edge id for every pixel. It is read back asynchronously, and drives hover and selection.
- **Edges:** screen-space quads of constant pixel width, so lines stay crisp at any zoom.
- **Transparency** uses weighted blended order-independent transparency. It is an approximation: against an exact sorted blend of two translucent layers the worst channel difference was 24/255.

## The path tracer

An LBVH built on the CPU (Morton codes and Karras's construction) feeds WGSL kernels for traversal, shading and accumulation. The denoiser is an à-trous wavelet filter that divides by first-hit albedo.

## Measured

| Check | Result |
|---|---|
| Frame time, 1,003,424 triangles (about 30 % transparent) with transparency, ambient occlusion and edges, 1080p | p95 **2.59 ms** (720p: 1.50 ms), over 600 orbit frames with GPU timestamps |
| Path tracer against a 16,384-sample f64 CPU reference | Relative MSE 0.79 % at 16 spp, 0.18 % at 64, 0.003 % at 4,096 |
| Denoised image quality | 30.53 dB PSNR at 64 spp (28.07 dB before denoising) |
| Lambert furnace test | 1.00000 on CPU and GPU |
| GGX directional albedo | Equal to quadrature within 0.03 % |

## Limits

- The path tracer was tested on a small scene, a Cornell box of 1,302 triangles at 96 × 96. The 1 % MSE criterion is met at 16 spp because the metric is dominated by bright regions; PSNR is the better noise measure.
- The denoiser divides by the first-hit albedo, so a glossy ball is filtered as if diffuse and its reflections are smoothed.
- Render uses its own GPU device and a **fixed 960 × 600** image.
- BVH construction runs on the CPU.
