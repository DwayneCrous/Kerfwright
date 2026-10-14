---
title: Simulation
description: "Finite-element studies: static, modal, thermal, thermal stress and nonlinear, with supports, loads and results on the model."
sidebar:
  order: 3
---

Simulation (FEA) solves how a part behaves under load. Open it from the workspace menu, or start with `--workspace fea`.

## The flow

1. **Choose bodies and materials.** A study runs on the included bodies of the design or the posed assembly, each with a material from the library.
2. **Pick supports and loads on faces** in the 3-D view.
3. **Mesh and solve.** Meshing and solving run on a background thread, with progress and cancel.
4. **Read the result** coloured straight on the model.

A project holds a **list of studies**.

## Study types

| Study | Gives you |
|---|---|
| Linear static | Displacement and von Mises stress, with a safety factor against the yield strength |
| Modal | Natural frequencies and animated mode shapes |
| Steady-state thermal | Temperature and flux, with convection and heat generation |
| Thermal stress | A thermal solve, then the stress from the expansion |
| Nonlinear static | Large displacements, moderate strain and von Mises plasticity with linear hardening, dead loads, load stepping and a load-displacement curve |

Topology optimisation (SIMP) is implemented in `kw-fem` and verified (see Accuracy). The nonlinear study uses the first body's yield strength and tangent modulus.

## Supports and loads

Supports can be **fixed**, **per-axis rollers**, or **prescribed displacements**. Gravity is a load. Bodies that touch are **bonded** by node-to-surface ties with a penalty; overlapping bodies are refused.

A support or load stays on its face. The study keeps the face's *signature* (body, centre of the surface, area, normal) beside its tag. If numbering shifts, because a body was left out or a face was added, the study finds the face again. It follows a face through small edits: the centre may move by 2 % of the body's size and the area by 5 %. A face it cannot find is shown as **gone**, the study will not run until you remove it or put it on a face again, and the face comes back by itself if the model does.

:::note
This matching is geometric. It is *not* the kernel's [persistent naming](/docs/concepts/topological-naming/). A face that moves far in a parametric edit has to be picked again.
:::

## How it works

- **Meshing** (`kw-mesh3d`) builds a constrained Delaunay tetrahedral mesh with exact predicates, refines it for quality (radius-edge ratio 3, then 2), removes slivers by edge flips and smoothing, snaps nodes to the true surface at the end and promotes the mesh to **TET10** (ten-node quadratic tetrahedra) with a Jacobian guard.
- **Solving** (`kw-fem`) assembles TET10 elements in parallel and solves with a direct sparse Cholesky for moderate sizes or Jacobi-preconditioned conjugate gradients for large ones. Stress is recovered by L2 smoothing. Modal analysis uses shift-invert.

## Accuracy

Checked against closed-form elasticity solutions:

| Case | Error |
|---|---|
| Cantilever tip deflection (Timoshenko–Goodier) | −0.015 % (−0.001 % on a finer mesh) |
| Plate with a hole, peak hoop stress (Kirsch) | −0.085 % |
| Thick cylinder under pressure, radial displacement / hoop stress (Lamé) | 0.018 % / 0.93 % (1.31 % at edge nodes) |
| Thick sphere, displacement / hoop stress | 0.045 % / 1.26 % (1.82 % at edge nodes) |
| Bending frequencies 1 to 3 against Euler–Bernoulli with shear and rotary inertia | within 0.3 % |

Speed: a 107,811-DOF static solve takes 3.6 s, and a 1,075,275-DOF solve to a 9.0e-9 residual takes 81 s on a structured mesh. The mesh quality target (at least 99 % of elements with radius-edge ≤ 2, none inverted) holds on every measured case (99.32 % to 100 %, with 0 inverted).

## Limits

- **Mesh:** the mesher works on the *faceted* tessellation of the bodies, so curved faces are not projected onto the true surface. Steep size grading on thick curved parts is weak, and unfixed elements sit at trihedral corners. Stress at edge nodes is accurate to 1.3 % to 1.8 %.
- **Results are not saved** in the `.kwf` file.
- **Contact** is bonded only. There is no buckling analysis.
- A 1-million-DOF run on an unstructured mesh was not measured.
- The references are exact elasticity solutions, not NAFEMS tables.
- On the SIMP reference: the published paper prints no compliance number for the MBB beam. The value used (216.81) is quoted by a third-party port, and the implementation reproduces it (216.8137 in 106 iterations).
