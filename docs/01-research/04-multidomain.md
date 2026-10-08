# 04 — Multi-Domain Modules: CAM, FEA, ECAD, Viewport

## A. CAM

### A.1 Offsetting, Minkowski sums, configuration space
* **Offset of a 2D region by tool radius R** = Minkowski sum with a disk (outward) / erosion (inward). Exact offsets of lines+arcs stay lines+arcs: offset each segment, trim/extend at intersections, discard invalid loops via winding-number test. [Public]
* **Configuration-space obstacle** = obstacle ⊕ (−tool). General (non-round) tools: Minkowski sum of convex decomposition pieces; used for collision-free linking moves and for holder/shank clearance.
* Rust today: `cavalier_contours` (arc-aware polyline offset), `i_overlay`/Clipper2 ports (polygon booleans, line-segment offsets). Evaluate; build in-house only for the gaps (offset of splines within tolerance).

### A.2 Medial axis / Voronoi for 2D adaptive clearing
* The **Voronoi diagram of the boundary segments/arcs** (Held's VRONI; Boost.Polygon voronoi; OpenVoronoi) restricted to the interior is the **medial axis (MA)**. The **MAT** attaches `r(p)` = largest inscribed disc radius at each MA point. [Public]
* Uses in CAM: (1) find where the pocket is narrower than the tool (`r < R`) → rest-machining regions; (2) place helical/ramp entry at the maximum-clearance point (`argmax r`); (3) pocket topology (branches = slots, leaves = corners) for ordering; (4) estimate local stepover limits.
* Rust: `spade` gives point/CDT Voronoi; segment-Voronoi is a **gap** to build (see 05).

### A.3 Adaptive (constant-engagement) clearing
Public principle: bound the tool's **radial engagement angle** φ so chip load and force stay constant. For radial width of cut `a_e` and tool radius `R`: `φ = arccos(1 − a_e/R)`. [Public]
Algorithm sketch **[Design]**:
1. Maintain a polygon `Cleared` (initially empty) and `Stock` = region − `Cleared`.
2. From the current tool position, search the direction θ that makes the *area removed per step* match the target (equivalently, the engagement arc against `Stock` = target φ); step forward; update `Cleared ← Cleared ∪ disc(path)` with polygon booleans.
3. When no direction satisfies the bound (narrow slot, corner) insert a **trochoidal** loop (circular path advanced along the slot) to keep engagement ≤ φ.
4. When the local region is exhausted, **link** to the next uncleared pocket at stay-down depth if the straight link lies inside `Cleared`; otherwise retract to safe height.
5. Finish with a constant-offset contour pass.
* ⚠ Several adaptive-clearing methods and medial-axis toolpath methods are patented (e.g. US 9,164,503). Implement from the open literature on engagement-angle control; obtain a legal review before shipping.

### A.4 3D/surface strategies
* **Drop-cutter / push-cutter** (Anders Wallin's OpenCAMLib thesis): lower the tool onto a triangle mesh using per-triangle vertex/edge/facet contact formulas for flat, ball, bull, and conical cutters; BVH accelerated. Waterline = push-cutter slices at constant Z. Scallop/constant-stepover = offset on the surface in `(u,v)` or geodesic distance field. [Public/OSS]
* **Stock simulation**: tri-dexel or heightfield voxel model; GPU compute for display, CPU for gouge/collision verification.

### A.5 5-axis
* **Tool-axis control**: lead/tilt relative to surface normal, fixed-tilt, point/line-drive, collision-avoidance by C-space search (tool, holder, spindle vs. part/fixtures via BVH distance queries).
* **Swarf** (flank): ruled-surface approximation; tool axis aligned with ruling; deviation check along the ruling.
* **Sweep volume** of tool along path (for gouge detection): union of convex hull of consecutive tool poses.
* **Machine kinematics**: model as a kinematic chain (screw/product-of-exponentials). For common table-table / head-table / head-head 5-axis machines with rotary axes A/B/C, the **analytic inverse kinematics** from (tool-tip position **p**, tool axis **k**) in workpiece frame gives the two rotary solutions (±); pick by continuity and axis limits, handle the **singularity** when the tool axis aligns with the first rotary axis (the second axis angle becomes undefined) by interpolating. General machines: damped least squares `Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e`. Output either RTCP/TCPC (machine does compensation) or pre-computed XYZ with inverse-time feed (G93). [Public]
* **Post-processing**: template DSL → G-code dialects (Fanuc, Siemens 840D, Heidenhain, Haas, Mach3/LinuxCNC).

## B. FEA / CAE

### B.1 Mesh generation
| Method | Notes |
|---|---|
| **Delaunay 3D** (Bowyer–Watson) + boundary recovery (constrained Delaunay à la TetGen, Si) | Quality via refinement (Shewchuk/Ruppert 3D, circumradius-to-shortest-edge bound ≈ 2); slivers need flips/perturbation/optimisation |
| **Advancing front** (Löhner; Lo) | Good boundary layers/gradation; harder robust implementation |
| **Octree / hex-dominant** | Preferred for topology optimisation, voxel FEA |
| Element order | Linear TET4 locks/over-stiff; default **TET10** for stress. HEX8/HEX20 for voxel work. |
* Rust today: `tritet` wraps C Triangle/TetGen (not pure Rust); `delaunay` crate (D-dimensional, exact predicates) — evaluate. Pure-Rust quality tet mesher with surface recovery is a **gap**.

### B.2 Assembly and solve: `K·u = F`
* Element stiffness `Kₑ = ∫ BᵀDB dV` (TET4: constant B ⇒ `V·BᵀDB`; TET10: 4-point Gauss).
* Assembly: per-thread COO triplets → sort-merge into CSR (or colouring for lock-free scatter). Boundary conditions by elimination (preferred) or penalty.
* Solvers:
  * **Direct** sparse Cholesky `K = LLᵀ` with AMD/nested-dissection ordering: robust, up to ~10⁶ DOF in 3D (fill-in memory ~ n^{4/3}). `faer-sparse` supplies supernodal LLT/LDLT. [Public]
  * **PCG** with Jacobi / IC(0) / smoothed-aggregation **AMG** for larger models; convergence ∝ √κ. Rigid-body modes → singular K unless constrained.
  * Modal: LOBPCG / shift-invert Lanczos. Thermal, nonlinear (Newton + line-search) later.
* Pre/post: von Mises, principal stress, safety factor, displacement; results mapped to the viewport as per-vertex scalar fields.

### B.3 Generative design
SIMP topology optimisation (`ρ^p` penalisation, density filter, optimality-criteria or MMA update) on a voxel/hex grid; then iso-surface extraction (marching cubes / dual contouring) → smoothing → SubD fit → B-rep ("Finish"). Manufacturing constraints (min member size, draw direction, overhang) as filters. [Public: Sigmund 2001, 2007]

## C. ECAD
* **Placement**: force-directed (Fruchterman–Reingold) for initial layout; simulated annealing with HPWL cost; analytic quadratic placement (spreading) for large boards.
* **Routing**: Lee–Moore BFS wave propagation (optimal on grid), **A\*** with via and layer-change costs and a Manhattan/octilinear heuristic; Hightower line-probe; negotiated congestion (PathFinder) with rip-up-and-reroute; push-and-shove for interactive routing; length-matching meanders and differential pairs.
* **DRC**: R-tree broad phase; exact clearance via segment/arc distance; copper pour = polygon booleans minus clearance offsets.
* **Outputs**: Gerber RS-274X + Excellon (public specs), IPC-2581, ODB++ (spec access varies), **IDF 3.0** (board/component 3D exchange), **STEP AP214/AP242** board assembly with component models. [Public]

## D. Graphics engine (`wgpu`)
* **Transparency**: Weighted-Blended OIT (McGuire & Bavoil 2013) — two render targets (accum RGBA16F, reveal R8), single geometry pass, composite pass; approximate but cheap and robust. Exact alternatives: per-pixel linked lists (storage buffers + atomics; memory heavy), depth peeling (N passes). **[Design]** default WBOIT, exact mode for stills.
* **Ambient occlusion**: SSAO baseline; **GTAO** (Jimenez et al. 2016) for quality at similar cost; bilateral blur; temporal accumulation.
* **GPU-driven B-rep edges**: edges rendered from *topological* B-rep edges (polylines from tessellation `edge_id`), not mesh dihedral heuristics; instanced screen-space thick-line quads in a vertex shader; depth-bias against the face pass; silhouette edges via view-dependent pass; selection/hover through an **ID buffer** (R32Uint target holding `face/edge/vertex` id) read back asynchronously.
* **Scene scale**: compute-shader frustum + Hi-Z occlusion culling, indirect draws, per-face material table; 64-bit→camera-relative 32-bit transform to avoid jitter on large models.
* **Ray tracing / Render workspace**: stable `wgpu` does not expose hardware ray-tracing as a portable feature (ray queries are experimental/native-only at best). **[Design]** build an LBVH (Karras 2012) in compute shaders and a progressive path tracer (GGX microfacet BRDF, MIS, IBL via split-sum prefiltering) in WGSL; hardware RT behind a feature flag if/when stable. Denoise with À-Trous/SVGF-style filter (OIDN is C++ ⇒ gap).
* Targets: Vulkan, Metal, DX12, WebGPU (via WASM) from one codebase.

## Sources
- Autodesk Fusion help, Adaptive Clearing — https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/3D-ADAPTIVE-READ.htm (user-facing behaviour only)
- Medial-axis toolpath patent (diligence reference, not an implementation source) — https://patents.google.com/patent/US9164503
- Held, "VRONI: An engineering approach to the reliable and efficient computation of Voronoi diagrams of points and line segments," CGTA 2001.
- Wallin, OpenCAMLib thesis (drop/push cutter); Kim/Choi on 5-axis tool-axis planning; Siciliano et al., *Robotics: Modelling, Planning and Control* (IK).
- Shewchuk, "Delaunay refinement algorithms for triangular mesh generation" and "Tetrahedral mesh generation by Delaunay refinement"; TetGen manual — https://wias-berlin.de/software/tetgen
- Sigmund, "A 99 line topology optimization code written in Matlab," 2001.
- McGuire & Bavoil, "Weighted Blended Order-Independent Transparency," JCGT 2(2), 2013 — https://www.jcgt.org/published/0002/02/09/paper.pdf
- Jimenez et al., "Practical Real-Time Strategies for Accurate Indirect Occlusion," SIGGRAPH 2016 (GTAO).
- Karras, "Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d Trees," HPG 2012.
- Lee (1961), Hart/Nilsson/Raphael (1968, A*), McMurchie & Ebeling (PathFinder, 1995).
