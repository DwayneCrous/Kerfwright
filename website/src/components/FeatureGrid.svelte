<script lang="ts">
  import { animate, inView, stagger } from 'motion';
  import { onMount } from 'svelte';

  const features = [
    { k: 'brep', title: 'Parametric B-rep Modelling', crates: 'kw-brep · kw-model · kw-feature',
      body: 'Own B-rep kernel: exact quadrics and NURBS, SSI-based Booleans, fillets, shell/draft, persistent topological names and a feature timeline with incremental rebuild.',
      icon: 'M12 3l8 4.5v9L12 21l-8-4.5v-9L12 3zm0 0v9m8-4.5L12 12m-8-4.5L12 12' },
    { k: 'sketch', title: 'Sketcher & Constraint Solver', crates: 'kw-sketch · kw-dxf',
      body: 'Sparse LM solver on a Dulmage–Mendelsohn decomposition. Live DOF colouring, minimal conflict sets, G0/G1/G2 joints, deterministic across thread counts.',
      icon: 'M4 20L20 4M4 4h4v4H4zM16 16h4v4h-4zM4 14a6 6 0 016-6' },
    { k: 'cam', title: 'Multi-Axis CAM', crates: 'kw-cam · kw-gsim',
      body: 'Adaptive clearing, drop-cutter, tri-dexel stock, five-axis kinematics and five post-processors, checked by independent G-code interpreters.',
      icon: 'M12 2v6m0 0a4 4 0 100 8 4 4 0 000-8zm0 8v6m-8-8h4m8 0h4' },
    { k: 'fea', title: 'FEA Simulation', crates: 'kw-mesh3d · kw-fem',
      body: 'Tetrahedral meshing, TET10 static and modal analysis, stress recovery and SIMP topology optimisation, verified against closed-form references.',
      icon: 'M3 17l5-8 4 5 3-4 6 7M3 21h18' },
    { k: 'ecad', title: 'ECAD Routing', crates: 'kw-ecad · kw-fabcheck',
      body: 'Placement, router, DRC and copper pour, with Gerber, Excellon, IDF and STEP outputs cross-checked by separate fab viewers.',
      icon: 'M4 6h6v6h4v6h6M4 18h4M14 6h6' },
  ] as const;

  let grid: HTMLDivElement;

  onMount(() => {
    const cards = grid.querySelectorAll<HTMLElement>('[data-card]');
    const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (reduce) return;
    cards.forEach((c) => (c.style.opacity = '0'));
    return inView(grid, () => {
      animate(cards, { opacity: [0, 1], y: [28, 0] },
        { delay: stagger(0.08), type: 'spring', stiffness: 120, damping: 20 });
    }, { amount: 0.15 });
  });

  // pointer tracking: spotlight + a few degrees of tilt, spring-smoothed by CSS vars
  function move(e: PointerEvent) {
    const el = e.currentTarget as HTMLElement;
    const r = el.getBoundingClientRect();
    const x = (e.clientX - r.left) / r.width, y = (e.clientY - r.top) / r.height;
    el.style.setProperty('--mx', `${x * 100}%`);
    el.style.setProperty('--my', `${y * 100}%`);
    el.style.transform = `perspective(900px) rotateX(${(0.5 - y) * 5}deg) rotateY(${(x - 0.5) * 6}deg)`;
  }
  function leave(e: PointerEvent) {
    const el = e.currentTarget as HTMLElement;
    animate(el, { transform: 'perspective(900px) rotateX(0deg) rotateY(0deg)' },
      { type: 'spring', stiffness: 160, damping: 18 });
  }
</script>

<div bind:this={grid} class="grid gap-6 md:grid-cols-6">
  {#each features as f, i}
    <article data-card
      class="neu spot relative overflow-hidden p-7 will-change-transform {i < 2 ? 'md:col-span-3' : 'md:col-span-2'}"
      onpointermove={move} onpointerleave={leave}>
      <!-- liquid-glass sheen -->
      <div class="pointer-events-none absolute -right-10 -top-10 size-40 rounded-full bg-[radial-gradient(circle,var(--kw-glow),transparent_70%)]"></div>
      <div class="glass mb-5 grid size-11 place-items-center rounded-2xl">
        <svg viewBox="0 0 24 24" class="size-5 text-brass" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d={f.icon} /></svg>
      </div>
      <h3 class="text-lg font-semibold tracking-tight">{f.title}</h3>
      <p class="mt-2 text-[.95rem] leading-relaxed text-soft">{f.body}</p>
      <p class="mt-5 font-mono text-[11px] tracking-wider text-brass/90">{f.crates}</p>
    </article>
  {/each}
</div>
