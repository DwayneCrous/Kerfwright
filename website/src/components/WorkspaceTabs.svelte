<script lang="ts">
  import { fade, fly } from 'svelte/transition';
  import { cubicOut } from 'svelte/easing';

  // Content mirrors docs/app-shell.md (the ribbon of each workspace in kw-app).
  const spaces = [
    { id: 'design', label: 'Design', accent: '#8fa9c9',
      blurb: 'Sketch and Model as one workspace: constrained sketches, extrude / revolve / sweep / loft, fillets, shell, assemblies with joints, sheet metal and SubD.',
      tools: ['Sketch', 'Extrude', 'Revolve', 'Fillet', 'Shell', 'Assemble'], panel: ['Browser', 'Timeline', 'Parameters'] },
    { id: 'manufacture', label: 'Manufacture', accent: '#d2ae78',
      blurb: 'Setups with work coordinates and stock, 2.5-D and 3-D operations, tri-dexel stock simulation and a G-code post per setup.',
      tools: ['Setup', 'Pocket', 'Adaptive', 'Contour', 'Drill', 'Simulate'], panel: ['Setups', 'Operations', 'Post'] },
    { id: 'simulation', label: 'Simulation', accent: '#b98a78',
      blurb: 'Pick supports and loads on faces, solve a static or modal study, and colour the result straight on the model.',
      tools: ['Supports', 'Loads', 'Mesh', 'Solve', 'Results'], panel: ['Studies', 'Materials', 'Results'] },
    { id: 'electronics', label: 'Electronics', accent: '#86ab97',
      blurb: 'Place, route, pour, run DRC and write fabrication files, with the board available as STEP for the mechanical design.',
      tools: ['Place', 'Route All', 'Check', 'Pour', 'Fab files'], panel: ['Layers', 'Violations', 'Nets'] },
    { id: 'render', label: 'Render', accent: '#a79ac4',
      blurb: 'Real-time viewport or progressive path tracing with a denoiser; save a PNG when it converges.',
      tools: ['Realtime', 'Path Trace', 'Denoise', 'Save PNG'], panel: ['Appearance', 'Lighting', 'Camera'] },
  ];

  let active = $state(0);
  const cur = $derived(spaces[active]);

  function key(e: KeyboardEvent) {
    if (e.key === 'ArrowRight') active = (active + 1) % spaces.length;
    else if (e.key === 'ArrowLeft') active = (active - 1 + spaces.length) % spaces.length;
    else return;
    e.preventDefault();
    (document.getElementById(`tab-${spaces[active].id}`) as HTMLElement)?.focus();
  }
</script>

<div class="neu p-3 md:p-5">
  <div role="tablist" aria-label="Kerfwright workspaces" class="neu-inset mb-5 flex gap-1 overflow-x-auto p-1.5" tabindex="-1" onkeydown={key}>
    {#each spaces as s, i}
      <button role="tab" id="tab-{s.id}" aria-selected={active === i} aria-controls="panel-{s.id}"
        tabindex={active === i ? 0 : -1} onclick={() => (active = i)}
        class="relative whitespace-nowrap rounded-xl px-4 py-2.5 text-sm font-semibold uppercase tracking-[.12em] transition-colors {active === i ? 'text-ink' : 'text-soft hover:text-ink'}">
        {#if active === i}
          <span class="neu absolute inset-0 -z-0 rounded-xl" style="border-radius:.75rem" in:fade={{ duration: 180 }}></span>
        {/if}
        <span class="relative z-10 flex items-center gap-2">
          <span class="size-1.5 rounded-full" style="background:{s.accent}"></span>{s.label}
        </span>
      </button>
    {/each}
  </div>

  <div role="tabpanel" id="panel-{cur.id}" aria-labelledby="tab-{cur.id}" class="grid gap-5 md:grid-cols-[1.1fr_1.4fr]">
    {#key active}
      <div in:fly={{ y: 14, duration: 320, easing: cubicOut }}>
        <h3 class="text-2xl font-semibold tracking-tight">{cur.label}</h3>
        <p class="mt-3 leading-relaxed text-soft">{cur.blurb}</p>
        <ul class="mt-5 flex flex-wrap gap-2">
          {#each cur.tools as t}
            <li class="glass rounded-full px-3 py-1 font-mono text-xs">{t}</li>
          {/each}
        </ul>
      </div>

      <!-- schematic window chrome, not a screenshot -->
      <div class="neu-inset overflow-hidden p-0" in:fly={{ y: 14, duration: 380, delay: 60, easing: cubicOut }} aria-hidden="true">
        <div class="flex items-center gap-2 border-b border-line px-4 py-2.5">
          <span class="size-2.5 rounded-full bg-line"></span><span class="size-2.5 rounded-full bg-line"></span><span class="size-2.5 rounded-full bg-line"></span>
          <span class="ml-3 font-mono text-[11px] text-soft">kerfwright — {cur.label.toUpperCase()}</span>
        </div>
        <div class="flex gap-1 border-b border-line px-3 py-2">
          {#each cur.tools as t}<span class="rounded-md px-2 py-1 font-mono text-[10px] text-soft">{t}</span>{/each}
        </div>
        <div class="grid min-h-56 grid-cols-[7.5rem_1fr]">
          <div class="space-y-2 border-r border-line p-3">
            {#each cur.panel as p, i}
              <div class="rounded-md px-2 py-1.5 font-mono text-[10px] {i === 0 ? 'glass text-ink' : 'text-soft'}">{p}</div>
            {/each}
          </div>
          <div class="relative">
            <svg viewBox="0 0 240 150" class="absolute inset-0 size-full" preserveAspectRatio="xMidYMid slice">
              <defs><pattern id="g-{cur.id}" width="12" height="12" patternUnits="userSpaceOnUse"><path d="M12 0H0V12" fill="none" stroke="var(--kw-line)" stroke-width=".6" /></pattern></defs>
              <rect width="240" height="150" fill="url(#g-{cur.id})" />
              <g fill="none" stroke={cur.accent} stroke-width="1.6" stroke-linejoin="round" stroke-linecap="round">
                <path d="M60 100 L110 60 L170 76 L170 112 L120 128 Z" />
                <path d="M110 60 L120 92 L170 112M120 92 L60 100" opacity=".55" />
                <circle cx="142" cy="92" r="9" opacity=".8" />
              </g>
            </svg>
          </div>
        </div>
      </div>
    {/key}
  </div>
</div>
