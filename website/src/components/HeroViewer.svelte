<script lang="ts">
  import { onMount } from 'svelte';
  import type { KwViewer } from '../lib/viewer';

  let host: HTMLDivElement;
  let viewer: KwViewer | null = null;
  let failed = $state(false);
  let ready = $state(false);

  let teeth = $state(20);
  let bolts = $state(6);
  let exploded = $state(false);
  let wire = $state(false);

  onMount(() => {
    let dead = false;
    // three.js is ~600 kB: load it after first paint so the hero text is instant.
    import('../lib/viewer')
      .then(({ KwViewer }) => {
        if (dead) return;
        viewer = new KwViewer(host, { teeth, bolts });
        ready = true;
      })
      .catch(() => (failed = true));
    return () => { dead = true; viewer?.destroy(); };
  });

  // reactive parameter rebuild — the "parametric" part of the demo
  $effect(() => { viewer?.setParams({ teeth, bolts }); });
  $effect(() => { viewer?.setExploded(exploded); });
  $effect(() => { viewer?.setWireframe(wire); });
</script>

<div class="neu relative aspect-[4/3] w-full overflow-hidden p-2 md:aspect-[16/11]">
  <div class="neu-inset absolute inset-2 overflow-hidden">
    <!-- blueprint grid under the transparent canvas -->
    <div class="absolute inset-0 opacity-60 [background-image:linear-gradient(var(--kw-line)_1px,transparent_1px),linear-gradient(90deg,var(--kw-line)_1px,transparent_1px)] [background-size:32px_32px] [mask-image:radial-gradient(70%_70%_at_50%_50%,#000,transparent)]"></div>
    <div bind:this={host} class="absolute inset-0 cursor-grab active:cursor-grabbing" role="img"
         aria-label="Interactive 3D preview of a flanged gearbox assembly. Drag to orbit."></div>
    {#if !ready && !failed}
      <div class="absolute inset-0 grid place-items-center font-mono text-xs tracking-widest text-soft">LOADING KERNEL PREVIEW…</div>
    {/if}
    {#if failed}
      <div class="absolute inset-0 grid place-items-center p-6 text-center text-sm text-soft">WebGL isn't available in this browser, so the live preview is disabled.</div>
    {/if}
  </div>

  <div class="glass absolute left-5 top-5 rounded-full px-3 py-1.5 font-mono text-[11px] tracking-wider text-soft">
    gearbox.kwf <span class="text-brass">· procedural preview</span>
  </div>

  <div class="glass absolute inset-x-4 bottom-4 flex flex-wrap items-center gap-x-4 gap-y-2 rounded-2xl px-4 py-3 text-xs">
    <label class="flex items-center gap-2">
      <span class="w-10 font-mono text-soft">teeth</span>
      <input type="range" min="8" max="40" bind:value={teeth} class="h-1 w-20 accent-[var(--kw-brass)]" aria-label="Gear tooth count" />
      <output class="w-5 font-mono tabular-nums">{teeth}</output>
    </label>
    <label class="flex items-center gap-2">
      <span class="w-10 font-mono text-soft">bolts</span>
      <input type="range" min="3" max="12" bind:value={bolts} class="h-1 w-20 accent-[var(--kw-brass)]" aria-label="Flange bolt count" />
      <output class="w-5 font-mono tabular-nums">{bolts}</output>
    </label>
    <div class="ml-auto flex gap-2">
      <button class="rounded-full px-3 py-1.5 transition {exploded ? 'neu-inset text-brass' : 'neu'}" aria-pressed={exploded} onclick={() => (exploded = !exploded)}>Explode</button>
      <button class="rounded-full px-3 py-1.5 transition {wire ? 'neu-inset text-brass' : 'neu'}" aria-pressed={wire} onclick={() => (wire = !wire)}>Wireframe</button>
      <button class="neu rounded-full px-3 py-1.5" onclick={() => viewer?.resetView()}>Reset</button>
    </div>
  </div>
</div>
