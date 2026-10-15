<script lang="ts">
  import { onMount } from 'svelte';
  import type { KwViewer } from '../lib/viewer';

  let host: HTMLDivElement;
  let viewer = $state.raw<KwViewer | null>(null);
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

  // Changing a parameter rebuilds the model, the same idea as editing a dimension in the app.
  // Read the state before the null check: `viewer?.f(x)` short-circuits and would leave
  // the effect with no dependencies, so it would never re-run.
  $effect(() => { const p = { teeth, bolts }; viewer?.setParams(p); });
  $effect(() => { const on = exploded; viewer?.setExploded(on); });
  $effect(() => { const on = wire; viewer?.setWireframe(on); });
</script>

<figure class="sheet m-0">
  <div class="relative aspect-[4/3] w-full overflow-hidden border-b border-rule-strong md:aspect-[16/11]" style="border-color: var(--kw-rule-strong)">
    <!-- 8 mm drafting grid under the transparent canvas -->
    <div class="absolute inset-0 opacity-70 [background-image:linear-gradient(var(--kw-rule)_1px,transparent_1px),linear-gradient(90deg,var(--kw-rule)_1px,transparent_1px)] [background-size:32px_32px]" aria-hidden="true"></div>
    <div bind:this={host} class="absolute inset-0 cursor-grab active:cursor-grabbing" role="img"
         aria-label="Interactive 3D preview of a flanged gearbox assembly. Drag to orbit. The controls below change the tooth count and bolt count, explode the assembly, and switch to wireframe."></div>
    {#if !ready && !failed}
      <div class="absolute inset-0 grid place-items-center font-mono text-[.8125rem] text-soft" aria-live="polite">Loading preview…</div>
    {/if}
    {#if failed}
      <div class="absolute inset-0 grid place-items-center p-6 text-center text-[.9375rem] text-soft">WebGL isn't available in this browser, so the live preview is off.</div>
    {/if}
  </div>

  <!-- Controls: a plain toolbar under the drawing, not floating over it. -->
  <div class="flex flex-wrap items-center gap-x-6 gap-y-3 border-b px-4 py-3 text-[.9375rem]" style="border-color: var(--kw-rule-strong)">
    <label class="flex items-center gap-3">
      <span class="w-14 text-soft">Teeth</span>
      <input type="range" min="8" max="40" bind:value={teeth} class="h-6 w-28 accent-[var(--kw-dykem)]" />
      <output class="w-6 font-mono tabular-nums">{teeth}</output>
    </label>
    <label class="flex items-center gap-3">
      <span class="w-14 text-soft">Bolts</span>
      <input type="range" min="3" max="12" bind:value={bolts} class="h-6 w-28 accent-[var(--kw-dykem)]" />
      <output class="w-6 font-mono tabular-nums">{bolts}</output>
    </label>
    <div class="ml-auto flex gap-2">
      <button type="button" class="btn btn-quiet !min-h-9 !px-3 {exploded ? '!bg-panel' : ''}" aria-pressed={exploded} onclick={() => (exploded = !exploded)}>Explode</button>
      <button type="button" class="btn btn-quiet !min-h-9 !px-3 {wire ? '!bg-panel' : ''}" aria-pressed={wire} onclick={() => (wire = !wire)}>Wireframe</button>
      <button type="button" class="btn btn-quiet !min-h-9 !px-3" onclick={() => viewer?.resetView()}>Reset view</button>
    </div>
  </div>

  <!-- Title block, as on a drawing. It reports the live parameters. -->
  <figcaption class="grid grid-cols-2 font-mono text-[.8125rem] sm:grid-cols-4">
    <div class="border-r border-rule p-3"><div class="text-soft">Part</div><div>gearbox.kwf</div></div>
    <div class="p-3 sm:border-r sm:border-rule"><div class="text-soft">Parameters</div><div class="tabular-nums">z = {teeth} · n = {bolts}</div></div>
    <div class="col-span-2 border-t border-rule p-3 sm:col-span-2 sm:border-t-0"><div class="text-soft">Source</div><div>Procedural preview, not read from a .kwf file</div></div>
  </figcaption>
</figure>
