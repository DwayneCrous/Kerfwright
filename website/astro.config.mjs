import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import svelte from '@astrojs/svelte';
import tailwindcss from '@tailwindcss/vite';

// Landing page  → src/pages/index.astro   (Astro + a Svelte island + Tailwind)
// Docs portal   → src/content/docs/docs/* (Starlight, served under /docs/)
export default defineConfig({
  integrations: [
    svelte(),
    starlight({
      title: 'Kerfwright',
      description: 'CAD, CAM, FEA and ECAD in one Rust program: user guides, concepts, architecture and measured results.',
      logo: { src: './src/assets/mark.svg', replacesTitle: false },
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/DwayneCrous/Kerfwright' },
      ],
      customCss: ['./src/styles/starlight.css'],
      tableOfContents: { minHeadingLevel: 2, maxHeadingLevel: 3 },
      components: {
        // Adds Lenis smooth scrolling to the docs as well as the landing page.
        Head: './src/components/starlight/Head.astro',
      },
      sidebar: [
        { label: 'Getting started', items: [{ autogenerate: { directory: 'docs/getting-started' } }] },
        { label: 'Concepts', items: [{ autogenerate: { directory: 'docs/concepts' } }] },
        { label: 'Workspaces', items: [{ autogenerate: { directory: 'docs/workspaces' } }] },
        { label: 'Architecture', items: [{ autogenerate: { directory: 'docs/architecture' } }] },
        { label: 'Reference', items: [{ autogenerate: { directory: 'docs/reference' } }] },
        { label: 'Benchmarks', items: [{ autogenerate: { directory: 'docs/benchmarks' } }] },
        { label: 'Project', items: [{ autogenerate: { directory: 'docs/project' } }] },
      ],
      editLink: {
        baseUrl: 'https://github.com/DwayneCrous/Kerfwright/edit/main/website/',
      },
    }),
  ],
  vite: {
    // Tailwind is imported only by the landing layout; Starlight keeps its own CSS.
    plugins: [tailwindcss()],
  },
});
