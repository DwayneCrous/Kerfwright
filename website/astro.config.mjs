import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import svelte from '@astrojs/svelte';
import tailwindcss from '@tailwindcss/vite';

// Landing page  → src/pages/index.astro   (Astro + Svelte islands + Tailwind)
// Docs portal   → src/content/docs/docs/* (Starlight, served under /docs/)
export default defineConfig({
  integrations: [
    svelte(),
    starlight({
      title: 'Kerfwright',
      description: 'Pure-Rust, clean-room CAD / CAM / CAE / ECAD suite.',
      logo: { src: './src/assets/mark.svg', replacesTitle: false },
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/DwayneCrous/Kerfwright' },
      ],
      customCss: ['./src/styles/starlight.css'],
      components: {
        // Adds Lenis smooth scrolling to the docs as well as the landing page.
        Head: './src/components/starlight/Head.astro',
      },
      sidebar: [
        { label: 'Getting started', items: [{ autogenerate: { directory: 'docs/getting-started' } }] },
        { label: 'Architecture', items: [{ autogenerate: { directory: 'docs/architecture' } }] },
        { label: 'Kernel benchmarks', items: [{ autogenerate: { directory: 'docs/benchmarks' } }] },
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
