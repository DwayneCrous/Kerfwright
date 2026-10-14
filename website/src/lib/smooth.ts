import Lenis from 'lenis';

/** One Lenis instance for the whole page; no-op for reduced-motion users. */
export function initSmoothScroll(): Lenis | null {
  if (typeof window === 'undefined') return null;
  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return null;
  if ((window as any).__lenis) return (window as any).__lenis;

  const lenis = new Lenis({
    duration: 1.15,
    easing: (t) => 1 - Math.pow(1 - t, 4), // easeOutQuart
    anchors: true, // in-page #links glide instead of jumping
  });
  (window as any).__lenis = lenis;

  const raf = (t: number) => { lenis.raf(t); requestAnimationFrame(raf); };
  requestAnimationFrame(raf);
  return lenis;
}
