import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { animate } from 'motion';
import { buildAssembly, type Assembly, type AssemblyParams } from './assembly';

/**
 * Framework-agnostic Three.js viewer. The Svelte island owns the UI;
 * this class owns the GL context and guarantees cleanup.
 */
// Aim slightly below the model centre so it sits above the glass control bar.
const TARGET_Y = -16;
// Resting zoom; the exploded view pulls back so every part stays in frame.
const ZOOM_REST = 1.05;
const ZOOM_EXPLODED = 0.78;

export class KwViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(32, 1, 1, 2000);
  private controls: OrbitControls;
  private assembly: Assembly | null = null;
  private pivot = new THREE.Group(); // turntable
  private explode = 0;
  private explodeAnim?: { stop(): void };
  private wireframe = false;
  private running = false;
  private visible = true;
  private raf = 0;
  private reduceMotion: boolean;
  private ro: ResizeObserver;
  private io: IntersectionObserver;
  private mo: MutationObserver;
  private pmrem: THREE.PMREMGenerator;
  private envTex: THREE.Texture;

  constructor(private host: HTMLElement, params: AssemblyParams) {
    this.reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.domElement.style.cssText = 'display:block;width:100%;height:100%;touch-action:pan-y';
    host.appendChild(this.renderer.domElement);

    // Image-based lighting from a procedural studio room: no HDR download.
    this.pmrem = new THREE.PMREMGenerator(this.renderer);
    this.envTex = this.pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environment = this.envTex;
    const key = new THREE.DirectionalLight(0xffffff, 1.4);
    key.position.set(80, 140, 90);
    this.scene.add(key, this.pivot);

    this.camera.position.set(150, 105, 170);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.controls.enablePan = false;
    this.controls.enableZoom = false; // keep page scroll (Lenis) unhijacked
    this.controls.minPolarAngle = 0.25;
    this.controls.maxPolarAngle = Math.PI * 0.62;
    this.controls.target.set(0, TARGET_Y, 0);
    this.controls.autoRotate = !this.reduceMotion;
    this.controls.autoRotateSpeed = 0.9;
    this.controls.addEventListener('start', () => (this.controls.autoRotate = false));

    this.setParams(params);
    this.syncTheme();

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(host);
    this.io = new IntersectionObserver(([e]) => {
      this.visible = e.isIntersecting;
      this.visible ? this.start() : this.stop();
    });
    this.io.observe(host);
    // follow the site theme toggle (data-theme on <html>)
    this.mo = new MutationObserver(() => this.syncTheme());
    this.mo.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    document.addEventListener('visibilitychange', this.onVis);

    this.resize();
    this.renderOnce();
    this.start();
  }

  setParams(params: AssemblyParams) {
    if (this.assembly) {
      this.pivot.remove(this.assembly.root);
      this.assembly.dispose();
    }
    this.assembly = buildAssembly(params);
    this.pivot.add(this.assembly.root);
    this.applyExplode();
    this.applyWireframe();
    this.syncTheme();
    this.renderOnce();
  }

  /** Spring the exploded view in or out. */
  setExploded(on: boolean) {
    this.explodeAnim?.stop();
    this.explodeAnim = animate(this.explode, on ? 1 : 0, {
      type: 'spring', stiffness: 90, damping: 14, mass: 0.9,
      onUpdate: (v: number) => { this.explode = v; this.applyExplode(); this.renderOnce(); },
    });
  }

  setWireframe(on: boolean) {
    this.wireframe = on;
    this.applyWireframe();
    this.renderOnce();
  }

  resetView() {
    this.controls.reset();
    this.camera.position.set(150, 105, 170);
    this.controls.target.set(0, TARGET_Y, 0);
    this.controls.autoRotate = !this.reduceMotion;
  }

  destroy() {
    this.stop();
    this.explodeAnim?.stop();
    this.ro.disconnect(); this.io.disconnect(); this.mo.disconnect();
    document.removeEventListener('visibilitychange', this.onVis);
    this.controls.dispose();
    this.assembly?.dispose();
    this.envTex.dispose(); this.pmrem.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  // ── internals ──
  private applyExplode() {
    this.assembly?.parts.forEach((p) => (p.object.position.z = p.explode * this.explode));
    this.camera.zoom = ZOOM_REST + (ZOOM_EXPLODED - ZOOM_REST) * this.explode;
    this.camera.updateProjectionMatrix();
  }
  private applyWireframe() {
    this.assembly?.materials.forEach((m) => { m.wireframe = this.wireframe; });
  }
  private syncTheme() {
    const dark = document.documentElement.dataset.theme !== 'light';
    this.renderer.toneMappingExposure = dark ? 0.95 : 1.1;
    if (this.assembly) {
      this.assembly.edgeMaterial.color.set(dark ? 0x05080c : 0x20262e);
      this.assembly.edgeMaterial.opacity = dark ? 0.7 : 0.5;
    }
    this.renderOnce();
  }
  private onVis = () => (document.hidden ? this.stop() : this.visible && this.start());
  private resize() {
    const w = this.host.clientWidth || 1, h = this.host.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderOnce();
  }
  private renderOnce() {
    if (!this.running) this.renderer.render(this.scene, this.camera);
  }
  private start() {
    if (this.running) return;
    this.running = true;
    this.raf = requestAnimationFrame(this.tick);
  }
  private stop() {
    this.running = false;
    cancelAnimationFrame(this.raf);
  }
  private tick = () => {
    if (!this.running) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.raf = requestAnimationFrame(this.tick);
  };
}
