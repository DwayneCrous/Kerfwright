import * as THREE from 'three';

export interface AssemblyParams {
  teeth: number; // gear tooth count
  bolts: number; // flange bolt count
}

export interface Part {
  name: string;
  object: THREE.Object3D;
  /** Displacement (in model units, +Z = up the shaft axis) when fully exploded. */
  explode: number;
}

export interface Assembly {
  root: THREE.Group;
  parts: Part[];
  materials: THREE.MeshStandardMaterial[];
  edgeMaterial: THREE.LineBasicMaterial;
  dispose(): void;
}

// Model space is Z-up like the CAD app; the root is rotated so Z points up in Three's Y-up world.
const TAU = Math.PI * 2;

function ring(r: number, n: number, i: number, phase = 0): [number, number] {
  const a = phase + (i / n) * TAU;
  return [Math.cos(a) * r, Math.sin(a) * r];
}

function circlePath(r: number, cx = 0, cy = 0): THREE.Path {
  const p = new THREE.Path();
  p.absarc(cx, cy, r, 0, TAU, true); // holes wind the opposite way
  return p;
}

function plateShape(w: number, h: number, r: number): THREE.Shape {
  const s = new THREE.Shape();
  const x = -w / 2, y = -h / 2;
  s.moveTo(x + r, y);
  s.lineTo(x + w - r, y); s.absarc(x + w - r, y + r, r, -Math.PI / 2, 0, false);
  s.lineTo(x + w, y + h - r); s.absarc(x + w - r, y + h - r, r, 0, Math.PI / 2, false);
  s.lineTo(x + r, y + h); s.absarc(x + r, y + h - r, r, Math.PI / 2, Math.PI, false);
  s.lineTo(x, y + r); s.absarc(x + r, y + r, r, Math.PI, Math.PI * 1.5, false);
  return s;
}

/** Spur-gear outline: trapezoidal teeth, bore and lightening holes. */
function gearShape(teeth: number, rRoot: number, rTip: number, bore: number): THREE.Shape {
  const s = new THREE.Shape();
  const pitch = TAU / teeth;
  for (let i = 0; i < teeth; i++) {
    const a = i * pitch;
    const pts: [number, number][] = [
      [rRoot, a], [rTip, a + pitch * 0.2], [rTip, a + pitch * 0.5], [rRoot, a + pitch * 0.7],
    ];
    pts.forEach(([r, ang], k) => {
      const x = Math.cos(ang) * r, y = Math.sin(ang) * r;
      if (i === 0 && k === 0) s.moveTo(x, y); else s.lineTo(x, y);
    });
  }
  s.closePath();
  s.holes.push(circlePath(bore));
  const web = (rRoot + bore) / 2;
  const holeR = Math.max(1.5, (rRoot - bore) * 0.22);
  for (let i = 0; i < 5; i++) {
    const [x, y] = ring(web, 5, i);
    s.holes.push(circlePath(holeR, x, y));
  }
  return s;
}

function extrude(shape: THREE.Shape, depth: number, z0: number, curveSegments = 48): THREE.BufferGeometry {
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments });
  g.translate(0, 0, z0);
  return g;
}

function lathe(profile: [number, number][], segments = 64): THREE.BufferGeometry {
  const g = new THREE.LatheGeometry(profile.map(([r, z]) => new THREE.Vector2(r, z)), segments);
  g.rotateX(Math.PI / 2); // LatheGeometry spins about Y; we want Z
  return g;
}

export function buildAssembly(params: AssemblyParams): Assembly {
  const teeth = Math.max(8, Math.min(48, Math.round(params.teeth)));
  const bolts = Math.max(3, Math.min(12, Math.round(params.bolts)));

  const geometries: THREE.BufferGeometry[] = [];
  const materials: THREE.MeshStandardMaterial[] = [];
  const mat = (color: number, metalness: number, roughness: number) => {
    const m = new THREE.MeshStandardMaterial({ color, metalness, roughness });
    materials.push(m);
    return m;
  };
  const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x0d1116, transparent: true, opacity: 0.55 });

  const steel = mat(0x6b7480, 0.85, 0.42);
  const brass = mat(0xb48f58, 0.9, 0.34);
  const bright = mat(0xaeb8c6, 0.9, 0.28);
  const chrome = mat(0xd3dae3, 1.0, 0.18);
  const dark = mat(0x2d323a, 0.75, 0.45);

  const root = new THREE.Group();
  const parts: Part[] = [];

  const addPart = (name: string, explode: number, items: [THREE.BufferGeometry, THREE.MeshStandardMaterial][]) => {
    const g = new THREE.Group();
    g.name = name;
    for (const [geo, m] of items) {
      geometries.push(geo);
      g.add(new THREE.Mesh(geo, m));
      const e = new THREE.EdgesGeometry(geo, 32);
      geometries.push(e);
      g.add(new THREE.LineSegments(e, edgeMaterial));
    }
    root.add(g);
    parts.push({ name, object: g, explode });
  };

  // 1 ─ base plate (z 0‥6) with four mounting holes
  const plate = plateShape(132, 92, 10);
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) plate.holes.push(circlePath(4.2, sx * 56, sy * 36));
  addPart('Base plate', -26, [[extrude(plate, 6, 0), steel]]);

  // 2 ─ flanged housing: bolt-circle flange + turned boss
  const flange = new THREE.Shape();
  flange.absarc(0, 0, 38, 0, TAU, false);
  flange.holes.push(circlePath(13));
  for (let i = 0; i < bolts; i++) {
    const [x, y] = ring(30, bolts, i, Math.PI / bolts);
    flange.holes.push(circlePath(2.7, x, y));
  }
  const boss = lathe([[13, 11], [20, 11], [20, 22], [17, 25], [13, 25]]);
  addPart('Flanged housing', -9, [[extrude(flange, 5, 6), brass], [boss, brass]]);

  // 3 ─ flange bolts (hex head + shank)
  const boltItems: [THREE.BufferGeometry, THREE.MeshStandardMaterial][] = [];
  for (let i = 0; i < bolts; i++) {
    const [x, y] = ring(30, bolts, i, Math.PI / bolts);
    const head = new THREE.CylinderGeometry(4.6, 4.6, 3.6, 6).rotateX(Math.PI / 2).translate(x, y, 12.8);
    const shank = new THREE.CylinderGeometry(2.5, 2.5, 12, 24).rotateX(Math.PI / 2).translate(x, y, 5);
    boltItems.push([head, dark], [shank, dark]);
  }
  addPart('Bolts', 24, boltItems);

  // 4 ─ stepped shaft
  const shaft = lathe([[0, 6], [7.5, 6], [7.5, 37], [6, 37], [6, 58], [0, 58]], 48);
  addPart('Shaft', 0, [[shaft, chrome]]);

  // 5 ─ spur gear
  const rTip = 22;
  addPart('Gear', 18, [[extrude(gearShape(teeth, rTip - 3.4, rTip, 7.5), 9, 27, 24), bright]]);

  // 6 ─ retaining nut + washer (extruded with a bore so they sit on the shaft)
  const hex = new THREE.Shape();
  for (let i = 0; i < 6; i++) {
    const [x, y] = ring(9.6, 6, i);
    if (i === 0) hex.moveTo(x, y); else hex.lineTo(x, y);
  }
  hex.closePath();
  hex.holes.push(circlePath(6));
  const washer = new THREE.Shape();
  washer.absarc(0, 0, 10.5, 0, TAU, false);
  washer.holes.push(circlePath(6));
  addPart('Retaining nut', 40, [[extrude(hex, 5, 38.2), brass], [extrude(washer, 1.2, 37), steel]]);

  // centre on the plate/shaft stack, put Z up
  root.rotation.x = -Math.PI / 2;
  root.position.y = -22;

  return {
    root, parts, materials, edgeMaterial,
    dispose() {
      geometries.forEach((g) => g.dispose());
      materials.forEach((m) => m.dispose());
      edgeMaterial.dispose();
    },
  };
}
