#!/usr/bin/env python3
"""Independent-tool check #2: open every Kerfwright STEP export with Gmsh's OpenCASCADE kernel
(`gmsh.model.occ.importShapes`), compare volume / centroid / face count with the manifest, and
mesh the solid (surface mesh) to prove the B-rep is consumable by a mesher.

Gmsh exposes only OCCT's default-precision mass integration (no eps argument). That is ~3e-4 off for
swept NURBS bodies even though OCCT itself, asked for eps=1e-10 (occt_check_ours.py), agrees with
Kerfwright to 1e-7 and Gmsh's own surface meshes converge quadratically to the Kerfwright volume. The
mass tolerance here is therefore 1e-3; the 1e-5 criterion is checked by occt_check_ours.py.

usage: gmsh_check.py <dir> [tol=1e-3]
"""
import json, os, sys
import gmsh

d = sys.argv[1]
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 1e-3
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
bad, worst, n = [], 0.0, 0
for line in open(os.path.join(d, "manifest.jsonl")):
    m = json.loads(line)
    n += 1
    gmsh.clear()
    try:
        ents = gmsh.model.occ.importShapes(os.path.join(d, m["file"]))
        gmsh.model.occ.synchronize()
        vols = [t for dim, t in ents if dim == 3]
        faces = gmsh.model.getEntities(2)
        if len(vols) != 1:
            bad.append((m["file"], f"{len(vols)} volumes"))
            continue
        vol = gmsh.model.occ.getMass(3, vols[0])
        c = gmsh.model.occ.getCenterOfMass(3, vols[0])
        ext = max(m["bbox_max"][i] - m["bbox_min"][i] for i in range(3))
        dv = abs(vol - m["volume"]) / m["volume"]
        dc = max(abs(c[i] - m["centroid"][i]) for i in range(3)) / ext
        worst = max(worst, dv, dc)
        gmsh.option.setNumber("Mesh.MeshSizeMax", ext / 6)
        gmsh.model.mesh.generate(2)
        nt = len(gmsh.model.mesh.getElementsByType(2)[0])
        if dv > tol or dc > tol or len(faces) != m["faces"] or nt == 0:
            bad.append((m["file"], dict(dvol=dv, dcen=dc, faces=len(faces), exp_faces=m["faces"], tris=nt)))
    except Exception as e:  # noqa: BLE001
        bad.append((m["file"], f"exception {e}"))
gmsh.finalize()
print(f"files checked: {n}   failures: {len(bad)}   worst relative deviation: {worst:.2e}")
for b in bad[:30]:
    print("FAIL", *b)
sys.exit(1 if bad else 0)
