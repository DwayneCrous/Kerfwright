---
title: The .kwf project file
description: One container for the design, CAM, FEA and board data.
sidebar:
  order: 2
---

A project is a single `.kwf` file. It holds the Phase 4 feature document (sketches, features, timeline, parameters) plus extra entries written through `kw-store`:

| Entry | Contents |
|---|---|
| `cam.cbor` | setups, stock, operations, posts |
| `fea.cbor` | studies, supports, loads, results |
| `board.cbor` | the ECAD board |

Interchange formats: STEP (AP203/214/242 import and export), IGES 5.3 import, DXF R2000, STL, OBJ+MTL, glTF/GLB, Gerber, Excellon, IDF, and PDF/DXF drawings.

:::note
The landing-page 3D preview is generated procedurally in the browser. It does not read `.kwf` files; the `kw-io` crate can export glTF/GLB, which is the route for showing a real model.
:::
