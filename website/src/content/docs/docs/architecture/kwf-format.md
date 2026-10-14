---
title: The .kwf project file
description: "The container layout, the schema versions, what is stored, compatibility rules and the interchange formats."
sidebar:
  order: 2
---

A project is a single `.kwf` file, which is a **ZIP archive**. Writing is deterministic: the same document always gives the same bytes.

## Contents

| Entry | Contents |
|---|---|
| `manifest.json` | The format name, the schema version, `min_reader_version`, the kernel version and a hash of the recipe |
| `recipe.cbor` | The feature document: sketches, features, timeline order, parameters, suppression flags and the rollback marker |
| `assembly.cbor` | Components, joints, rigid groups and links |
| `cam.cbor` | Setups, stock, operations, and embedded copies of the machine, post and tools used |
| `fea.cbor` | Studies, supports, loads (results are not saved) |
| `board.cbor` | The ECAD board |

Entries other than the manifest and the recipe are written through `kw-store`, which also holds the drawing sheets. CBOR is a compact binary encoding of JSON-like data.

## The recipe is the only truth

Geometry is never stored. The file holds the *recipe*: the sketches, the features and how they were specified. Opening a file rebuilds the model. The file format does not write cache entries, and a file with no caches is exactly what the determinism test rebuilds on 1, 4 and 32 threads. Thumbnails are not written either.

The consequence is that a file is small, and a model is always consistent with the kernel that opens it. The cost is that opening a large model takes the time of a cold rebuild (see [How the engine rebuilds](/docs/concepts/parametric-engine/)).

## Schema versions and compatibility

| Schema | Adds |
|---|---|
| 1 | The base document |
| 2 | Suppression flags and the rollback bar |
| 3 | The tilt of datum planes, with the sheet-metal, SubD and assembly entries |

Migration is a chain of functions applied to the decoded tree, so an old file passes through each step in turn. The rules:

- **Older files open.** Files written by frozen v1, v2 and v3 writers all read to identical models.
- **Newer files are read.** A file from a *newer* writer opens. **Unknown fields and unknown ZIP entries survive a re-save** because the freshly written recipe is merged over the one that was read.
- **Too-new files are refused.** If `min_reader_version` is above what your build understands, the file is refused with a message and is not touched.
- **Hostile files are bounded.** The reader limits entry sizes and CBOR nesting depth. In testing, 400 mutated or truncated files and arbitrary byte strings never caused a panic.

## What the app does around it

The welcome page creates a project by saving `<name>.kwf` at once, so a project always has a file. Imports (STEP, IGES, STL) are stored *inside* the project as the file's text, so the `.kwf` stays self-contained. A linked component keeps its own copy in the project so a missing file never stops it opening.

## Interchange formats

| Direction | Formats |
|---|---|
| Import | STEP AP203, AP214, AP242; IGES 5.3; STL (binary and ASCII); DXF R2000 (sketch) |
| Export | STEP; STL; OBJ with MTL; glTF and GLB; DXF (sketch, flat pattern, drawing); PDF (drawing); Gerber RS-274X; Excellon; IDF 3.0; pick-and-place CSV; bill of materials CSV |

See [File formats](/docs/reference/file-formats/) for the details and limits of each.

:::note
The landing-page 3D preview is generated procedurally in the browser. It does not read `.kwf` files. The `kw-io` crate can export glTF and GLB, which is the route for showing a real model on the web.
:::
