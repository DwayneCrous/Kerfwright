---
title: Scripting
description: "Build and edit documents from Rhai scripts with the kw-script builder API."
sidebar:
  order: 3
---

`kw-script` embeds the [Rhai](https://rhai.rs) scripting language over the document builder. A script builds a document, evaluates it and can save it as a `.kwf` file. A scripted document is the same document the Rust builder makes, and saves to the same bytes (that is a test).

There is no scripting window in the application yet; scripts run through the crate.

## Example

```rust
let d = new_doc();
let w = d.param("width", "40 mm");
let s = d.rect_sketch("Base sketch", "xy", 0, 0, "width", "width / 2");
let base = d.extrude("Base", s, "10 mm", "new");          // new | join | cut | cut_down | join_down | intersect
let hs = d.circle_sketch("Hole sketch", "xy:10", 20, 10, 6);
d.extrude("Hole", hs, 12, "cut_down");
let boss = d.extrude("Boss", s, "5 mm", "join");
d.linear_pattern("Row", boss, 8, 0, 0, 3);
let r = d.evaluate();            // { ok, evaluated, failures }
let f = d.fillet("F", [[d.side_ref(base, 0), d.side_ref(base, 1)]], 2);
d.sheet("Bracket", "t", 0.44, "t * 1.5", 30, ["F40", "B90", "F25"]);
d.subd("Pillow", "cube", 20, 0.01);
d.set_param(w, "50 mm"); d.suppress(f, true); d.move_to(f, 2); d.rollback(f); d.save("part.kwf");
d
```

## Conventions

- **Numbers are millimetres**, and counts are plain numbers.
- **Strings are expressions with units**, such as `"width / 2"` or `"10 mm"`.
- `evaluate()` returns whether it succeeded, how many features were evaluated and the list of failures.
- Edit calls (`set_param`, `suppress`, `move_to`, `rollback`) go through the same validation as the timeline, so an invalid move is refused in the same way.

## Safety

Scripts run under limits on operations, depth and size. They **cannot reach the file system** (only `save` is allowed) and **cannot start processes**.

## Coverage

The script API covers the common builder calls (parameters, sketches, extrude, patterns, fillet, sheet metal, subdivision, and the edit calls above), not the whole document API.
