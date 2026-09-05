# ige-obb

Oriented bounding box solvers extracted from [IGE](https://github.com/GIS-Inscribed-Geometry/IGE) `ige-core/src/solvers/obb`.

Enclosing boxes: the box contains the polygon. This is the opposite direction from inscribed solvers such as LIR and MIC.

## API

- `solve_obb_axis_aligned` - axis-aligned bounding rect
- `solve_obb` - minimal-area oriented box via rotating calipers
- `solve_obb_constrained` - min-area solve plus shorter-side expansion to `max_ratio` / `min_ratio`
- `solve_obb_aspect_fit` + `build_obb_frame` - exact AFR optimal rotation for a fixed `ratio_w` / `ratio_h` frame

## Provenance

Source files under `src/obb/` are byte copies of the IGE OBB solver at extraction time. `src/shared.rs` carries only the items OBB actually uses (`LirError`, `Result`, rotation helpers). No grid, SDF, expand, or certify machinery was carried over.
