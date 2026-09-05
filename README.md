# oriented-bounding-box

Oriented bounding box solvers extracted from [IGE](https://github.com/GIS-Inscribed-Geometry/IGE) `ige-core/src/solvers/obb`.

Enclosing boxes: the box contains the polygon. This is the opposite direction from inscribed solvers such as LIR and MIC.

## API

- `solve_obb_axis_aligned` - axis-aligned bounding rect
- `solve_obb` - minimal-area oriented box via rotating calipers
- `solve_obb_constrained` - min-area solve plus shorter-side expansion to `max_ratio` / `min_ratio`
- `solve_obb_aspect_fit` + `build_obb_frame` - exact AFR optimal rotation for a fixed `ratio_w` / `ratio_h` frame

## C ABI

`src/ffi.rs` exposes `obb_solve`, `obb_solve_constrained`,
`obb_solve_aspect_fit`, and `obb_options_default`. Flat coordinate
arrays in, `#[repr(C)]` structs out, 0 on success, -1 on error, panics
caught at the boundary. The crate builds `cdylib` + `staticlib` + `rlib`.

## Python

Feature `python` provides the `oriented_bounding_box` module with
`solve_obb_py`, `solve_obb_constrained_py`, and
`solve_obb_aspect_fit_py`. Build an importable module with
`cargo build --features python,extension-module` (or maturin).

## Geo conventions

The Rust API solves over `geo_types::Polygon` (re-exported as
`oriented_bounding_box::geo_types` for version lock-in). Feature
`geo-traits` adds generic solves over any foreign polygon implementing
`geo_traits::PolygonTrait` plus `Geometry` dispatch
(`solve_obb_geometry`): Polygon solves directly, MultiPolygon solves the
largest-area component, anything else is rejected with a named error.

## Provenance

Source files under `src/obb/` are byte copies of the IGE OBB solver at extraction time. `src/shared.rs` carries only the items OBB actually uses (`LirError`, `Result`, rotation helpers). No grid, SDF, expand, or certify machinery was carried over.
