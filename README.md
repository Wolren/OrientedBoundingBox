<div align="center">

# OrientedBoundingBox

Finds the smallest rectangle that contains a polygon at any orientation.

[![crate](https://img.shields.io/crates/v/oriented-bounding-box.svg)](https://crates.io/crates/oriented-bounding-box)
[![docs](https://docs.rs/oriented-bounding-box/badge.svg)](https://docs.rs/oriented-bounding-box)
![MSRV](https://img.shields.io/badge/rustc-1.88+-ab6000.svg)
[![Tests](https://img.shields.io/badge/tests-21%20passing-brightgreen.svg)](https://github.com/Wolren/OrientedBoundingBox/actions/workflows/ci.yml)
[![GEOS](https://img.shields.io/badge/vs%20GEOS-4.28x%20faster-blue.svg)](https://github.com/Wolren/OrientedBoundingBox#performance)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://github.com/Wolren/OrientedBoundingBox?tab=License-1-ov-file)
[![Ko-fi](https://img.shields.io/badge/Ko--fi-Support%20Wolren-FF5E5B?logo=ko-fi&logoColor=white)](https://ko-fi.com/wolren)

</div>

An oriented bounding box is the smallest rectangle of a chosen orientation that
contains a polygon. The axis-aligned box is the zero-degree case. This is the
enclosing direction, the opposite of an inscribed solver, which places a shape
inside the polygon. Two related solves are exposed: one expands the short side
until it reaches an aspect ratio, and one searches the rotation that best fits a
frame of fixed ratio. The minimum-area box is unique, so it is the answer rather
than an approximation. A non-polygonal variant is rejected with an error that
names it. The full API reference and the algorithm detail are in the crate
documentation: `cargo doc --open`, no feature needed.

## Correctness

`tests/corpus_invariants.rs` checks two properties.

The first property is minimality. The caliper area must match an independent
brute-force sweep over 20,000 angles on the hull, to 1e-4 relative. That sweep
knows nothing about calipers, so it catches a regression in the bookkeeping.

The second property is enclosure. A dense sample of interior points must fall
inside the returned box.

The workflow in `.github/workflows/ci.yml` runs the suite on every push and pull
request. It runs four feature sets, the 1.88 MSRV check, and a clippy, fmt, and
rustdoc job. No check needs an external solver.

## Performance

The corpus holds 290 Polish cadastral parcels in
`benches/data/realworld_290.geojson`. Only the solve is timed, because the GEOS
geometry is built before the clock starts. GEOS runs `MinimumRotatedRectangle`,
its equivalent of the minimum-area box. Ratio is GEOS divided by native, so above 1
means this crate wins. Both sides run serially, so the table measures the solver
and not the threading.

| Measure | OrientedBoundingBox | GEOS | Ratio |
|---------|--------------------:|-----:|------:|
| total, 290 parcels | **0.5 ms** | 2.2 ms | **4.28x** |
| cases native solves faster | 290 of 290 | 0 of 290 | 100% |

The ratio varies little by case. Across the 290 parcels:

| Per-case ratio | Value |
|----------------|------:|
| p10 | 3.50x |
| median | 4.26x |
| p90 | 5.29x |

The margin holds across size, because the solve does not converge:

| Exterior vertices | n | OrientedBoundingBox | GEOS | Ratio |
|------------------:|--:|--------------------:|-----:|------:|
| 0-7 | 42 | 0.03 ms | 0.12 ms | 4.08x |
| 8-16 | 111 | 0.13 ms | 0.50 ms | 3.96x |
| 17-32 | 75 | 0.11 ms | 0.54 ms | 4.78x |
| 129+ | 14 | 0.12 ms | 0.50 ms | 4.25x |

The `rayon` feature adds `solve_obb_slice`, which returns bit-identical results in
input order and runs 4.2x faster over the same parcels on 12 cores.

The two solvers agree on the answer as well. Box areas differ by a median of
2.1e-8 relative, and by at most 3.1e-5. The minimum-area box is unique, so a
larger gap would mean that one side is wrong.

Regenerate with
`cargo run --release --features bench-geos --example compare_apples`. The build
needs the GEOS C library. The GEOS dependency reads `GEOS_LIB_DIR` and
`GEOS_VERSION` when pkg-config cannot find an install.

## Integration with the geo ecosystem

Enable `geo-traits` to accept a foreign polygon directly. WKB views, GeoArrow
arrays, and GeoZero streams then need no conversion.

| Function | Accepts |
|----------|---------|
| `solve_obb_polygon_trait` | any `PolygonTrait` with `f64` coordinates, minimum area |
| `solve_obb_constrained_polygon_trait` | any `PolygonTrait`, with the ratio constraints |
| `solve_obb_aspect_fit_polygon_trait` | any `PolygonTrait`, with a fixed frame ratio |
| `solve_obb_geometry` | `geo_types::Geometry`, polygon only, minimum area |
| `solve_obb_constrained_geometry` | `geo_types::Geometry`, polygon only, with ratio constraints |

The dispatch functions reject a non-polygonal variant with
`LirError::NotSupported`, and the message names the variant.

## Python bindings

The `python` feature builds the extension module `oriented_bounding_box`. Build it
with maturin, or with `cargo build --features python,extension-module`.

| Function | Signature |
|----------|-----------|
| `solve_obb_py` | `(exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None, angle_samples=None, use_pca=None, use_refinement=None, xatol_deg=None)` |
| `solve_obb_constrained_py` | `(exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None)` |
| `solve_obb_aspect_fit_py` | `(exterior, ratio_w, ratio_h, holes=None)` |

Rings arrive as `(x, y)` tuples. A rejected input raises `ValueError`.

## C API

`src/ffi.rs` is always compiled, so the C ABI needs no feature. Build with
`cargo build --release`.

| Symbol | Signature |
|--------|-----------|
| `obb_solve` | `(coords, coords_len, holes, holes_len, options, out) -> c_int` |
| `obb_solve_constrained` | `(coords, coords_len, holes, holes_len, options, out) -> c_int` |
| `obb_solve_aspect_fit` | `(coords, coords_len, holes, holes_len, ratio_w, ratio_h, out) -> c_int` |
| `obb_options_default` | `() -> ObbOptionsC` |

`coords` is flat `[x0, y0, ...]` exterior ring data. Holes arrive as an array of
`HoleInput`, each holding its own flat coordinate pointer. `ObbResultC`,
`ObbOptionsC`, and `HoleInput` are `#[repr(C)]`.

Each function returns 0 on success and -1 on failure. A panic becomes -1, and a
null or short buffer is rejected instead of crashing.

## Features

| Feature | Description | Default |
|---------|-------------|---------|
| `geo-traits` | Generic solves over any `geo_traits` polygon, plus `geo_types::Geometry` dispatch | no |
| `rayon` | Parallel `solve_obb_slice` batch | no |
| `hotpath` | `hotpath::measure` instrumentation on the solver entry points | no |
| `python` | PyO3 extension module named `oriented_bounding_box` | no |
| `extension-module` | Build the Python module as an importable extension | no |
| `bench-geos` | Comparison only: links the GEOS C library so benchmarks can cross-check the native solver | no |

## License

Apache-2.0
