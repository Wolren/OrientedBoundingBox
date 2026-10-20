<div align="center">

# OrientedBoundingBox

Finds the smallest rectangle that contains a polygon at any orientation. It also
finds the rotation that best fits a fixed aspect-ratio frame.

![MSRV](https://img.shields.io/badge/rustc-1.88+-ab6000.svg)
[![Tests](https://img.shields.io/badge/tests-22%20passing-brightgreen.svg)](https://github.com/Wolren/OrientedBoundingBox/actions/workflows/ci.yml)
[![GEOS](https://img.shields.io/badge/vs%20GEOS-4.28x%20faster-blue.svg)](https://github.com/Wolren/OrientedBoundingBox#performance)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://github.com/Wolren/OrientedBoundingBox?tab=License-1-ov-file)
[![Ko-fi](https://img.shields.io/badge/Ko--fi-Support%20Wolren-FF5E5B?logo=ko-fi&logoColor=white)](https://ko-fi.com/wolren)

</div>

An oriented bounding box is the smallest rectangle of a chosen orientation that
contains a polygon. The axis-aligned box is the zero-degree case. This is the
enclosing direction, the opposite of an inscribed solver, which places a shape
inside the polygon.

## API

| Function | Box |
|----------|-----|
| `solve_obb_axis_aligned` | Axis-aligned, via `geo::BoundingRect` |
| `solve_obb` | Minimum area, by rotating calipers over the hull |
| `solve_obb_constrained` | Minimum area, then expand the short side to a ratio |
| `solve_obb_aspect_fit` | Best rotation for a frame of fixed `ratio_w` to `ratio_h` |
| `build_obb_frame` | The frame at one known angle, as an `ObbFrame` |
| `solve_obb_slice` | Solve a batch with rayon, behind the `rayon` feature |

`ObbResult` reports the box polygon and the area. It also reports the perimeter,
the width and height, the angle in degrees, the centroid, the aspect ratio, and
the fill ratio. On the aspect-fit path it also reports the north-up fill, the
gain over it, and the caliper interval count.

`build_obb_frame` returns an `ObbFrame`. That struct reports the frame polygon,
the frame width and height after expansion, the body width and height, and the
frame centre.

`solve_obb_axis_aligned` ignores its options argument.

Run `cargo run --example quickstart` for a complete program.

## How it solves

The minimum-area solve rotates calipers over the convex hull. The optimum is
always edge-aligned, so the calipers visit each hull edge and evaluate one
candidate angle per edge. The answer is exact, not sampled. A concave polygon
needs no special case, because its hull has the same minimal box.

A hull is convex, so each support vertex advances monotonically as the angle
grows. Both solves track four support indices and walk them forward, so the
work is O(h) rather than the O(h^2) of projecting every vertex at every
angle. A hull of 4096 vertices solves in 1.3 ms, and the aspect-fit solve in
2.0 ms.

The constrained solve takes that box and expands the shorter side until the
ratio satisfies `max_ratio` and `min_ratio`. The aspect-fit solve searches the
rotation that maximises how much of a fixed-ratio frame the polygon fills.

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
its equivalent of the minimum-area box. Ratio is GEOS divided by native. Both
sides run serially, so the table measures the solver and not the threading.

| Measure | native | GEOS | Ratio |
|---------|-------:|-----:|------:|
| total, 290 parcels | **0.5 ms** | 2.2 ms | **4.28x** |
| cases native solves faster | 290 of 290 | 0 of 290 | 100% |

The ratio varies little by case. Across the 290 parcels:

| Per-case ratio | Value |
|----------------|------:|
| p10 | 3.50x |
| median | 4.26x |
| p90 | 5.29x |

The margin holds across size, because the solve does not converge:

| Exterior vertices | n | native | GEOS | Ratio |
|------------------:|--:|-------:|-----:|------:|
| 0-7 | 42 | 0.03 ms | 0.12 ms | 4.08x |
| 8-16 | 111 | 0.13 ms | 0.50 ms | 3.96x |
| 17-32 | 75 | 0.11 ms | 0.54 ms | 4.78x |
| 129+ | 14 | 0.12 ms | 0.50 ms | 4.25x |

The `rayon` feature adds `solve_obb_slice`, which returns bit-identical results
in input order and runs 4.2x faster over the same parcels on 12 cores.

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

The `python` feature builds the extension module `oriented_bounding_box`. Build
it with maturin, or with `cargo build --features python,extension-module`.

| Function | Signature |
|----------|-----------|
| `solve_obb_py` | `(exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None, angle_samples=None, use_pca=None, use_refinement=None, xatol_deg=None)` |
| `solve_obb_constrained_py` | `(exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None)` |
| `solve_obb_aspect_fit_py` | `(exterior, ratio_w, ratio_h, holes=None)` |

The result class reports `area`, `perimeter`, `angle_deg`, `width`, `height`,
`aspect_ratio`, `fill_ratio`, `centroid_x`, `centroid_y`, `polygon_wkt`,
`north_fill`, `improve_pct`, and `n_intervals`. The box polygon arrives as WKT
text, and it is empty when no polygon was produced. A rejected input raises
`ValueError`, and so does a solver panic.

## C API

`src/ffi.rs` is always compiled. It exposes `obb_solve`, `obb_solve_constrained`,
`obb_solve_aspect_fit`, and `obb_options_default`. Flat `[x0, y0, ...]`
coordinate arrays plus `HoleInput` entries go in, and `ObbResultC` comes out.

`ObbResultC` carries the box as ten doubles, four corners plus the first corner
repeated. The ring is zero-filled on the aspect-fit path, where no polygon is
produced.

Each function returns 0 on success and -1 on failure. A panic becomes -1, and a
null or short buffer is rejected instead of crashing.

The crate builds as `rlib`, `cdylib`, and `staticlib`, so `cargo build
--release` produces the shared library without an extra feature.

## Features

| Feature | Description | Default |
|---------|-------------|---------|
| `bench-geos` | Comparison only: links the GEOS C library so benchmarks can cross-check the native solver. Never compiled into the solve path. | no |
| `geo-traits` | Generic solves over any `geo_traits` polygon, plus `geo_types::Geometry` dispatch | no |
| `rayon` | Adds `solve_obb_slice` for parallel batches | no |
| `hotpath` | `hotpath::measure` instrumentation on the solver entry points | no |
| `python` | PyO3 extension module named `oriented_bounding_box` | no |
| `extension-module` | Build the Python module as an importable extension | no |

## License

Apache-2.0
