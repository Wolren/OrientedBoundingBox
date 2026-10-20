# oriented-bounding-box

The smallest rectangle that contains a polygon at any orientation, plus the
rotation that best fits a fixed aspect-ratio frame. The axis-aligned bounding
box is the zero-degree case.

```rust
use oriented_bounding_box::geo_types::{LineString, Polygon};
use oriented_bounding_box::{solve_obb, ObbOptions};

fn main() -> Result<(), oriented_bounding_box::LirError> {
    let l_shape = Polygon::new(
        LineString::from(vec![
            (0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (1.0, 1.0), (1.0, 3.0), (0.0, 3.0), (0.0, 0.0),
        ]),
        vec![],
    );

    let obb = solve_obb(&l_shape, &ObbOptions::default())?;
    assert!(obb.area <= 12.0 + 1e-9);
    Ok(())
}
```

## Install

```toml
[dependencies]
oriented-bounding-box = "0.1"
```

Rust 1.88 or newer. Default features are empty; no C library and no system
dependency.

## API

| Function | Box |
|----------|-----|
| `solve_obb_axis_aligned` | axis-aligned, via `geo::BoundingRect` |
| `solve_obb` | minimum area, rotating calipers over the convex hull |
| `solve_obb_constrained` | minimum area, then the shorter side is expanded to satisfy `max_ratio` / `min_ratio` |
| `solve_obb_aspect_fit` | optimal rotation for a frame with a fixed `ratio_w` to `ratio_h` |
| `build_obb_frame` | the frame at one known angle, as an `ObbFrame` |

`ObbResult` carries the box polygon, its area, perimeter, width, height,
rotation angle in degrees, centroid, aspect ratio and fill ratio. On the
aspect-fit path it also carries the fill ratio at zero degrees, the improvement
over north-up and the caliper interval count.

`build_obb_frame` returns an `ObbFrame`: the frame polygon, the frame width and
height after expansion to the target ratio, the tight bounding-box width and
height of the rotated body, and the frame centre.

## Correctness

Two properties are checked on every push in `tests/corpus_invariants.rs`:

- **minimal**: the caliper area matches an independent brute-force rotation
  sweep over 20,000 angles on the hull, to 1e-4 relative. The sweep knows
  nothing about rotating calipers.
- **enclosing**: a dense sample of interior points of the polygon falls inside
  the returned box.

The min-area solve is exact for convex polygons because the optimum is always
edge-aligned, and a concave polygon has the same minimal box as its hull. The
brute-force sweep is there to catch a regression in the caliper bookkeeping,
not to approximate the answer.

## Performance

### Against GEOS, same polygon

290 Polish cadastral parcels (BDOT10k, TERYT 2261) vendored in
`benches/data/realworld_290.geojson`. Both sides get the same polygon and only
the solve is timed: the GEOS geometry is built before the clock starts, and
neither side includes coordinate conversion. GEOS runs
`MinimumRotatedRectangle`, its equivalent of the minimum-area box. Best of 5
per case.

Ratio is GEOS / native, so above 1 means we win.

| Measure | native | GEOS | Ratio |
|---------|-------:|-----:|------:|
| total, 290 parcels | **0.5 ms** | 2.2 ms | **4.23x** |
| per-case median | | | 4.25x |
| per-case p10 | | | 3.35x |
| per-case p90 | | | 5.67x |
| native faster on | 290/290 | | 100% |

The advantage is stable across size, unlike a convergent search:

| Exterior vertices | n | native | GEOS | Ratio |
|------------------:|--:|-------:|-----:|------:|
| 0-7 | 42 | 0.03 ms | 0.11 ms | 4.17x |
| 8-16 | 111 | 0.12 ms | 0.49 ms | 3.99x |
| 17-32 | 75 | 0.12 ms | 0.60 ms | 4.83x |
| 33-64 | 28 | 0.07 ms | 0.33 ms | 4.87x |
| 65-128 | 20 | 0.07 ms | 0.24 ms | 3.55x |
| 129+ | 14 | 0.12 ms | 0.44 ms | 3.85x |

Box areas agree: median 2.1e-8, p90 6.5e-7, max 3.1e-5 relative. The min-area
box is unique, so a larger disagreement would mean one side is wrong.

Regenerate with `cargo run --release --features bench-geos --example compare_apples`,
which needs `GEOS_LIB_DIR`, `GEOS_VERSION` and GEOS on `PATH`.

## Features

| Feature | Effect |
|---------|--------|
| `geo-traits` | solve over any `geo_traits::PolygonTrait`, plus `geo_types::Geometry` dispatch |
| `hotpath` | `#[hotpath::measure]` instrumentation on the solver entry points |
| `python` | PyO3 extension module |
| `extension-module` | build the Python module as an importable extension |

## Geo-traits interop

With `geo-traits` the solvers accept any foreign polygon without materialising
a `geo_types::Polygon` first, so WKB views, GeoArrow arrays and GeoZero streams
go straight in:

```rust
use oriented_bounding_box::geo_interop::solve_obb_polygon_trait;
use oriented_bounding_box::ObbOptions;

let obb = solve_obb_polygon_trait(&foreign_polygon, &ObbOptions::default())?;
```

`solve_obb_geometry` and `solve_obb_constrained_geometry` dispatch a
`geo_types::Geometry`: `Polygon` solves directly, `MultiPolygon` solves the
largest-area component, and every other variant is rejected with an error
naming it rather than silently mis-solved.

## Other interfaces

`obb_solve`, `obb_solve_constrained`, `obb_solve_aspect_fit` and
`obb_options_default` form a C ABI in `src/ffi.rs`, and the crate builds as
`rlib`, `cdylib` and `staticlib`. Panics are caught at the boundary. The
`python` feature exposes `solve_obb_py`, `solve_obb_constrained_py` and
`solve_obb_aspect_fit_py`.

## Limits

- Coordinates must be finite. A NaN or infinite coordinate is rejected with
  `LirError::InvalidPolygon`.
- A degenerate ring, such as zero area or fewer than three distinct vertices,
  is rejected.
- Holes do not affect the box, because the box encloses the exterior ring and
  a hole cannot extend past it.
- `ObbOptions::angle_samples`, `use_pca`, `use_refinement` and `xatol_deg` are
  carried fields. The caliper solve is exact and does not sample angles, so
  those four do not change the answer.

## Examples

`cargo run --example quickstart` solves an L shape and a thin rectangle at
45 degrees, where the oriented box is far tighter than the axis-aligned one.

## License

Apache-2.0
