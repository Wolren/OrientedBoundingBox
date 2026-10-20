# oriented-bounding-box

Finds the smallest rectangle that contains a polygon at any orientation. It also
finds the rotation that best fits a fixed aspect-ratio frame.

The axis-aligned box is the zero-degree case.

Pure Rust. The crate calls no external solver.

## Install

Add the dependency:

```toml
[dependencies]
oriented-bounding-box = "0.1"
```

Rust 1.88 or newer. Default features are empty. The crate links no C library.

## API

| Function | Box |
|----------|-----|
| `solve_obb_axis_aligned` | Axis-aligned, via `geo::BoundingRect` |
| `solve_obb` | Minimum area, by rotating calipers over the hull |
| `solve_obb_constrained` | Minimum area, then expand the short side to a ratio |
| `solve_obb_aspect_fit` | Best rotation for a frame of fixed `ratio_w` to `ratio_h` |
| `build_obb_frame` | The frame at one known angle, as an `ObbFrame` |
| `solve_obb_slice` | Solve a batch with rayon |

`ObbResult` reports the box polygon, the area, the perimeter, the width and
height, the angle in degrees, the centroid, the aspect ratio, and the fill
ratio. On the aspect-fit path it also reports the north-up fill, the gain over
it, and the caliper interval count.

`build_obb_frame` returns an `ObbFrame`. That struct reports the frame polygon,
the frame width and height after expansion, the body width and height, and the
frame centre.

Run `cargo run --example quickstart` for a complete program.

## How it solves

The minimum-area solve rotates calipers over the convex hull. The optimum is
always edge-aligned, so the calipers visit each hull edge and evaluate one
candidate angle per edge. The answer is exact, not sampled. A concave polygon
needs no special case, because its hull has the same minimal box.

The constrained solve takes that box and expands the shorter side until the
ratio satisfies `max_ratio` and `min_ratio`. The aspect-fit solve searches the
rotation that maximises how much of a fixed-ratio frame the polygon fills.

## Correctness

`tests/corpus_invariants.rs` checks two properties on every push.

The first property is minimality. The caliper area must match an independent
brute-force sweep over 20,000 angles on the hull, to 1e-4 relative. That sweep
knows nothing about calipers, so it catches a regression in the bookkeeping.

The second property is enclosure. A dense sample of interior points must fall
inside the returned box.

## Performance

The corpus holds 290 Polish cadastral parcels in
`benches/data/realworld_290.geojson`. Only the solve is timed, because the GEOS
geometry is built before the clock starts. GEOS runs `MinimumRotatedRectangle`,
its equivalent of the minimum-area box. Ratio is GEOS divided by native.

| Measure | native | GEOS | Ratio |
|---------|-------:|-----:|------:|
| total, 290 parcels | **0.5 ms** | 2.2 ms | **4.23x** |
| per-case median | | | 4.25x |
| per-case p90 | | | 5.67x |
| native faster on | 290/290 | | 100% |

The margin holds across size, because the solve does not converge:

| Exterior vertices | n | native | GEOS | Ratio |
|------------------:|--:|-------:|-----:|------:|
| 0-7 | 42 | 0.03 ms | 0.11 ms | 4.17x |
| 8-16 | 111 | 0.12 ms | 0.49 ms | 3.99x |
| 17-32 | 75 | 0.12 ms | 0.60 ms | 4.83x |
| 129+ | 14 | 0.12 ms | 0.44 ms | 3.85x |

Both sides run serially, so the table measures the solver and not the threading.
The `rayon` feature adds `solve_obb_slice`, which returns bit-identical results
in input order and runs 4.2x faster over the same parcels on 12 cores.

The two solvers agree on the answer as well. Box areas differ by a median of
2.1e-8 relative, and by at most 3.1e-5. The minimum-area box is unique, so a
larger gap would mean one side is wrong.

Regenerate with `cargo run --release --features bench-geos --example compare_apples`.

## Features

| Feature | Effect |
|---------|--------|
| `geo-traits` | Solve over any `PolygonTrait` |
| `rayon` | Solve many polygons in parallel |
| `hotpath` | Instrument the entry points with `hotpath` |
| `python` | Build a PyO3 extension module |
| `bench-geos` | Link GEOS for benchmark comparison only |

Enable `geo-traits` to accept a foreign polygon directly. WKB views, GeoArrow
arrays, and GeoZero streams then need no conversion. The dispatch function
rejects a non-polygonal geometry with an error that names the variant.

The C ABI lives in `src/ffi.rs`, and it catches panic at the boundary.

## Limits

The crate rejects a NaN or infinite coordinate with `LirError::InvalidPolygon`.
A degenerate ring fails the same way. A ring with fewer than three distinct
vertices counts as degenerate.

Holes do not affect the box. The box encloses the exterior ring, and a hole
cannot extend past it.

`angle_samples`, `use_pca`, `use_refinement`, and `xatol_deg` are carried
fields. The caliper solve is exact and samples no angles, so those four do not
change the answer.

## License

Apache-2.0
