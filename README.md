# oriented-bounding-box

The smallest rectangle of a chosen orientation that contains a polygon, plus
the rotation that best fits a fixed aspect-ratio frame. The axis-aligned
bounding box is the zero-degree case.

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

## API

| Function | Box |
|----------|-----|
| `solve_obb_axis_aligned` | axis-aligned, via `geo::BoundingRect`; the options argument is ignored |
| `solve_obb` | minimum area, rotating calipers over the convex hull |
| `solve_obb_constrained` | minimum area, then the shorter side is expanded to satisfy `max_ratio` / `min_ratio` |
| `solve_obb_aspect_fit` | optimal rotation for a frame with a fixed `ratio_w` to `ratio_h` |
| `build_obb_frame` | the frame polygon for one known angle |

`ObbResult` carries the box polygon, its area, perimeter, width, height,
rotation angle in degrees, centroid, aspect ratio, fill ratio, and, on the
aspect-fit path, the fill ratio at zero degrees, the improvement over north-up
and the number of caliper intervals.

## Features

| Feature | Effect |
|---------|--------|
| `geo-traits` | generic solves over any `geo_traits` polygon, plus `geo_types::Geometry` dispatch |
| `python` | PyO3 extension module |

Default features are empty.

## Other interfaces

`obb_solve`, `obb_solve_constrained`, `obb_solve_aspect_fit` and
`obb_options_default` form a C ABI in `src/ffi.rs`, and the crate builds as
`rlib`, `cdylib` and `staticlib`. The `python` feature exposes
`solve_obb_py`, `solve_obb_constrained_py` and `solve_obb_aspect_fit_py`.

## Examples

`examples/quickstart.rs` solves an L shape and a thin rectangle that sits at
45 degrees, where the oriented box is far tighter than the axis-aligned one.

## Provenance

The solver sources come from the IGE project, split out here as a standalone
crate.

## License

Apache-2.0
