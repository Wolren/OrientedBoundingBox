//! Compare the axis-aligned and oriented boxes on two shapes.
//!
//! Run with `cargo run --example quickstart`.

use oriented_bounding_box::geo_types::{LineString, Polygon};
use oriented_bounding_box::{solve_obb, solve_obb_aspect_fit, solve_obb_axis_aligned, ObbOptions};

/// An L shape: 4 wide, 3 tall, area 6.
fn l_shape() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 3.0),
            (0.0, 3.0),
            (0.0, 0.0),
        ]),
        vec![],
    )
}

/// A 10 by 1 rectangle rotated by 45 degrees.
fn thin_rectangle_45deg() -> Polygon<f64> {
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let rotate = |x: f64, y: f64| (x * c - y * c, x * c + y * c);
    let (long, half) = (5.0, 0.5);
    Polygon::new(
        LineString::from(vec![
            rotate(-long, -half),
            rotate(long, -half),
            rotate(long, half),
            rotate(-long, half),
            rotate(-long, -half),
        ]),
        vec![],
    )
}

fn main() -> Result<(), oriented_bounding_box::LirError> {
    let opts = ObbOptions::default();

    let l = l_shape();
    let aabb = solve_obb_axis_aligned(&l, &opts)?;
    let obb = solve_obb(&l, &opts)?;
    println!(
        "L shape: axis-aligned area {} ({}x{}), min-area area {} at {} deg",
        aabb.area, aabb.width, aabb.height, obb.area, obb.angle_deg
    );
    assert!((aabb.area - 12.0).abs() < 1e-9);
    assert!(obb.area <= 12.0 + 1e-9);

    let thin = thin_rectangle_45deg();
    let aabb = solve_obb_axis_aligned(&thin, &opts)?;
    let obb = solve_obb(&thin, &opts)?;
    println!(
        "45 deg rectangle: axis-aligned area {} ({:.3}x{:.3}), min-area area {:.6} at {:.4} deg",
        aabb.area, aabb.width, aabb.height, obb.area, obb.angle_deg
    );
    assert!(obb.area < aabb.area / 4.0);

    let fitted = solve_obb_aspect_fit(&thin, 16.0, 9.0).expect("aspect-fit solve");
    println!(
        "45 deg rectangle in a 16:9 frame: area {:.6}, fill ratio {:.4}, {} caliper intervals",
        fitted.area, fitted.fill_ratio, fitted.n_intervals
    );

    Ok(())
}
