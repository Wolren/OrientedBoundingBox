//! Minimal-area OBB by rotating calipers over convex-hull edge directions.

use super::super::{ObbOptions, ObbResult};
use super::common::{hull_ccw, width_depth_seed, width_depth_walk};
use crate::shared::{rotate_polygon_around, LirError, Result};
use geo::Area;
use geo_types::{Coord, LineString, Point, Polygon};

/// Minimal-area OBB via rotating calipers over convex-hull edge directions.
///
/// For each convex-hull edge direction the bounding-box area follows in closed
/// form from the extent of the hull along that direction and its normal.
///
/// The extent along `theta` is also the extent along `theta + pi`, so the loop
/// folds the axis into `[0, pi/2)` and reads both extents from one projection.
/// A hull edge and the edge perpendicular to it then share work, which halves
/// the projection passes without changing the answer.
///
/// Complexity: O(h^2) in the number of distinct hull edge directions, because
/// each candidate angle projects every hull vertex.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb_rotating_calipers(
    poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    let pts = hull_ccw(poly).ok_or_else(|| LirError::InvalidPolygon("degenerate hull".into()))?;

    let n = pts.len();
    let mut angles: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let dx = pts[j].x - pts[i].x;
        let dy = pts[j].y - pts[i].y;
        if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
            continue;
        }
        // A box at angle a and one at a + pi/2 have the same area, so only the
        // first quadrant of orientations needs a candidate.
        let a = dy.atan2(dx).rem_euclid(std::f64::consts::FRAC_PI_2);
        angles.push(a);
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

    if angles.is_empty() {
        return Err(LirError::InvalidPolygon("no valid edges".into()));
    }

    // Centre the hull once. Projected extents are translation invariant, but the
    // reconstruction below works in this frame.
    let mut cx = 0.0f64;
    let mut cy = 0.0f64;
    for pt in &pts {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= n as f64;
    cy /= n as f64;

    let mut best_area = f64::INFINITY;
    let mut best_angle = 0.0;
    let mut best_w = 0.0;
    let mut best_h = 0.0;
    let mut best_bb_min = Coord { x: 0.0, y: 0.0 };
    let mut best_bb_max = Coord { x: 0.0, y: 0.0 };

    // The hull is convex, so each support index advances monotonically as the
    // angle grows. Seeding once and walking gives O(h) overall instead of the
    // O(h^2) that a full projection per angle costs.
    let mut support = width_depth_seed(&pts, angles[0]);
    for &angle in &angles {
        let ((w, h), next) = width_depth_walk(&pts, angle, support);
        support = next;

        if w * h < best_area {
            best_area = w * h;
            best_angle = angle;
            best_w = w;
            best_h = h;

            let (c, s) = (angle.cos(), angle.sin());
            let mut min_x = f64::INFINITY;
            let mut max_x = f64::NEG_INFINITY;
            let mut min_y = f64::INFINITY;
            let mut max_y = f64::NEG_INFINITY;
            for pt in &pts {
                let dx = pt.x - cx;
                let dy = pt.y - cy;
                let rx = dx * c + dy * s;
                let ry = -dx * s + dy * c;
                if rx < min_x {
                    min_x = rx;
                }
                if rx > max_x {
                    max_x = rx;
                }
                if ry < min_y {
                    min_y = ry;
                }
                if ry > max_y {
                    max_y = ry;
                }
            }
            best_bb_min = Coord { x: min_x, y: min_y };
            best_bb_max = Coord { x: max_x, y: max_y };
        }
    }

    let centroid = Point::new(cx, cy);

    let bb_poly = Polygon::new(
        LineString::from(vec![
            Coord {
                x: best_bb_min.x,
                y: best_bb_min.y,
            },
            Coord {
                x: best_bb_max.x,
                y: best_bb_min.y,
            },
            Coord {
                x: best_bb_max.x,
                y: best_bb_max.y,
            },
            Coord {
                x: best_bb_min.x,
                y: best_bb_max.y,
            },
            Coord {
                x: best_bb_min.x,
                y: best_bb_min.y,
            },
        ]),
        vec![],
    );

    let angle_deg = best_angle.to_degrees();
    let rotated = rotate_polygon_around(&bb_poly, angle_deg, &Point::new(0.0, 0.0));
    let ext: Vec<Coord<f64>> = rotated
        .exterior()
        .0
        .iter()
        .map(|c| Coord {
            x: c.x + centroid.x(),
            y: c.y + centroid.y(),
        })
        .collect();
    let obb_poly = Polygon::new(LineString::from(ext), vec![]);

    let poly_area = poly.unsigned_area();
    let fill = if best_area > 0.0 {
        poly_area / best_area
    } else {
        0.0
    };
    let aspect = if best_h > 0.0 {
        (best_w / best_h).max(best_h / best_w)
    } else {
        1.0
    };

    Ok(ObbResult {
        polygon: Some(obb_poly),
        area: best_area,
        perimeter: 2.0 * (best_w + best_h),
        angle_deg,
        width: best_w,
        height: best_h,
        centroid: Some(centroid),
        aspect_ratio: aspect,
        fill_ratio: fill,
        north_fill: 0.0,
        improve_pct: 0.0,
        n_intervals: 0,
    })
}
