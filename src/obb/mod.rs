//! Oriented Bounding Box (OBB) solver.
//!
//! Finds the minimal area oriented bounding box that encloses a polygon,
//! or the optimal rotation for a fixed-aspect-ratio frame.
//!
//! ## Sub-modules
//! - `axis_aligned` - axis-aligned bounding box
//! - `oriented` - oriented solvers (min-area calipers, aspect-ratio fit)

pub mod axis_aligned;
pub mod oriented;

use crate::shared::{LirError, Result};
use geo::Area;
use geo_types::{Coord, LineString, Point, Polygon};

/// Configuration for OBB solvers.
#[derive(Debug, Clone)]
pub struct ObbOptions {
    /// Max aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Min aspect ratio; 0.0 = unconstrained.
    pub min_ratio: f64,
    /// Number of angle samples for search.
    pub angle_samples: usize,
    /// If true, use PCA for initial angle guess.
    pub use_pca: bool,
    /// If true, enable refinement after initial find.
    pub use_refinement: bool,
    /// Convergence tolerance for refinement (degrees).
    pub xatol_deg: f64,
}

impl Default for ObbOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            angle_samples: 90,
            use_pca: true,
            use_refinement: true,
            xatol_deg: 0.1,
        }
    }
}

/// Result of an OBB solve.
#[derive(Debug, Clone)]
pub struct ObbResult {
    /// The oriented bounding box as a polygon.
    pub polygon: Option<Polygon<f64>>,
    /// Area of the bounding box.
    pub area: f64,
    /// Perimeter of the bounding box.
    pub perimeter: f64,
    /// Rotation angle in degrees.
    pub angle_deg: f64,
    /// Width of the bounding box.
    pub width: f64,
    /// Height of the bounding box.
    pub height: f64,
    /// Centroid of the bounding box.
    pub centroid: Option<Point<f64>>,
    /// Aspect ratio (longer/shorter, always ≥ 1).
    pub aspect_ratio: f64,
    /// Fill ratio. On the min-area and axis-aligned paths this is polygon area
    /// divided by box area. On the aspect-fit path it is the area of the
    /// rotated bounding rectangle divided by frame area, at most 1.0 and equal
    /// to 1.0 when a rotation matches the target ratio exactly.
    pub fill_ratio: f64,
    /// On the aspect-fit path, `fill_ratio` at angle 0; 0.0 on the other paths.
    pub north_fill: f64,
    /// Percent improvement of `fill_ratio` over `north_fill`; aspect fit only.
    pub improve_pct: f64,
    /// Number of caliper intervals (aspect-fit only).
    pub n_intervals: usize,
}

impl ObbResult {
    /// Build a result with no polygon, zeroed metrics, and zero counters.
    pub fn empty() -> Self {
        Self {
            polygon: None,
            area: 0.0,
            perimeter: 0.0,
            angle_deg: 0.0,
            width: 0.0,
            height: 0.0,
            centroid: None,
            aspect_ratio: 1.0,
            fill_ratio: 0.0,
            north_fill: 0.0,
            improve_pct: 0.0,
            n_intervals: 0,
        }
    }
}

impl Default for ObbResult {
    fn default() -> Self {
        Self::empty()
    }
}

/// One frame at a known angle: the box polygon plus the measurements behind it.
///
/// `body_*` describe the tight bounding box of the rotated polygon, and
/// `frame_*` describe that box expanded to the requested aspect ratio.
#[derive(Debug, Clone)]
pub struct ObbFrame {
    /// The frame polygon, rotated back into input coordinates.
    pub polygon: Polygon<f64>,
    /// Frame width after expansion to the target ratio.
    pub frame_w: f64,
    /// Frame height after expansion to the target ratio.
    pub frame_h: f64,
    /// Tight bounding-box width of the rotated polygon.
    pub body_w: f64,
    /// Tight bounding-box height of the rotated polygon.
    pub body_h: f64,
    /// Centre of the frame in input coordinates.
    pub center: Point<f64>,
}

/// Solve many polygons in parallel using rayon.
///
/// Each polygon is independent, so results come back in input order and are
/// bit-identical to calling [`solve_obb`] on each one. Use this when the caller
/// has a batch; it does not change any single solve.
#[cfg(feature = "rayon")]
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb_slice(polys: &[Polygon<f64>], options: &ObbOptions) -> Vec<Result<ObbResult>> {
    use rayon::prelude::*;
    polys
        .par_iter()
        .map(|poly| solve_obb(poly, options))
        .collect()
}

/// Minimal-area oriented bounding box via rotating calipers.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb(poly: &Polygon<f64>, options: &ObbOptions) -> Result<ObbResult> {
    // Pre-filter NaN/Inf coordinates to avoid geo crate panic in convex hull
    for c in poly.exterior().0.iter() {
        if !c.x.is_finite() || !c.y.is_finite() {
            return Err(LirError::InvalidPolygon(
                "NaN or Inf coordinate in polygon".into(),
            ));
        }
    }
    oriented::calipers::solve_obb_rotating_calipers(poly, options)
}

/// Minimal OBB with aspect-ratio constraints.
///
/// Applies max_ratio / min_ratio from `ObbOptions` by expanding the
/// shorter side after the min-area solve.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb_constrained(poly: &Polygon<f64>, options: &ObbOptions) -> Result<ObbResult> {
    let mut result = oriented::calipers::solve_obb_rotating_calipers(poly, options)?;
    if options.max_ratio > 0.0 || options.min_ratio > 0.0 {
        let w = result.width;
        let h = result.height;
        let current_ar = (w / h).max(h / w);
        let needs_clamp = (options.max_ratio > 0.0 && current_ar > options.max_ratio)
            || (options.min_ratio > 0.0 && current_ar < options.min_ratio);
        if needs_clamp {
            let target_ar = if options.max_ratio > 0.0 && current_ar > options.max_ratio {
                options.max_ratio
            } else {
                options.min_ratio
            };
            let (new_w, new_h) = if w >= h {
                (w, w / target_ar)
            } else {
                (h * target_ar, h)
            };
            result.width = new_w;
            result.height = new_h;
            result.area = new_w * new_h;
            result.aspect_ratio = if new_h > 0.0 {
                (new_w / new_h).max(new_h / new_w)
            } else {
                1.0
            };
            result.fill_ratio = poly.unsigned_area() / result.area.max(1e-12);
            // Rebuild polygon to match the new dimensions
            if let Some(ref centroid) = result.centroid {
                let bb_poly = Polygon::new(
                    LineString::from(vec![
                        Coord {
                            x: -new_w / 2.0,
                            y: -new_h / 2.0,
                        },
                        Coord {
                            x: new_w / 2.0,
                            y: -new_h / 2.0,
                        },
                        Coord {
                            x: new_w / 2.0,
                            y: new_h / 2.0,
                        },
                        Coord {
                            x: -new_w / 2.0,
                            y: new_h / 2.0,
                        },
                        Coord {
                            x: -new_w / 2.0,
                            y: -new_h / 2.0,
                        },
                    ]),
                    vec![],
                );
                let rotated = crate::shared::rotate_polygon_around(
                    &bb_poly,
                    result.angle_deg,
                    &Point::new(0.0, 0.0),
                );
                let ext: Vec<Coord<f64>> = rotated
                    .exterior()
                    .0
                    .iter()
                    .map(|c| Coord {
                        x: c.x + centroid.x(),
                        y: c.y + centroid.y(),
                    })
                    .collect();
                result.polygon = Some(Polygon::new(LineString::from(ext), vec![]));
            }
        }
    }
    Ok(result)
}

/// Optimal rotation for a fixed-aspect-ratio frame.
///
/// Uses the exact AFR algorithm: O(n log n) rotating-calipers support
/// function with closed-form crossing roots per caliper interval.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb_aspect_fit(poly: &Polygon<f64>, ratio_w: f64, ratio_h: f64) -> Option<ObbResult> {
    oriented::aspect_fit::solve_obb_aspect_fit(poly, ratio_w, ratio_h)
}

/// Build the COVER frame polygon for a given optimal angle.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn build_obb_frame(
    poly: &Polygon<f64>,
    theta_rad: f64,
    ratio_w: f64,
    ratio_h: f64,
) -> Option<ObbFrame> {
    oriented::aspect_fit::build_obb_frame(poly, theta_rad, ratio_w, ratio_h)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{Coord, LineString, Polygon};

    fn square() -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 10.0, y: 0.0 },
                Coord { x: 10.0, y: 10.0 },
                Coord { x: 0.0, y: 10.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![],
        )
    }

    #[test]
    fn obb_default_options() {
        let opts = ObbOptions::default();
        assert_eq!(opts.max_ratio, 0.0); // 0 = unconstrained
        assert_eq!(opts.min_ratio, 0.0);
    }

    #[test]
    fn obb_square_area() {
        let r = solve_obb(&square(), &ObbOptions::default()).unwrap();
        assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
        assert!((r.aspect_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn obb_degenerate_returns_err() {
        let bad = Polygon::new(
            LineString::from(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 5.0, y: 0.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        assert!(solve_obb(&bad, &ObbOptions::default()).is_err());
    }

    #[test]
    fn obb_aspect_fit_square_a4() {
        let res = solve_obb_aspect_fit(&square(), 297.0, 210.0).unwrap();
        assert!(res.fill_ratio > 0.5, "fill={}", res.fill_ratio);
    }

    #[test]
    fn obb_constrained_max_ratio() {
        let poly = Polygon::new(
            LineString::from(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 20.0, y: 0.0 },
                Coord { x: 20.0, y: 5.0 },
                Coord { x: 0.0, y: 5.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        let opts = ObbOptions {
            max_ratio: 2.0,
            ..Default::default()
        };
        let r = solve_obb_constrained(&poly, &opts).unwrap();
        assert!(r.aspect_ratio <= 2.0 + 1e-6, "ar={}", r.aspect_ratio);
    }

    #[test]
    fn obb_empty_result_defaults() {
        let r = ObbResult::empty();
        assert!(r.polygon.is_none());
        assert_eq!(r.area, 0.0);
        assert_eq!(r.perimeter, 0.0);
        assert_eq!(r.angle_deg, 0.0);
        assert_eq!(r.width, 0.0);
        assert_eq!(r.height, 0.0);
    }
}
