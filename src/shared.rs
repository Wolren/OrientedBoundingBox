//! Minimal shared plumbing carried over from IGE `ige-core::shared`.
//!
//! Only what `obb` actually uses: the error type, the `Result` alias, and
//! the rotation helpers. No grid, SDF, expand, or certify machinery.

use geo::Centroid;
use geo_types::{Coord, LineString, Point, Polygon};
use thiserror::Error;

/// Errors returned by the solvers and their interop layers.
#[derive(Debug, Error)]
pub enum LirError {
    /// The polygon is degenerate or holds a non-finite coordinate.
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    /// No rectangle satisfying the requested constraints exists.
    #[error("No rectangle found")]
    NoRectangleFound,
    /// A GPU backend reported an error.
    #[error("GPU error: {0}")]
    GpuError(String),
    /// The input geometry kind is not handled by the requested solver.
    #[error("Algorithm not supported: {0}")]
    NotSupported(String),
    /// An internal solver invariant was violated.
    #[error("Internal error: {0}")]
    Internal(String),
}

/// Solver result alias over [`LirError`].
pub type Result<T> = std::result::Result<T, LirError>;

/// Rotate a polygon about its own centroid by `angle_deg` degrees.
pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    match poly.centroid() {
        Some(centroid) => rotate_polygon_around(poly, angle_deg, &centroid),
        None => poly.clone(),
    }
}

/// Rotate a polygon about `center` by `angle_deg` degrees.
pub fn rotate_polygon_around(
    poly: &Polygon<f64>,
    angle_deg: f64,
    center: &Point<f64>,
) -> Polygon<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let cx = center.x();
    let cy = center.y();

    let rotate = |c: &Coord<f64>| Coord {
        x: cx + (c.x - cx) * cos_a - (c.y - cy) * sin_a,
        y: cy + (c.x - cx) * sin_a + (c.y - cy) * cos_a,
    };

    let ext = LineString::from(poly.exterior().0.iter().map(&rotate).collect::<Vec<_>>());
    let interiors: Vec<LineString<f64>> = poly
        .interiors()
        .iter()
        .map(|r| LineString::from(r.0.iter().map(&rotate).collect::<Vec<_>>()))
        .collect();

    Polygon::new(ext, interiors)
}
