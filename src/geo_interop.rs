//! georust ecosystem interop (feature `geo-traits`).
//!
//! Two surfaces. First, the solvers accept any foreign polygon that
//! implements `geo_traits::PolygonTrait` (wkb views, geoarrow arrays,
//! geozero streams) without materializing a `geo_types::Polygon` first;
//! conversion uses `to_polygon()` and keeps only the first two dimensions.
//! Second, [`solve_obb_geometry`] dispatches over `geo_types::Geometry` so
//! callers holding mixed geometry enums get one entry point.
//!
//! MultiPolygon policy: the largest-area component is solved. Other
//! geometry variants are rejected with an error naming the variant,
//! never silently mis-solved.

use super::obb::{solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions, ObbResult};
use super::shared::{LirError, Result};
use geo::Area;
use geo_traits::to_geo::ToGeoPolygon;
use geo_traits::{GeometryTrait, PolygonTrait};
use geo_types::{Geometry, Polygon};

/// Solve min-area OBB on any foreign polygon implementing `PolygonTrait`.
pub fn solve_obb_polygon_trait<G>(geom: &G, options: &ObbOptions) -> Result<ObbResult>
where
    G: PolygonTrait<T = f64>,
{
    let poly: Polygon<f64> = geom.to_polygon();
    solve_obb(&poly, options)
}

/// Solve constrained OBB on any foreign polygon implementing `PolygonTrait`.
pub fn solve_obb_constrained_polygon_trait<G>(geom: &G, options: &ObbOptions) -> Result<ObbResult>
where
    G: PolygonTrait<T = f64>,
{
    let poly: Polygon<f64> = geom.to_polygon();
    solve_obb_constrained(&poly, options)
}

/// Solve aspect-fit OBB on any foreign polygon implementing `PolygonTrait`.
pub fn solve_obb_aspect_fit_polygon_trait<G>(
    geom: &G,
    ratio_w: f64,
    ratio_h: f64,
) -> Option<ObbResult>
where
    G: PolygonTrait<T = f64>,
{
    let poly: Polygon<f64> = geom.to_polygon();
    solve_obb_aspect_fit(&poly, ratio_w, ratio_h)
}

fn variant_name(geom: &Geometry<f64>) -> &'static str {
    match geom {
        Geometry::Point(_) => "Point",
        Geometry::Line(_) => "Line",
        Geometry::LineString(_) => "LineString",
        Geometry::Polygon(_) => "Polygon",
        Geometry::MultiPoint(_) => "MultiPoint",
        Geometry::MultiLineString(_) => "MultiLineString",
        Geometry::MultiPolygon(_) => "MultiPolygon",
        Geometry::GeometryCollection(_) => "GeometryCollection",
        Geometry::Rect(_) => "Rect",
        Geometry::Triangle(_) => "Triangle",
    }
}

/// Solve min-area OBB on a `geo_types::Geometry`.
///
/// Polygon solves directly. MultiPolygon solves the largest-area
/// component. Every other variant is rejected with an error naming it.
pub fn solve_obb_geometry(geom: &Geometry<f64>, options: &ObbOptions) -> Result<ObbResult> {
    match geom {
        Geometry::Polygon(p) => solve_obb(p, options),
        Geometry::MultiPolygon(mp) => {
            let best =
                mp.0.iter()
                    .max_by(|a, b| {
                        a.unsigned_area()
                            .partial_cmp(&b.unsigned_area())
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .ok_or_else(|| LirError::InvalidPolygon("empty MultiPolygon".into()))?;
            solve_obb(best, options)
        }
        other => Err(LirError::NotSupported(format!(
            "solve_obb_geometry needs Polygon or MultiPolygon, got {}",
            variant_name(other)
        ))),
    }
}

/// Solve constrained OBB on a `geo_types::Geometry`. Same dispatch policy
/// as [`solve_obb_geometry`].
pub fn solve_obb_constrained_geometry(
    geom: &Geometry<f64>,
    options: &ObbOptions,
) -> Result<ObbResult> {
    match geom {
        Geometry::Polygon(p) => solve_obb_constrained(p, options),
        Geometry::MultiPolygon(mp) => {
            let best =
                mp.0.iter()
                    .max_by(|a, b| {
                        a.unsigned_area()
                            .partial_cmp(&b.unsigned_area())
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .ok_or_else(|| LirError::InvalidPolygon("empty MultiPolygon".into()))?;
            solve_obb_constrained(best, options)
        }
        other => Err(LirError::NotSupported(format!(
            "solve_obb_constrained_geometry needs Polygon or MultiPolygon, got {}",
            variant_name(other)
        ))),
    }
}

/// Assert a foreign polygon roundtrips through `to_polygon()` for use as a
/// differential oracle against the native path in tests.
pub fn polygon_trait_roundtrip<G>(geom: &G) -> Polygon<f64>
where
    G: PolygonTrait<T = f64>,
{
    geom.to_polygon()
}
