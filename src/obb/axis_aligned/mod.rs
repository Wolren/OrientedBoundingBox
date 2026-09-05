use super::{ObbOptions, ObbResult};
use crate::shared::Result;
use geo::{Area, BoundingRect, Centroid};
use geo_types::Polygon;

#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn solve_obb_axis_aligned(poly: &Polygon<f64>, _options: &ObbOptions) -> Result<ObbResult> {
    let area = poly.unsigned_area();
    let bb = poly
        .bounding_rect()
        .ok_or(crate::shared::LirError::InvalidPolygon(
            "degenerate polygon".into(),
        ))?;

    let width = bb.max().x - bb.min().x;
    let height = bb.max().y - bb.min().y;
    let aspect_ratio = if height > 0.0 { width / height } else { 1.0 };
    let aspect_ratio = aspect_ratio.max(1.0 / aspect_ratio.max(1e-10));

    let fill_ratio = if width * height > 0.0 {
        area / (width * height)
    } else {
        0.0
    };

    let perimeter = poly
        .exterior()
        .0
        .windows(2)
        .map(|w| {
            let dx = w[1].x - w[0].x;
            let dy = w[1].y - w[0].y;
            (dx * dx + dy * dy).sqrt()
        })
        .sum::<f64>();

    Ok(ObbResult {
        polygon: Some(poly.clone()),
        area: width * height,
        perimeter,
        angle_deg: 0.0,
        width,
        height,
        centroid: poly.centroid(),
        aspect_ratio,
        fill_ratio,
        north_fill: 0.0,
        improve_pct: 0.0,
        n_intervals: 0,
    })
}
