use super::super::ObbResult;
use super::common::{
    antipodal_coeffs, caliper_breakpoints, fill_ratio, hull_ccw, width_depth, EPS,
};
use geo::{BoundingRect, Centroid};
use geo_types::{Coord, Polygon};

/// Exact optimal rotation solver for a fixed-aspect-ratio frame.
///
/// Uses rotating-calipers support function with closed-form aspect-ratio
/// crossing root per caliper interval.  O(n log n), no grid search.
///
/// Returns `None` for degenerate polygons or invalid aspect ratio.
pub fn solve_obb_aspect_fit(poly: &Polygon<f64>, ratio_w: f64, ratio_h: f64) -> Option<ObbResult> {
    if ratio_h < EPS {
        return None;
    }
    let r = ratio_w / ratio_h;
    let pts = hull_ccw(poly)?;
    if pts.len() < 3 {
        return None;
    }

    let raw = caliper_breakpoints(&pts);
    let mut bps: Vec<f64> = Vec::with_capacity(raw.len() + 2);
    bps.push(0.0);
    for b in raw {
        if b - bps.last().unwrap_or(&0.0) > EPS {
            bps.push(b);
        }
    }
    let last = *bps.last().unwrap_or(&0.0);
    if std::f64::consts::PI - EPS - last > EPS {
        bps.push(std::f64::consts::PI - EPS);
    }

    let mut cands = bps.clone();
    for i in 0..bps.len() - 1 {
        let lo = bps[i];
        let hi = bps[i + 1];
        let mid = (lo + hi) / 2.0;
        let (aw, bw, ad, bd) = antipodal_coeffs(&pts, mid);
        let p = aw - r * ad;
        let q = bw - r * bd;
        if p.hypot(q) < EPS {
            cands.push(mid);
            continue;
        }
        let tb = (-p).atan2(q);
        for k in -1..=2 {
            let tc = tb + k as f64 * std::f64::consts::PI;
            if tc > lo + EPS && tc < hi - EPS {
                cands.push(tc);
            }
        }
    }

    let mut best_fill = -1.0;
    let mut best_theta = 0.0;
    for &t in &cands {
        let t = t.rem_euclid(std::f64::consts::PI);
        let (w, d) = width_depth(&pts, t);
        let f = fill_ratio(w, d, r);
        if f > best_fill + 1e-16 {
            best_fill = f;
            best_theta = t;
        }
    }

    let (w0, d0) = width_depth(&pts, 0.0);
    let nfill = fill_ratio(w0, d0, r);

    let (w_opt, d_opt) = width_depth(&pts, best_theta);
    let fw = w_opt.max(d_opt * r);
    let fh = d_opt.max(w_opt / r);
    let imp = if nfill > EPS {
        (best_fill - nfill) / nfill * 100.0
    } else {
        0.0
    };
    let aspect = (fw / fh).max(fh / fw);

    Some(ObbResult {
        polygon: None,
        area: fw * fh,
        perimeter: 2.0 * (fw + fh),
        angle_deg: best_theta.to_degrees(),
        width: fw,
        height: fh,
        centroid: None,
        aspect_ratio: aspect,
        fill_ratio: best_fill,
        north_fill: nfill,
        improve_pct: imp,
        n_intervals: bps.len() - 1,
    })
}

/// Build the COVER frame polygon for a given optimal angle.
///
/// Rotates the polygon by -θ around its centroid, computes the tight AABB,
/// expands to the target aspect ratio, then rotates back.
pub fn build_obb_frame(
    poly: &Polygon<f64>,
    theta_rad: f64,
    ratio_w: f64,
    ratio_h: f64,
) -> Option<(Polygon<f64>, f64, f64, f64, f64, f64, f64)> {
    let centroid = poly.centroid()?;
    let r = ratio_w / ratio_h;
    let theta_deg = theta_rad.to_degrees();

    let poly_rot = crate::shared::rotate_polygon_around(poly, -theta_deg, &centroid);
    let rect = poly_rot.bounding_rect()?;
    let (minx, miny, maxx, maxy) = (rect.min().x, rect.min().y, rect.max().x, rect.max().y);
    let wb = maxx - minx;
    let hb = maxy - miny;

    let fw = wb.max(hb * r);
    let fh = hb.max(wb / r);

    let cx_bb = (minx + maxx) / 2.0;
    let cy_bb = (miny + maxy) / 2.0;

    let frame_rot = Polygon::new(
        geo_types::LineString::from(vec![
            Coord {
                x: cx_bb - fw / 2.0,
                y: cy_bb - fh / 2.0,
            },
            Coord {
                x: cx_bb + fw / 2.0,
                y: cy_bb - fh / 2.0,
            },
            Coord {
                x: cx_bb + fw / 2.0,
                y: cy_bb + fh / 2.0,
            },
            Coord {
                x: cx_bb - fw / 2.0,
                y: cy_bb + fh / 2.0,
            },
            Coord {
                x: cx_bb - fw / 2.0,
                y: cy_bb - fh / 2.0,
            },
        ]),
        vec![],
    );

    let frame = crate::shared::rotate_polygon_around(&frame_rot, theta_deg, &centroid);
    let fc = frame.centroid()?;

    Some((frame, fw, fh, wb, hb, fc.x(), fc.y()))
}
