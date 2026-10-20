//! Exact optimal rotation for a fixed-aspect-ratio frame.
//!
//! Rotating-calipers support function with a closed-form aspect-ratio
//! crossing root per caliper interval: O(n log n), no grid search.

use super::super::{ObbFrame, ObbResult};
use super::common::{
    antipodal_coeffs_at, caliper_breakpoints, fill_ratio, hull_ccw, width_depth, width_depth_seed,
    width_depth_walk, EPS,
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
    // One support walk across the intervals: the coefficients per interval come
    // from the indices, so no interval rescans the hull.
    let mut support = width_depth_seed(&pts, bps[0]);
    for i in 0..bps.len() - 1 {
        let lo = bps[i];
        let hi = bps[i + 1];
        let mid = (lo + hi) / 2.0;
        let (_, next) = width_depth_walk(&pts, mid, support);
        support = next;
        let (aw, bw, ad, bd) = antipodal_coeffs_at(&pts, support);
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
    // The hull is convex, so the support vertices advance monotonically as the
    // candidate angle grows. Sort the candidates and walk the supports across
    // them, which costs O(h) for the whole sweep instead of a full projection
    // per candidate. Without this the search is O(h^2) and dominated it.
    let mut ordered: Vec<f64> = cands
        .iter()
        .map(|t| t.rem_euclid(std::f64::consts::PI))
        .collect();
    ordered.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ordered.dedup_by(|a, b| (*a - *b).abs() < EPS);

    if ordered.is_empty() {
        ordered.push(0.0);
    }
    let mut support = width_depth_seed(&pts, ordered[0]);
    for &t in &ordered {
        let ((w, d), next) = width_depth_walk(&pts, t, support);
        support = next;
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
) -> Option<ObbFrame> {
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

    Some(ObbFrame {
        polygon: frame,
        frame_w: fw,
        frame_h: fh,
        body_w: wb,
        body_h: hb,
        center: fc,
    })
}
