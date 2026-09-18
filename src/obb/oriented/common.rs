//! Shared geometry helpers for the oriented solvers.

use geo::ConvexHull;
use geo_types::Coord;
use geo_types::Polygon;

/// Tolerance below which a quantity counts as zero.
pub const EPS: f64 = 1e-12;

/// Extract convex hull vertices in CCW order.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn hull_ccw(poly: &Polygon<f64>) -> Option<Vec<Coord<f64>>> {
    let h = poly.convex_hull();
    let mut coords: Vec<Coord<f64>> = h.exterior().0.clone();
    if coords.len() < 4 {
        return None;
    }
    coords.pop();

    let mut area = 0.0;
    for i in 0..coords.len() {
        let j = (i + 1) % coords.len();
        area += coords[i].x * coords[j].y;
        area -= coords[j].x * coords[i].y;
    }
    if area < 0.0 {
        coords.reverse();
    }

    if coords.len() < 3 {
        return None;
    }
    Some(coords)
}

#[cfg_attr(feature = "hotpath", hotpath::measure)]
/// Width (w) and depth (d) of point set projected along angle θ.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn width_depth(pts: &[Coord<f64>], theta: f64) -> (f64, f64) {
    let (c, s) = (theta.cos(), theta.sin());
    let (c2, s2) = (-s, c);

    let mut p_min = f64::INFINITY;
    let mut p_max = f64::NEG_INFINITY;
    let mut pd_min = f64::INFINITY;
    let mut pd_max = f64::NEG_INFINITY;

    for pt in pts {
        let p = pt.x * c + pt.y * s;
        let pd = pt.x * c2 + pt.y * s2;
        if p < p_min {
            p_min = p;
        }
        if p > p_max {
            p_max = p;
        }
        if pd < pd_min {
            pd_min = pd;
        }
        if pd > pd_max {
            pd_max = pd;
        }
    }

    (p_max - p_min, pd_max - pd_min)
}

/// Fill ratio: F = w·d / (max(w, d·r) · max(d, w/r))
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn fill_ratio(w: f64, d: f64, r: f64) -> f64 {
    let fa = w.max(d * r) * d.max(w / r);
    if fa > EPS {
        (w * d) / fa
    } else {
        0.0
    }
}
#[cfg_attr(feature = "hotpath", hotpath::measure)]

/// Caliper breakpoints: edge directions (mod π) plus their π/2 offsets.
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn caliper_breakpoints(pts: &[Coord<f64>]) -> Vec<f64> {
    let n = pts.len();
    let mut set: Vec<f64> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let dx = pts[j].x - pts[i].x;
        let dy = pts[j].y - pts[i].y;
        if dx.abs() < EPS && dy.abs() < EPS {
            continue;
        }
        let a = dy.atan2(dx).rem_euclid(std::f64::consts::PI);
        set.push(a);
        set.push((a + std::f64::consts::PI / 2.0).rem_euclid(std::f64::consts::PI));
    }
    set.sort_by(|a, b| a.partial_cmp(b).unwrap());
    set.dedup_by(|a, b| (*a - *b).abs() < EPS);
    set
}

/// Antipodal coefficients (Aw, Bw, Ad, Bd) at a given angle.
///
#[cfg_attr(feature = "hotpath", hotpath::measure)]
/// Within a caliper interval the antipodal pairs are fixed, so:
///   w(θ) = Aw·cosθ + Bw·sinθ
///   d(θ) = Ad·cosθ + Bd·sinθ
#[cfg_attr(feature = "hotpath", hotpath::measure)]
pub fn antipodal_coeffs(pts: &[Coord<f64>], theta: f64) -> (f64, f64, f64, f64) {
    let (c, s) = (theta.cos(), theta.sin());
    let (c2, s2) = (-s, c);

    let mut p_max = f64::NEG_INFINITY;
    let mut p_min = f64::INFINITY;
    let mut p2_max = f64::NEG_INFINITY;
    let mut p2_min = f64::INFINITY;
    let mut vt = 0usize;
    let mut vb = 0usize;
    let mut vr = 0usize;
    let mut vl = 0usize;

    for (i, pt) in pts.iter().enumerate() {
        let p = pt.x * c + pt.y * s;
        let p2 = pt.x * c2 + pt.y * s2;
        if p > p_max {
            p_max = p;
            vt = i;
        }
        if p < p_min {
            p_min = p;
            vb = i;
        }
        if p2 > p2_max {
            p2_max = p2;
            vr = i;
        }
        if p2 < p2_min {
            p2_min = p2;
            vl = i;
        }
    }

    let aw = pts[vt].x - pts[vb].x;
    let bw = pts[vt].y - pts[vb].y;
    let ad = pts[vr].y - pts[vl].y;
    let bd = -(pts[vr].x - pts[vl].x);

    (aw, bw, ad, bd)
}
