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

#[inline]
/// Width and depth of a point set projected along `theta` and its normal.
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

/// Extreme hull vertices for a direction, found by a monotone advance.
///
/// The hull is convex and CCW, so as `theta` grows the vertex that maximises the
/// projection advances monotonically. Tracking that index costs O(1) amortised
/// per query instead of the O(h) full scan, which turns the whole minimum-area
/// sweep from O(h^2) into O(h). `start` is the index found for the previous
/// angle, and the search continues from there.
#[inline]
fn advance<F: Fn(usize) -> f64>(count: usize, mut idx: usize, proj: F) -> usize {
    let mut advanced = 0usize;
    // A full lap is the guard: a degenerate hull cannot trap the walk.
    while advanced < count {
        let next = (idx + 1) % count;
        if proj(next) > proj(idx) {
            idx = next;
            advanced += 1;
        } else {
            break;
        }
    }
    idx
}

/// Width and depth by monotone support points, seeded by `prev` indices.
///
/// Returns the extents and the four indices that produced them, so the next
/// angle can resume the walk from there.
pub fn width_depth_walk(
    pts: &[Coord<f64>],
    theta: f64,
    prev: [usize; 4],
) -> ((f64, f64), [usize; 4]) {
    let (c, s) = (theta.cos(), theta.sin());
    let n = pts.len();
    let proj_min = |i: usize| -(pts[i].x * c + pts[i].y * s);
    let proj_max = |i: usize| pts[i].x * c + pts[i].y * s;
    let depth_min = |i: usize| -(pts[i].x * -s + pts[i].y * c);
    let depth_max = |i: usize| pts[i].x * -s + pts[i].y * c;

    let i_max = advance(n, prev[0], proj_max);
    let i_min = advance(n, prev[1], proj_min);
    let i_dmax = advance(n, prev[2], depth_max);
    let i_dmin = advance(n, prev[3], depth_min);

    let p_max = pts[i_max].x * c + pts[i_max].y * s;
    let p_min = pts[i_min].x * c + pts[i_min].y * s;
    let pd_max = pts[i_dmax].x * -s + pts[i_dmax].y * c;
    let pd_min = pts[i_dmin].x * -s + pts[i_dmin].y * c;

    (
        (p_max - p_min, pd_max - pd_min),
        [i_max, i_min, i_dmax, i_dmin],
    )
}

/// Seeds for [`width_depth_walk`] at the first angle, by a full scan.
pub fn width_depth_seed(pts: &[Coord<f64>], theta: f64) -> [usize; 4] {
    let (c, s) = (theta.cos(), theta.sin());
    let mut out = [0usize; 4];
    let (mut p_max, mut p_min) = (f64::NEG_INFINITY, f64::INFINITY);
    let (mut d_max, mut d_min) = (f64::NEG_INFINITY, f64::INFINITY);
    for (i, pt) in pts.iter().enumerate() {
        let p = pt.x * c + pt.y * s;
        let d = pt.x * -s + pt.y * c;
        if p > p_max {
            p_max = p;
            out[0] = i;
        }
        if p < p_min {
            p_min = p;
            out[1] = i;
        }
        if d > d_max {
            d_max = d;
            out[2] = i;
        }
        if d < d_min {
            d_min = d;
            out[3] = i;
        }
    }
    out
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
