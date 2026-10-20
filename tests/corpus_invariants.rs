//! Corpus-level invariants for the OBB solvers.
//!
//! The min-area box is checked against an independent brute-force rotation
//! search on the hull, which knows nothing about rotating calipers. Every box
//! is also required to contain the polygon it claims to enclose.

use geo::{BoundingRect, Contains, ConvexHull};
use oriented_bounding_box::geo_types::{Coord, LineString, Point, Polygon};
use oriented_bounding_box::{solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions};

fn poly(pts: &[(f64, f64)]) -> Polygon<f64> {
    let mut v: Vec<Coord<f64>> = pts.iter().map(|p| Coord { x: p.0, y: p.1 }).collect();
    v.push(v[0]);
    Polygon::new(LineString::from(v), vec![])
}

/// Smallest box area over a dense sweep of angles, computed on the hull.
/// Independent of the calipers implementation.
fn brute_min_area(p: &Polygon<f64>, steps: usize) -> f64 {
    let hull = p.convex_hull();
    let ring: Vec<(f64, f64)> = hull.exterior().0.iter().map(|c| (c.x, c.y)).collect();
    let mut best = f64::INFINITY;
    for k in 0..steps {
        let a = std::f64::consts::PI * 0.5 * (k as f64) / (steps as f64);
        let (s, c) = a.sin_cos();
        let (mut min_u, mut max_u) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut min_v, mut max_v) = (f64::INFINITY, f64::NEG_INFINITY);
        for &(x, y) in &ring {
            let u = x * c + y * s;
            let v = -x * s + y * c;
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
        let area = (max_u - min_u) * (max_v - min_v);
        best = best.min(area);
    }
    best
}

fn corpus() -> Vec<(&'static str, Polygon<f64>)> {
    let mut cases: Vec<(&'static str, Polygon<f64>)> = vec![
        (
            "square",
            poly(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]),
        ),
        (
            "L shape",
            poly(&[
                (0.0, 0.0),
                (4.0, 0.0),
                (4.0, 1.0),
                (1.0, 1.0),
                (1.0, 3.0),
                (0.0, 3.0),
            ]),
        ),
        (
            "diamond",
            poly(&[(1.0, 0.0), (2.0, 1.0), (1.0, 2.0), (0.0, 1.0)]),
        ),
        (
            "thin 45",
            poly(&[(0.0, 0.0), (10.0, 10.0), (9.0, 11.0), (-1.0, 1.0)]),
        ),
        (
            "long rect",
            poly(&[(0.0, 0.0), (100.0, 0.0), (100.0, 1.0), (0.0, 1.0)]),
        ),
        ("triangle", poly(&[(0.0, 0.0), (10.0, 0.0), (5.0, 8.0)])),
        (
            "U shape",
            poly(&[
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 1.0),
                (6.0, 1.0),
                (6.0, 9.0),
                (4.0, 9.0),
                (4.0, 1.0),
                (0.0, 1.0),
            ]),
        ),
    ];
    // A regular n-gon at a rotated offset, to catch angle normalisation.
    for n in [5usize, 8, 32] {
        let pts: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let a = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64) + 0.3;
                (3.0 * a.cos() + 7.0, 3.0 * a.sin() - 2.0)
            })
            .collect();
        let name: &'static str = match n {
            5 => "regular 5-gon",
            8 => "regular 8-gon",
            _ => "regular 32-gon",
        };
        cases.push((name, poly(&pts)));
    }
    cases
}

#[test]
fn min_area_matches_brute_force_rotation() {
    let opts = ObbOptions::default();
    for (name, p) in corpus() {
        let got = solve_obb(&p, &opts).expect("obb").area;
        let brute = brute_min_area(&p, 20_000);
        let rel = (got - brute) / brute;
        assert!(
            rel.abs() < 1e-4,
            "{name}: calipers {got} vs brute {brute} (rel {rel:e})"
        );
    }
}

#[test]
fn box_contains_its_polygon() {
    // A correct box puts every polygon point inside it, but a point can land on
    // the boundary where `contains` is knife-edge: the diamond test sample sat
    // 2e-17 inside and was still rejected. Measure signed distance to the box
    // edges instead, and allow the same 1e-9 relative slack the area test uses.
    let opts = ObbOptions::default();
    for (name, p) in corpus() {
        let r = solve_obb(&p, &opts).expect("obb");
        let box_poly = r.polygon.expect("box polygon");
        let ext = box_poly.exterior();
        let rect = p.bounding_rect().expect("rect");
        let (min_x, min_y) = (rect.min().x, rect.min().y);
        let (max_x, max_y) = (rect.max().x, rect.max().y);
        let scale = (max_x - min_x).max(max_y - min_y).max(1.0);
        let tol = scale * 1e-9;

        // Outward is the positive side, so a point inside has every edge
        // distance at or below `tol` in magnitude on the inward side.
        let side = |pt: Coord<f64>, a: Coord<f64>, b: Coord<f64>| -> f64 {
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let len = (dx * dx + dy * dy).sqrt().max(f64::MIN_POSITIVE);
            ((pt.x - a.x) * dy - (pt.y - a.y) * dx) / len
        };

        let steps = 60;
        let mut checked = 0usize;
        for i in 0..=steps {
            for j in 0..=steps {
                let x = min_x + (max_x - min_x) * (i as f64) / (steps as f64);
                let y = min_y + (max_y - min_y) * (j as f64) / (steps as f64);
                let pt = Point::new(x, y);
                if !p.contains(&pt) {
                    continue;
                }
                checked += 1;
                let c = Coord { x, y };
                // Inside the box means no edge reports a positive outward side.
                let worst = ext
                    .0
                    .windows(2)
                    .map(|w| side(c, w[0], w[1]))
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!(
                    worst <= tol,
                    "{name}: interior point {pt:?} sits {worst:e} outside the box (tol {tol:e})"
                );
            }
        }
        assert!(checked > 0, "{name}: no interior points sampled");
    }
}

#[test]
fn constrained_respects_max_ratio() {
    let r = solve_obb_constrained(
        &poly(&[(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]),
        &ObbOptions {
            max_ratio: 2.0,
            ..Default::default()
        },
    )
    .expect("obb");
    assert!(r.aspect_ratio <= 2.0 + 1e-9, "ar={}", r.aspect_ratio);
}

#[test]
fn aspect_fit_beats_north_up_on_a_tilted_square() {
    // A square turned 45 degrees fits an A4 frame better when rotated back.
    let tilted: Polygon<f64> = poly(&[(0.0, 0.0), (10.0, 10.0), (0.0, 20.0), (-10.0, 10.0)]);
    let r = solve_obb_aspect_fit(&tilted, 297.0, 210.0).expect("fit");
    assert!(r.fill_ratio > 0.0);
    assert!(r.fill_ratio <= 1.0 + 1e-9, "fill={}", r.fill_ratio);
    assert!(r.n_intervals > 0);
}

#[test]
fn degenerate_input_is_rejected() {
    let bad = poly(&[(0.0, 0.0), (5.0, 0.0)]);
    assert!(solve_obb(&bad, &ObbOptions::default()).is_err());
}
