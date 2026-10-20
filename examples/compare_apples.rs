//! Apples-to-apples oriented bounding box benchmark against GEOS.
//!
//! Both sides get the same polygon, and only the solve is timed. The native
//! side runs the real public entry point. GEOS gets a geometry built before the
//! clock starts and runs `MinimumRotatedRectangle`, which is its equivalent of
//! the minimum-area box. Neither side includes coordinate conversion.
//!
//! Run: cargo run --release --features bench-geos --example compare_apples

use std::time::Instant;

use geos::{Geom, Geometry};
use oriented_bounding_box::geo_types::Polygon;
use oriented_bounding_box::{solve_obb, ObbOptions};

#[path = "../benches/corpus.rs"]
mod corpus;

fn to_wkt(poly: &Polygon<f64>) -> String {
    let mut s = String::from("POLYGON (");
    let mut first = true;
    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
        if !first {
            s.push_str(", ");
        }
        first = false;
        s.push('(');
        for (i, c) in ring.0.iter().enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push_str(&format!("{} {}", c.x, c.y));
        }
        s.push(')');
    }
    s.push(')');
    s
}

struct Row {
    verts: usize,
    native_us: f64,
    geos_us: f64,
    area_native: f64,
    area_geos: f64,
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let i = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[i.min(sorted.len() - 1)]
}

fn main() {
    let parcels = corpus::load_parcels();
    let opts = ObbOptions::default();

    struct Case {
        verts: usize,
        poly: Polygon<f64>,
        geom: Geometry,
    }
    let mut cases: Vec<Case> = Vec::new();
    for p in &parcels {
        let Ok(geom) = Geometry::new_from_wkt(&to_wkt(&p.polygon)) else {
            continue;
        };
        cases.push(Case {
            verts: corpus::exterior_len(&p.polygon),
            poly: p.polygon.clone(),
            geom,
        });
    }
    println!("cases: {}\n", cases.len());

    for c in cases.iter().take(30) {
        let _ = solve_obb(&c.poly, &opts);
        let _ = c.geom.minimum_rotated_rectangle();
    }

    let mut rows: Vec<Row> = Vec::with_capacity(cases.len());
    for c in &cases {
        let mut native = f64::INFINITY;
        let mut geos = f64::INFINITY;
        let (mut an, mut ag) = (0.0, 0.0);
        for _ in 0..5 {
            let t = Instant::now();
            let out = solve_obb(&c.poly, &opts);
            let us = t.elapsed().as_secs_f64() * 1e6;
            if us < native {
                native = us;
            }
            if let Ok(r) = out {
                an = r.width * r.height;
            }

            let t = Instant::now();
            let gout = c.geom.minimum_rotated_rectangle();
            let us = t.elapsed().as_secs_f64() * 1e6;
            if us < geos {
                geos = us;
            }
            if let Ok(g) = gout {
                if let Ok(a) = g.area() {
                    ag = a;
                }
            }
        }
        rows.push(Row {
            verts: c.verts,
            native_us: native,
            geos_us: geos,
            area_native: an,
            area_geos: ag,
        });
    }

    let mut ratios: Vec<f64> = rows
        .iter()
        .filter(|r| r.native_us > 0.0 && r.geos_us > 0.0)
        .map(|r| r.geos_us / r.native_us)
        .collect();
    ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let total_n: f64 = rows.iter().map(|r| r.native_us).sum();
    let total_g: f64 = rows.iter().map(|r| r.geos_us).sum();

    println!("=== per-case ratio geos/native ===");
    println!(
        "p10 {:.2}x   p25 {:.2}x   median {:.2}x   p75 {:.2}x   p90 {:.2}x",
        pct(&ratios, 0.10),
        pct(&ratios, 0.25),
        pct(&ratios, 0.50),
        pct(&ratios, 0.75),
        pct(&ratios, 0.90)
    );
    println!(
        "\ntotal native {:.1} ms   total geos {:.1} ms   ratio {:.2}x",
        total_n / 1e3,
        total_g / 1e3,
        total_g / total_n
    );
    let wins = rows.iter().filter(|r| r.native_us < r.geos_us).count();
    println!(
        "native faster on {wins}/{} cases ({:.0}%)",
        rows.len(),
        100.0 * wins as f64 / rows.len() as f64
    );

    let buckets = [
        (0usize, 8usize),
        (8, 17),
        (17, 33),
        (33, 65),
        (65, 129),
        (129, 1_000_000),
    ];
    println!("\n=== by exterior vertex count ===");
    println!(
        "{:>10} {:>6} {:>12} {:>12} {:>10}",
        "verts", "n", "native ms", "geos ms", "ratio"
    );
    for (lo, hi) in buckets {
        let sel: Vec<&Row> = rows
            .iter()
            .filter(|r| r.verts >= lo && r.verts < hi)
            .collect();
        if sel.is_empty() {
            continue;
        }
        let n: f64 = sel.iter().map(|r| r.native_us).sum();
        let g: f64 = sel.iter().map(|r| r.geos_us).sum();
        println!(
            "{:>10} {:>6} {:>12.2} {:>12.2} {:>9.2}x",
            format!("{lo}-{hi}"),
            sel.len(),
            n / 1e3,
            g / 1e3,
            g / n
        );
    }

    let mut diffs: Vec<f64> = rows
        .iter()
        .filter(|r| r.area_native > 0.0 && r.area_geos > 0.0)
        .map(|r| (r.area_native - r.area_geos).abs() / r.area_native.max(r.area_geos))
        .collect();
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n=== box area agreement ===");
    println!(
        "median {:.2e}   p90 {:.2e}   max {:.2e}",
        pct(&diffs, 0.5),
        pct(&diffs, 0.9),
        diffs[diffs.len() - 1]
    );
    let better = rows
        .iter()
        .filter(|r| r.area_native > 0.0 && r.area_geos > 0.0 && r.area_native < r.area_geos)
        .count();
    println!("native box smaller on {better}/{} cases", rows.len());
}
