//! geo-traits interop tests: foreign polygons solve identically to native.
#![cfg(feature = "geo-traits")]

use geo_types::{Coord, LineString, Polygon};
use oriented_bounding_box::geo_interop::{
    solve_obb_aspect_fit_polygon_trait, solve_obb_constrained_polygon_trait, solve_obb_geometry,
    solve_obb_polygon_trait,
};
use oriented_bounding_box::{solve_obb, ObbOptions};

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
fn polygon_trait_matches_native() {
    let poly = square();
    let opts = ObbOptions::default();
    let native = solve_obb(&poly, &opts).unwrap();
    // `&Polygon` implements `PolygonTrait` via geo-traits' geo-types impls,
    // exercising the foreign-type path with a borrowed view.
    let foreign = solve_obb_polygon_trait(&&poly, &opts).unwrap();
    assert!((native.area - foreign.area).abs() < 1e-9);
    assert!((native.angle_deg - foreign.angle_deg).abs() < 1e-9);
}

#[test]
fn geometry_dispatch_polygon() {
    let geom = geo_types::Geometry::Polygon(square());
    let r = solve_obb_geometry(&geom, &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
}

#[test]
fn geometry_dispatch_multipolygon_picks_largest() {
    let big = square();
    let small = Polygon::new(
        LineString::from(vec![
            Coord { x: 20.0, y: 20.0 },
            Coord { x: 22.0, y: 20.0 },
            Coord { x: 22.0, y: 22.0 },
            Coord { x: 20.0, y: 22.0 },
            Coord { x: 20.0, y: 20.0 },
        ]),
        vec![],
    );
    let geom = geo_types::Geometry::MultiPolygon(geo_types::MultiPolygon(vec![small, big]));
    let r = solve_obb_geometry(&geom, &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
}

#[test]
fn geometry_dispatch_rejects_point() {
    let geom = geo_types::Geometry::Point(geo_types::Point::new(1.0, 2.0));
    let err = solve_obb_geometry(&geom, &ObbOptions::default()).unwrap_err();
    assert!(err.to_string().contains("Point"));
}

#[test]
fn constrained_and_aspect_fit_trait_paths() {
    let poly = square();
    let mut opts = ObbOptions::default();
    opts.max_ratio = 1.5;
    let r = solve_obb_constrained_polygon_trait(&&poly, &opts).unwrap();
    assert!(r.aspect_ratio <= 1.5 + 1e-6);
    let r = solve_obb_aspect_fit_polygon_trait(&&poly, 297.0, 210.0).unwrap();
    assert!(r.fill_ratio > 0.5);
}
