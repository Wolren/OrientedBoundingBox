//! Python bindings for the OBB solvers (feature `python`).
//!
//! Thin facade over the Rust API: exterior ring plus optional holes in,
//! result object out. Panics become `ValueError`, never a host crash.
//! Ported from IGE `ige-py` with `ige_core` paths pointed at this crate.

use super::obb::{solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions};
use geo_types::{Coord, LineString, Polygon};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Build a polygon from exterior ring plus optional holes.
pub fn build_polygon(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
) -> PyResult<Polygon<f64>> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err(
            "polygon exterior must contain at least 3 points",
        ));
    }
    let coords: Vec<Coord<f64>> = exterior.into_iter().map(|(x, y)| Coord { x, y }).collect();
    let exterior_ls = LineString::from(coords);
    let interiors: Vec<LineString<f64>> = holes
        .unwrap_or_default()
        .into_iter()
        .map(|ring| {
            let ring_coords: Vec<Coord<f64>> =
                ring.into_iter().map(|(x, y)| Coord { x, y }).collect();
            LineString::from(ring_coords)
        })
        .collect();
    Ok(Polygon::new(exterior_ls, interiors))
}

/// Serialize a result polygon to WKT. Returns empty string when None.
pub fn polygon_to_wkt(poly: &Option<Polygon<f64>>) -> String {
    match poly {
        None => String::new(),
        Some(p) => {
            let ext: Vec<String> = p
                .exterior()
                .0
                .iter()
                .map(|c| format!("{:.6} {:.6}", c.x, c.y))
                .collect();
            let mut out = format!("POLYGON(({}", ext.join(", "));
            for hole in p.interiors() {
                let h: Vec<String> = hole
                    .0
                    .iter()
                    .map(|c| format!("{:.6} {:.6}", c.x, c.y))
                    .collect();
                out.push_str(&format!("),({}", h.join(", ")));
            }
            out.push_str("))");
            out
        }
    }
}

/// Map a core error into a Python ValueError preserving the message.
pub fn core_err<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(format!("solve failed: {e}"))
}

/// Run a closure behind the panic boundary. A Rust panic becomes a
/// Python ValueError instead of aborting the host process.
/// NOTE: effective only when the cdylib is built with panic=unwind.
pub fn guard<T>(f: impl FnOnce() -> PyResult<T> + std::panic::UnwindSafe) -> PyResult<T> {
    match std::panic::catch_unwind(f) {
        Ok(r) => r,
        Err(_) => Err(PyValueError::new_err(
            "solver panicked; input rejected without crashing host",
        )),
    }
}

#[pyclass]
#[derive(Clone)]
/// Result of an OBB solve, exposed to Python as read-only attributes.
pub struct PyObbResult {
    /// Area of the bounding box in squared input units.
    #[pyo3(get)]
    pub area: f64,
    /// Perimeter of the bounding box.
    #[pyo3(get)]
    pub perimeter: f64,
    /// Rotation angle of the box in degrees.
    #[pyo3(get)]
    pub angle_deg: f64,
    /// Width of the box along its rotated axis.
    #[pyo3(get)]
    pub width: f64,
    /// Height of the box along its rotated axis.
    #[pyo3(get)]
    pub height: f64,
    /// Aspect ratio, longer side over shorter, always at least 1.
    #[pyo3(get)]
    pub aspect_ratio: f64,
    /// Fill ratio. Polygon area divided by box area on the min-area and
    /// axis-aligned paths; on the aspect-fit path, the rotated bounding
    /// rectangle area divided by frame area.
    #[pyo3(get)]
    pub fill_ratio: f64,
    /// Box centroid x coordinate.
    #[pyo3(get)]
    pub centroid_x: f64,
    /// Box centroid y coordinate.
    #[pyo3(get)]
    pub centroid_y: f64,
    /// Box polygon in WKT; empty when no polygon was produced.
    #[pyo3(get)]
    pub polygon_wkt: String,
    /// On the aspect-fit path, `fill_ratio` at angle 0; 0.0 on other paths.
    #[pyo3(get)]
    pub north_fill: f64,
    /// Percent improvement over north-up; aspect fit only, 0.0 otherwise.
    #[pyo3(get)]
    pub improve_pct: f64,
    /// Number of caliper intervals; aspect fit only, 0 otherwise.
    #[pyo3(get)]
    pub n_intervals: usize,
}

fn obb_to_py(r: &super::obb::ObbResult) -> PyObbResult {
    let (cx, cy) = match &r.centroid {
        Some(p) => (p.x(), p.y()),
        None => (0.0, 0.0),
    };
    PyObbResult {
        area: r.area,
        perimeter: r.perimeter,
        angle_deg: r.angle_deg,
        width: r.width,
        height: r.height,
        aspect_ratio: r.aspect_ratio,
        fill_ratio: r.fill_ratio,
        centroid_x: cx,
        centroid_y: cy,
        polygon_wkt: polygon_to_wkt(&r.polygon),
        north_fill: r.north_fill,
        improve_pct: r.improve_pct,
        n_intervals: r.n_intervals,
    }
}

fn obb_opts(
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    angle_samples: Option<usize>,
    use_pca: Option<bool>,
    use_refinement: Option<bool>,
    xatol_deg: Option<f64>,
) -> ObbOptions {
    let mut o = ObbOptions::default();
    if let Some(v) = max_aspect_ratio {
        o.max_ratio = v;
    }
    if let Some(v) = min_aspect_ratio {
        o.min_ratio = v;
    }
    if let Some(v) = angle_samples {
        o.angle_samples = v;
    }
    if let Some(v) = use_pca {
        o.use_pca = v;
    }
    if let Some(v) = use_refinement {
        o.use_refinement = v;
    }
    if let Some(v) = xatol_deg {
        o.xatol_deg = v;
    }
    o
}

/// Solve the minimal-area OBB for an exterior ring and optional holes.
#[pyfunction(signature = (exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None, angle_samples=None, use_pca=None, use_refinement=None, xatol_deg=None))]
#[allow(clippy::too_many_arguments)]
pub fn solve_obb_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    angle_samples: Option<usize>,
    use_pca: Option<bool>,
    use_refinement: Option<bool>,
    xatol_deg: Option<f64>,
) -> PyResult<PyObbResult> {
    guard(|| {
        let poly = build_polygon(exterior, holes)?;
        let opts = obb_opts(
            max_aspect_ratio,
            min_aspect_ratio,
            angle_samples,
            use_pca,
            use_refinement,
            xatol_deg,
        );
        let r = solve_obb(&poly, &opts).map_err(core_err)?;
        Ok(obb_to_py(&r))
    })
}

/// Solve the minimal-area OBB with aspect-ratio clamps applied.
#[pyfunction(signature = (exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None))]
pub fn solve_obb_constrained_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
) -> PyResult<PyObbResult> {
    guard(|| {
        let poly = build_polygon(exterior, holes)?;
        let mut opts = ObbOptions::default();
        if let Some(v) = max_aspect_ratio {
            opts.max_ratio = v;
        }
        if let Some(v) = min_aspect_ratio {
            opts.min_ratio = v;
        }
        let r = solve_obb_constrained(&poly, &opts).map_err(core_err)?;
        Ok(obb_to_py(&r))
    })
}

/// Find the best rotation of a fixed-ratio frame around the exterior ring.
#[pyfunction(signature = (exterior, ratio_w, ratio_h, holes=None))]
pub fn solve_obb_aspect_fit_py(
    exterior: Vec<(f64, f64)>,
    ratio_w: f64,
    ratio_h: f64,
    holes: Option<Vec<Vec<(f64, f64)>>>,
) -> PyResult<PyObbResult> {
    guard(|| {
        let poly = build_polygon(exterior, holes)?;
        let r = solve_obb_aspect_fit(&poly, ratio_w, ratio_h)
            .ok_or_else(|| PyValueError::new_err("solve failed: no fit found"))?;
        Ok(obb_to_py(&r))
    })
}

/// Register the result class and the three solver functions on `m`.
pub fn register(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    let _ = py;
    m.add_class::<PyObbResult>()?;
    m.add_function(wrap_pyfunction!(solve_obb_py, m)?)?;
    m.add_function(wrap_pyfunction!(solve_obb_constrained_py, m)?)?;
    m.add_function(wrap_pyfunction!(solve_obb_aspect_fit_py, m)?)?;
    Ok(())
}
