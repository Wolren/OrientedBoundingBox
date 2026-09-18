//! C ABI for the OBB solvers.
//!
//! Every exported function returns 0 on success, -1 on error, and never
//! lets a Rust panic cross into the host process. Ported from IGE `ige-c`
//! with the `ige_` prefix replaced by `obb_`.

use super::obb::{solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions};
use geo_types::{Coord, LineString, Polygon};
use libc::{c_double, c_int, size_t};
use std::slice;

/// One hole ring: flat coords + length.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct HoleInput {
    /// Pointer to the hole ring's flat x/y coordinate doubles.
    pub coords: *const c_double,
    /// Number of doubles in `coords`; must be even and at least 6.
    pub coords_len: size_t,
}

/// OBB options.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ObbOptionsC {
    /// Max aspect ratio, longer side over shorter; 0.0 means unconstrained.
    pub max_aspect_ratio: c_double,
    /// Min aspect ratio, longer side over shorter; 0.0 means unconstrained.
    pub min_aspect_ratio: c_double,
    /// Number of angle samples for the search; 0 selects the Rust default.
    pub angle_samples: size_t,
    /// Nonzero enables the PCA initial angle guess.
    pub use_pca: c_int,
    /// Nonzero enables refinement after the initial angle search.
    pub use_refinement: c_int,
    /// Refinement tolerance in degrees; 0.0 or less selects the Rust default.
    pub xatol_deg: c_double,
}

impl Default for ObbOptionsC {
    fn default() -> Self {
        let d = ObbOptions::default();
        Self {
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            angle_samples: d.angle_samples,
            use_pca: 1,
            use_refinement: 1,
            xatol_deg: d.xatol_deg,
        }
    }
}

/// OBB result: ring + full metrics.
#[repr(C)]
pub struct ObbResultC {
    /// Box corners as flat x/y doubles: four points, then the first repeated.
    /// Zero-filled on the aspect-fit path, where no polygon is produced.
    pub ring: [c_double; 10],
    /// Box area in squared units of the input coordinates.
    pub area: c_double,
    /// Box perimeter in input coordinate units.
    pub perimeter: c_double,
    /// Box rotation angle in degrees.
    pub angle_deg: c_double,
    /// Box width along its rotated axis.
    pub width: c_double,
    /// Box height along its rotated axis.
    pub height: c_double,
    /// Aspect ratio, longer side over shorter, always at least 1.
    pub aspect_ratio: c_double,
    /// Fill ratio. Polygon area divided by box area on the min-area and
    /// axis-aligned paths; on the aspect-fit path, the rotated bounding
    /// rectangle area divided by frame area.
    pub fill_ratio: c_double,
    /// Box centroid x, or 0.0 when no centroid was computed.
    pub centroid_x: c_double,
    /// Box centroid y, or 0.0 when no centroid was computed.
    pub centroid_y: c_double,
    /// On the aspect-fit path, `fill_ratio` at angle 0; 0.0 on other paths.
    pub north_fill: c_double,
    /// Percent improvement over north-up; aspect fit only, 0.0 otherwise.
    pub improve_pct: c_double,
    /// Number of caliper intervals; aspect fit only, 0 otherwise.
    pub n_intervals: size_t,
}

/// Decode a flat [x0, y0, x1, y1, ...] array into coords. Validates length.
///
/// # Safety
/// `coords` must point to `coords_len` readable doubles, or be null
/// (null returns None).
pub unsafe fn decode_coords(
    coords: *const c_double,
    coords_len: size_t,
) -> Option<Vec<Coord<f64>>> {
    if coords.is_null() {
        return None;
    }
    if coords_len < 6 || !coords_len.is_multiple_of(2) {
        return None;
    }
    let s = slice::from_raw_parts(coords, coords_len);
    let mut out = Vec::with_capacity(coords_len / 2);
    for i in (0..coords_len).step_by(2) {
        let (x, y) = (s[i], s[i + 1]);
        if !x.is_finite() || !y.is_finite() {
            return None;
        }
        out.push(Coord { x, y });
    }
    Some(out)
}

/// Decode exterior + optional holes into a polygon.
///
/// # Safety
/// Same contract as [`decode_coords`]; `holes` must point to `holes_len`
/// readable entries, or be null (null means no holes).
pub unsafe fn decode_polygon(
    coords: *const c_double,
    coords_len: size_t,
    holes: *const HoleInput,
    holes_len: size_t,
) -> Option<Polygon<f64>> {
    let ext = decode_coords(coords, coords_len)?;
    let mut interiors = Vec::new();
    if !holes.is_null() {
        let hs = slice::from_raw_parts(holes, holes_len);
        for h in hs {
            let ring = decode_coords(h.coords, h.coords_len)?;
            if ring.len() < 3 {
                return None;
            }
            interiors.push(LineString::from(ring));
        }
    }
    Some(Polygon::new(LineString::from(ext), interiors))
}

/// Run a closure behind the panic boundary. Panic becomes -1.
pub fn guard(f: impl FnOnce() -> c_int + std::panic::UnwindSafe) -> c_int {
    std::panic::catch_unwind(f).unwrap_or(-1)
}

fn opts_from(raw: ObbOptionsC) -> ObbOptions {
    let d = ObbOptions::default();
    ObbOptions {
        max_ratio: raw.max_aspect_ratio,
        min_ratio: raw.min_aspect_ratio,
        angle_samples: if raw.angle_samples > 0 {
            raw.angle_samples
        } else {
            d.angle_samples
        },
        use_pca: raw.use_pca != 0,
        use_refinement: raw.use_refinement != 0,
        xatol_deg: if raw.xatol_deg > 0.0 {
            raw.xatol_deg
        } else {
            d.xatol_deg
        },
    }
}

fn fill(r: &super::obb::ObbResult, out: *mut ObbResultC) -> c_int {
    if out.is_null() {
        return -1;
    }
    let (cx, cy) = match &r.centroid {
        Some(p) => (p.x(), p.y()),
        None => (0.0, 0.0),
    };
    let mut ring = [0.0f64; 10];
    if let Some(p) = r.polygon.as_ref() {
        for (i, c) in p.exterior().0.iter().take(5).enumerate() {
            ring[i * 2] = c.x;
            ring[i * 2 + 1] = c.y;
        }
    }
    unsafe {
        *out = ObbResultC {
            ring,
            area: r.area,
            perimeter: r.perimeter,
            angle_deg: r.angle_deg,
            width: r.width,
            height: r.height,
            aspect_ratio: r.aspect_ratio,
            fill_ratio: r.fill_ratio,
            centroid_x: cx,
            centroid_y: cy,
            north_fill: r.north_fill,
            improve_pct: r.improve_pct,
            n_intervals: r.n_intervals,
        };
    }
    0
}

/// Solve min-area OBB. Holes optional (null = none).
///
/// Returns 0 on success, -1 on error.
///
/// # Safety
/// All pointer arguments must be valid for reads of their declared lengths
/// for the duration of the call: coord arrays hold coords_len doubles with
/// an even length of at least 6, hole arrays hold holes_len entries, and
/// out-pointers must be non-null with room for the documented result
/// struct. Null or short inputs return -1 instead of crashing.
#[no_mangle]
pub unsafe extern "C" fn obb_solve(
    coords: *const c_double,
    coords_len: size_t,
    holes: *const HoleInput,
    holes_len: size_t,
    options: *const ObbOptionsC,
    result: *mut ObbResultC,
) -> c_int {
    guard(|| {
        if result.is_null() {
            return -1;
        }
        let poly = match decode_polygon(coords, coords_len, holes, holes_len) {
            Some(p) => p,
            None => return -1,
        };
        let opts = if options.is_null() {
            ObbOptions::default()
        } else {
            opts_from(*options)
        };
        match solve_obb(&poly, &opts) {
            Ok(r) => fill(&r, result),
            Err(_) => -1,
        }
    })
}

/// Solve OBB with aspect-ratio clamps.
///
/// Returns 0 on success, -1 on error.
///
/// # Safety
/// Same contract as [`obb_solve`].
#[no_mangle]
pub unsafe extern "C" fn obb_solve_constrained(
    coords: *const c_double,
    coords_len: size_t,
    holes: *const HoleInput,
    holes_len: size_t,
    options: *const ObbOptionsC,
    result: *mut ObbResultC,
) -> c_int {
    guard(|| {
        if result.is_null() {
            return -1;
        }
        let poly = match decode_polygon(coords, coords_len, holes, holes_len) {
            Some(p) => p,
            None => return -1,
        };
        let opts = if options.is_null() {
            ObbOptions::default()
        } else {
            opts_from(*options)
        };
        match solve_obb_constrained(&poly, &opts) {
            Ok(r) => fill(&r, result),
            Err(_) => -1,
        }
    })
}

/// Solve OBB aspect fit for a fixed ratio frame.
///
/// Returns 0 on success, -1 on error.
///
/// # Safety
/// Same contract as [`obb_solve`].
#[no_mangle]
pub unsafe extern "C" fn obb_solve_aspect_fit(
    coords: *const c_double,
    coords_len: size_t,
    holes: *const HoleInput,
    holes_len: size_t,
    ratio_w: c_double,
    ratio_h: c_double,
    result: *mut ObbResultC,
) -> c_int {
    guard(|| {
        if result.is_null() {
            return -1;
        }
        let poly = match decode_polygon(coords, coords_len, holes, holes_len) {
            Some(p) => p,
            None => return -1,
        };
        match solve_obb_aspect_fit(&poly, ratio_w, ratio_h) {
            Some(r) => fill(&r, result),
            None => -1,
        }
    })
}

/// Return OBB options holding the crate's default values.
#[no_mangle]
pub extern "C" fn obb_options_default() -> ObbOptionsC {
    ObbOptionsC::default()
}
