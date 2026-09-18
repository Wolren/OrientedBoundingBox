//! Oriented bounding box (OBB) solvers for polygons.
//!
//! An OBB is the smallest rectangle of a chosen orientation that contains a
//! polygon. The axis-aligned bounding box is the zero-degree case. This is the
//! enclosing direction, the opposite of inscribed solvers, which place shapes
//! inside the polygon.
//!
//! # Quick start
//!
//! ```
//! use oriented_bounding_box::geo_types::{LineString, Polygon};
//! use oriented_bounding_box::{solve_obb, ObbOptions};
//!
//! let l_shape = Polygon::new(
//!     LineString::from(vec![
//!         (0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (1.0, 1.0), (1.0, 3.0), (0.0, 3.0), (0.0, 0.0),
//!     ]),
//!     vec![],
//! );
//!
//! let obb = solve_obb(&l_shape, &ObbOptions::default())?;
//! assert!(obb.area <= 12.0 + 1e-9);
//! # Ok::<(), oriented_bounding_box::LirError>(())
//! ```
//!
//! # Solvers
//!
//! | Function | Box |
//! |----------|-----|
//! | [`solve_obb_axis_aligned`] | axis-aligned, via `geo::BoundingRect`; the options argument is ignored |
//! | [`solve_obb`] | minimum area, rotating calipers over the convex hull |
//! | [`solve_obb_constrained`] | minimum area, then the shorter side is expanded to satisfy [`ObbOptions::max_ratio`] or [`ObbOptions::min_ratio`] |
//! | [`solve_obb_aspect_fit`] | optimal rotation for a frame with a fixed `ratio_w` to `ratio_h`, via the caliper support function |
//! | [`build_obb_frame`] | the frame polygon for one known angle |
//!
//! [`solve_obb_aspect_fit`] and [`build_obb_frame`] report the frame with the
//! unrotated bounding extents and the frame centroid, so callers can draw the
//! box without re-deriving the geometry.
//!
//! Non-finite coordinates are rejected with [`LirError::InvalidPolygon`] before
//! the hull is built, instead of panicking inside the hull routine.
//!
//! # Features
//!
//! | Feature | Effect |
//! |---------|--------|
//! | `python` | PyO3 extension module named `oriented_bounding_box` |
//! | `extension-module` | build the Python module as an importable extension |
//! | `geo-traits` | generic solves over any `geo_traits` polygon, plus `geo_types::Geometry` dispatch |
//!
//! Default features are empty.
//!
//! # C ABI
//!
//! [`ffi`] exposes `obb_solve`, `obb_solve_constrained`, `obb_solve_aspect_fit`
//! and `obb_options_default`: flat coordinate arrays in, `#[repr(C)]` structs
//! out, `0` on success and `-1` on error, with panics caught at the boundary.
//! The crate builds as `rlib`, `cdylib` and `staticlib`.

#![warn(missing_docs)]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod ffi;
pub mod obb;
pub mod shared;

#[cfg(feature = "python")]
#[cfg_attr(docsrs, doc(cfg(feature = "python")))]
pub mod python;

#[cfg(feature = "geo-traits")]
#[cfg_attr(docsrs, doc(cfg(feature = "geo-traits")))]
pub mod geo_interop;

/// Pinned `geo-types` re-export so downstream crates lock to the same
/// version this crate solves over.
pub use geo_types;

pub use obb::axis_aligned::solve_obb_axis_aligned;
pub use obb::{
    build_obb_frame, solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions, ObbResult,
};
pub use shared::{LirError, Result};

#[cfg(feature = "python")]
use pyo3::prelude::*;

/// Python extension module (feature `python`).
///
/// Build with maturin or with
/// `cargo build --features python,extension-module`, then import
/// `oriented_bounding_box`.
#[cfg(feature = "python")]
#[pymodule]
pub fn oriented_bounding_box(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    python::register(py, m)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
