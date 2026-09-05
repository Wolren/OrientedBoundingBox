//! Oriented bounding box (OBB) solvers extracted from IGE.
//!
//! Enclosing-box solvers: the box contains the polygon. This is the opposite
//! direction from inscribed solvers (LIR, MIC), which compute shapes inside
//! the polygon.
//!
//! ## Entry points
//! - `solve_obb_axis_aligned` - axis-aligned bounding rect via `geo::BoundingRect`
//! - `solve_obb` - minimal-area oriented box via rotating calipers
//! - `solve_obb_constrained` - min-area solve plus shorter-side expansion to
//!   satisfy `ObbOptions::max_ratio` / `min_ratio`
//! - `solve_obb_aspect_fit` + `build_obb_frame` - exact AFR optimal rotation
//!   for a fixed `ratio_w` / `ratio_h` frame
//!
//! ## Features
//! | Feature | Effect |
//! |---------|--------|
//! | `python` | PyO3 bindings module `oriented_bounding_box` |
//! | `extension-module` | Build the Python module as an importable extension |
//! | `geo-traits` | Generic solves over `geo_traits::PolygonTrait` plus `Geometry` dispatch |
//!
//! ## Foreign-function interface
//!
//! The C ABI lives in [`ffi`] (`obb_solve`, `obb_solve_constrained`,
//! `obb_solve_aspect_fit`): flat coordinate arrays in, `#[repr(C)]`
//! result structs out, 0 on success, -1 on error, panics caught at the
//! boundary.

pub mod ffi;
pub mod obb;
pub mod shared;

#[cfg(feature = "python")]
pub mod python;

#[cfg(feature = "geo-traits")]
pub mod geo_interop;

/// Pinned `geo-types` re-export so downstream crates lock to the same
/// version this crate solves over.
pub use geo_types;

pub use obb::{
    build_obb_frame, solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions, ObbResult,
};
pub use obb::axis_aligned::solve_obb_axis_aligned;
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
