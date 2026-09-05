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

pub mod obb;
pub mod shared;

pub use obb::{
    build_obb_frame, solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions, ObbResult,
};
pub use obb::axis_aligned::solve_obb_axis_aligned;
pub use shared::{LirError, Result};
