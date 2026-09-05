//! In-process smoke tests for the C ABI: flat arrays in, structs out.

use oriented_bounding_box::ffi::{
    obb_options_default, obb_solve, obb_solve_aspect_fit, obb_solve_constrained, ObbResultC,
};
use std::mem::MaybeUninit;

fn square() -> Vec<f64> {
    vec![
        0.0, 0.0, //
        10.0, 0.0, //
        10.0, 10.0, //
        0.0, 10.0, //
        0.0, 0.0,
    ]
}

fn call_result() -> ObbResultC {
    unsafe {
        let mut out = MaybeUninit::<ObbResultC>::uninit();
        let rc = obb_solve(
            square().as_ptr(),
            10,
            std::ptr::null(),
            0,
            std::ptr::null(),
            out.as_mut_ptr(),
        );
        assert_eq!(rc, 0);
        out.assume_init()
    }
}

#[test]
fn ffi_min_area_square() {
    let r = call_result();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
    assert!((r.aspect_ratio - 1.0).abs() < 1e-6, "ar={}", r.aspect_ratio);
}

#[test]
fn ffi_constrained_clamps_ratio() {
    let rect = vec![
        0.0, 0.0, //
        20.0, 0.0, //
        20.0, 5.0, //
        0.0, 5.0, //
        0.0, 0.0,
    ];
    let mut opts = obb_options_default();
    opts.max_aspect_ratio = 2.0;
    unsafe {
        let mut out = MaybeUninit::<ObbResultC>::uninit();
        let rc = obb_solve_constrained(
            rect.as_ptr(),
            10,
            std::ptr::null(),
            0,
            &opts,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, 0);
        let r = out.assume_init();
        assert!(r.aspect_ratio <= 2.0 + 1e-6, "ar={}", r.aspect_ratio);
    }
}

#[test]
fn ffi_aspect_fit_a4_square() {
    unsafe {
        let sq = square();
        let mut out = MaybeUninit::<ObbResultC>::uninit();
        let rc = obb_solve_aspect_fit(
            sq.as_ptr(),
            10,
            std::ptr::null(),
            0,
            297.0,
            210.0,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, 0);
        let r = out.assume_init();
        assert!(r.fill_ratio > 0.5, "fill={}", r.fill_ratio);
    }
}

#[test]
fn ffi_rejects_garbage() {
    unsafe {
        let mut out = MaybeUninit::<ObbResultC>::uninit();
        // Null coords.
        let rc = obb_solve(
            std::ptr::null(),
            0,
            std::ptr::null(),
            0,
            std::ptr::null(),
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        // Null out-pointer.
        let sq = square();
        let rc = obb_solve(
            sq.as_ptr(),
            10,
            std::ptr::null(),
            0,
            std::ptr::null(),
            std::ptr::null_mut(),
        );
        assert_eq!(rc, -1);
    }
}
