//! Point type definitions and utilities
//!
//! This module provides FFI bindings for PCL's various point types.

use crate::ffi;
use crate::{PointXYZ, PointXYZI, PointXYZRGB};

impl PointXYZ {
    pub fn x(&self) -> f32 {
        ffi::get_x(self)
    }

    pub fn y(&self) -> f32 {
        ffi::get_y(self)
    }

    pub fn z(&self) -> f32 {
        ffi::get_z(self)
    }
}

impl PointXYZRGB {
    pub fn x(&self) -> f32 {
        ffi::get_x_xyzrgb(self)
    }

    pub fn y(&self) -> f32 {
        ffi::get_y_xyzrgb(self)
    }

    pub fn z(&self) -> f32 {
        ffi::get_z_xyzrgb(self)
    }

    pub fn r(&self) -> u8 {
        ffi::get_r(self)
    }

    pub fn g(&self) -> u8 {
        ffi::get_g(self)
    }

    pub fn b(&self) -> u8 {
        ffi::get_b(self)
    }
}
