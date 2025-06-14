//! Safe wrappers for PCL point types
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's point types.

use pcl_sys::ffi;

/// A 3D point with x, y, z coordinates
pub struct PointXYZ {
    pub(crate) inner: ffi::PointXYZ,
}

impl PointXYZ {
    /// Get the x coordinate
    pub fn x(&self) -> f32 {
        ffi::get_x(&self.inner)
    }

    /// Get the y coordinate
    pub fn y(&self) -> f32 {
        ffi::get_y(&self.inner)
    }

    /// Get the z coordinate
    pub fn z(&self) -> f32 {
        ffi::get_z(&self.inner)
    }

    /// Get coordinates as a tuple
    pub fn xyz(&self) -> (f32, f32, f32) {
        (self.x(), self.y(), self.z())
    }
}

/// A 3D point with x, y, z coordinates and intensity
pub struct PointXYZI {
    pub(crate) inner: ffi::PointXYZI,
}

impl PointXYZI {
    /// Get the x coordinate
    pub fn x(&self) -> f32 {
        ffi::get_x_xyzi(&self.inner)
    }

    /// Get the y coordinate
    pub fn y(&self) -> f32 {
        ffi::get_y_xyzi(&self.inner)
    }

    /// Get the z coordinate
    pub fn z(&self) -> f32 {
        ffi::get_z_xyzi(&self.inner)
    }

    /// Get the intensity value
    pub fn intensity(&self) -> f32 {
        ffi::get_intensity(&self.inner)
    }

    /// Get coordinates as a tuple
    pub fn xyz(&self) -> (f32, f32, f32) {
        (self.x(), self.y(), self.z())
    }

    /// Get all components as a tuple
    pub fn xyzi(&self) -> (f32, f32, f32, f32) {
        (self.x(), self.y(), self.z(), self.intensity())
    }
}

// Note: Point creation is not currently supported due to cxx limitations
// Points must be created and managed by PCL C++ code

/// A 3D point with x, y, z coordinates and RGB color
pub struct PointXYZRGB {
    pub(crate) inner: ffi::PointXYZRGB,
}

impl PointXYZRGB {
    /// Get the x coordinate
    pub fn x(&self) -> f32 {
        ffi::get_x_xyzrgb(&self.inner)
    }

    /// Get the y coordinate
    pub fn y(&self) -> f32 {
        ffi::get_y_xyzrgb(&self.inner)
    }

    /// Get the z coordinate
    pub fn z(&self) -> f32 {
        ffi::get_z_xyzrgb(&self.inner)
    }

    /// Get the red component
    pub fn r(&self) -> u8 {
        ffi::get_r(&self.inner)
    }

    /// Get the green component
    pub fn g(&self) -> u8 {
        ffi::get_g(&self.inner)
    }

    /// Get the blue component
    pub fn b(&self) -> u8 {
        ffi::get_b(&self.inner)
    }

    /// Get coordinates as a tuple
    pub fn xyz(&self) -> (f32, f32, f32) {
        (self.x(), self.y(), self.z())
    }

    /// Get RGB values as a tuple
    pub fn rgb(&self) -> (u8, u8, u8) {
        (self.r(), self.g(), self.b())
    }

    /// Get all components as a tuple
    pub fn xyzrgb(&self) -> (f32, f32, f32, u8, u8, u8) {
        (self.x(), self.y(), self.z(), self.r(), self.g(), self.b())
    }
}
