//! Point type definitions for PCL
//!
//! This module implements:
//! - Owned types (PointXYZ, etc.) as simple Rust structs
//! - Marker types (XYZ, etc.) for the new PointCloud<T> API
//! - Trait implementations for compatibility with existing code

use cxx::memory::UniquePtrTarget;
use pcl_sys::ffi;
use std::fmt::Debug;
// Removed unused imports

// ============================================================================
// Marker Types
// ============================================================================

/// Marker type for XYZ point types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZ;

/// Marker type for XYZI point types  
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZI;

/// Marker type for XYZRGB point types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZRGB;

/// Marker type for PointNormal types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Normal;

// ============================================================================
// Core Trait
// ============================================================================

/// Trait for converting reference types to owned types
pub trait ToPointOwned {
    /// The owned type
    type Owned;

    /// Convert to owned type
    fn to_owned(&self) -> Self::Owned;
}

/// Trait that connects marker types to their owned and reference types
pub trait PointType: Debug + Clone + Copy + 'static {
    /// The owned point type (e.g., PointXYZ)
    type Owned: Clone + Copy + Debug + PartialEq + 'static;

    /// The reference point type (e.g., PointXYZRef)
    type Ref: ?Sized + ToPointOwned<Owned = Self::Owned>;

    /// The FFI cloud type
    type CloudType: UniquePtrTarget;

    /// The FFI point type
    type FfiPointType: UniquePtrTarget;

    /// Get the type name for debugging
    fn type_name() -> &'static str;
}

// ============================================================================
// Owned Point Types
// ============================================================================

/// A 3D point with x, y, z coordinates (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl PointXYZ {
    /// Create a new point
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Calculate squared distance to another point
    pub fn distance_squared_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance to another point
    pub fn distance_to(&self, other: &Self) -> f32 {
        self.distance_squared_to(other).sqrt()
    }
}

/// A 3D point with x, y, z coordinates and intensity (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl PointXYZI {
    /// Create a new point with intensity
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }
}

/// A 3D point with x, y, z coordinates and RGB color (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    _padding: u8, // To match PCL's 32-bit alignment
}

impl PointXYZRGB {
    /// Create a new colored point
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self {
            x,
            y,
            z,
            r,
            g,
            b,
            _padding: 0,
        }
    }
}

/// A 3D point with normal information (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
    pub curvature: f32,
}

impl PointNormal {
    /// Create a new point with normal
    pub fn new(x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        Self {
            x,
            y,
            z,
            normal_x: nx,
            normal_y: ny,
            normal_z: nz,
            curvature: 0.0,
        }
    }
}

// ============================================================================
// PointType Implementations
// ============================================================================

impl PointType for XYZ {
    type Owned = PointXYZ;
    type Ref = PointXYZ; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }
}

impl PointType for XYZI {
    type Owned = PointXYZI;
    type Ref = PointXYZI; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }
}

impl PointType for XYZRGB {
    type Owned = PointXYZRGB;
    type Ref = PointXYZRGB; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }
}

impl PointType for Normal {
    type Owned = PointNormal;
    type Ref = PointNormal; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }
}

// ============================================================================
// ToPointOwned implementations for owned types (self-conversion)
// ============================================================================

impl ToPointOwned for PointXYZ {
    type Owned = PointXYZ;

    fn to_owned(&self) -> Self::Owned {
        *self
    }
}

impl ToPointOwned for PointXYZI {
    type Owned = PointXYZI;

    fn to_owned(&self) -> Self::Owned {
        *self
    }
}

impl ToPointOwned for PointXYZRGB {
    type Owned = PointXYZRGB;

    fn to_owned(&self) -> Self::Owned {
        *self
    }
}

impl ToPointOwned for PointNormal {
    type Owned = PointNormal;

    fn to_owned(&self) -> Self::Owned {
        *self
    }
}

// ============================================================================
// Removed deprecated trait implementations
// ============================================================================
// All trait implementations for Point, Xyz, Rgb, Intensity, NormalXyz,
// PointXyzOps, PointRgbOps, and PointIntensityOps have been removed.
// Use the new marker type system with PointType trait instead.

// ============================================================================
// PointType Implementations for Owned Types (for compatibility)
// ============================================================================

// These allow owned types to work with new PointCloud<T> API
// For example: PointCloud<PointXYZ> instead of just PointCloud<XYZ>

impl PointType for PointXYZ {
    type Owned = PointXYZ;
    type Ref = PointXYZ; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }
}

impl PointType for PointXYZRGB {
    type Owned = PointXYZRGB;
    type Ref = PointXYZRGB; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }
}

impl PointType for PointXYZI {
    type Owned = PointXYZI;
    type Ref = PointXYZI; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }
}

impl PointType for PointNormal {
    type Owned = PointNormal;
    type Ref = PointNormal; // Using owned type as reference for now
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }
}

// ============================================================================
// Conversion Utilities
// ============================================================================

/// Convert owned point to FFI point for pushing to cloud
pub fn owned_to_ffi_xyz(point: PointXYZ) -> cxx::UniquePtr<ffi::PointXYZ> {
    ffi::create_point_xyz(point.x, point.y, point.z)
}

pub fn owned_to_ffi_xyzi(point: PointXYZI) -> cxx::UniquePtr<ffi::PointXYZI> {
    ffi::create_point_xyzi(point.x, point.y, point.z, point.intensity)
}

pub fn owned_to_ffi_xyzrgb(point: PointXYZRGB) -> cxx::UniquePtr<ffi::PointXYZRGB> {
    ffi::create_point_xyzrgb(point.x, point.y, point.z, point.r, point.g, point.b)
}

pub fn owned_to_ffi_normal(point: PointNormal) -> cxx::UniquePtr<ffi::PointNormal> {
    ffi::create_point_normal(
        point.x,
        point.y,
        point.z,
        point.normal_x,
        point.normal_y,
        point.normal_z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_owned_point_creation() {
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.z, 3.0);
    }

    #[test]
    fn test_point_distance() {
        let p1 = PointXYZ::new(0.0, 0.0, 0.0);
        let p2 = PointXYZ::new(3.0, 4.0, 0.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
    }

    #[test]
    fn test_marker_types() {
        // Verify marker types work correctly
        assert_eq!(XYZ::type_name(), "PointXYZ");
        assert_eq!(XYZI::type_name(), "PointXYZI");
    }

    #[test]
    fn test_conversions() {
        // Test owned to FFI conversion
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        let ffi_point = owned_to_ffi_xyz(p);
        assert!(!ffi_point.is_null());
    }
}
