//! Core PCL data structures and point types
//!
//! This module provides FFI bindings for PCL's fundamental data structures
//! including point types and the PointCloud container.

pub mod point_cloud;
pub mod point_types;

// Re-export types from the main FFI module
pub use crate::ffi::{
    PointCloud_PointXYZ as PointCloudXYZ, PointCloud_PointXYZRGB as PointCloudXYZRGB, PointXYZ,
    PointXYZI, PointXYZRGB,
};
