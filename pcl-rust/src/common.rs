//! Core PCL types and utilities
//!
//! This module provides safe wrappers around PCL's fundamental data structures
//! including point types and the PointCloud container.

pub mod builders;
pub mod point_cloud;
pub mod point_types;
pub mod transform;

#[cfg(test)]
mod tests;

pub use builders::{PointCloudNormalBuilder, PointCloudXYZBuilder, PointCloudXYZRGBBuilder};
// Export the new PointCloud with marker types
pub use point_cloud::{PointCloud, PointCloudIter};
pub use transform::{TransformBuilder, transform_point_cloud};

// New point type system - these are the primary types now
pub use point_types::{
    Normal,
    PointNormal,
    // Compatibility trait
    PointNormalOps,
    PointNormalRef,
    PointType,
    PointXYZ,
    PointXYZI,
    PointXYZIRef,
    PointXYZRGB,
    PointXYZRGBRef,
    // Reference types for compatibility
    PointXYZRef,
    ToPointOwned,
    XYZ,
    XYZI,
    XYZRGB,
};

// Compatibility aliases for the old API - these will eventually be removed

// Type aliases for old PointCloud API
pub type PointCloudXYZ = PointCloud<XYZ>;
pub type PointCloudXYZI = PointCloud<XYZI>;
pub type PointCloudXYZRGB = PointCloud<XYZRGB>;
pub type PointCloudNormal = PointCloud<Normal>;
