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
// Export generic PointCloud and type aliases
pub use point_cloud::{
    PointCloud, PointCloudNormal, PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB,
};
pub use point_types::*;
pub use transform::{TransformBuilder, transform_point_cloud};
