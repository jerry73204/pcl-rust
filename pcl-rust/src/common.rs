//! Core PCL types and utilities
//!
//! This module provides safe wrappers around PCL's fundamental data structures
//! including point types and the PointCloud container.

pub mod builders;
pub mod point_cloud_generic;
pub mod point_types;

#[cfg(test)]
mod tests;

pub use builders::{PointCloudNormalBuilder, PointCloudXYZBuilder, PointCloudXYZRGBBuilder};
// Export generic PointCloud and type aliases
pub use point_cloud_generic::{
    PointCloud, PointCloudNormal, PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB,
};
pub use point_types::*;
