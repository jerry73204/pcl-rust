//! Safe Rust wrappers for PCL filters
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's filtering algorithms.
//! All filters follow a builder pattern for configuration and provide comprehensive
//! error handling.

pub mod pass_through;

pub use pass_through::{
    PassThroughXYZ, PassThroughXYZBuilder, PassThroughXYZRGB, PassThroughXYZRGBBuilder,
};

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::PclResult;

/// Trait for all PCL filters with PointXYZ
pub trait FilterXYZ {
    /// Set the input point cloud for filtering
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()>;

    /// Apply the filter and return the filtered point cloud
    fn filter(&mut self) -> PclResult<PointCloudXYZ>;
}

/// Trait for all PCL filters with PointXYZRGB
pub trait FilterXYZRGB {
    /// Set the input point cloud for filtering
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()>;

    /// Apply the filter and return the filtered point cloud
    fn filter(&mut self) -> PclResult<PointCloudXYZRGB>;
}

/// Common filter field names used in PCL
pub mod field_names {
    /// X coordinate field
    pub const X: &str = "x";
    /// Y coordinate field
    pub const Y: &str = "y";
    /// Z coordinate field
    pub const Z: &str = "z";
    /// RGB field for colored point clouds
    pub const RGB: &str = "rgb";
    /// Intensity field for intensity point clouds
    pub const INTENSITY: &str = "intensity";
}
