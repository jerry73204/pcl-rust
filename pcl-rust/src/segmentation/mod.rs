//! Point cloud segmentation algorithms
//!
//! This module provides safe Rust interfaces for various point cloud segmentation
//! algorithms including region growing, clustering, and model-based segmentation.

pub mod clustering;
pub mod region_growing;
pub mod sac_segmentation;

pub use clustering::{ClusteringXYZ, EuclideanClusterExtractionXYZ};
pub use region_growing::{
    RegionGrowingRgbXYZRGB, RegionGrowingXYZ, SegmentationXYZ, SegmentationXYZRGB,
};
pub use sac_segmentation::{MethodType, ModelType, SacSegmentationXYZ};

use crate::error::PclResult;

/// Common trait for point cloud segmentation algorithms
pub trait Segmentation<T> {
    /// Set the input point cloud for segmentation
    fn set_input_cloud(&mut self, cloud: &T) -> PclResult<()>;

    /// Perform segmentation and return cluster indices
    fn segment(&mut self) -> PclResult<Vec<Vec<i32>>>;
}

/// Constants for SAC model types
pub mod model_types {
    pub const SACMODEL_PLANE: i32 = 0;
    pub const SACMODEL_LINE: i32 = 1;
    pub const SACMODEL_CIRCLE2D: i32 = 2;
    pub const SACMODEL_CIRCLE3D: i32 = 3;
    pub const SACMODEL_SPHERE: i32 = 4;
    pub const SACMODEL_CYLINDER: i32 = 5;
    pub const SACMODEL_CONE: i32 = 6;
}

/// Constants for SAC method types
pub mod method_types {
    pub const SAC_RANSAC: i32 = 0;
    pub const SAC_LMEDS: i32 = 1;
    pub const SAC_MSAC: i32 = 2;
    pub const SAC_RRANSAC: i32 = 3;
    pub const SAC_RMSAC: i32 = 4;
    pub const SAC_MLESAC: i32 = 5;
    pub const SAC_PROSAC: i32 = 6;
}
