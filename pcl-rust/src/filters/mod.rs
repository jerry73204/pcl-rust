//! Safe Rust wrappers for PCL filters
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's filtering algorithms.
//! All filters follow a builder pattern for configuration and provide comprehensive
//! error handling.

pub mod generic;
pub mod pass_through;
pub mod radius_outlier_removal;
pub mod statistical_outlier_removal;
pub mod voxel_grid;

#[cfg(test)]
mod tests;

pub use pass_through::{
    PassThroughXYZ, PassThroughXYZBuilder, PassThroughXYZRGB, PassThroughXYZRGBBuilder,
};
pub use radius_outlier_removal::{
    RadiusOutlierRemovalXYZ, RadiusOutlierRemovalXYZBuilder, RadiusOutlierRemovalXYZRGB,
    RadiusOutlierRemovalXYZRGBBuilder,
};
pub use statistical_outlier_removal::{
    StatisticalOutlierRemovalXYZ, StatisticalOutlierRemovalXYZBuilder,
    StatisticalOutlierRemovalXYZRGB, StatisticalOutlierRemovalXYZRGBBuilder,
};
pub use voxel_grid::{VoxelGridXYZ, VoxelGridXYZBuilder, VoxelGridXYZRGB, VoxelGridXYZRGBBuilder};

// Re-export generic filters and traits
pub use generic::{
    Filter, PassThrough, PassThroughXYZ as GenericPassThroughXYZ,
    PassThroughXYZRGB as GenericPassThroughXYZRGB, VoxelGrid, VoxelGridBuilder,
    VoxelGridXYZ as GenericVoxelGridXYZ, VoxelGridXYZRGB as GenericVoxelGridXYZRGB,
};

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
