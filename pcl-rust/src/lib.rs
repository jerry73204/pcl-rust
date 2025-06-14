//! Safe Rust bindings for Point Cloud Library (PCL)
//!
//! This crate provides safe, idiomatic Rust interfaces to the Point Cloud Library.
//! It builds on top of the `pcl-sys` crate which provides low-level FFI bindings.
//!
//! # Examples
//!
//! ```rust
//! use pcl::PointCloudXYZ;
//!
//! let mut cloud = PointCloudXYZ::new();
//! // Add points, perform operations...
//! ```

pub mod common;
pub mod error;
pub mod octree;
pub mod search;

// Re-export common types for convenience
pub use common::{PointCloudXYZ, PointCloudXYZRGB, PointXYZ, PointXYZRGB};
pub use error::{PclError, PclResult};

#[cfg(test)]
mod tests {
    use super::*;

    // Note: Point creation tests are disabled since point creation
    // is not currently supported due to cxx limitations

    #[test]
    fn test_point_cloud_creation() {
        let cloud = PointCloudXYZ::new().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
    }
}
