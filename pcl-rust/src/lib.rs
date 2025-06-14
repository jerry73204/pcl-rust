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
pub use common::{PointCloudXYZ, PointCloudXYZRGB, PointXYZ, PointXYZI, PointXYZRGB};
pub use error::{PclError, PclResult};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::octree::{OctreeSearchXYZ, OctreeVoxelCentroidXYZ};
    use crate::search::KdTreeXYZ;

    // Note: Point creation tests are disabled since point creation
    // is not currently supported due to cxx limitations

    #[test]
    fn test_point_cloud_creation() {
        let cloud = PointCloudXYZ::new().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
        assert_eq!(cloud.width(), 0);
        assert_eq!(cloud.height(), 0);
        assert!(!cloud.is_organized());
    }

    #[test]
    fn test_point_cloud_xyzrgb_creation() {
        let cloud = PointCloudXYZRGB::new().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
        assert_eq!(cloud.width(), 0);
        assert_eq!(cloud.height(), 0);
        assert!(!cloud.is_organized());
    }

    #[test]
    fn test_point_cloud_operations() {
        let mut cloud = PointCloudXYZ::new().unwrap();

        // Test initial state
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);

        // Test reserve
        cloud.reserve(100).unwrap();
        assert!(cloud.empty()); // Still empty after reserve

        // Test resize
        cloud.resize(50).unwrap();
        assert!(!cloud.empty());
        assert_eq!(cloud.size(), 50);

        // Test clear operation
        cloud.clear().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
    }

    #[test]
    fn test_point_cloud_builders() {
        use crate::common::PointCloudXYZBuilder;

        // Test XYZ builder
        let cloud = PointCloudXYZBuilder::new()
            .add_point(1.0, 2.0, 3.0)
            .add_points(vec![(4.0, 5.0, 6.0), (7.0, 8.0, 9.0)])
            .build()
            .unwrap();
        assert_eq!(cloud.size(), 3);

        // Test empty builder
        let empty_cloud = PointCloudXYZBuilder::new().build().unwrap();
        assert!(empty_cloud.empty());
    }

    #[test]
    fn test_point_xyzi() {
        // Since we can't create points directly, this test is limited
        // but ensures the type exists and can be used
        use crate::PointXYZI;

        // The type should be available
        fn _accept_point(_point: &PointXYZI) {
            // This function just checks that the type exists
        }
    }

    #[test]
    fn test_kdtree_creation() {
        let kdtree = KdTreeXYZ::new().unwrap();
        // Basic creation test - more functionality would require points
        drop(kdtree);
    }

    #[test]
    fn test_octree_creation() {
        let octree = OctreeSearchXYZ::new(0.1).unwrap();
        drop(octree);

        // Test invalid resolution
        let result = OctreeSearchXYZ::new(-1.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_octree_voxel_centroid_creation() {
        let octree = OctreeVoxelCentroidXYZ::new(0.1).unwrap();
        drop(octree);

        // Test invalid resolution
        let result = OctreeVoxelCentroidXYZ::new(0.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_error_handling() {
        // Test parameter validation
        let kdtree = KdTreeXYZ::new().unwrap();

        // This would require actual points to test properly, but we can test parameter validation
        // KdTree search methods would need points to work with
    }
}
