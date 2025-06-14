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
pub mod filters;
pub mod io;
pub mod octree;
pub mod registration;
pub mod sample_consensus;
pub mod search;

#[cfg(test)]
mod error_tests;

// Re-export common types for convenience
pub use common::{
    PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB, PointXYZ, PointXYZI, PointXYZRGB,
};
pub use error::{PclError, PclResult};
pub use filters::{FilterXYZ, FilterXYZRGB, PassThroughXYZ, PassThroughXYZRGB};
pub use io::{
    BinaryFormat, FileFormat, PcdIoXYZ, PcdIoXYZI, PcdIoXYZRGB, PlyIoXYZ, PlyIoXYZI, PlyIoXYZRGB,
};
pub use registration::{IcpXYZ, IcpXYZRGB, Transform3D, TransformationMatrix};
pub use sample_consensus::{
    RansacPlaneXYZ, RansacPlaneXYZRGB, RansacSphereXYZ, RansacSphereXYZRGB,
};

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
        use crate::search::{KdTreeXYZ, KdTreeXYZRGB, SearchInputCloud};

        let kdtree = KdTreeXYZ::new().unwrap();
        assert!(!kdtree.has_input_cloud());
        drop(kdtree);

        let kdtree_rgb = KdTreeXYZRGB::new().unwrap();
        assert!(!kdtree_rgb.has_input_cloud());
        drop(kdtree_rgb);
    }

    #[test]
    fn test_kdtree_configuration() {
        use crate::search::SearchConfiguration;

        let mut kdtree = KdTreeXYZ::new().unwrap();

        // Test epsilon configuration
        let initial_epsilon = kdtree.epsilon();
        assert!(initial_epsilon >= 0.0);

        kdtree.set_epsilon(0.5).unwrap();
        assert_eq!(kdtree.epsilon(), 0.5);

        // Test invalid epsilon
        let result = kdtree.set_epsilon(-1.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_search_input_cloud() {
        use crate::search::SearchInputCloud;

        let mut kdtree = KdTreeXYZ::new().unwrap();
        let cloud = PointCloudXYZ::new().unwrap();

        assert!(!kdtree.has_input_cloud());
        kdtree.set_input_cloud(&cloud).unwrap();
        assert!(kdtree.has_input_cloud());
    }

    #[test]
    fn test_unified_search_interface() {
        use crate::search::{SearchMethod, SearchXYZ, SearchXYZRGB};

        // Test XYZ search
        let search_xyz = SearchXYZ::new(SearchMethod::KdTree).unwrap();
        assert_eq!(search_xyz.method(), SearchMethod::KdTree);

        // Test XYZRGB search
        let search_xyzrgb = SearchXYZRGB::new(SearchMethod::KdTree).unwrap();
        assert_eq!(search_xyzrgb.method(), SearchMethod::KdTree);

        // Test unsupported method
        let result = SearchXYZ::new(SearchMethod::Octree);
        assert!(result.is_err());
    }

    #[test]
    fn test_search_traits() {
        use crate::search::{NearestNeighborSearch, SearchConfiguration};

        let kdtree = KdTreeXYZ::new().unwrap();

        // Test trait implementation
        fn accept_search<T: NearestNeighborSearch<PointXYZ> + SearchConfiguration>(_search: &T) {
            // This just tests that the traits are implemented
        }

        accept_search(&kdtree);
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
