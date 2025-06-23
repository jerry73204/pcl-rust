//! Safe Rust bindings for Point Cloud Library (PCL)
//!
//! This crate provides safe, idiomatic Rust interfaces to the Point Cloud Library.
//! It builds on top of the `pcl-sys` crate which provides low-level FFI bindings.
//!
//! # Migration Guide (v0.2.0)
//!
//! **IMPORTANT**: This version introduces a new point type system using marker types.
//! The old trait-based system is deprecated but still supported with warnings.
//!
//! ## New Point Type System
//!
//! ### Before (Deprecated):
//! ```rust,ignore
//! use pcl::{PointCloud, PointXYZ};
//! use pcl::traits::{Point, Xyz, Rgb};
//!
//! // Old trait-based approach (now deprecated)
//! fn process_cloud<T: Point + Xyz>(cloud: &PointCloud<T>) {
//!     // Process any point type with XYZ coordinates
//! }
//! ```
//!
//! ### After (Recommended):
//! ```rust
//! use pcl::{PointCloud, PointXYZ, PointXYZRGB, XYZ, XYZRGB};
//!
//! // New marker type approach
//! fn process_xyz_cloud(cloud: &PointCloud<XYZ>) {
//!     // Process specifically XYZ point clouds
//! }
//!
//! fn process_xyzrgb_cloud(cloud: &PointCloud<XYZRGB>) {
//!     // Process specifically XYZRGB point clouds
//! }
//! ```
//!
//! ## Point Creation
//!
//! ### Before:
//! ```rust,ignore
//! use pcl::PointCloudXYZBuilder;
//!
//! let cloud = PointCloudXYZBuilder::default()
//!     .add_point(1.0, 2.0, 3.0)  // Deprecated
//!     .build()?;
//! ```
//!
//! ### After:
//! ```rust
//! use pcl::{PointCloud, PointXYZ, XYZ};
//!
//! // Method 1: Direct creation
//! let mut cloud = PointCloud::<XYZ>::new()?;
//! cloud.push(PointXYZ::new(1.0, 2.0, 3.0))?;
//!
//! // Method 2: Builder pattern
//! let cloud = PointCloudXYZBuilder::default()
//!     .point(1.0, 2.0, 3.0)
//!     .build()?;
//! ```
//!
//! ## Algorithm Implementation
//! ```rust
//! use pcl::{PointXYZ, PointXYZRGB};
//!
//! // Specific implementations for each point type
//! fn compute_centroid_xyz(points: &[PointXYZ]) -> (f32, f32, f32) {
//!     // Implementation for XYZ points
//! }
//!
//! fn compute_centroid_xyzrgb(points: &[PointXYZRGB]) -> (f32, f32, f32) {
//!     // Implementation for XYZRGB points
//! }
//! ```
//!
//! # Examples
//!
//! ```rust
//! use pcl::{PointCloud, PointXYZ, XYZ};
//!
//! // Create a point cloud with the new marker type system
//! let mut cloud = PointCloud::<XYZ>::new()?;
//! cloud.push(PointXYZ::new(1.0, 2.0, 3.0))?;
//!
//! println!("Cloud has {} points", cloud.size());
//! # Ok::<(), pcl::error::PclError>(())
//! ```

pub mod common;
pub mod error;
#[cfg(feature = "filters")]
pub mod filters;
#[cfg(feature = "io")]
pub mod io;
#[cfg(feature = "octree")]
pub mod octree;
#[cfg(feature = "search")]
pub mod search;
pub mod traits;

#[cfg(feature = "features")]
pub mod features;
#[cfg(feature = "keypoints")]
pub mod keypoints;
#[cfg(feature = "registration")]
pub mod registration;
#[cfg(feature = "sample_consensus")]
pub mod sample_consensus;
#[cfg(feature = "segmentation")]
pub mod segmentation;
#[cfg(feature = "surface")]
pub mod surface;
// #[cfg(feature = "visualization")]
// pub mod visualization;  // Disabled due to VTK linkage issues - see CLAUDE.md

#[cfg(test)]
mod error_tests;

// Re-export common types for convenience
pub use common::{
    Normal, PointCloud, PointCloudNormalBuilder, PointCloudXYZBuilder, PointCloudXYZRGBBuilder,
    PointNormal, PointXYZ, PointXYZI, PointXYZRGB, XYZ, XYZI, XYZRGB,
};
pub use error::{PclError, PclResult};
// Re-export commonly used traits
#[cfg(feature = "io")]
pub use io::{
    BinaryFormat, FileFormat, PcdIoXYZ, PcdIoXYZI, PcdIoXYZRGB, PlyIoXYZ, PlyIoXYZI, PlyIoXYZRGB,
};
#[cfg(feature = "octree")]
pub use octree::{OctreeSearch, OctreeVoxelCentroid};
#[cfg(feature = "search")]
pub use search::{KdTree, SearchMethod};
// Deprecated traits have been removed - use the new marker type system instead

// Deprecated type aliases have been removed

#[cfg(feature = "filters")]
pub use filters::{
    Filter, PassThrough, PassThroughXYZ, PassThroughXYZRGB, RadiusOutlierRemovalXYZ,
    RadiusOutlierRemovalXYZRGB, StatisticalOutlierRemovalXYZ, StatisticalOutlierRemovalXYZRGB,
    VoxelGrid, VoxelGridBuilder, VoxelGridXYZ, VoxelGridXYZRGB,
};

#[cfg(feature = "features")]
pub use features::{
    FpfhEstimation, FpfhEstimationOmp, FpfhSignature, NormalCloud, NormalEstimation,
    NormalEstimationOmp, PfhEstimation, PfhSignature,
};
#[cfg(feature = "keypoints")]
pub use keypoints::{
    Harris3D, Harris3DBuilder, Iss3D, Iss3DBuilder, KeypointBuilder, KeypointDetector,
    PointCloudWithScale, PointWithScale, SiftKeypoint, SiftKeypointBuilder,
};
#[cfg(feature = "registration")]
pub use registration::{
    IcpXYZ, IcpXYZRGB, RegistrationXYZ, RegistrationXYZRGB, Transform3D, TransformationMatrix,
};
#[cfg(feature = "sample_consensus")]
pub use sample_consensus::{
    PlaneModelXYZ, PlaneModelXYZRGB, RansacPlaneXYZ, RansacPlaneXYZRGB, RansacSphereXYZ,
    RansacSphereXYZRGB, SphereModelXYZ, SphereModelXYZRGB,
};
#[cfg(feature = "segmentation")]
pub use segmentation::{
    ClusteringXYZ, ConditionalEuclideanClusteringXYZ, EuclideanClusterExtractionXYZ,
    ExtractPolygonalPrismDataXYZ, MethodType, MinCutSegmentationXYZ, ModelType,
    ProgressiveMorphologicalFilterXYZ, RegionGrowingRgbXYZRGB, RegionGrowingXYZ,
    SacSegmentationXYZ, Segmentation, SegmentationResult,
};
#[cfg(feature = "surface")]
pub use surface::{
    GreedyProjectionTriangulation, MarchingCubesHoppeXYZ, MarchingCubesRbfXYZ, MovingLeastSquares,
    OrganizedFastMeshXYZ, PointCloudSmoothing, PoissonReconstruction, PolygonMesh,
    SurfaceReconstruction, TriangulationType, UpsampleMethod,
};
// Visualization exports disabled due to VTK linkage issues - see CLAUDE.md
// #[cfg(feature = "visualization")]
// pub use visualization::{
//     AdvancedViewer,
//     // Advanced visualization features
//     AnimationController,
//     // Core viewers
//     CameraControl,
//     // Configuration
//     CameraPosition,
//     CloudViewer,
//     CloudViewerBuilder,
//     ColorMap,
//     ComparisonViewer,
//     FeatureVisualizer,
//     HistogramVisualizer,
//     InteractiveViewer,
//     MultiCloudViewer,
//     NormalVisualization,
//     PclVisualizer,
//     PclVisualizerBuilder,
//     RangeImageVisualizer,
//     RenderingProperties,
//     Representation,
//     Shading,
//     ShapeVisualization,
//     // Generic visualization traits
//     Viewer,
//     ViewerXYZ,
//     ViewerXYZRGB,
//     ViewportControl,
//     VisualizablePoint,
//     VisualizationConfig,
//     VisualizationConfigBuilder,
//     VisualizationControl,
// };

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "octree")]
    use crate::octree::{OctreeSearch, OctreeVoxelCentroid};
    #[cfg(feature = "search")]
    use crate::search::KdTree;

    // Note: Point creation tests are disabled since point creation
    // is not currently supported due to cxx limitations

    #[test]
    fn test_point_cloud_creation() {
        let cloud = PointCloud::<XYZ>::new().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
        assert_eq!(cloud.width(), 0);
        assert_eq!(cloud.height(), 0);
        assert!(!cloud.is_organized());
    }

    #[test]
    fn test_point_cloud_xyzrgb_creation() {
        let cloud = PointCloud::<XYZRGB>::new().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
        assert_eq!(cloud.width(), 0);
        assert_eq!(cloud.height(), 0);
        assert!(!cloud.is_organized());
    }

    #[test]
    fn test_point_cloud_operations() {
        let mut cloud = PointCloud::<XYZ>::new().unwrap();

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
    #[cfg(feature = "search")]
    fn test_kdtree_creation() {
        use crate::search::SearchInputCloud;

        let kdtree = KdTree::<XYZ>::new().unwrap();
        assert!(!kdtree.has_input_cloud());
        drop(kdtree);

        let kdtree_xyzi = KdTree::<XYZI>::new().unwrap();
        assert!(!kdtree_xyzi.has_input_cloud());
        drop(kdtree_xyzi);

        let kdtree_rgb = KdTree::<XYZRGB>::new().unwrap();
        assert!(!kdtree_rgb.has_input_cloud());
        drop(kdtree_rgb);
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_kdtree_configuration() {
        use crate::search::SearchConfiguration;

        let mut kdtree = KdTree::<XYZ>::new().unwrap();

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
    #[cfg(feature = "search")]
    fn test_search_input_cloud() {
        use crate::search::SearchInputCloud;

        let mut kdtree = KdTree::<XYZ>::new().unwrap();
        let cloud = PointCloud::<XYZ>::new().unwrap();

        assert!(!kdtree.has_input_cloud());
        kdtree.set_input_cloud(&cloud).unwrap();
        assert!(kdtree.has_input_cloud());
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_search_method() {
        // Test that SearchMethod enum exists and can be used
        let _method = SearchMethod::KdTree;
        // More comprehensive testing would require unified search interface
        // which is not yet implemented in minimal FFI
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_search_traits() {
        use crate::search::{NearestNeighborSearch, SearchConfiguration};

        let kdtree = KdTree::<PointXYZ>::new().unwrap();

        // Test trait implementation
        fn accept_search<T: NearestNeighborSearch<PointXYZ> + SearchConfiguration>(_search: &T) {
            // This just tests that the traits are implemented
        }

        accept_search(&kdtree);
    }

    #[test]
    #[cfg(feature = "octree")]
    fn test_octree_creation() {
        let octree = OctreeSearch::new(0.1).unwrap();
        drop(octree);

        // Test invalid resolution
        let result = OctreeSearch::new(-1.0);
        assert!(result.is_err());
    }

    #[test]
    #[cfg(feature = "octree")]
    fn test_octree_voxel_centroid_creation() {
        let octree = OctreeVoxelCentroid::new(0.1).unwrap();
        drop(octree);

        // Test invalid resolution
        let result = OctreeVoxelCentroid::new(0.0);
        assert!(result.is_err());
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_error_handling() {
        // Test parameter validation
        let _kdtree = KdTree::<PointXYZ>::new().unwrap();

        // This would require actual points to test properly, but we can test parameter validation
        // KdTree search methods would need points to work with
    }
}
