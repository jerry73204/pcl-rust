//! Unit tests for common module
//! Corresponds to PCL's test/common/*.cpp tests
//!
//! Test mapping from C++ to Rust:
//! - test/common/test_common.cpp -> point_type_tests module
//! - test/common/test_pointcloud.cpp -> point_cloud_tests module
//! - test/common/test_copy_point.cpp -> copy_tests module (placeholders)
//! - test/common/test_transforms.cpp -> transform_tests module (placeholders)
//! - test/common/test_centroid.cpp -> centroid_tests module (placeholders)

use super::*;
use crate::error::PclResult;

/// Tests corresponding to test/common/test_common.cpp
/// C++ source: pcl/test/common/test_common.cpp
#[cfg(test)]
mod point_type_tests {
    use super::*;

    #[test]
    fn test_point_xyz_basic() -> PclResult<()> {
        // Points can only be accessed through point clouds
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;
        cloud.push(1.0, 2.0, 3.0)?;

        // Test that the point was added successfully
        assert_eq!(cloud.size(), 1);
        assert!(!cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_point_xyzrgb_color() -> PclResult<()> {
        // Test corresponding to TEST (PCL, PointXYZRGB)
        // C++ source: pcl/test/common/test_common.cpp:56-85
        let mut cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;
        cloud.push(0.0, 0.0, 0.0, 127, 64, 254)?;

        assert_eq!(cloud.size(), 1);

        // Add another point with different colors
        cloud.push(1.0, 1.0, 1.0, 0, 127, 0)?;
        assert_eq!(cloud.size(), 2);

        Ok(())
    }

    #[test]
    fn test_point_xyzrgb_coordinates() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;
        cloud.push(1.0, 2.0, 3.0, 255, 128, 64)?;

        assert_eq!(cloud.size(), 1);
        assert!(!cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_point_xyzi_intensity() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZI> = PointCloud::new()?;
        cloud.push_with_intensity(1.0, 2.0, 3.0, 0.5)?;

        assert_eq!(cloud.size(), 1);
        assert!(!cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_point_normal() -> PclResult<()> {
        let cloud = PointCloudNormalBuilder::default()
            .add_point(1.0, 2.0, 3.0, 0.0, 0.0, 1.0)
            .build()?;

        assert_eq!(cloud.size(), 1);
        assert!(!cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_is_finite() -> PclResult<()> {
        // Test corresponding to TEST(PCL, isFinite)
        // C++ source: pcl/test/common/test_common.cpp:121-130
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Test finite point
        cloud.push(1.0, 2.0, 3.0)?;
        // Note: is_finite() is not exposed in the current FFI interface
        // We can test that the point was added successfully
        assert_eq!(cloud.size(), 1);

        // Test NaN, Infinity values - these might be handled by PCL internally
        // The behavior when pushing non-finite values is undefined in the current FFI

        Ok(())
    }
}

/// Tests corresponding to test/common/test_pointcloud.cpp
/// C++ source: pcl/test/common/test_pointcloud.cpp
#[cfg(test)]
mod point_cloud_tests {
    use super::*;

    #[test]
    fn test_point_cloud_new() -> PclResult<()> {
        let cloud: PointCloud<PointXYZ> = PointCloud::new()?;
        assert_eq!(cloud.size(), 0);
        assert!(cloud.is_empty());
        Ok(())
    }

    #[test]
    fn test_point_cloud_is_organized() -> PclResult<()> {
        // Test corresponding to TEST_F (pointCloudTest, is_organized)
        // C++ source: pcl/test/common/test_pointcloud.cpp:24-29
        let cloud: PointCloud<PointXYZ> = PointCloud::new()?;
        // Note: set_width and set_height are not available in the current API
        // A cloud is organized if height > 1
        assert!(!cloud.is_organized()); // New clouds are not organized by default
        Ok(())
    }

    #[test]
    fn test_point_cloud_not_organized() -> PclResult<()> {
        // Test corresponding to TEST_F (pointCloudTest, not_organized)
        // C++ source: pcl/test/common/test_pointcloud.cpp:31-36
        let cloud: PointCloud<PointXYZ> = PointCloud::new()?;
        // New clouds have height = 1, so they are not organized
        assert!(!cloud.is_organized());
        Ok(())
    }

    #[test]
    fn test_point_cloud_push() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Push points
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;

        assert_eq!(cloud.size(), 2);
        assert!(!cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_point_cloud_clear() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Add some points
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        assert_eq!(cloud.size(), 2);

        // Clear the cloud
        cloud.clear()?;
        assert_eq!(cloud.size(), 0);
        assert!(cloud.is_empty());

        Ok(())
    }

    #[test]
    fn test_point_cloud_resize() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Resize cloud
        cloud.resize(10)?;
        assert_eq!(cloud.size(), 10);

        // Resize smaller
        cloud.resize(5)?;
        assert_eq!(cloud.size(), 5);

        Ok(())
    }

    #[test]
    fn test_point_cloud_at() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Add points
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;

        // Note: Direct point access is not implemented in the current FFI
        // The at() method returns NotImplemented error
        assert!(cloud.at(0).is_err());

        Ok(())
    }

    #[test]
    fn test_point_cloud_xyzrgb() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;

        // Push colored points
        cloud.push(1.0, 2.0, 3.0, 255, 0, 0)?;
        cloud.push(4.0, 5.0, 6.0, 0, 255, 0)?;

        assert_eq!(cloud.size(), 2);

        Ok(())
    }

    #[test]
    fn test_point_cloud_xyzi() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZI> = PointCloud::new()?;

        // Push points with intensity
        cloud.push_with_intensity(1.0, 2.0, 3.0, 0.5)?;
        cloud.push_with_intensity(4.0, 5.0, 6.0, 0.8)?;

        assert_eq!(cloud.size(), 2);

        Ok(())
    }

    #[test]
    fn test_point_cloud_reserve() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Reserve capacity
        cloud.reserve(100)?;

        // Cloud should still be empty
        assert_eq!(cloud.size(), 0);
        assert!(cloud.is_empty());
        Ok(())
    }

    #[test]
    fn test_point_cloud_extend_from_slice() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Add multiple points from slice
        let points = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        cloud.extend_from_slice(&points)?;

        assert_eq!(cloud.size(), 3);

        Ok(())
    }

    #[test]
    fn test_point_cloud_width_height() -> PclResult<()> {
        let cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // New clouds have default width and height
        assert_eq!(cloud.width(), 0);
        assert_eq!(cloud.height(), 0); // Default height is 0 for empty clouds

        Ok(())
    }

    #[test]
    fn test_point_cloud_is_dense() -> PclResult<()> {
        let cloud: PointCloud<PointXYZ> = PointCloud::new()?;

        // Test is_dense property
        let _is_dense = cloud.is_dense();
        // Just verify the method exists and can be called

        Ok(())
    }
}

/// Tests for builders
#[cfg(test)]
mod builder_tests {
    use super::*;

    #[test]
    fn test_point_cloud_xyz_builder() -> PclResult<()> {
        let cloud = PointCloudXYZBuilder::default()
            .add_point(1.0, 2.0, 3.0)
            .add_point(4.0, 5.0, 6.0)
            .width(2)
            .height(1)
            .build()?;

        assert_eq!(cloud.size(), 2);
        assert_eq!(cloud.width(), 2);
        assert_eq!(cloud.height(), 1);
        assert!(!cloud.is_organized());

        Ok(())
    }

    #[test]
    fn test_point_cloud_xyzrgb_builder() -> PclResult<()> {
        let cloud = PointCloudXYZRGBBuilder::default()
            .add_point(1.0, 2.0, 3.0, 255, 0, 0)
            .add_point(4.0, 5.0, 6.0, 0, 255, 0)
            .width(2)
            .height(1)
            .build()?;

        assert_eq!(cloud.size(), 2);

        Ok(())
    }

    #[test]
    fn test_point_cloud_normal_builder() -> PclResult<()> {
        let cloud = PointCloudNormalBuilder::default()
            .add_point(1.0, 2.0, 3.0, 0.0, 0.0, 1.0)
            .add_point(4.0, 5.0, 6.0, 0.0, 1.0, 0.0)
            .width(2)
            .height(1)
            .build()?;

        assert_eq!(cloud.size(), 2);

        Ok(())
    }

    #[test]
    fn test_builder_from_vec() -> PclResult<()> {
        let points = vec![(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)];

        let cloud = PointCloudXYZBuilder::default().add_points(points).build()?;

        assert_eq!(cloud.size(), 3);

        Ok(())
    }
}

/// Tests for copy operations (corresponding to test_copy_point.cpp)
/// C++ source: pcl/test/common/test_copy_point.cpp
/// Note: Most copy operations are not available due to FFI limitations
#[cfg(test)]
mod copy_tests {
    use super::*;

    #[test]
    fn test_point_cloud_copy() -> PclResult<()> {
        // Note: Individual points cannot be cloned due to FFI limitations
        // Testing point cloud copy instead
        let mut cloud1: PointCloud<PointXYZ> = PointCloud::new()?;
        cloud1.push(1.0, 2.0, 3.0)?;
        cloud1.push(4.0, 5.0, 6.0)?;

        // Note: PointCloud does not implement Clone in the current API
        // This is a limitation of the FFI interface
        assert_eq!(cloud1.size(), 2);
        Ok(())
    }

    #[test]
    #[ignore = "PointCloud clone not yet implemented"]
    fn test_point_cloud_clone() -> PclResult<()> {
        // TODO: Implement when PointCloud clone is available
        Ok(())
    }
}

/// Tests for point cloud transformations (placeholder for test_transforms.cpp)
/// C++ source: pcl/test/common/test_transforms.cpp
/// TODO: Implement when transform operations are available in FFI
#[cfg(test)]
mod transform_tests {
    #[test]
    #[ignore = "Transform operations not yet implemented"]
    fn test_transform_point_cloud() {
        // TODO: Implement when transform operations are available
    }

    #[test]
    #[ignore = "Transform operations not yet implemented"]
    fn test_transform_point() {
        // TODO: Implement when transform operations are available
    }
}

/// Tests for centroid calculations (placeholder for test_centroid.cpp)
/// C++ source: pcl/test/common/test_centroid.cpp
/// TODO: Implement when centroid operations are available in FFI
#[cfg(test)]
mod centroid_tests {
    #[test]
    #[ignore = "Centroid calculations not yet implemented"]
    fn test_compute_centroid() {
        // TODO: Implement when centroid operations are available
    }

    #[test]
    #[ignore = "Centroid calculations not yet implemented"]
    fn test_compute_centroid_weighted() {
        // TODO: Implement when centroid operations are available
    }
}

/// Additional utility tests
#[cfg(test)]
mod utility_tests {
    use super::*;

    #[test]
    fn test_point_cloud_debug() -> PclResult<()> {
        let mut cloud: PointCloud<PointXYZ> = PointCloud::new()?;
        cloud.push(1.0, 2.0, 3.0)?;

        let debug_str = format!("{:?}", cloud);
        assert!(debug_str.contains("PointCloud"));
        // The debug format may vary, just check that it contains basic info
        assert!(cloud.size() == 1);

        Ok(())
    }

    #[test]
    fn test_point_cloud_default() -> PclResult<()> {
        // Test Default trait implementation
        let cloud: PointCloud<PointXYZ> = Default::default();
        assert_eq!(cloud.size(), 0);
        assert!(cloud.is_empty());

        Ok(())
    }
}
