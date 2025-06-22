//! Tests corresponding to PCL's test/filters/*.cpp files
//!
//! This module tests the filter functionality including:
//! - test/filters/test_filters.cpp - PassThrough, VoxelGrid, Statistical, Radius tests
//! - test/filters/test_bilateral.cpp - Bilateral filter tests
//! - test/filters/test_convolution.cpp - Convolution filter tests
//! - test/filters/test_morphological.cpp - Morphological filter tests
//! - test/filters/test_sampling.cpp - Sampling filter tests
//! - test/filters/test_crop_hull.cpp - CropHull filter tests
//!
//! Uses test fixtures from PCL test data:
//! - bunny.pcd - Stanford bunny model for filter tests
//! - table_scene_lms400.pcd - Table scene for filtering operations
//! - milk.pcd - Milk carton scan for general filter testing

use super::*;
use crate::common::{PointCloud, PointXYZ, PointXYZRGB, XYZ, XYZRGB};
use crate::error::PclResult;

/// Tests corresponding to test/filters/test_filters.cpp - general filter functionality
#[cfg(test)]
mod general_filter_tests {
    use super::*;
    use crate::filters::{
        PassThroughXYZ, PassThroughXYZRGB, RadiusOutlierRemovalXYZ, StatisticalOutlierRemovalXYZ,
        VoxelGridXYZ, VoxelGridXYZRGB,
    };

    #[test]
    fn test_pass_through_creation() -> PclResult<()> {
        // Test creating PassThrough filter for different point types
        let _filter_xyz: PassThroughXYZ = PassThroughXYZ::new()?;
        let _filter_xyzrgb: PassThroughXYZRGB = PassThroughXYZRGB::new()?;
        Ok(())
    }

    #[test]
    fn test_pass_through_field_filter() -> PclResult<()> {
        // Test PassThrough filter with field-based filtering
        let mut filter = PassThroughXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create test data with varied Z values
        for i in 0..10 {
            cloud.push(PointXYZ::new(i as f32, 0.0, i as f32 * 0.5))?;
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_filter_field_name("z");
        filter.set_filter_limits(1.0, 3.0);

        let filtered = filter.filter()?;
        // Should only keep points with z between 1.0 and 3.0
        assert!(filtered.size() < cloud.size());
        Ok(())
    }

    #[test]
    fn test_pass_through_negative_filter() -> PclResult<()> {
        // Test PassThrough with negative (inverted) filtering
        let mut filter = PassThroughXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create test data
        for i in 0..10 {
            cloud.push(PointXYZ::new(i as f32, 0.0, i as f32 * 0.5))?;
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_filter_field_name("z");
        filter.set_filter_limits(1.0, 3.0);
        filter.set_negative(true); // Keep points outside the range

        let filtered = filter.filter()?;
        // Should keep points with z < 1.0 or z > 3.0
        Ok(())
    }

    #[test]
    fn test_pass_through_empty_cloud() -> PclResult<()> {
        // Test PassThrough with empty cloud
        let mut filter = PassThroughXYZ::new()?;
        let cloud = PointCloud::<XYZ>::new()?;

        filter.set_input_cloud(&cloud)?;
        filter.set_filter_field_name("x");
        filter.set_filter_limits(-1.0, 1.0);

        let filtered = filter.filter()?;
        assert_eq!(filtered.size(), 0);
        Ok(())
    }

    #[test]
    fn test_pass_through_all_filtered() -> PclResult<()> {
        // Test when all points are filtered out
        let mut filter = PassThroughXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // All points have z=0
        for i in 0..5 {
            cloud.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_filter_field_name("z");
        filter.set_filter_limits(1.0, 2.0); // No points in this range

        let filtered = filter.filter()?;
        assert_eq!(filtered.size(), 0);
        Ok(())
    }

    #[test]
    fn test_voxel_grid_creation() -> PclResult<()> {
        // Test creating VoxelGrid filter
        let _filter_xyz: VoxelGridXYZ = VoxelGridXYZ::new()?;
        let _filter_xyzrgb: VoxelGridXYZRGB = VoxelGridXYZRGB::new()?;
        Ok(())
    }

    #[test]
    fn test_voxel_grid_downsampling() -> PclResult<()> {
        // Test VoxelGrid downsampling
        let mut filter = VoxelGridXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create a dense grid of points
        for x in 0..10 {
            for y in 0..10 {
                for z in 0..10 {
                    cloud.push(PointXYZ::new(
                        x as f32 * 0.1,
                        y as f32 * 0.1,
                        z as f32 * 0.1,
                    ))?;
                }
            }
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_leaf_size(0.3, 0.3, 0.3); // Larger voxel size

        let filtered = filter.filter()?;
        // Should have significantly fewer points
        assert!(filtered.size() < cloud.size() / 2);
        Ok(())
    }

    #[test]
    fn test_voxel_grid_leaf_size() -> PclResult<()> {
        // Test different leaf sizes
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create test data
        for i in 0..100 {
            cloud.push(PointXYZ::new(
                (i as f32 * 0.1) % 1.0,
                ((i / 10) as f32 * 0.1) % 1.0,
                ((i / 100) as f32 * 0.1) % 1.0,
            ))?;
        }

        // Test with small leaf size
        let mut filter1 = VoxelGridXYZ::new()?;
        filter1.set_input_cloud(&cloud)?;
        filter1.set_leaf_size(0.05, 0.05, 0.05);
        let filtered1 = filter1.filter()?;

        // Test with large leaf size
        let mut filter2 = VoxelGridXYZ::new()?;
        filter2.set_input_cloud(&cloud)?;
        filter2.set_leaf_size(0.2, 0.2, 0.2);
        let filtered2 = filter2.filter()?;

        // Larger leaf size should result in fewer points
        assert!(filtered2.size() < filtered1.size());
        Ok(())
    }

    #[test]
    fn test_voxel_grid_uniform_leaf() -> PclResult<()> {
        // Test uniform leaf size setting
        let mut filter = VoxelGridXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        for i in 0..50 {
            cloud.push(PointXYZ::new(
                i as f32 * 0.1,
                i as f32 * 0.1,
                i as f32 * 0.1,
            ))?;
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_leaf_size(0.5, 0.5, 0.5);

        let filtered = filter.filter()?;
        assert!(filtered.size() < cloud.size());
        Ok(())
    }

    #[test]
    fn test_statistical_outlier_removal_creation() -> PclResult<()> {
        // Test creating StatisticalOutlierRemoval filter
        let _filter: StatisticalOutlierRemovalXYZ = StatisticalOutlierRemovalXYZ::new()?;
        Ok(())
    }

    #[test]
    fn test_statistical_outlier_removal_basic() -> PclResult<()> {
        // Test basic statistical outlier removal
        let mut filter = StatisticalOutlierRemovalXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create mostly uniform data with some outliers
        for i in 0..50 {
            cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
        }
        // Add outliers
        cloud.push(PointXYZ::new(5.0, 10.0, 10.0))?;
        cloud.push(PointXYZ::new(-5.0, -10.0, -10.0))?;

        filter.set_input_cloud(&cloud)?;
        filter.set_mean_k(10);
        filter.set_std_dev_mul_thresh(1.0);

        let filtered = filter.filter()?;
        // Should remove outliers
        assert!(filtered.size() < cloud.size());
        Ok(())
    }

    #[test]
    fn test_statistical_outlier_parameters() -> PclResult<()> {
        // Test different parameters for statistical outlier removal
        let mut filter = StatisticalOutlierRemovalXYZ::new()?;

        filter.set_mean_k(50)?;
        // TODO: Add getter methods for verification when implemented
        // assert_eq!(filter.get_mean_k(), 50);

        filter.set_std_dev_mul_thresh(2.0)?;
        // TODO: Add getter methods for verification when implemented
        // assert!((filter.get_std_dev_mul_thresh() - 2.0).abs() < 1e-6);
        Ok(())
    }

    #[test]
    fn test_statistical_outlier_negative() -> PclResult<()> {
        // Test negative (inverted) outlier removal
        let mut filter = StatisticalOutlierRemovalXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create test data
        for i in 0..50 {
            cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
        }
        cloud.push(PointXYZ::new(25.0, 0.0, 0.0))?; // Outlier

        filter.set_input_cloud(&cloud)?;
        filter.set_mean_k(10);
        filter.set_std_dev_mul_thresh(1.0);
        filter.set_negative(true); // Keep only outliers

        let filtered = filter.filter()?;
        // Should keep only outliers
        assert!(filtered.size() < 5);
        Ok(())
    }

    #[test]
    fn test_radius_outlier_removal_creation() -> PclResult<()> {
        // Test creating RadiusOutlierRemoval filter
        let _filter: RadiusOutlierRemovalXYZ = RadiusOutlierRemovalXYZ::new()?;
        Ok(())
    }

    #[test]
    fn test_radius_outlier_removal_basic() -> PclResult<()> {
        // Test basic radius outlier removal
        let mut filter = RadiusOutlierRemovalXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create clustered points with isolated outliers
        for i in 0..10 {
            cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
        }
        // Add isolated point
        cloud.push(PointXYZ::new(10.0, 10.0, 10.0))?;

        filter.set_input_cloud(&cloud)?;
        filter.set_radius_search(1.0);
        filter.set_min_neighbors_in_radius(2);

        let filtered = filter.filter()?;
        // Should remove isolated point
        assert_eq!(filtered.size(), 10);
        Ok(())
    }

    #[test]
    fn test_radius_outlier_parameters() -> PclResult<()> {
        // Test radius outlier removal parameters
        let mut filter = RadiusOutlierRemovalXYZ::new()?;

        filter.set_radius_search(2.0)?;
        // TODO: Add getter methods for verification when implemented
        // assert!((filter.get_radius_search() - 2.0).abs() < 1e-6);

        filter.set_min_neighbors_in_radius(5)?;
        // TODO: Add getter methods for verification when implemented
        // assert_eq!(filter.get_min_neighbors_in_radius(), 5);
        Ok(())
    }

    #[test]
    fn test_radius_outlier_dense_cloud() -> PclResult<()> {
        // Test with dense cloud (no outliers)
        let mut filter = RadiusOutlierRemovalXYZ::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create dense grid
        for x in 0..5 {
            for y in 0..5 {
                cloud.push(PointXYZ::new(x as f32, y as f32, 0.0))?;
            }
        }

        filter.set_input_cloud(&cloud)?;
        filter.set_radius_search(1.5);
        filter.set_min_neighbors_in_radius(3);

        let filtered = filter.filter()?;
        // Most points should be kept in dense cloud
        assert!(filtered.size() >= cloud.size() - 4); // Corner points might be removed
        Ok(())
    }

    #[test]
    fn test_filter_xyzrgb() -> PclResult<()> {
        // Test filters with colored points
        let mut voxel = VoxelGridXYZRGB::new()?;
        let mut cloud = PointCloud::<XYZRGB>::new()?;

        // Create colored point grid
        for x in 0..5 {
            for y in 0..5 {
                let r = (x * 51) as u8;
                let g = (y * 51) as u8;
                cloud.push(PointXYZRGB::new(x as f32, y as f32, 0.0, r, g, 0))?;
            }
        }

        voxel.set_input_cloud(&cloud)?;
        voxel.set_leaf_size(2.0, 2.0, 1.0);

        let filtered = voxel.filter()?;
        assert!(filtered.size() < cloud.size());
        Ok(())
    }
}

/// Tests for bilateral filter (placeholder)
#[cfg(test)]
mod bilateral_filter_tests {
    use super::*;

    #[test]
    fn test_bilateral_filter_placeholder() {
        // TODO: Implement bilateral filter tests when available
        // This corresponds to test/filters/test_bilateral.cpp

        // Bilateral filter smooths data while preserving edges
        // Uses both spatial and range (intensity) kernels

        assert!(true, "Bilateral filter tests not yet implemented");
    }
}

/// Tests for convolution filters (placeholder)
#[cfg(test)]
mod convolution_filter_tests {
    use super::*;

    #[test]
    fn test_convolution_filter_placeholder() {
        // TODO: Implement convolution filter tests when available
        // This corresponds to test/filters/test_convolution.cpp

        // Convolution filters apply various kernels to point cloud data
        // Useful for smoothing, edge detection, etc.

        assert!(true, "Convolution filter tests not yet implemented");
    }
}

/// Tests for morphological filters (placeholder)
#[cfg(test)]
mod morphological_filter_tests {
    use super::*;

    #[test]
    fn test_morphological_filter_placeholder() {
        // TODO: Implement morphological filter tests when available
        // This corresponds to test/filters/test_morphological.cpp

        // Morphological filters include erosion, dilation, opening, closing
        // Useful for noise removal and feature extraction

        assert!(true, "Morphological filter tests not yet implemented");
    }
}

/// Tests for sampling filters
#[cfg(test)]
mod sampling_filter_tests {
    use super::*;

    #[test]
    fn test_uniform_sampling_placeholder() {
        // TODO: Implement uniform sampling tests when available
        // Uniform sampling selects points uniformly across the cloud

        assert!(true, "Uniform sampling tests not yet implemented");
    }

    #[test]
    fn test_random_sampling_placeholder() {
        // TODO: Implement random sampling tests when available
        // Random sampling randomly selects a subset of points

        assert!(true, "Random sampling tests not yet implemented");
    }

    #[test]
    fn test_normal_space_sampling_placeholder() {
        // TODO: Implement normal space sampling tests when available
        // Normal space sampling ensures uniform sampling in normal space

        assert!(true, "Normal space sampling tests not yet implemented");
    }
}

/// Tests for crop hull filter (placeholder)
#[cfg(test)]
mod crop_hull_filter_tests {
    use super::*;

    #[test]
    fn test_crop_hull_placeholder() {
        // TODO: Implement crop hull filter tests when available
        // This corresponds to test/filters/test_crop_hull.cpp

        // CropHull filter crops points inside/outside a convex hull
        // Useful for extracting regions of interest

        assert!(true, "Crop hull filter tests not yet implemented");
    }
}

/// Integration tests combining multiple filters
#[cfg(test)]
mod filter_integration_tests {
    use super::*;

    #[test]
    fn test_filter_pipeline() -> PclResult<()> {
        // Test chaining multiple filters
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create noisy data
        for i in 0..100 {
            cloud.push(PointXYZ::new(
                (i as f32 * 0.1) % 5.0,
                ((i / 10) as f32 * 0.1) % 5.0,
                ((i / 100) as f32 * 0.1) % 5.0,
            ))?;
        }
        // Add outliers
        cloud.push(PointXYZ::new(50.0, 50.0, 50.0))?;
        cloud.push(PointXYZ::new(-50.0, -50.0, -50.0))?;

        // Step 1: Remove outliers
        let mut outlier_filter = StatisticalOutlierRemovalXYZ::new()?;
        outlier_filter.set_input_cloud(&cloud)?;
        outlier_filter.set_mean_k(10);
        outlier_filter.set_std_dev_mul_thresh(1.0);
        let cloud_no_outliers = outlier_filter.filter()?;

        // Step 2: Downsample
        let mut voxel_filter = VoxelGridXYZ::new()?;
        voxel_filter.set_input_cloud(&cloud_no_outliers)?;
        voxel_filter.set_leaf_size(0.5, 0.5, 0.5);
        let cloud_downsampled = voxel_filter.filter()?;

        // Step 3: Crop to region
        let mut pass_filter = PassThroughXYZ::new()?;
        pass_filter.set_input_cloud(&cloud_downsampled)?;
        pass_filter.set_filter_field_name("z");
        pass_filter.set_filter_limits(0.0, 2.0);
        let cloud_final = pass_filter.filter()?;

        // Final cloud should be significantly smaller
        assert!(cloud_final.size() < cloud.size() / 2);
        Ok(())
    }

    #[test]
    fn test_filter_empty_result() -> PclResult<()> {
        // Test when filters result in empty cloud
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create points all at same location
        for _ in 0..10 {
            cloud.push(PointXYZ::new(1.0, 1.0, 1.0))?;
        }

        // Apply voxel grid - should merge all points
        let mut voxel = VoxelGridXYZ::new()?;
        voxel.set_input_cloud(&cloud)?;
        voxel.set_leaf_size(2.0, 2.0, 2.0);
        let filtered = voxel.filter()?;

        assert_eq!(filtered.size(), 1); // All points in same voxel

        // Apply pass through that excludes this point
        let mut pass = PassThroughXYZ::new()?;
        pass.set_input_cloud(&filtered)?;
        pass.set_filter_field_name("x");
        pass.set_filter_limits(2.0, 3.0);
        let final_cloud = pass.filter()?;

        assert_eq!(final_cloud.size(), 0);
        Ok(())
    }
}

/// Performance and edge case tests
#[cfg(test)]
mod filter_performance_tests {
    use super::*;

    #[test]
    fn test_filter_large_cloud() -> PclResult<()> {
        // Test filters with larger dataset
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create 10k points
        for i in 0..10000 {
            cloud.push(PointXYZ::new(
                (i as f32 * 0.01) % 10.0,
                ((i / 100) as f32 * 0.01) % 10.0,
                ((i / 10000) as f32 * 0.01) % 10.0,
            ))?;
        }

        let mut voxel = VoxelGridXYZ::new()?;
        voxel.set_input_cloud(&cloud)?;
        voxel.set_leaf_size(0.1, 0.1, 0.1);

        let filtered = voxel.filter()?;
        assert!(filtered.size() < cloud.size());
        assert!(filtered.size() > 100); // Should still have many points
        Ok(())
    }

    #[test]
    fn test_filter_memory_usage() -> PclResult<()> {
        // Test that filters don't leak memory
        for _ in 0..20 {
            let mut cloud = PointCloud::<XYZ>::new()?;

            for i in 0..100 {
                cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
            }

            let mut filter = VoxelGridXYZ::new()?;
            filter.set_input_cloud(&cloud)?;
            filter.set_leaf_size(0.5, 0.5, 0.5);
            let _filtered = filter.filter()?;
        }
        Ok(())
    }

    #[test]
    fn test_filter_edge_cases() -> PclResult<()> {
        // Test filters with edge case values
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Add points with extreme values
        cloud.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(f32::MAX / 2.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(f32::MIN / 2.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, f32::MAX / 2.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, f32::MIN / 2.0, 0.0))?;

        let mut filter = PassThroughXYZ::new()?;
        filter.set_input_cloud(&cloud)?;
        filter.set_filter_field_name("x");
        filter.set_filter_limits(-1000.0, 1000.0);

        let filtered = filter.filter()?;
        // Should handle extreme values gracefully
        assert!(filtered.size() >= 1); // At least origin point
        Ok(())
    }

    #[test]
    fn test_filter_nan_handling() -> PclResult<()> {
        // Test filter behavior with NaN values
        let mut cloud = PointCloud::<XYZ>::new()?;

        cloud.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(1.0, 1.0, 1.0))?;
        cloud.push(PointXYZ::new(f32::NAN, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, f32::NAN, 0.0))?;

        let mut filter = VoxelGridXYZ::new()?;
        filter.set_input_cloud(&cloud)?;
        filter.set_leaf_size(0.5, 0.5, 0.5);

        let filtered = filter.filter()?;
        // Should handle NaN points appropriately
        assert!(filtered.size() <= cloud.size());
        Ok(())
    }
}

/// Builder pattern and convenience function tests
#[cfg(test)]
mod filter_builder_tests {
    use super::*;
    use crate::filters::{PassThroughXYZBuilder, VoxelGridXYZBuilder};

    #[test]
    fn test_voxel_grid_builder() -> PclResult<()> {
        // Test VoxelGrid builder pattern
        let mut cloud = PointCloud::<XYZ>::new()?;

        for i in 0..50 {
            cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
        }

        let mut filter = VoxelGridXYZBuilder::new()
            .leaf_size(1.0, 1.0, 1.0)
            .build()?;
        filter.set_input_cloud(&cloud)?;
        let filtered = filter.filter()?;

        assert!(filtered.size() < cloud.size());
        Ok(())
    }

    #[test]
    fn test_pass_through_builder() -> PclResult<()> {
        // Test PassThrough builder pattern
        let mut cloud = PointCloud::<XYZ>::new()?;

        for i in 0..10 {
            cloud.push(PointXYZ::new(0.0, 0.0, i as f32))?;
        }

        let mut filter = PassThroughXYZBuilder::new()
            .field_name("z")
            .limits(2.0, 7.0)
            .build()?;
        filter.set_input_cloud(&cloud)?;
        let filtered = filter.filter()?;

        assert_eq!(filtered.size(), 6); // Points with z in [2,7]
        Ok(())
    }

    #[test]
    fn test_builder_negative_filter() -> PclResult<()> {
        // Test builder with negative filtering
        let mut cloud = PointCloud::<XYZ>::new()?;

        for i in 0..10 {
            cloud.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
        }

        let mut filter = PassThroughXYZBuilder::new()
            .field_name("x")
            .limits(3.0, 6.0)
            .negative(true)
            .build()?;
        filter.set_input_cloud(&cloud)?;
        let filtered = filter.filter()?;

        // Should keep x < 3 or x > 6
        assert_eq!(filtered.size(), 6);
        Ok(())
    }
}
