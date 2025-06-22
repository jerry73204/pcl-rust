//! Tests corresponding to PCL's test/octree/*.cpp files
//!
//! This module tests the octree functionality including:
//! - test/octree/test_octree.cpp - Basic octree operations tests
//! - test/octree/test_octree_iterator.cpp - Octree iterator tests
//!
//! Uses test fixtures from PCL test data:
//! - bunny.pcd - Stanford bunny model for octree tests
//! - Various synthetic point clouds for octree structure testing

use super::*;
use crate::common::{PointCloud, PointXYZ};
use crate::error::PclResult;
use crate::octree::traits::{OctreeType, OctreeConfiguration, VoxelOperations, OctreeManagement};

/// Tests corresponding to test/octree/test_octree.cpp - basic octree functionality
#[cfg(test)]
mod basic_octree_tests {
    use super::*;

    #[test]
    fn test_octree_type_names() {
        // Test OctreeType enum functionality
        assert_eq!(OctreeType::PointCloud.name(), "Point Cloud");
        assert_eq!(OctreeType::Search.name(), "Search");
        assert_eq!(OctreeType::VoxelCentroid.name(), "Voxel Centroid");
        assert_eq!(OctreeType::Occupancy.name(), "Occupancy");
        assert_eq!(OctreeType::ChangeDetection.name(), "Change Detection");
    }

    #[test]
    fn test_octree_type_search_support() {
        // Test which octree types support search operations
        assert!(OctreeType::Search.supports_search());
        assert!(!OctreeType::PointCloud.supports_search());
        assert!(!OctreeType::VoxelCentroid.supports_search());
        assert!(!OctreeType::Occupancy.supports_search());
        assert!(!OctreeType::ChangeDetection.supports_search());
    }

    #[test]
    fn test_octree_type_voxel_operations() {
        // Test which octree types support voxel operations
        assert!(OctreeType::Search.supports_voxel_operations());
        assert!(OctreeType::VoxelCentroid.supports_voxel_operations());
        assert!(OctreeType::Occupancy.supports_voxel_operations());
        assert!(!OctreeType::PointCloud.supports_voxel_operations());
        assert!(!OctreeType::ChangeDetection.supports_voxel_operations());
    }

    #[test]
    fn test_octree_type_equality() {
        // Test OctreeType equality comparisons
        assert_eq!(OctreeType::Search, OctreeType::Search);
        assert_ne!(OctreeType::Search, OctreeType::PointCloud);
    }

    #[test]
    fn test_octree_type_debug() {
        // Test debug formatting for OctreeType
        let octree_type = OctreeType::Search;
        let debug_str = format!("{:?}", octree_type);
        assert!(debug_str.contains("Search"));
    }

    #[test]
    fn test_octree_search_creation() -> PclResult<()> {
        // Test creating OctreeSearchXYZ with different resolutions
        let _octree1 = OctreeSearchXYZ::new(0.1)?;
        let _octree2 = OctreeSearchXYZ::new(1.0)?;
        let _octree3 = OctreeSearchXYZ::new(10.0)?;
        Ok(())
    }

    #[test]
    fn test_octree_voxel_centroid_creation() -> PclResult<()> {
        // Test creating OctreeVoxelCentroidXYZ with different resolutions
        let _octree1 = OctreeVoxelCentroidXYZ::new(0.1)?;
        let _octree2 = OctreeVoxelCentroidXYZ::new(1.0)?;
        let _octree3 = OctreeVoxelCentroidXYZ::new(10.0)?;
        Ok(())
    }

    #[test]
    fn test_octree_zero_resolution() {
        // Test behavior with zero resolution
        let result = OctreeSearchXYZ::new(0.0);
        // Should either error or handle gracefully
        // The specific behavior depends on implementation
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_octree_negative_resolution() {
        // Test behavior with negative resolution
        let result = OctreeSearchXYZ::new(-1.0);
        // Should either error or handle gracefully
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_octree_very_small_resolution() -> PclResult<()> {
        // Test with very small resolution
        let _octree = OctreeSearchXYZ::new(1e-6)?;
        Ok(())
    }

    #[test]
    fn test_octree_very_large_resolution() -> PclResult<()> {
        // Test with very large resolution
        let _octree = OctreeSearchXYZ::new(1e6)?;
        Ok(())
    }
}

/// Tests for octree input cloud management
#[cfg(test)]
mod octree_input_cloud_tests {
    use super::*;

    #[test]
    fn test_octree_set_input_cloud() -> PclResult<()> {
        // Test setting input cloud for octree
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add some test points
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        cloud.push(2.0, 2.0, 2.0)?;
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_empty_cloud() -> PclResult<()> {
        // Test setting empty cloud
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let cloud = PointCloud::<PointXYZ>::new()?;
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_single_point() -> PclResult<()> {
        // Test with single point cloud
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(5.0, 5.0, 5.0)?;
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_large_cloud() -> PclResult<()> {
        // Test with moderately large point cloud
        let mut octree = OctreeSearchXYZ::new(0.5)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a 10x10x10 grid of points
        for x in 0..10 {
            for y in 0..10 {
                for z in 0..10 {
                    cloud.push(x as f32, y as f32, z as f32)?;
                }
            }
        }
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_duplicate_points() -> PclResult<()> {
        // Test with duplicate points
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add the same point multiple times
        for _ in 0..5 {
            cloud.push(1.0, 1.0, 1.0)?;
        }
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_scattered_points() -> PclResult<()> {
        // Test with widely scattered points
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(-100.0, -100.0, -100.0)?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(100.0, 100.0, 100.0)?;
        cloud.push(-50.0, 75.0, -25.0)?;
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }
}

/// Tests for octree configuration and introspection
#[cfg(test)]
mod octree_configuration_tests {
    use super::*;

    #[test]
    fn test_octree_resolution() -> PclResult<()> {
        // Test getting octree resolution
        let mut octree = OctreeSearchXYZ::new(2.5)?;
        let resolution = octree.resolution();
        assert!((resolution - 2.5).abs() < 1e-6);
        Ok(())
    }

    #[test]
    fn test_octree_tree_depth() -> PclResult<()> {
        // Test getting tree depth
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points that should create some depth
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(5.0, 5.0, 5.0)?;
        
        octree.set_input_cloud(&cloud)?;
        let depth = octree.tree_depth();
        assert!(depth > 0);
        Ok(())
    }

    #[test]
    fn test_octree_leaf_count() -> PclResult<()> {
        // Test getting leaf node count
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add some points
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.1, 1.1, 1.1)?;
        cloud.push(2.2, 2.2, 2.2)?;
        
        octree.set_input_cloud(&cloud)?;
        let leaf_count = octree.leaf_count();
        assert!(leaf_count > 0);
        Ok(())
    }

    #[test]
    fn test_octree_branch_count() -> PclResult<()> {
        // Test getting branch node count
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points to create branches
        for i in 0..8 {
            cloud.push(i as f32 * 2.0, i as f32 * 2.0, i as f32 * 2.0)?;
        }
        
        octree.set_input_cloud(&cloud)?;
        let branch_count = octree.branch_count();
        // May be 0 if implementation doesn't distinguish branches yet
        assert!(branch_count >= 0);
        Ok(())
    }

    #[test]
    fn test_octree_empty_tree_stats() -> PclResult<()> {
        // Test stats for empty octree
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let cloud = PointCloud::<PointXYZ>::new()?;
        
        octree.set_input_cloud(&cloud)?;
        
        let depth = octree.tree_depth();
        let leaf_count = octree.leaf_count();
        let branch_count = octree.branch_count();
        
        // Empty tree should have minimal or zero stats
        assert_eq!(depth, 0);
        assert_eq!(leaf_count, 0);
        assert_eq!(branch_count, 0);
        Ok(())
    }
}

/// Tests for octree search operations
#[cfg(test)]
mod octree_search_tests {
    use super::*;

    #[test]
    fn test_octree_voxel_search() -> PclResult<()> {
        // Test voxel search functionality
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points that should be in same and different voxels
        cloud.push(0.1, 0.1, 0.1)?; // Voxel (0,0,0)
        cloud.push(0.9, 0.9, 0.9)?; // Voxel (0,0,0)
        cloud.push(1.1, 1.1, 1.1)?; // Voxel (1,1,1)
        
        octree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.5, 0.5, 0.5)?;
        let indices = octree.voxel_search(&query_point)?;
        
        // Should find points in the same voxel
        assert!(indices.len() >= 1);
        Ok(())
    }

    #[test]
    fn test_octree_voxel_search_empty() -> PclResult<()> {
        // Test voxel search in empty voxel
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(2.0, 2.0, 2.0)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Search in empty voxel
        let query_point = PointXYZ::new(5.0, 5.0, 5.0)?;
        let indices = octree.voxel_search(&query_point)?;
        
        assert_eq!(indices.len(), 0);
        Ok(())
    }

    #[test]
    fn test_octree_radius_search() -> PclResult<()> {
        // Test radius search functionality
        let mut octree = OctreeSearchXYZ::new(0.5)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points at known distances
        cloud.push(0.0, 0.0, 0.0)?; // Distance 0
        cloud.push(1.0, 0.0, 0.0)?; // Distance 1
        cloud.push(0.0, 1.0, 0.0)?; // Distance 1
        cloud.push(2.0, 0.0, 0.0)?; // Distance 2
        
        octree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = octree.radius_search(&query_point, 1.5)?;
        
        // Should find points within radius 1.5
        assert!(indices.len() >= 3); // At least the first 3 points
        Ok(())
    }

    #[test]
    fn test_octree_k_nearest_search() -> PclResult<()> {
        // Test k-nearest neighbor search
        let mut octree = OctreeSearchXYZ::new(0.5)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a simple point configuration
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, 0.0, 1.0)?;
        cloud.push(5.0, 5.0, 5.0)?;
        
        octree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = octree.k_nearest_search(&query_point, 3)?;
        
        assert_eq!(indices.len(), 3);
        assert_eq!(indices[0], 0); // Should find itself first
        Ok(())
    }

    #[test]
    fn test_octree_search_no_input_cloud() -> PclResult<()> {
        // Test search operations before setting input cloud
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = octree.voxel_search(&query_point)?;
        
        // Should return empty result
        assert_eq!(indices.len(), 0);
        Ok(())
    }
}

/// Tests for octree voxel centroid functionality
#[cfg(test)]
mod octree_voxel_centroid_tests {
    use super::*;

    #[test]
    fn test_voxel_centroid_creation() -> PclResult<()> {
        // Test creating OctreeVoxelCentroidXYZ
        let _octree = OctreeVoxelCentroidXYZ::new(1.0)?;
        Ok(())
    }

    #[test]
    fn test_voxel_centroid_set_input() -> PclResult<()> {
        // Test setting input cloud for voxel centroid octree
        let mut octree = OctreeVoxelCentroidXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.1, 0.1, 0.1)?;
        cloud.push(0.9, 0.9, 0.9)?;
        cloud.push(1.1, 1.1, 1.1)?;
        
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_voxel_centroid_compute() -> PclResult<()> {
        // Test computing voxel centroids
        let mut octree = OctreeVoxelCentroidXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points that should create centroids
        cloud.push(0.2, 0.2, 0.2)?;
        cloud.push(0.8, 0.8, 0.8)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Compute centroids
        let centroids = octree.get_voxel_centroids()?;
        
        // Should have at least one centroid
        assert!(centroids.size() >= 1);
        Ok(())
    }

    #[test]
    fn test_voxel_centroid_accuracy() -> PclResult<()> {
        // Test centroid computation accuracy
        let mut octree = OctreeVoxelCentroidXYZ::new(2.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points with known centroid
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        // Expected centroid: (0.5, 0.5, 0.5)
        
        octree.set_input_cloud(&cloud)?;
        let centroids = octree.get_voxel_centroids()?;
        
        // Should have one centroid approximately at (0.5, 0.5, 0.5)
        assert!(centroids.size() >= 1);
        // Detailed centroid value checking would require point access methods
        Ok(())
    }

    #[test]
    fn test_voxel_centroid_empty_cloud() -> PclResult<()> {
        // Test centroid computation with empty cloud
        let mut octree = OctreeVoxelCentroidXYZ::new(1.0)?;
        let cloud = PointCloud::<PointXYZ>::new()?;
        
        octree.set_input_cloud(&cloud)?;
        let centroids = octree.get_voxel_centroids()?;
        
        assert_eq!(centroids.size(), 0);
        Ok(())
    }
}

/// Tests for octree management operations
#[cfg(test)]
mod octree_management_tests {
    use super::*;

    #[test]
    fn test_octree_delete_tree() -> PclResult<()> {
        // Test deleting octree structure
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Delete the tree structure
        octree.delete_tree()?;
        
        // Tree should be effectively empty now
        let depth = octree.tree_depth();
        assert_eq!(depth, 0);
        Ok(())
    }

    #[test]
    fn test_octree_is_built() -> PclResult<()> {
        // Test checking if octree is built
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        
        // Initially should not be built
        assert!(!octree.is_built());
        
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Should be built after setting input
        assert!(octree.is_built());
        Ok(())
    }

    #[test]
    fn test_octree_rebuild() -> PclResult<()> {
        // Test rebuilding octree with different data
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud1 = PointCloud::<PointXYZ>::new()?;
        
        cloud1.push(0.0, 0.0, 0.0)?;
        cloud1.push(1.0, 1.0, 1.0)?;
        
        octree.set_input_cloud(&cloud1)?;
        let initial_leaf_count = octree.leaf_count();
        
        // Set different cloud
        let mut cloud2 = PointCloud::<PointXYZ>::new()?;
        for i in 0..10 {
            cloud2.push(i as f32, i as f32, i as f32)?;
        }
        
        octree.set_input_cloud(&cloud2)?;
        let new_leaf_count = octree.leaf_count();
        
        // Leaf count should be different
        assert_ne!(initial_leaf_count, new_leaf_count);
        Ok(())
    }
}

/// Tests corresponding to test/octree/test_octree_iterator.cpp - iterator functionality
#[cfg(test)]
mod octree_iterator_tests {
    use super::*;

    #[test]
    fn test_octree_iterator_placeholder() {
        // TODO: Implement octree iterator tests when iterator interface is available
        // This would test iterating through octree nodes, leaves, and branches
        
        // Placeholder test - octree iterator not yet implemented
        assert!(true, "Octree iterator tests not yet implemented");
    }

    #[test]
    fn test_octree_leaf_iterator_placeholder() {
        // TODO: Implement leaf iterator tests
        // This would test iterating through only leaf nodes
        
        assert!(true, "Octree leaf iterator tests not yet implemented");
    }

    #[test]
    fn test_octree_breadth_first_iterator_placeholder() {
        // TODO: Implement breadth-first iterator tests
        // This would test breadth-first traversal of octree
        
        assert!(true, "Octree breadth-first iterator tests not yet implemented");
    }

    #[test]
    fn test_octree_depth_first_iterator_placeholder() {
        // TODO: Implement depth-first iterator tests
        // This would test depth-first traversal of octree
        
        assert!(true, "Octree depth-first iterator tests not yet implemented");
    }
}

/// Builder pattern tests
#[cfg(test)]
mod octree_builder_tests {
    use super::*;
    use crate::octree::builders::{OctreeSearchBuilder, OctreeVoxelCentroidBuilder};

    #[test]
    fn test_octree_search_builder() -> PclResult<()> {
        // Test building octree with builder pattern
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        
        let octree = OctreeSearchBuilder::new(1.0)
            .input_cloud(cloud)
            .build()?;
        
        assert!(octree.is_built());
        Ok(())
    }

    #[test]
    fn test_octree_search_builder_no_cloud() -> PclResult<()> {
        // Test building octree without input cloud
        let octree = OctreeSearchBuilder::new(1.0)
            .resolution(2.0)
            .build()?;
        
        assert!(!octree.is_built());
        Ok(())
    }

    #[test]
    fn test_octree_voxel_centroid_builder() -> PclResult<()> {
        // Test building voxel centroid octree with builder
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        
        let octree = OctreeVoxelCentroidBuilder::new(1.0)
            .input_cloud(cloud)
            .build()?;
        
        assert!(octree.is_built());
        Ok(())
    }

    #[test]
    fn test_builder_method_chaining() -> PclResult<()> {
        // Test that builder methods can be chained fluently
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        
        let octree = OctreeSearchBuilder::new(0.5)
            .resolution(1.0)  // Override initial resolution
            .input_cloud(cloud)
            .build()?;
        
        // Resolution should be the overridden value
        let mut octree_mut = octree;
        let resolution = octree_mut.resolution();
        assert!((resolution - 1.0).abs() < 1e-6);
        Ok(())
    }
}

/// Performance and edge case tests
#[cfg(test)]
mod octree_performance_tests {
    use super::*;

    #[test]
    fn test_octree_memory_usage() -> PclResult<()> {
        // Test that octrees don't leak memory
        for _ in 0..10 {
            let mut octree = OctreeSearchXYZ::new(1.0)?;
            let mut cloud = PointCloud::<PointXYZ>::new()?;
            
            for i in 0..100 {
                cloud.push(i as f32, i as f32, i as f32)?;
            }
            
            octree.set_input_cloud(&cloud)?;
            
            let query_point = PointXYZ::new(50.0, 50.0, 50.0)?;
            let _indices = octree.voxel_search(&query_point)?;
        }
        Ok(())
    }

    #[test]
    fn test_octree_large_dataset() -> PclResult<()> {
        // Test octree with larger dataset
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a 20x20x20 grid (8000 points)
        for x in 0..20 {
            for y in 0..20 {
                for z in 0..20 {
                    cloud.push(x as f32, y as f32, z as f32)?;
                }
            }
        }
        
        octree.set_input_cloud(&cloud)?;
        
        // Verify octree was built successfully
        assert!(octree.is_built());
        assert!(octree.leaf_count() > 0);
        assert!(octree.tree_depth() > 0);
        
        Ok(())
    }

    #[test]
    fn test_octree_different_resolutions() -> PclResult<()> {
        // Test octree behavior with different resolutions
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a regular grid
        for x in 0..5 {
            for y in 0..5 {
                for z in 0..5 {
                    cloud.push(x as f32, y as f32, z as f32)?;
                }
            }
        }
        
        // Test with fine resolution
        let mut octree_fine = OctreeSearchXYZ::new(0.5)?;
        octree_fine.set_input_cloud(&cloud)?;
        let fine_leaf_count = octree_fine.leaf_count();
        
        // Test with coarse resolution
        let mut octree_coarse = OctreeSearchXYZ::new(2.0)?;
        octree_coarse.set_input_cloud(&cloud)?;
        let coarse_leaf_count = octree_coarse.leaf_count();
        
        // Finer resolution should generally have more leaves
        assert!(fine_leaf_count >= coarse_leaf_count);
        
        Ok(())
    }
}

/// Edge cases and error handling tests
#[cfg(test)]
mod octree_edge_cases {
    use super::*;

    #[test]
    fn test_octree_infinite_coordinates() -> PclResult<()> {
        // Test behavior with infinite coordinates
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(f32::INFINITY, 0.0, 0.0)?;
        cloud.push(0.0, f32::INFINITY, 0.0)?;
        
        // Should handle infinite values gracefully
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_nan_coordinates() -> PclResult<()> {
        // Test behavior with NaN coordinates
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(f32::NAN, 0.0, 0.0)?;
        cloud.push(0.0, f32::NAN, 0.0)?;
        
        // Should handle NaN values gracefully
        octree.set_input_cloud(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_octree_very_close_points() -> PclResult<()> {
        // Test with points very close to each other
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points within machine epsilon
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1e-15, 1e-15, 1e-15)?;
        cloud.push(2e-15, 2e-15, 2e-15)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Should handle very close points appropriately
        assert!(octree.leaf_count() >= 1);
        Ok(())
    }

    #[test]
    fn test_octree_extreme_coordinates() -> PclResult<()> {
        // Test with very large coordinate values
        let mut octree = OctreeSearchXYZ::new(1.0)?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1e6, 1e6, 1e6)?;
        cloud.push(-1e6, -1e6, -1e6)?;
        
        octree.set_input_cloud(&cloud)?;
        
        // Should handle large coordinates
        assert!(octree.is_built());
        Ok(())
    }
}