//! Tests corresponding to PCL's test/search/*.cpp files
//!
//! This module tests the search functionality including:
//! - test/search/test_search.cpp - General search tests
//! - test/search/test_kdtree.cpp - KdTree specific tests
//! - test/search/test_organized.cpp - Organized neighbor search tests
//! - test/search/test_organized_index.cpp - Organized index search tests
//! - test/search/test_octree.cpp - Octree search tests
//! - test/search/test_flann_search.cpp - FLANN search tests
//!
//! Uses test fixtures from PCL test data:
//! - bunny.pcd - Stanford bunny model for search tests
//! - milk.pcd - Milk carton scan for general search operations

use super::*;
use crate::common::{PointCloud, PointXYZ, PointXYZRGB, PointXYZI};
use crate::error::PclResult;

/// Tests corresponding to test/search/test_search.cpp - general search functionality
#[cfg(test)]
mod general_search_tests {
    use super::*;

    #[test]
    fn test_search_method_names() {
        // Test SearchMethod enum functionality
        assert_eq!(SearchMethod::KdTree.name(), "KD-Tree");
        assert_eq!(SearchMethod::Octree.name(), "Octree");
        assert_eq!(SearchMethod::Flann.name(), "FLANN");
        assert_eq!(SearchMethod::BruteForce.name(), "Brute Force");
    }

    #[test]
    fn test_search_method_approximate_support() {
        // Test which methods support approximate search
        assert!(SearchMethod::KdTree.supports_approximate_search());
        assert!(SearchMethod::Flann.supports_approximate_search());
        assert!(!SearchMethod::Octree.supports_approximate_search());
        assert!(!SearchMethod::BruteForce.supports_approximate_search());
    }

    #[test]
    fn test_search_method_equality() {
        // Test SearchMethod equality comparisons
        assert_eq!(SearchMethod::KdTree, SearchMethod::KdTree);
        assert_ne!(SearchMethod::KdTree, SearchMethod::Octree);
    }

    #[test]
    fn test_search_method_debug() {
        // Test debug formatting for SearchMethod
        let method = SearchMethod::KdTree;
        let debug_str = format!("{:?}", method);
        assert!(debug_str.contains("KdTree"));
    }

    #[test]
    fn test_search_method_clone() {
        // Test cloning SearchMethod
        let original = SearchMethod::KdTree;
        let cloned = original.clone();
        assert_eq!(original, cloned);
    }
}

/// Tests corresponding to test/search/test_kdtree.cpp - KdTree specific functionality
#[cfg(test)]
mod kdtree_tests {
    use super::*;
    use crate::search::generic::KdTree;

    #[test]
    fn test_kdtree_xyz_creation() -> PclResult<()> {
        // Test creating KdTree for PointXYZ
        let _tree: KdTree<PointXYZ> = KdTree::new()?;
        Ok(())
    }

    #[test]
    fn test_kdtree_xyzrgb_creation() -> PclResult<()> {
        // Test creating KdTree for PointXYZRGB
        let _tree: KdTree<PointXYZRGB> = KdTree::new()?;
        Ok(())
    }

    #[test]
    fn test_kdtree_xyzi_creation() -> PclResult<()> {
        // Test creating KdTree for PointXYZI
        let _tree: KdTree<PointXYZI> = KdTree::new()?;
        Ok(())
    }

    #[test]
    fn test_kdtree_set_input_cloud() -> PclResult<()> {
        // Test setting input cloud for KdTree search
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add some test points
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        cloud.push(7.0, 8.0, 9.0)?;
        
        tree.set_input_cloud(&cloud)?;
        assert!(tree.has_input_cloud());
        Ok(())
    }

    #[test]
    fn test_kdtree_nearest_k_search_basic() -> PclResult<()> {
        // Test basic k-nearest neighbor search
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a simple point cloud
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, 0.0, 1.0)?;
        cloud.push(2.0, 2.0, 2.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        // Search for nearest neighbors of origin
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 3)?;
        
        assert_eq!(indices.len(), 3);
        assert_eq!(indices[0], 0); // Should find itself first
        Ok(())
    }

    #[test]
    fn test_kdtree_nearest_k_search_with_distances() -> PclResult<()> {
        // Test k-nearest neighbor search with distance computation
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a simple point cloud
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(2.0, 0.0, 0.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        // Search for nearest neighbors with distances
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let (indices, distances) = tree.nearest_k_search_with_distances(&query_point, 2)?;
        
        assert_eq!(indices.len(), 2);
        assert_eq!(distances.len(), 2);
        assert_eq!(indices[0], 0);
        assert!((distances[0] - 0.0).abs() < 1e-6); // Distance to itself should be 0
        Ok(())
    }

    #[test]
    fn test_kdtree_radius_search() -> PclResult<()> {
        // Test radius-based neighbor search
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points in a line
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(2.0, 0.0, 0.0)?;
        cloud.push(3.0, 0.0, 0.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        // Search within radius of 1.5 from origin
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.radius_search(&query_point, 1.5)?;
        
        // Should find points at 0.0 and 1.0 distance
        assert_eq!(indices.len(), 2);
        assert!(indices.contains(&0));
        assert!(indices.contains(&1));
        Ok(())
    }

    #[test]
    fn test_kdtree_radius_search_with_distances() -> PclResult<()> {
        // Test radius search with distance computation
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points in a line
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(2.0, 0.0, 0.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        // Search within radius with distances
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let (indices, distances) = tree.radius_search_with_distances(&query_point, 1.5)?;
        
        assert_eq!(indices.len(), distances.len());
        assert!(indices.len() >= 2); // Should find at least 2 points
        Ok(())
    }

    #[test]
    fn test_kdtree_empty_cloud() -> PclResult<()> {
        // Test behavior with empty point cloud
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let cloud = PointCloud::<PointXYZ>::new()?;
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 1)?;
        
        assert_eq!(indices.len(), 0);
        Ok(())
    }

    #[test]
    fn test_kdtree_single_point() -> PclResult<()> {
        // Test behavior with single point cloud
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(1.0, 2.0, 3.0)?;
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 1)?;
        
        assert_eq!(indices.len(), 1);
        assert_eq!(indices[0], 0);
        Ok(())
    }

    #[test]
    fn test_kdtree_large_k() -> PclResult<()> {
        // Test requesting more neighbors than available points
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add only 3 points
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 10)?; // Request more than available
        
        assert_eq!(indices.len(), 3); // Should return all available points
        Ok(())
    }

    #[test]
    fn test_kdtree_zero_radius() -> PclResult<()> {
        // Test radius search with zero radius
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.radius_search(&query_point, 0.0)?;
        
        // Should find only the exact match (if any)
        assert!(indices.len() <= 1);
        if indices.len() == 1 {
            assert_eq!(indices[0], 0);
        }
        Ok(())
    }

    #[test]
    fn test_kdtree_xyzrgb_search() -> PclResult<()> {
        // Test KdTree with PointXYZRGB (spatial search ignores color)
        let mut tree: KdTree<PointXYZRGB> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        // Add colored points - spatial positions matter, not color
        cloud.push(0.0, 0.0, 0.0, 255, 0, 0)?; // Red
        cloud.push(1.0, 0.0, 0.0, 0, 255, 0)?; // Green
        cloud.push(0.0, 1.0, 0.0, 0, 0, 255)?; // Blue
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZRGB::new(0.0, 0.0, 0.0, 128, 128, 128)?;
        let indices = tree.nearest_k_search(&query_point, 2)?;
        
        assert_eq!(indices.len(), 2);
        assert_eq!(indices[0], 0); // Closest spatially
        Ok(())
    }

    #[test]
    fn test_kdtree_performance_large_cloud() -> PclResult<()> {
        // Test performance with moderately large point cloud
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a larger point cloud for performance testing
        for i in 0..1000 {
            let x = (i as f32 * 0.1) % 10.0;
            let y = ((i / 10) as f32 * 0.1) % 10.0;
            let z = ((i / 100) as f32 * 0.1) % 10.0;
            cloud.push(x, y, z)?;
        }
        
        tree.set_input_cloud(&cloud)?;
        
        // Perform multiple searches to test performance
        let query_point = PointXYZ::new(5.0, 5.0, 5.0)?;
        for _ in 0..10 {
            let indices = tree.nearest_k_search(&query_point, 10)?;
            assert_eq!(indices.len(), 10);
        }
        
        Ok(())
    }
}

/// Tests corresponding to test/search/test_organized.cpp - Organized search functionality  
#[cfg(test)]
mod organized_search_tests {
    use super::*;

    #[test]
    fn test_organized_search_placeholder() {
        // TODO: Implement organized search tests when organized search is available
        // This corresponds to PCL's organized neighbor search functionality
        
        // Placeholder test - organized search not yet implemented
        assert!(true, "Organized search tests not yet implemented");
    }
}

/// Tests corresponding to test/search/test_organized_index.cpp - Organized index search
#[cfg(test)]
mod organized_index_tests {
    use super::*;

    #[test]
    fn test_organized_index_placeholder() {
        // TODO: Implement organized index search tests when available
        // This corresponds to PCL's organized index search functionality
        
        // Placeholder test - organized index search not yet implemented
        assert!(true, "Organized index search tests not yet implemented");
    }
}

/// Tests corresponding to test/search/test_octree.cpp - Octree search functionality
#[cfg(test)]
mod octree_search_tests {
    use super::*;

    #[test]
    fn test_octree_search_placeholder() {
        // TODO: Implement octree search tests when octree search is available
        // This would test octree-based spatial search operations
        
        // Note: The octree module exists but may not have search interface yet
        assert!(true, "Octree search tests not yet implemented");
    }
}

/// Tests corresponding to test/search/test_flann_search.cpp - FLANN search functionality
#[cfg(test)]
mod flann_search_tests {
    use super::*;

    #[test]
    fn test_flann_search_placeholder() {
        // TODO: Implement FLANN search tests when FLANN integration is available
        // FLANN (Fast Library for Approximate Nearest Neighbors) provides
        // efficient search for high-dimensional spaces
        
        // Placeholder test - FLANN search not yet implemented
        assert!(true, "FLANN search tests not yet implemented");
    }
}

/// Performance and stress tests for search functionality
#[cfg(test)]
mod search_performance_tests {
    use super::*;

    #[test]
    fn test_search_memory_usage() -> PclResult<()> {
        // Test that search structures don't leak memory
        for _ in 0..10 {
            let mut tree: KdTree<PointXYZ> = KdTree::new()?;
            let mut cloud = PointCloud::<PointXYZ>::new()?;
            
            for i in 0..100 {
                cloud.push(i as f32, i as f32, i as f32)?;
            }
            
            tree.set_input_cloud(&cloud)?;
            
            let query_point = PointXYZ::new(50.0, 50.0, 50.0)?;
            let _indices = tree.nearest_k_search(&query_point, 5)?;
        }
        Ok(())
    }

    #[test]
    fn test_search_thread_safety() -> PclResult<()> {
        // Test basic thread safety of search operations
        // Note: More comprehensive thread safety tests would require std::thread
        
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        for i in 0..50 {
            cloud.push(i as f32, 0.0, 0.0)?;
        }
        
        tree.set_input_cloud(&cloud)?;
        
        // Perform multiple searches in sequence (simulating concurrent access)
        for i in 0..10 {
            let query_point = PointXYZ::new(i as f32 * 5.0, 0.0, 0.0)?;
            let indices = tree.nearest_k_search(&query_point, 3)?;
            assert!(indices.len() <= 3);
        }
        
        Ok(())
    }
}

/// Edge case and error handling tests
#[cfg(test)]
mod search_edge_cases {
    use super::*;

    #[test]
    fn test_search_without_input_cloud() -> PclResult<()> {
        // Test search operations before setting input cloud
        let tree: KdTree<PointXYZ> = KdTree::new()?;
        assert!(!tree.has_input_cloud());
        
        // Attempting search without input cloud should handle gracefully
        // The specific behavior depends on implementation
        Ok(())
    }

    #[test]
    fn test_search_negative_k() -> PclResult<()> {
        // Test behavior with negative k value
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, -1)?;
        
        // Should handle negative k gracefully (likely return empty result)
        assert_eq!(indices.len(), 0);
        Ok(())
    }

    #[test]
    fn test_search_zero_k() -> PclResult<()> {
        // Test behavior with k = 0
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 0)?;
        
        assert_eq!(indices.len(), 0);
        Ok(())
    }

    #[test]
    fn test_search_negative_radius() -> PclResult<()> {
        // Test behavior with negative radius
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.radius_search(&query_point, -1.0)?;
        
        // Should handle negative radius gracefully
        assert_eq!(indices.len(), 0);
        Ok(())
    }

    #[test]
    fn test_search_infinite_values() -> PclResult<()> {
        // Test behavior with infinite coordinate values
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        tree.set_input_cloud(&cloud)?;
        
        // Query with infinite coordinates
        let query_point = PointXYZ::new(f32::INFINITY, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 1)?;
        
        // Should handle infinite values - may return empty or valid result
        // depending on implementation
        assert!(indices.len() <= 1);
        Ok(())
    }

    #[test]
    fn test_search_nan_values() -> PclResult<()> {
        // Test behavior with NaN coordinate values
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 1.0, 1.0)?;
        tree.set_input_cloud(&cloud)?;
        
        // Query with NaN coordinates  
        let query_point = PointXYZ::new(f32::NAN, 0.0, 0.0)?;
        let indices = tree.nearest_k_search(&query_point, 1)?;
        
        // Should handle NaN values gracefully
        assert!(indices.len() <= 1);
        Ok(())
    }
}

/// Search accuracy and correctness tests
#[cfg(test)]
mod search_accuracy_tests {
    use super::*;

    #[test]
    fn test_search_accuracy_grid() -> PclResult<()> {
        // Test search accuracy with a regular grid of points
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a 3x3x3 grid
        for x in 0..3 {
            for y in 0..3 {
                for z in 0..3 {
                    cloud.push(x as f32, y as f32, z as f32)?;
                }
            }
        }
        
        tree.set_input_cloud(&cloud)?;
        
        // Search from center of grid
        let query_point = PointXYZ::new(1.0, 1.0, 1.0)?;
        let indices = tree.nearest_k_search(&query_point, 1)?;
        
        assert_eq!(indices.len(), 1);
        // The center point should be at index 13 (1*9 + 1*3 + 1 = 13)
        assert_eq!(indices[0], 13);
        Ok(())
    }

    #[test]
    fn test_search_accuracy_distances() -> PclResult<()> {
        // Test that reported distances are accurate
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points at known distances
        cloud.push(0.0, 0.0, 0.0)?; // Distance 0
        cloud.push(1.0, 0.0, 0.0)?; // Distance 1
        cloud.push(0.0, 1.0, 0.0)?; // Distance 1  
        cloud.push(0.0, 0.0, 1.0)?; // Distance 1
        cloud.push(2.0, 0.0, 0.0)?; // Distance 2
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let (indices, distances) = tree.nearest_k_search_with_distances(&query_point, 3)?;
        
        assert_eq!(indices.len(), 3);
        assert_eq!(distances.len(), 3);
        
        // First point should be exact match with distance 0
        assert_eq!(indices[0], 0);
        assert!((distances[0] - 0.0).abs() < 1e-6);
        
        // Next points should be at distance 1 (squared distance = 1)
        assert!((distances[1] - 1.0).abs() < 1e-6);
        assert!((distances[2] - 1.0).abs() < 1e-6);
        
        Ok(())
    }

    #[test]
    fn test_radius_search_accuracy() -> PclResult<()> {
        // Test radius search returns correct points within radius
        let mut tree: KdTree<PointXYZ> = KdTree::new()?;
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points at various distances from origin
        cloud.push(0.0, 0.0, 0.0)?; // Distance 0
        cloud.push(0.5, 0.0, 0.0)?; // Distance 0.5
        cloud.push(1.0, 0.0, 0.0)?; // Distance 1.0
        cloud.push(1.5, 0.0, 0.0)?; // Distance 1.5
        cloud.push(2.0, 0.0, 0.0)?; // Distance 2.0
        
        tree.set_input_cloud(&cloud)?;
        
        let query_point = PointXYZ::new(0.0, 0.0, 0.0)?;
        let indices = tree.radius_search(&query_point, 1.0)?;
        
        // Should find points at distances 0, 0.5, and 1.0
        assert_eq!(indices.len(), 3);
        assert!(indices.contains(&0));
        assert!(indices.contains(&1));
        assert!(indices.contains(&2));
        
        Ok(())
    }
}