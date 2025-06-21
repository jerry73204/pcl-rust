//! Traits for octree operations
//!
//! This module provides trait definitions for various octree operations.

use crate::error::PclResult;

/// Configuration for octree data structures
pub trait OctreeConfiguration {
    /// Get the octree resolution
    fn resolution(&mut self) -> f64;

    /// Get the depth of the octree
    fn tree_depth(&mut self) -> u32;

    /// Get the number of leaf nodes
    fn leaf_count(&mut self) -> usize;

    /// Get the number of branch nodes  
    fn branch_count(&mut self) -> usize;
}

/// Trait for octree structures that support voxel operations
pub trait VoxelOperations {
    /// Find all points in the same voxel as the query point
    fn voxel_search(&mut self, point: &crate::common::PointXYZ) -> PclResult<Vec<i32>>;
}

/// Trait for octree structures that can be cleared
pub trait OctreeManagement {
    /// Delete the octree structure (keeps the capacity allocated)
    fn delete_tree(&mut self) -> PclResult<()>;

    /// Check if the octree has been built
    fn is_built(&self) -> bool;
}

/// Types of octree structures available
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OctreeType {
    /// Basic octree for point cloud storage
    PointCloud,
    /// Octree optimized for search operations
    Search,
    /// Octree that computes voxel centroids
    VoxelCentroid,
    /// Octree for occupancy detection
    Occupancy,
    /// Octree for change detection between clouds
    ChangeDetection,
}

impl OctreeType {
    /// Get a human-readable name for the octree type
    pub fn name(&self) -> &'static str {
        match self {
            OctreeType::PointCloud => "Point Cloud",
            OctreeType::Search => "Search",
            OctreeType::VoxelCentroid => "Voxel Centroid",
            OctreeType::Occupancy => "Occupancy",
            OctreeType::ChangeDetection => "Change Detection",
        }
    }

    /// Check if the octree type supports search operations
    pub fn supports_search(&self) -> bool {
        matches!(self, OctreeType::Search)
    }

    /// Check if the octree type supports voxel operations
    pub fn supports_voxel_operations(&self) -> bool {
        matches!(
            self,
            OctreeType::Search | OctreeType::VoxelCentroid | OctreeType::Occupancy
        )
    }
}
