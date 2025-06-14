//! Generic search interfaces and traits
//!
//! This module provides trait definitions for spatial search operations
//! that can be implemented by various search algorithms.

use crate::error::PclResult;

/// Generic trait for nearest neighbor search operations
pub trait NearestNeighborSearch<PointT> {
    /// Find the k nearest neighbors to a query point
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `k` - Number of nearest neighbors to find
    ///
    /// # Returns
    /// Vector of indices into the input cloud
    fn nearest_k_search(&self, point: &PointT, k: i32) -> PclResult<Vec<i32>>;

    /// Find the k nearest neighbors with distances
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `k` - Number of nearest neighbors to find
    ///
    /// # Returns
    /// Tuple of (indices, squared distances) vectors
    fn nearest_k_search_with_distances(
        &self,
        point: &PointT,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)>;

    /// Find all neighbors within a radius
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `radius` - Search radius
    ///
    /// # Returns
    /// Vector of indices into the input cloud
    fn radius_search(&self, point: &PointT, radius: f64) -> PclResult<Vec<i32>>;

    /// Find all neighbors within a radius with distances
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `radius` - Search radius
    ///
    /// # Returns
    /// Tuple of (indices, squared distances) vectors
    fn radius_search_with_distances(
        &self,
        point: &PointT,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)>;
}

/// Configuration for search algorithms
pub trait SearchConfiguration {
    /// Get the epsilon value for approximate search
    fn epsilon(&self) -> f32;

    /// Set the epsilon value for approximate search
    ///
    /// Epsilon is the error bound for approximate nearest neighbor search.
    /// A value of 0.0 means exact search.
    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()>;

    /// Check if the search structure is sorted
    fn is_sorted(&self) -> bool {
        true // Most search structures maintain sorted results by default
    }
}

/// Trait for search structures that require input cloud
pub trait SearchInputCloud<CloudT> {
    /// Set the input cloud for search operations
    fn set_input_cloud(&mut self, cloud: &CloudT) -> PclResult<()>;

    /// Check if an input cloud has been set
    fn has_input_cloud(&self) -> bool;
}

/// Search method enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchMethod {
    /// KD-tree search (good for low-dimensional data)
    KdTree,
    /// Octree search (good for 3D spatial queries)
    Octree,
    /// FLANN-based search (good for high-dimensional data)
    Flann,
    /// Brute force search (simple but slow)
    BruteForce,
}

impl SearchMethod {
    /// Get a human-readable name for the search method
    pub fn name(&self) -> &'static str {
        match self {
            SearchMethod::KdTree => "KD-Tree",
            SearchMethod::Octree => "Octree",
            SearchMethod::Flann => "FLANN",
            SearchMethod::BruteForce => "Brute Force",
        }
    }

    /// Check if the method supports approximate search
    pub fn supports_approximate_search(&self) -> bool {
        matches!(self, SearchMethod::KdTree | SearchMethod::Flann)
    }
}
