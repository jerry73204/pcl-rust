//! Generic surface reconstruction traits and implementations
//!
//! This module provides generic traits for surface reconstruction algorithms
//! that can work with different point types and mesh representations.

use crate::common::PointCloud;
use crate::error::PclResult;
use crate::traits::{NormalXyz, Point, Xyz};
use cxx::memory::UniquePtrTarget;

/// Generic trait for surface reconstruction algorithms
///
/// This trait defines the common interface for algorithms that reconstruct
/// surfaces from point clouds, producing mesh representations.
///
/// # Type Parameters
///
/// * `T` - The input point type
/// * `M` - The output mesh type
pub trait SurfaceReconstruction<T: Point, M>
where
    T::CloudType: UniquePtrTarget,
{
    /// Set the input point cloud for surface reconstruction
    fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()>;

    /// Perform surface reconstruction and return the resulting mesh
    fn reconstruct(&mut self, mesh: &mut M) -> PclResult<()>;
}

/// Generic trait for point cloud smoothing algorithms
///
/// This trait defines the common interface for algorithms that smooth
/// or resample point clouds, typically as a preprocessing step for
/// surface reconstruction.
///
/// # Type Parameters
///
/// * `T` - The input point type
/// * `U` - The output point type (may be different, e.g., with normals added)
pub trait PointCloudSmoothing<T: Point, U: Point>
where
    T::CloudType: UniquePtrTarget,
    U::CloudType: UniquePtrTarget,
{
    /// Set the input point cloud for smoothing
    fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()>;

    /// Process the point cloud and return the smoothed result
    fn process(&mut self) -> PclResult<PointCloud<U>>;
}

/// Trait for points that support surface reconstruction
///
/// This marker trait indicates that a point type has all the necessary
/// capabilities for surface reconstruction algorithms.
pub trait SurfacePoint: Point + Xyz + NormalXyz {}

// Implement SurfacePoint for types that have both coordinates and normals
use crate::common::PointNormal;

impl SurfacePoint for PointNormal {}

/// Configuration options for surface reconstruction algorithms
#[derive(Debug, Clone)]
pub struct SurfaceReconstructionConfig {
    /// Search radius for local operations
    pub search_radius: Option<f64>,

    /// Maximum number of neighbors to consider
    pub max_neighbors: Option<i32>,

    /// Smoothness threshold for surface fitting
    pub smoothness: Option<f64>,

    /// Whether to compute normals if not present
    pub compute_normals: bool,
}

impl Default for SurfaceReconstructionConfig {
    fn default() -> Self {
        Self {
            search_radius: None,
            max_neighbors: None,
            smoothness: None,
            compute_normals: false,
        }
    }
}

/// Builder for surface reconstruction configuration
pub struct SurfaceReconstructionConfigBuilder {
    config: SurfaceReconstructionConfig,
}

impl SurfaceReconstructionConfigBuilder {
    /// Create a new builder with default values
    pub fn new() -> Self {
        Self {
            config: SurfaceReconstructionConfig::default(),
        }
    }

    /// Set the search radius
    pub fn search_radius(mut self, radius: f64) -> Self {
        self.config.search_radius = Some(radius);
        self
    }

    /// Set the maximum number of neighbors
    pub fn max_neighbors(mut self, max: i32) -> Self {
        self.config.max_neighbors = Some(max);
        self
    }

    /// Set the smoothness threshold
    pub fn smoothness(mut self, smoothness: f64) -> Self {
        self.config.smoothness = Some(smoothness);
        self
    }

    /// Enable normal computation
    pub fn compute_normals(mut self, compute: bool) -> Self {
        self.config.compute_normals = compute;
        self
    }

    /// Build the configuration
    pub fn build(self) -> SurfaceReconstructionConfig {
        self.config
    }
}

impl Default for SurfaceReconstructionConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Generic mesh generation trait
///
/// This trait provides common operations for mesh generation from point clouds
pub trait MeshGeneration<T: SurfacePoint>
where
    T::CloudType: UniquePtrTarget,
{
    /// The mesh type produced by this algorithm
    type MeshType;

    /// Generate a mesh from the input point cloud
    fn generate_mesh(&mut self, cloud: &PointCloud<T>) -> PclResult<Self::MeshType>;

    /// Get the current configuration
    fn config(&self) -> &SurfaceReconstructionConfig;

    /// Set the configuration
    fn set_config(&mut self, config: SurfaceReconstructionConfig);
}

/// Extension trait for algorithms that support search method configuration
pub trait SearchConfiguration {
    /// Set the search method (e.g., KdTree)
    fn set_search_method<S>(&mut self, search: S) -> PclResult<()>
    where
        S: SearchMethod;
}

/// Marker trait for search methods
pub trait SearchMethod {}

// Note: Actual generic implementations would require updates to the FFI layer
// to support template instantiation for different point types. For now, these
// traits provide the blueprint for future generic implementations.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_builder() {
        let config = SurfaceReconstructionConfigBuilder::new()
            .search_radius(0.05)
            .max_neighbors(100)
            .smoothness(0.01)
            .compute_normals(true)
            .build();

        assert_eq!(config.search_radius, Some(0.05));
        assert_eq!(config.max_neighbors, Some(100));
        assert_eq!(config.smoothness, Some(0.01));
        assert!(config.compute_normals);
    }

    #[test]
    fn test_surface_point_implementation() {
        // Verify that PointNormal implements SurfacePoint
        fn requires_surface_point<T: SurfacePoint>() {}
        requires_surface_point::<PointNormal>();
    }
}
