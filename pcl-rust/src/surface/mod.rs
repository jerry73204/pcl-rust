//! Surface reconstruction algorithms
//!
//! This module provides safe Rust interfaces for various surface reconstruction algorithms
//! including Marching Cubes, Poisson reconstruction, Greedy Projection Triangulation,
//! and Moving Least Squares smoothing.

pub mod greedy_projection;
pub mod marching_cubes;
pub mod moving_least_squares;
pub mod organized_fast_mesh;
pub mod poisson;
pub mod polygon_mesh;

pub use greedy_projection::GreedyProjectionTriangulation;
pub use marching_cubes::{MarchingCubesHoppeXYZ, MarchingCubesRbfXYZ};
pub use moving_least_squares::{MovingLeastSquares, UpsampleMethod};
pub use organized_fast_mesh::{OrganizedFastMeshXYZ, TriangulationType};
pub use poisson::PoissonReconstruction;
pub use polygon_mesh::PolygonMesh;

use crate::error::PclResult;

/// Common trait for surface reconstruction algorithms
pub trait SurfaceReconstruction<T, M> {
    /// Set the input point cloud for surface reconstruction
    fn set_input_cloud(&mut self, cloud: &T) -> PclResult<()>;

    /// Perform surface reconstruction and return the resulting mesh
    fn reconstruct(&mut self, mesh: &mut M) -> PclResult<()>;
}

/// Common trait for point cloud smoothing algorithms
pub trait PointCloudSmoothing<T, U> {
    /// Set the input point cloud for smoothing
    fn set_input_cloud(&mut self, cloud: &T) -> PclResult<()>;

    /// Process the point cloud and return the smoothed result
    fn process(&mut self) -> PclResult<U>;
}
