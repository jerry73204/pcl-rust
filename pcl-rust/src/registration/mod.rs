//! Point cloud registration algorithms
//!
//! This module provides algorithms for aligning point clouds, including:
//! - ICP (Iterative Closest Point)
//! - NDT (Normal Distributions Transform)
//! - Feature-based registration using correspondences

pub mod correspondence;
pub mod icp;
pub mod ndt;
pub mod transform;

#[cfg(test)]
mod tests;

pub use correspondence::{
    Correspondence, CorrespondenceEstimation, CorrespondenceRejectorSampleConsensus,
    FeatureBasedRegistration, FeatureBasedRegistrationBuilder, RegistrationResult,
    TransformationEstimationSVD,
};
pub use icp::{IcpXYZ, IcpXYZBuilder, IcpXYZRGB, IcpXYZRGBBuilder};
pub use ndt::{NdtXYZ, NdtXYZBuilder, NdtXYZRGB, NdtXYZRGBBuilder};
pub use transform::{Transform3D, TransformationMatrix};

use crate::common::{PointCloud, XYZ, XYZRGB};
use crate::error::PclResult;

/// Trait for point cloud registration algorithms
pub trait RegistrationXYZ {
    /// Set the source (moving) point cloud
    fn set_input_source(&mut self, cloud: &PointCloud<XYZ>) -> PclResult<()>;

    /// Set the target (fixed) point cloud
    fn set_input_target(&mut self, cloud: &PointCloud<XYZ>) -> PclResult<()>;

    /// Align the source cloud to the target cloud
    fn align(&mut self) -> PclResult<PointCloud<XYZ>>;

    /// Align with initial transformation guess
    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloud<XYZ>>;

    /// Check if the registration has converged
    fn has_converged(&mut self) -> bool;

    /// Get the fitness score of the alignment
    fn get_fitness_score(&mut self) -> f64;

    /// Get the final transformation matrix
    fn get_final_transformation(&mut self) -> TransformationMatrix;
}

/// Trait for point cloud registration algorithms with RGB
pub trait RegistrationXYZRGB {
    /// Set the source (moving) point cloud
    fn set_input_source(&mut self, cloud: &PointCloud<XYZRGB>) -> PclResult<()>;

    /// Set the target (fixed) point cloud
    fn set_input_target(&mut self, cloud: &PointCloud<XYZRGB>) -> PclResult<()>;

    /// Align the source cloud to the target cloud
    fn align(&mut self) -> PclResult<PointCloud<XYZRGB>>;

    /// Align with initial transformation guess
    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloud<XYZRGB>>;

    /// Check if the registration has converged
    fn has_converged(&mut self) -> bool;

    /// Get the fitness score of the alignment
    fn get_fitness_score(&mut self) -> f64;

    /// Get the final transformation matrix
    fn get_final_transformation(&mut self) -> TransformationMatrix;
}
