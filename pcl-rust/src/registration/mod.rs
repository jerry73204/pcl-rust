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

pub use correspondence::{
    Correspondence, CorrespondenceEstimation, CorrespondenceRejectorSampleConsensus,
    FeatureBasedRegistration, FeatureBasedRegistrationBuilder, RegistrationResult,
    TransformationEstimationSVD,
};
pub use icp::{IcpXYZ, IcpXYZBuilder, IcpXYZRGB, IcpXYZRGBBuilder};
pub use ndt::{NdtXYZ, NdtXYZBuilder, NdtXYZRGB, NdtXYZRGBBuilder};
pub use transform::{Transform3D, TransformationMatrix};

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::PclResult;

/// Trait for point cloud registration algorithms
pub trait RegistrationXYZ {
    /// Set the source (moving) point cloud
    fn set_input_source(&mut self, cloud: &PointCloudXYZ) -> PclResult<()>;

    /// Set the target (fixed) point cloud
    fn set_input_target(&mut self, cloud: &PointCloudXYZ) -> PclResult<()>;

    /// Align the source cloud to the target cloud
    fn align(&mut self) -> PclResult<PointCloudXYZ>;

    /// Align with initial transformation guess
    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZ>;

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
    fn set_input_source(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()>;

    /// Set the target (fixed) point cloud
    fn set_input_target(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()>;

    /// Align the source cloud to the target cloud
    fn align(&mut self) -> PclResult<PointCloudXYZRGB>;

    /// Align with initial transformation guess
    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZRGB>;

    /// Check if the registration has converged
    fn has_converged(&mut self) -> bool;

    /// Get the fitness score of the alignment
    fn get_fitness_score(&mut self) -> f64;

    /// Get the final transformation matrix
    fn get_final_transformation(&mut self) -> TransformationMatrix;
}
