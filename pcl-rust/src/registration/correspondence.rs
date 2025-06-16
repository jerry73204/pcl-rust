//! Feature-based registration using correspondence estimation and rejection
//!
//! This module provides tools for feature-based point cloud registration that uses
//! correspondences between feature points to estimate transformations.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::registration::TransformationMatrix;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Correspondence between two points (source index, target index)
#[derive(Debug, Clone, PartialEq)]
pub struct Correspondence {
    pub source_index: i32,
    pub target_index: i32,
    pub distance: f32,
}

impl Correspondence {
    /// Create a new correspondence
    pub fn new(source_index: i32, target_index: i32, distance: f32) -> Self {
        Self {
            source_index,
            target_index,
            distance,
        }
    }
}

/// Correspondence estimation between point clouds
pub struct CorrespondenceEstimation {
    inner: UniquePtr<ffi::CorrespondenceEstimation_PointXYZ>,
}

impl CorrespondenceEstimation {
    /// Create a new correspondence estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_correspondence_estimation_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "CorrespondenceEstimation".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the input source cloud
    pub fn set_input_source(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_source_correspondence_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    /// Set the input target cloud
    pub fn set_input_target(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_target_correspondence_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    /// Determine correspondences between source and target clouds
    pub fn determine_correspondences(&mut self) -> PclResult<Vec<Correspondence>> {
        let mut indices = Vec::new();
        let mut distances = Vec::new();
        ffi::determine_correspondences_xyz(self.inner.pin_mut(), &mut indices, &mut distances);

        // Convert flattened arrays to Correspondence structs
        let mut correspondences = Vec::new();
        for i in (0..indices.len()).step_by(2) {
            if i + 1 < indices.len() && i / 2 < distances.len() {
                correspondences.push(Correspondence {
                    source_index: indices[i],
                    target_index: indices[i + 1],
                    distance: distances[i / 2],
                });
            }
        }

        Ok(correspondences)
    }

    /// Determine reciprocal correspondences between source and target clouds
    pub fn determine_reciprocal_correspondences(&mut self) -> PclResult<Vec<Correspondence>> {
        let mut indices = Vec::new();
        let mut distances = Vec::new();
        ffi::determine_reciprocal_correspondences_xyz(
            self.inner.pin_mut(),
            &mut indices,
            &mut distances,
        );

        // Convert flattened arrays to Correspondence structs
        let mut correspondences = Vec::new();
        for i in (0..indices.len()).step_by(2) {
            if i + 1 < indices.len() && i / 2 < distances.len() {
                correspondences.push(Correspondence {
                    source_index: indices[i],
                    target_index: indices[i + 1],
                    distance: distances[i / 2],
                });
            }
        }

        Ok(correspondences)
    }
}

/// RANSAC-based correspondence rejector
pub struct CorrespondenceRejectorSampleConsensus {
    inner: UniquePtr<ffi::CorrespondenceRejectorSampleConsensus_PointXYZ>,
}

impl CorrespondenceRejectorSampleConsensus {
    /// Create a new correspondence rejector
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_correspondence_rejector_sac_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "CorrespondenceRejectorSampleConsensus".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the input source cloud
    pub fn set_input_source(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_source_rejector_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    /// Set the input target cloud
    pub fn set_input_target(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_target_rejector_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    /// Set the inlier threshold for RANSAC
    pub fn set_inlier_threshold(&mut self, threshold: f64) {
        ffi::set_inlier_threshold_rejector_xyz(self.inner.pin_mut(), threshold);
    }

    /// Get the inlier threshold
    pub fn get_inlier_threshold(&mut self) -> f64 {
        ffi::get_inlier_threshold_rejector_xyz(self.inner.pin_mut())
    }

    /// Filter correspondences using RANSAC
    pub fn filter_correspondences(
        &mut self,
        correspondences: &[Correspondence],
    ) -> PclResult<Vec<Correspondence>> {
        // Flatten correspondences for FFI
        let mut indices = Vec::new();
        let mut distances = Vec::new();
        for corr in correspondences {
            indices.push(corr.source_index);
            indices.push(corr.target_index);
            distances.push(corr.distance);
        }

        let mut remaining_indices = Vec::new();
        let mut remaining_distances = Vec::new();
        ffi::get_correspondences_rejector_xyz(
            self.inner.pin_mut(),
            &indices,
            &distances,
            &mut remaining_indices,
            &mut remaining_distances,
        );

        // Convert flattened arrays back to Correspondence structs
        let mut filtered_correspondences = Vec::new();
        for i in (0..remaining_indices.len()).step_by(2) {
            if i + 1 < remaining_indices.len() && i / 2 < remaining_distances.len() {
                filtered_correspondences.push(Correspondence {
                    source_index: remaining_indices[i],
                    target_index: remaining_indices[i + 1],
                    distance: remaining_distances[i / 2],
                });
            }
        }

        Ok(filtered_correspondences)
    }
}

/// SVD-based transformation estimation
pub struct TransformationEstimationSVD {
    inner: UniquePtr<ffi::TransformationEstimationSVD_PointXYZ>,
}

impl TransformationEstimationSVD {
    /// Create a new transformation estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_transformation_estimation_svd_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "TransformationEstimationSVD".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Estimate rigid transformation from correspondences
    pub fn estimate_rigid_transformation(
        &mut self,
        source: &PointCloudXYZ,
        target: &PointCloudXYZ,
        correspondences: &[Correspondence],
    ) -> PclResult<TransformationMatrix> {
        // Flatten correspondences for FFI
        let mut indices = Vec::new();
        let mut distances = Vec::new();
        for corr in correspondences {
            indices.push(corr.source_index);
            indices.push(corr.target_index);
            distances.push(corr.distance);
        }

        let mut transformation_data = Vec::new();
        ffi::estimate_rigid_transformation_xyz(
            self.inner.pin_mut(),
            &source.inner,
            &target.inner,
            &indices,
            &distances,
            &mut transformation_data,
        );

        if transformation_data.len() == 16 {
            let mut data = [0.0f32; 16];
            data.copy_from_slice(&transformation_data);
            Ok(TransformationMatrix::from_array(&data))
        } else {
            Err(PclError::InvalidState {
                message: "Invalid transformation matrix size".into(),
                expected_state: "16 elements".into(),
                actual_state: format!("{} elements", transformation_data.len()),
            })
        }
    }
}

/// Complete feature-based registration pipeline
pub struct FeatureBasedRegistration {
    correspondence_estimator: CorrespondenceEstimation,
    correspondence_rejector: CorrespondenceRejectorSampleConsensus,
    transformation_estimator: TransformationEstimationSVD,
}

impl FeatureBasedRegistration {
    /// Create a new feature-based registration pipeline
    pub fn new() -> PclResult<Self> {
        Ok(Self {
            correspondence_estimator: CorrespondenceEstimation::new()?,
            correspondence_rejector: CorrespondenceRejectorSampleConsensus::new()?,
            transformation_estimator: TransformationEstimationSVD::new()?,
        })
    }

    /// Set the inlier threshold for correspondence rejection
    pub fn set_inlier_threshold(&mut self, threshold: f64) {
        self.correspondence_rejector.set_inlier_threshold(threshold);
    }

    /// Get the inlier threshold
    pub fn get_inlier_threshold(&mut self) -> f64 {
        self.correspondence_rejector.get_inlier_threshold()
    }

    /// Perform complete feature-based registration
    pub fn register(
        &mut self,
        source: &PointCloudXYZ,
        target: &PointCloudXYZ,
    ) -> PclResult<RegistrationResult> {
        // Set input clouds
        self.correspondence_estimator.set_input_source(source)?;
        self.correspondence_estimator.set_input_target(target)?;
        self.correspondence_rejector.set_input_source(source)?;
        self.correspondence_rejector.set_input_target(target)?;

        // Estimate correspondences
        let correspondences = self.correspondence_estimator.determine_correspondences()?;
        let total_correspondences = correspondences.len();

        // Filter correspondences using RANSAC
        let filtered_correspondences = self
            .correspondence_rejector
            .filter_correspondences(&correspondences)?;
        let inlier_correspondences = filtered_correspondences.len();

        // Estimate transformation
        let transformation = self
            .transformation_estimator
            .estimate_rigid_transformation(source, target, &filtered_correspondences)?;

        Ok(RegistrationResult {
            transformation,
            correspondences: filtered_correspondences,
            total_correspondences,
            inlier_correspondences,
        })
    }
}

/// Result of feature-based registration
#[derive(Debug)]
pub struct RegistrationResult {
    /// The estimated transformation matrix
    pub transformation: TransformationMatrix,
    /// The inlier correspondences used for transformation estimation
    pub correspondences: Vec<Correspondence>,
    /// Total number of correspondences found
    pub total_correspondences: usize,
    /// Number of inlier correspondences after filtering
    pub inlier_correspondences: usize,
}

impl RegistrationResult {
    /// Get the inlier ratio (inliers / total)
    pub fn inlier_ratio(&self) -> f32 {
        if self.total_correspondences == 0 {
            0.0
        } else {
            self.inlier_correspondences as f32 / self.total_correspondences as f32
        }
    }
}

/// Builder for feature-based registration
pub struct FeatureBasedRegistrationBuilder {
    inlier_threshold: Option<f64>,
}

impl FeatureBasedRegistrationBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            inlier_threshold: None,
        }
    }

    /// Set the inlier threshold for RANSAC
    pub fn inlier_threshold(mut self, threshold: f64) -> Self {
        self.inlier_threshold = Some(threshold);
        self
    }

    /// Build the configured feature-based registration instance
    pub fn build(self) -> PclResult<FeatureBasedRegistration> {
        let mut registration = FeatureBasedRegistration::new()?;

        if let Some(threshold) = self.inlier_threshold {
            registration.set_inlier_threshold(threshold);
        }

        Ok(registration)
    }
}

impl Default for FeatureBasedRegistrationBuilder {
    fn default() -> Self {
        Self::new()
    }
}
