//! Normal Distributions Transform (NDT) registration implementation
//!
//! NDT is a registration method that represents the point cloud as a set of normal
//! distributions and finds the transformation that maximizes the likelihood of the
//! source cloud given the target cloud's normal distributions.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use crate::registration::{RegistrationXYZ, RegistrationXYZRGB, TransformationMatrix};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// NDT registration for PointXYZ clouds
pub struct NdtXYZ {
    inner: UniquePtr<ffi::NormalDistributionsTransform_PointXYZ>,
}

impl NdtXYZ {
    /// Create a new NDT registration instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_ndt_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "NormalDistributionsTransform<PointXYZ>".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the transformation epsilon (convergence criteria)
    pub fn set_transformation_epsilon(&mut self, epsilon: f64) {
        ffi::set_transformation_epsilon_ndt_xyz(self.inner.pin_mut(), epsilon);
    }

    /// Get the transformation epsilon
    pub fn get_transformation_epsilon(&mut self) -> f64 {
        ffi::get_transformation_epsilon_ndt_xyz(self.inner.pin_mut())
    }

    /// Set the step size for the optimization algorithm
    pub fn set_step_size(&mut self, step_size: f64) {
        ffi::set_step_size_ndt_xyz(self.inner.pin_mut(), step_size);
    }

    /// Get the step size
    pub fn get_step_size(&mut self) -> f64 {
        ffi::get_step_size_ndt_xyz(self.inner.pin_mut())
    }

    /// Set the resolution of the voxel grid used by NDT
    pub fn set_resolution(&mut self, resolution: f32) {
        ffi::set_resolution_ndt_xyz(self.inner.pin_mut(), resolution as f64);
    }

    /// Get the resolution
    pub fn get_resolution(&mut self) -> f32 {
        ffi::get_resolution_ndt_xyz(self.inner.pin_mut()) as f32
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iter: i32) {
        ffi::set_max_iterations_ndt_xyz(self.inner.pin_mut(), max_iter);
    }

    /// Get the maximum number of iterations
    pub fn get_max_iterations(&mut self) -> i32 {
        ffi::get_max_iterations_ndt_xyz(self.inner.pin_mut())
    }
}

impl RegistrationXYZ for NdtXYZ {
    fn set_input_source(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_source_ndt_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn set_input_target(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_target_ndt_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn align(&mut self) -> PclResult<PointCloudXYZ> {
        let aligned = ffi::align_ndt_xyz(self.inner.pin_mut());
        if aligned.is_null() {
            Err(PclError::ProcessingFailed {
                message: "NDT alignment failed".into(),
            })
        } else {
            Ok(PointCloudXYZ { inner: aligned })
        }
    }

    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZ> {
        let aligned = ffi::align_with_guess_ndt_xyz(self.inner.pin_mut(), &initial_guess.to_vec());
        if aligned.is_null() {
            Err(PclError::ProcessingFailed {
                message: "NDT alignment with guess failed".into(),
            })
        } else {
            Ok(PointCloudXYZ { inner: aligned })
        }
    }

    fn has_converged(&mut self) -> bool {
        ffi::has_converged_ndt_xyz(self.inner.pin_mut())
    }

    fn get_fitness_score(&mut self) -> f64 {
        ffi::get_fitness_score_ndt_xyz(self.inner.pin_mut())
    }

    fn get_final_transformation(&mut self) -> TransformationMatrix {
        let transform = ffi::get_final_transformation_ndt_xyz(self.inner.pin_mut());
        TransformationMatrix::from_vec(&transform).unwrap_or_else(TransformationMatrix::identity)
    }
}

/// NDT registration for PointXYZRGB clouds
pub struct NdtXYZRGB {
    inner: UniquePtr<ffi::NormalDistributionsTransform_PointXYZRGB>,
}

impl NdtXYZRGB {
    /// Create a new NDT registration instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_ndt_xyzrgb();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "NormalDistributionsTransform<PointXYZRGB>".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the transformation epsilon (convergence criteria)
    pub fn set_transformation_epsilon(&mut self, epsilon: f64) {
        ffi::set_transformation_epsilon_ndt_xyzrgb(self.inner.pin_mut(), epsilon);
    }

    /// Get the transformation epsilon
    pub fn get_transformation_epsilon(&mut self) -> f64 {
        ffi::get_transformation_epsilon_ndt_xyzrgb(self.inner.pin_mut())
    }

    /// Set the step size for the optimization algorithm
    pub fn set_step_size(&mut self, step_size: f64) {
        ffi::set_step_size_ndt_xyzrgb(self.inner.pin_mut(), step_size);
    }

    /// Get the step size
    pub fn get_step_size(&mut self) -> f64 {
        ffi::get_step_size_ndt_xyzrgb(self.inner.pin_mut())
    }

    /// Set the resolution of the voxel grid used by NDT
    pub fn set_resolution(&mut self, resolution: f32) {
        ffi::set_resolution_ndt_xyzrgb(self.inner.pin_mut(), resolution as f64);
    }

    /// Get the resolution
    pub fn get_resolution(&mut self) -> f32 {
        ffi::get_resolution_ndt_xyzrgb(self.inner.pin_mut()) as f32
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iter: i32) {
        ffi::set_max_iterations_ndt_xyzrgb(self.inner.pin_mut(), max_iter);
    }

    /// Get the maximum number of iterations
    pub fn get_max_iterations(&mut self) -> i32 {
        ffi::get_max_iterations_ndt_xyzrgb(self.inner.pin_mut())
    }
}

impl RegistrationXYZRGB for NdtXYZRGB {
    fn set_input_source(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_source_ndt_xyzrgb(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn set_input_target(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_target_ndt_xyzrgb(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn align(&mut self) -> PclResult<PointCloudXYZRGB> {
        let aligned = ffi::align_ndt_xyzrgb(self.inner.pin_mut());
        if aligned.is_null() {
            Err(PclError::ProcessingFailed {
                message: "NDT alignment failed".into(),
            })
        } else {
            Ok(PointCloudXYZRGB { inner: aligned })
        }
    }

    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZRGB> {
        let aligned =
            ffi::align_with_guess_ndt_xyzrgb(self.inner.pin_mut(), &initial_guess.to_vec());
        if aligned.is_null() {
            Err(PclError::ProcessingFailed {
                message: "NDT alignment with guess failed".into(),
            })
        } else {
            Ok(PointCloudXYZRGB { inner: aligned })
        }
    }

    fn has_converged(&mut self) -> bool {
        ffi::has_converged_ndt_xyzrgb(self.inner.pin_mut())
    }

    fn get_fitness_score(&mut self) -> f64 {
        ffi::get_fitness_score_ndt_xyzrgb(self.inner.pin_mut())
    }

    fn get_final_transformation(&mut self) -> TransformationMatrix {
        let transform = ffi::get_final_transformation_ndt_xyzrgb(self.inner.pin_mut());
        TransformationMatrix::from_vec(&transform).unwrap_or_else(TransformationMatrix::identity)
    }
}

/// Builder for NDT registration
pub struct NdtXYZBuilder {
    transformation_epsilon: Option<f64>,
    step_size: Option<f64>,
    resolution: Option<f32>,
    max_iterations: Option<i32>,
}

impl NdtXYZBuilder {
    /// Create a new NDT builder
    pub fn new() -> Self {
        Self {
            transformation_epsilon: None,
            step_size: None,
            resolution: None,
            max_iterations: None,
        }
    }

    /// Set the transformation epsilon
    pub fn transformation_epsilon(mut self, epsilon: f64) -> Self {
        self.transformation_epsilon = Some(epsilon);
        self
    }

    /// Set the step size
    pub fn step_size(mut self, step_size: f64) -> Self {
        self.step_size = Some(step_size);
        self
    }

    /// Set the resolution
    pub fn resolution(mut self, resolution: f32) -> Self {
        self.resolution = Some(resolution);
        self
    }

    /// Set the maximum iterations
    pub fn max_iterations(mut self, max_iter: i32) -> Self {
        self.max_iterations = Some(max_iter);
        self
    }

    /// Build the configured NDT instance
    pub fn build(self) -> PclResult<NdtXYZ> {
        let mut ndt = NdtXYZ::new()?;

        if let Some(epsilon) = self.transformation_epsilon {
            ndt.set_transformation_epsilon(epsilon);
        }
        if let Some(step_size) = self.step_size {
            ndt.set_step_size(step_size);
        }
        if let Some(resolution) = self.resolution {
            ndt.set_resolution(resolution);
        }
        if let Some(max_iter) = self.max_iterations {
            ndt.set_max_iterations(max_iter);
        }

        Ok(ndt)
    }
}

/// Builder for NDT registration with RGB
pub struct NdtXYZRGBBuilder {
    transformation_epsilon: Option<f64>,
    step_size: Option<f64>,
    resolution: Option<f32>,
    max_iterations: Option<i32>,
}

impl NdtXYZRGBBuilder {
    /// Create a new NDT RGB builder
    pub fn new() -> Self {
        Self {
            transformation_epsilon: None,
            step_size: None,
            resolution: None,
            max_iterations: None,
        }
    }

    /// Set the transformation epsilon
    pub fn transformation_epsilon(mut self, epsilon: f64) -> Self {
        self.transformation_epsilon = Some(epsilon);
        self
    }

    /// Set the step size
    pub fn step_size(mut self, step_size: f64) -> Self {
        self.step_size = Some(step_size);
        self
    }

    /// Set the resolution
    pub fn resolution(mut self, resolution: f32) -> Self {
        self.resolution = Some(resolution);
        self
    }

    /// Set the maximum iterations
    pub fn max_iterations(mut self, max_iter: i32) -> Self {
        self.max_iterations = Some(max_iter);
        self
    }

    /// Build the configured NDT instance
    pub fn build(self) -> PclResult<NdtXYZRGB> {
        let mut ndt = NdtXYZRGB::new()?;

        if let Some(epsilon) = self.transformation_epsilon {
            ndt.set_transformation_epsilon(epsilon);
        }
        if let Some(step_size) = self.step_size {
            ndt.set_step_size(step_size);
        }
        if let Some(resolution) = self.resolution {
            ndt.set_resolution(resolution);
        }
        if let Some(max_iter) = self.max_iterations {
            ndt.set_max_iterations(max_iter);
        }

        Ok(ndt)
    }
}

impl Default for NdtXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for NdtXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}
