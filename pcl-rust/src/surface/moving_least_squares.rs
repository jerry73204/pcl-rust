//! Moving Least Squares surface smoothing
//!
//! This module provides Moving Least Squares (MLS) surface smoothing and upsampling,
//! which can be used to smooth noisy point clouds and compute surface normals.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::surface::{PointCloudSmoothing, poisson::PointCloudWithNormals};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Upsampling method for Moving Least Squares
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpsampleMethod {
    /// No upsampling
    None = 0,
    /// Sample local plane
    SampleLocalPlane = 1,
    /// Random uniform density
    RandomUniformDensity = 2,
    /// Voxel grid dilation
    VoxelGridDilation = 3,
}

impl UpsampleMethod {
    /// Convert from integer value
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(UpsampleMethod::None),
            1 => Some(UpsampleMethod::SampleLocalPlane),
            2 => Some(UpsampleMethod::RandomUniformDensity),
            3 => Some(UpsampleMethod::VoxelGridDilation),
            _ => None,
        }
    }

    /// Convert to integer value
    pub fn to_i32(self) -> i32 {
        self as i32
    }
}

/// Placeholder for PointCloudNormal - this would be properly implemented when features module is available
pub struct PointCloudNormal {
    inner: UniquePtr<ffi::PointCloud_PointNormal>,
}

impl PointCloudNormal {
    /// Create from a unique pointer (internal use)
    pub(crate) fn from_ptr(ptr: UniquePtr<ffi::PointCloud_PointNormal>) -> Self {
        Self { inner: ptr }
    }

    /// Check if the point cloud is empty
    pub fn is_empty(&self) -> bool {
        // This would be properly implemented when features module is available
        // For now, return false as a placeholder
        false
    }

    /// Get the size of the point cloud
    pub fn size(&self) -> usize {
        // This would be properly implemented when features module is available
        // For now, return 0 as a placeholder
        0
    }

    /// Get the raw PCL point cloud reference
    pub fn as_raw(&self) -> &ffi::PointCloud_PointNormal {
        &self.inner
    }
}

impl PointCloudWithNormals for PointCloudNormal {
    fn is_empty(&self) -> bool {
        self.is_empty()
    }

    fn as_raw(&self) -> &ffi::PointCloud_PointNormal {
        &self.inner
    }
}

/// Moving Least Squares surface smoothing and upsampling
pub struct MovingLeastSquares {
    inner: UniquePtr<ffi::MovingLeastSquares_PointXYZ_PointNormal>,
}

impl MovingLeastSquares {
    /// Create a new Moving Least Squares instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_moving_least_squares();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "MovingLeastSquares".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the search radius for the local plane computation
    pub fn set_search_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid search radius",
                "radius",
                "positive value",
                &radius.to_string(),
            ));
        }
        ffi::set_search_radius_mls(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Get the current search radius
    pub fn search_radius(&self) -> f64 {
        ffi::get_search_radius_mls(&self.inner)
    }

    /// Set the polynomial order for the polynomial fitting
    pub fn set_polynomial_order(&mut self, order: i32) -> PclResult<()> {
        if order < 0 {
            return Err(PclError::invalid_parameters(
                "Invalid polynomial order",
                "order",
                "non-negative value",
                &order.to_string(),
            ));
        }
        ffi::set_polynomial_order_mls(self.inner.pin_mut(), order);
        Ok(())
    }

    /// Get the current polynomial order
    pub fn polynomial_order(&self) -> i32 {
        ffi::get_polynomial_order_mls(&self.inner)
    }

    /// Set the parameter used for the Gaussian weight function
    pub fn set_sqr_gauss_param(&mut self, param: f64) -> PclResult<()> {
        if param <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid Gaussian parameter",
                "param",
                "positive value",
                &param.to_string(),
            ));
        }
        ffi::set_sqr_gauss_param_mls(self.inner.pin_mut(), param);
        Ok(())
    }

    /// Get the current Gaussian parameter
    pub fn sqr_gauss_param(&self) -> f64 {
        ffi::get_sqr_gauss_param_mls(&self.inner)
    }

    /// Set whether to compute normals for the output point cloud
    pub fn set_compute_normals(&mut self, compute: bool) {
        ffi::set_compute_normals_mls(self.inner.pin_mut(), compute);
    }

    /// Set the upsampling method
    pub fn set_upsample_method(&mut self, method: UpsampleMethod) {
        ffi::set_upsample_method_mls(self.inner.pin_mut(), method.to_i32());
    }

    /// Set the radius for upsampling (used with SampleLocalPlane method)
    pub fn set_upsampling_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid upsampling radius",
                "radius",
                "positive value",
                &radius.to_string(),
            ));
        }
        ffi::set_upsampling_radius_mls(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Get the current upsampling radius
    pub fn upsampling_radius(&self) -> f64 {
        ffi::get_upsampling_radius_mls(&self.inner)
    }

    /// Set the step size for upsampling (used with SampleLocalPlane method)
    pub fn set_upsampling_step_size(&mut self, step_size: f64) -> PclResult<()> {
        if step_size <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid upsampling step size",
                "step_size",
                "positive value",
                &step_size.to_string(),
            ));
        }
        ffi::set_upsampling_step_size_mls(self.inner.pin_mut(), step_size);
        Ok(())
    }

    /// Get the current upsampling step size
    pub fn upsampling_step_size(&self) -> f64 {
        ffi::get_upsampling_step_size_mls(&self.inner)
    }

    /// Set the desired number of points in radius (used with RandomUniformDensity method)
    pub fn set_desired_num_points_in_radius(&mut self, num_points: i32) -> PclResult<()> {
        if num_points <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid desired number of points",
                "num_points",
                "positive value",
                &num_points.to_string(),
            ));
        }
        ffi::set_desired_num_points_in_radius_mls(self.inner.pin_mut(), num_points);
        Ok(())
    }

    /// Set the voxel size for dilation (used with VoxelGridDilation method)
    pub fn set_dilation_voxel_size(&mut self, voxel_size: f32) -> PclResult<()> {
        if voxel_size <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid dilation voxel size",
                "voxel_size",
                "positive value",
                &voxel_size.to_string(),
            ));
        }
        ffi::set_dilation_voxel_size_mls(self.inner.pin_mut(), voxel_size);
        Ok(())
    }

    /// Get the current dilation voxel size
    pub fn dilation_voxel_size(&self) -> f32 {
        ffi::get_dilation_voxel_size_mls(&self.inner)
    }

    /// Set the number of dilation iterations (used with VoxelGridDilation method)
    pub fn set_dilation_iterations(&mut self, iterations: i32) -> PclResult<()> {
        if iterations < 0 {
            return Err(PclError::invalid_parameters(
                "Invalid dilation iterations",
                "iterations",
                "non-negative value",
                &iterations.to_string(),
            ));
        }
        ffi::set_dilation_iterations_mls(self.inner.pin_mut(), iterations);
        Ok(())
    }

    /// Get the current dilation iterations
    pub fn dilation_iterations(&self) -> i32 {
        ffi::get_dilation_iterations_mls(&self.inner)
    }
}

impl PointCloudSmoothing<PointCloudXYZ, PointCloudNormal> for MovingLeastSquares {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_mls(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn process(&mut self) -> PclResult<PointCloudNormal> {
        let result = ffi::process_mls(self.inner.pin_mut());
        if result.is_null() {
            return Err(PclError::ProcessingFailed {
                message: "Moving Least Squares processing failed".to_string(),
            });
        }
        Ok(PointCloudNormal::from_ptr(result))
    }
}

impl Default for MovingLeastSquares {
    fn default() -> Self {
        Self::new().expect("Failed to create default MovingLeastSquares")
    }
}

/// Builder for MovingLeastSquares configuration
pub struct MovingLeastSquaresBuilder {
    search_radius: Option<f64>,
    polynomial_order: Option<i32>,
    sqr_gauss_param: Option<f64>,
    compute_normals: Option<bool>,
    upsample_method: Option<UpsampleMethod>,
    upsampling_radius: Option<f64>,
    upsampling_step_size: Option<f64>,
    desired_num_points_in_radius: Option<i32>,
    dilation_voxel_size: Option<f32>,
    dilation_iterations: Option<i32>,
}

impl MovingLeastSquaresBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            search_radius: None,
            polynomial_order: None,
            sqr_gauss_param: None,
            compute_normals: None,
            upsample_method: None,
            upsampling_radius: None,
            upsampling_step_size: None,
            desired_num_points_in_radius: None,
            dilation_voxel_size: None,
            dilation_iterations: None,
        }
    }

    /// Set the search radius
    pub fn search_radius(mut self, radius: f64) -> Self {
        self.search_radius = Some(radius);
        self
    }

    /// Set the polynomial order
    pub fn polynomial_order(mut self, order: i32) -> Self {
        self.polynomial_order = Some(order);
        self
    }

    /// Set the Gaussian parameter
    pub fn sqr_gauss_param(mut self, param: f64) -> Self {
        self.sqr_gauss_param = Some(param);
        self
    }

    /// Set whether to compute normals
    pub fn compute_normals(mut self, compute: bool) -> Self {
        self.compute_normals = Some(compute);
        self
    }

    /// Set the upsampling method
    pub fn upsample_method(mut self, method: UpsampleMethod) -> Self {
        self.upsample_method = Some(method);
        self
    }

    /// Set the upsampling radius
    pub fn upsampling_radius(mut self, radius: f64) -> Self {
        self.upsampling_radius = Some(radius);
        self
    }

    /// Set the upsampling step size
    pub fn upsampling_step_size(mut self, step_size: f64) -> Self {
        self.upsampling_step_size = Some(step_size);
        self
    }

    /// Set the desired number of points in radius
    pub fn desired_num_points_in_radius(mut self, num_points: i32) -> Self {
        self.desired_num_points_in_radius = Some(num_points);
        self
    }

    /// Set the dilation voxel size
    pub fn dilation_voxel_size(mut self, voxel_size: f32) -> Self {
        self.dilation_voxel_size = Some(voxel_size);
        self
    }

    /// Set the dilation iterations
    pub fn dilation_iterations(mut self, iterations: i32) -> Self {
        self.dilation_iterations = Some(iterations);
        self
    }

    /// Build the MovingLeastSquares instance
    pub fn build(self) -> PclResult<MovingLeastSquares> {
        let mut mls = MovingLeastSquares::new()?;

        if let Some(radius) = self.search_radius {
            mls.set_search_radius(radius)?;
        }
        if let Some(order) = self.polynomial_order {
            mls.set_polynomial_order(order)?;
        }
        if let Some(param) = self.sqr_gauss_param {
            mls.set_sqr_gauss_param(param)?;
        }
        if let Some(compute) = self.compute_normals {
            mls.set_compute_normals(compute);
        }
        if let Some(method) = self.upsample_method {
            mls.set_upsample_method(method);
        }
        if let Some(radius) = self.upsampling_radius {
            mls.set_upsampling_radius(radius)?;
        }
        if let Some(step_size) = self.upsampling_step_size {
            mls.set_upsampling_step_size(step_size)?;
        }
        if let Some(num_points) = self.desired_num_points_in_radius {
            mls.set_desired_num_points_in_radius(num_points)?;
        }
        if let Some(voxel_size) = self.dilation_voxel_size {
            mls.set_dilation_voxel_size(voxel_size)?;
        }
        if let Some(iterations) = self.dilation_iterations {
            mls.set_dilation_iterations(iterations)?;
        }

        Ok(mls)
    }
}

impl Default for MovingLeastSquaresBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_moving_least_squares_creation() {
        let mls = MovingLeastSquares::new();
        assert!(mls.is_ok());
    }

    #[test]
    fn test_upsample_method() {
        assert_eq!(UpsampleMethod::None.to_i32(), 0);
        assert_eq!(UpsampleMethod::SampleLocalPlane.to_i32(), 1);
        assert_eq!(UpsampleMethod::RandomUniformDensity.to_i32(), 2);
        assert_eq!(UpsampleMethod::VoxelGridDilation.to_i32(), 3);

        assert_eq!(UpsampleMethod::from_i32(0), Some(UpsampleMethod::None));
        assert_eq!(
            UpsampleMethod::from_i32(1),
            Some(UpsampleMethod::SampleLocalPlane)
        );
        assert_eq!(
            UpsampleMethod::from_i32(2),
            Some(UpsampleMethod::RandomUniformDensity)
        );
        assert_eq!(
            UpsampleMethod::from_i32(3),
            Some(UpsampleMethod::VoxelGridDilation)
        );
        assert_eq!(UpsampleMethod::from_i32(99), None);
    }

    #[test]
    fn test_moving_least_squares_builder() {
        let mls = MovingLeastSquaresBuilder::new()
            .search_radius(0.03)
            .polynomial_order(2)
            .sqr_gauss_param(0.001)
            .compute_normals(true)
            .upsample_method(UpsampleMethod::SampleLocalPlane)
            .upsampling_radius(0.005)
            .upsampling_step_size(0.002)
            .build();

        assert!(mls.is_ok());
        let mls = mls.unwrap();
        assert_eq!(mls.search_radius(), 0.03);
        assert_eq!(mls.polynomial_order(), 2);
        assert_eq!(mls.sqr_gauss_param(), 0.001);
        assert_eq!(mls.upsampling_radius(), 0.005);
        assert_eq!(mls.upsampling_step_size(), 0.002);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut mls = MovingLeastSquares::new().unwrap();

        assert!(mls.set_search_radius(-1.0).is_err());
        assert!(mls.set_search_radius(0.0).is_err());
        assert!(mls.set_polynomial_order(-1).is_err());
        assert!(mls.set_sqr_gauss_param(-1.0).is_err());
        assert!(mls.set_sqr_gauss_param(0.0).is_err());
        assert!(mls.set_upsampling_radius(-1.0).is_err());
        assert!(mls.set_upsampling_radius(0.0).is_err());
        assert!(mls.set_upsampling_step_size(-1.0).is_err());
        assert!(mls.set_upsampling_step_size(0.0).is_err());
        assert!(mls.set_desired_num_points_in_radius(-1).is_err());
        assert!(mls.set_desired_num_points_in_radius(0).is_err());
        assert!(mls.set_dilation_voxel_size(-1.0).is_err());
        assert!(mls.set_dilation_voxel_size(0.0).is_err());
        assert!(mls.set_dilation_iterations(-1).is_err());
    }
}
