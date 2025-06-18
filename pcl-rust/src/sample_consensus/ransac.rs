//! RANSAC algorithm implementation

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use pcl_sys::UniquePtr;
use pcl_sys::ffi;

/// RANSAC algorithm for plane and sphere fitting with PointXYZ
pub struct RansacPlaneXYZ {
    inner: UniquePtr<ffi::RandomSampleConsensus_PointXYZ>,
}

impl RansacPlaneXYZ {
    /// Create a new RANSAC plane fitter for a PointXYZ cloud
    pub fn new(cloud: &PointCloudXYZ) -> PclResult<Self> {
        let inner = ffi::new_ransac_plane_xyz(cloud.inner());
        if inner.is_null() {
            return Err(PclError::RansacError(
                "Failed to create RANSAC plane fitter".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Set the distance threshold for inlier determination
    pub fn set_distance_threshold(&mut self, threshold: f64) {
        ffi::set_distance_threshold_xyz(self.inner.pin_mut(), threshold);
    }

    /// Get the current distance threshold
    pub fn distance_threshold(&self) -> f64 {
        ffi::get_distance_threshold_xyz(&self.inner)
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iterations: i32) {
        ffi::set_max_iterations_xyz(self.inner.pin_mut(), max_iterations);
    }

    /// Get the maximum number of iterations
    pub fn max_iterations(&self) -> i32 {
        ffi::get_max_iterations_xyz(&self.inner)
    }

    /// Set the desired probability of finding the correct model
    pub fn set_probability(&mut self, probability: f64) {
        ffi::set_probability_xyz(self.inner.pin_mut(), probability);
    }

    /// Get the desired probability
    pub fn probability(&self) -> f64 {
        ffi::get_probability_xyz(&self.inner)
    }

    /// Compute the model using RANSAC
    pub fn compute_model(&mut self) -> PclResult<()> {
        if ffi::compute_model_xyz(self.inner.pin_mut()) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to compute RANSAC model".to_string(),
            ))
        }
    }

    /// Refine the model using non-linear optimization
    pub fn refine_model(&mut self, sigma: f64, max_iterations: u32) -> PclResult<()> {
        if ffi::refine_model_xyz(self.inner.pin_mut(), sigma, max_iterations) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to refine RANSAC model".to_string(),
            ))
        }
    }

    /// Get the indices of inlier points
    pub fn inliers(&self) -> Vec<i32> {
        ffi::get_inliers_xyz(&self.inner)
    }

    /// Get the model coefficients
    /// For planes: [a, b, c, d] where ax + by + cz + d = 0
    pub fn model_coefficients(&self) -> Vec<f32> {
        ffi::get_model_coefficients_xyz(&self.inner)
    }

    /// Get the number of inliers
    pub fn inliers_count(&self) -> usize {
        ffi::get_inliers_count_xyz(&self.inner)
    }
}

/// RANSAC algorithm for sphere fitting with PointXYZ
pub struct RansacSphereXYZ {
    inner: UniquePtr<ffi::RandomSampleConsensus_PointXYZ>,
}

impl RansacSphereXYZ {
    /// Create a new RANSAC sphere fitter for a PointXYZ cloud
    pub fn new(cloud: &PointCloudXYZ) -> PclResult<Self> {
        let inner = ffi::new_ransac_sphere_xyz(cloud.inner());
        if inner.is_null() {
            return Err(PclError::RansacError(
                "Failed to create RANSAC sphere fitter".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Set the distance threshold for inlier determination
    pub fn set_distance_threshold(&mut self, threshold: f64) {
        ffi::set_distance_threshold_xyz(self.inner.pin_mut(), threshold);
    }

    /// Get the current distance threshold
    pub fn distance_threshold(&self) -> f64 {
        ffi::get_distance_threshold_xyz(&self.inner)
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iterations: i32) {
        ffi::set_max_iterations_xyz(self.inner.pin_mut(), max_iterations);
    }

    /// Get the maximum number of iterations
    pub fn max_iterations(&self) -> i32 {
        ffi::get_max_iterations_xyz(&self.inner)
    }

    /// Set the desired probability of finding the correct model
    pub fn set_probability(&mut self, probability: f64) {
        ffi::set_probability_xyz(self.inner.pin_mut(), probability);
    }

    /// Get the desired probability
    pub fn probability(&self) -> f64 {
        ffi::get_probability_xyz(&self.inner)
    }

    /// Compute the model using RANSAC
    pub fn compute_model(&mut self) -> PclResult<()> {
        if ffi::compute_model_xyz(self.inner.pin_mut()) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to compute RANSAC model".to_string(),
            ))
        }
    }

    /// Refine the model using non-linear optimization
    pub fn refine_model(&mut self, sigma: f64, max_iterations: u32) -> PclResult<()> {
        if ffi::refine_model_xyz(self.inner.pin_mut(), sigma, max_iterations) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to refine RANSAC model".to_string(),
            ))
        }
    }

    /// Get the indices of inlier points
    pub fn inliers(&self) -> Vec<i32> {
        ffi::get_inliers_xyz(&self.inner)
    }

    /// Get the model coefficients
    /// For spheres: [center_x, center_y, center_z, radius]
    pub fn model_coefficients(&self) -> Vec<f32> {
        ffi::get_model_coefficients_xyz(&self.inner)
    }

    /// Get the number of inliers
    pub fn inliers_count(&self) -> usize {
        ffi::get_inliers_count_xyz(&self.inner)
    }
}

/// RANSAC algorithm for plane fitting with PointXYZRGB
pub struct RansacPlaneXYZRGB {
    inner: UniquePtr<ffi::RandomSampleConsensus_PointXYZRGB>,
}

impl RansacPlaneXYZRGB {
    /// Create a new RANSAC plane fitter for a PointXYZRGB cloud
    pub fn new(cloud: &PointCloudXYZRGB) -> PclResult<Self> {
        let inner = ffi::new_ransac_plane_xyzrgb(cloud.inner());
        if inner.is_null() {
            return Err(PclError::RansacError(
                "Failed to create RANSAC plane fitter".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Set the distance threshold for inlier determination
    pub fn set_distance_threshold(&mut self, threshold: f64) {
        ffi::set_distance_threshold_xyzrgb(self.inner.pin_mut(), threshold);
    }

    /// Get the current distance threshold
    pub fn distance_threshold(&self) -> f64 {
        ffi::get_distance_threshold_xyzrgb(&self.inner)
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iterations: i32) {
        ffi::set_max_iterations_xyzrgb(self.inner.pin_mut(), max_iterations);
    }

    /// Get the maximum number of iterations
    pub fn max_iterations(&self) -> i32 {
        ffi::get_max_iterations_xyzrgb(&self.inner)
    }

    /// Set the desired probability of finding the correct model
    pub fn set_probability(&mut self, probability: f64) {
        ffi::set_probability_xyzrgb(self.inner.pin_mut(), probability);
    }

    /// Get the desired probability
    pub fn probability(&self) -> f64 {
        ffi::get_probability_xyzrgb(&self.inner)
    }

    /// Compute the model using RANSAC
    pub fn compute_model(&mut self) -> PclResult<()> {
        if ffi::compute_model_xyzrgb(self.inner.pin_mut()) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to compute RANSAC model".to_string(),
            ))
        }
    }

    /// Refine the model using non-linear optimization
    pub fn refine_model(&mut self, sigma: f64, max_iterations: u32) -> PclResult<()> {
        if ffi::refine_model_xyzrgb(self.inner.pin_mut(), sigma, max_iterations) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to refine RANSAC model".to_string(),
            ))
        }
    }

    /// Get the indices of inlier points
    pub fn inliers(&self) -> Vec<i32> {
        ffi::get_inliers_xyzrgb(&self.inner)
    }

    /// Get the model coefficients
    /// For planes: [a, b, c, d] where ax + by + cz + d = 0
    pub fn model_coefficients(&self) -> Vec<f32> {
        ffi::get_model_coefficients_xyzrgb(&self.inner)
    }

    /// Get the number of inliers
    pub fn inliers_count(&self) -> usize {
        ffi::get_inliers_count_xyzrgb(&self.inner)
    }
}

/// RANSAC algorithm for sphere fitting with PointXYZRGB
pub struct RansacSphereXYZRGB {
    inner: UniquePtr<ffi::RandomSampleConsensus_PointXYZRGB>,
}

impl RansacSphereXYZRGB {
    /// Create a new RANSAC sphere fitter for a PointXYZRGB cloud
    pub fn new(cloud: &PointCloudXYZRGB) -> PclResult<Self> {
        let inner = ffi::new_ransac_sphere_xyzrgb(cloud.inner());
        if inner.is_null() {
            return Err(PclError::RansacError(
                "Failed to create RANSAC sphere fitter".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Set the distance threshold for inlier determination
    pub fn set_distance_threshold(&mut self, threshold: f64) {
        ffi::set_distance_threshold_xyzrgb(self.inner.pin_mut(), threshold);
    }

    /// Get the current distance threshold
    pub fn distance_threshold(&self) -> f64 {
        ffi::get_distance_threshold_xyzrgb(&self.inner)
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iterations: i32) {
        ffi::set_max_iterations_xyzrgb(self.inner.pin_mut(), max_iterations);
    }

    /// Get the maximum number of iterations
    pub fn max_iterations(&self) -> i32 {
        ffi::get_max_iterations_xyzrgb(&self.inner)
    }

    /// Set the desired probability of finding the correct model
    pub fn set_probability(&mut self, probability: f64) {
        ffi::set_probability_xyzrgb(self.inner.pin_mut(), probability);
    }

    /// Get the desired probability
    pub fn probability(&self) -> f64 {
        ffi::get_probability_xyzrgb(&self.inner)
    }

    /// Compute the model using RANSAC
    pub fn compute_model(&mut self) -> PclResult<()> {
        if ffi::compute_model_xyzrgb(self.inner.pin_mut()) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to compute RANSAC model".to_string(),
            ))
        }
    }

    /// Refine the model using non-linear optimization
    pub fn refine_model(&mut self, sigma: f64, max_iterations: u32) -> PclResult<()> {
        if ffi::refine_model_xyzrgb(self.inner.pin_mut(), sigma, max_iterations) {
            Ok(())
        } else {
            Err(PclError::RansacError(
                "Failed to refine RANSAC model".to_string(),
            ))
        }
    }

    /// Get the indices of inlier points
    pub fn inliers(&self) -> Vec<i32> {
        ffi::get_inliers_xyzrgb(&self.inner)
    }

    /// Get the model coefficients
    /// For spheres: [center_x, center_y, center_z, radius]
    pub fn model_coefficients(&self) -> Vec<f32> {
        ffi::get_model_coefficients_xyzrgb(&self.inner)
    }

    /// Get the number of inliers
    pub fn inliers_count(&self) -> usize {
        ffi::get_inliers_count_xyzrgb(&self.inner)
    }
}
