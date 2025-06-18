//! Geometric model implementations for sample consensus
//!
//! This module provides safe wrappers for PCL's geometric models used in
//! sample consensus algorithms like RANSAC.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use pcl_sys::UniquePtr;
use pcl_sys::ffi;

/// Plane model for PointXYZ clouds
///
/// Represents a 3D plane with equation ax + by + cz + d = 0
pub struct PlaneModelXYZ {
    inner: UniquePtr<ffi::SampleConsensusModelPlane_PointXYZ>,
}

impl PlaneModelXYZ {
    /// Create a new plane model for the given point cloud
    pub fn new(cloud: &PointCloudXYZ) -> PclResult<Self> {
        let inner = ffi::new_sac_model_plane_xyz(cloud.inner());
        if inner.is_null() {
            return Err(PclError::ModelError(
                "Failed to create plane model".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Compute model coefficients from a set of sample points
    pub fn compute_model_coefficients(&mut self, samples: &[i32]) -> PclResult<Vec<f32>> {
        let sample_vec: Vec<i32> = samples.to_vec();
        let mut coefficients = Vec::new();
        if ffi::compute_model_coefficients_plane_xyz(
            self.inner.pin_mut(),
            &sample_vec,
            &mut coefficients,
        ) {
            Ok(coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to compute plane model coefficients".to_string(),
            ))
        }
    }

    /// Get distances from all points in the cloud to the model
    pub fn get_distances_to_model(&mut self, coefficients: &[f32]) -> Vec<f64> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut distances = Vec::new();
        ffi::get_distances_to_model_plane_xyz(self.inner.pin_mut(), &coeff_vec, &mut distances);
        distances
    }

    /// Select points within a distance threshold from the model
    pub fn select_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> Vec<i32> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::select_within_distance_plane_xyz(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Count points within a distance threshold from the model
    pub fn count_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> i32 {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::count_within_distance_plane_xyz(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Optimize model coefficients using a set of inlier points
    pub fn optimize_model_coefficients(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
    ) -> PclResult<Vec<f32>> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut optimized_coefficients = Vec::new();
        if ffi::optimize_model_coefficients_plane_xyz(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            &mut optimized_coefficients,
        ) {
            Ok(optimized_coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to optimize plane model coefficients".to_string(),
            ))
        }
    }

    /// Project points onto the plane surface
    pub fn project_points(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
        copy_data_fields: bool,
    ) -> PclResult<PointCloudXYZ> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let projected = ffi::project_points_plane_xyz(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            copy_data_fields,
        );
        if projected.is_null() {
            return Err(PclError::ModelError(
                "Failed to project points onto plane".to_string(),
            ));
        }
        Ok(PointCloudXYZ::from_unique_ptr(projected))
    }
}

/// Sphere model for PointXYZ clouds
///
/// Represents a 3D sphere with center (x, y, z) and radius r
pub struct SphereModelXYZ {
    inner: UniquePtr<ffi::SampleConsensusModelSphere_PointXYZ>,
}

impl SphereModelXYZ {
    /// Create a new sphere model for the given point cloud
    pub fn new(cloud: &PointCloudXYZ) -> PclResult<Self> {
        let inner = ffi::new_sac_model_sphere_xyz(cloud.inner());
        if inner.is_null() {
            return Err(PclError::ModelError(
                "Failed to create sphere model".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Compute model coefficients from a set of sample points
    pub fn compute_model_coefficients(&mut self, samples: &[i32]) -> PclResult<Vec<f32>> {
        let sample_vec: Vec<i32> = samples.to_vec();
        let mut coefficients = Vec::new();
        if ffi::compute_model_coefficients_sphere_xyz(
            self.inner.pin_mut(),
            &sample_vec,
            &mut coefficients,
        ) {
            Ok(coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to compute sphere model coefficients".to_string(),
            ))
        }
    }

    /// Get distances from all points in the cloud to the model surface
    pub fn get_distances_to_model(&mut self, coefficients: &[f32]) -> Vec<f64> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut distances = Vec::new();
        ffi::get_distances_to_model_sphere_xyz(self.inner.pin_mut(), &coeff_vec, &mut distances);
        distances
    }

    /// Select points within a distance threshold from the model surface
    pub fn select_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> Vec<i32> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::select_within_distance_sphere_xyz(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Count points within a distance threshold from the model surface
    pub fn count_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> i32 {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::count_within_distance_sphere_xyz(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Optimize model coefficients using a set of inlier points
    pub fn optimize_model_coefficients(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
    ) -> PclResult<Vec<f32>> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut optimized_coefficients = Vec::new();
        if ffi::optimize_model_coefficients_sphere_xyz(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            &mut optimized_coefficients,
        ) {
            Ok(optimized_coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to optimize sphere model coefficients".to_string(),
            ))
        }
    }

    /// Set radius limits for sphere fitting
    pub fn set_radius_limits(&mut self, min_radius: f64, max_radius: f64) {
        ffi::set_radius_limits_sphere_xyz(self.inner.pin_mut(), min_radius, max_radius);
    }

    /// Get radius limits for sphere fitting
    pub fn get_radius_limits(&mut self) -> (f64, f64) {
        let mut min_radius = 0.0;
        let mut max_radius = 0.0;
        ffi::get_radius_limits_sphere_xyz(self.inner.pin_mut(), &mut min_radius, &mut max_radius);
        (min_radius, max_radius)
    }

    /// Project points onto the sphere surface
    pub fn project_points(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
        copy_data_fields: bool,
    ) -> PclResult<PointCloudXYZ> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let projected = ffi::project_points_sphere_xyz(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            copy_data_fields,
        );
        if projected.is_null() {
            return Err(PclError::ModelError(
                "Failed to project points onto sphere".to_string(),
            ));
        }
        Ok(PointCloudXYZ::from_unique_ptr(projected))
    }
}

/// Plane model for PointXYZRGB clouds
///
/// Represents a 3D plane with equation ax + by + cz + d = 0
pub struct PlaneModelXYZRGB {
    inner: UniquePtr<ffi::SampleConsensusModelPlane_PointXYZRGB>,
}

impl PlaneModelXYZRGB {
    /// Create a new plane model for the given point cloud
    pub fn new(cloud: &PointCloudXYZRGB) -> PclResult<Self> {
        let inner = ffi::new_sac_model_plane_xyzrgb(cloud.inner());
        if inner.is_null() {
            return Err(PclError::ModelError(
                "Failed to create plane model".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Compute model coefficients from a set of sample points
    pub fn compute_model_coefficients(&mut self, samples: &[i32]) -> PclResult<Vec<f32>> {
        let sample_vec: Vec<i32> = samples.to_vec();
        let mut coefficients = Vec::new();
        if ffi::compute_model_coefficients_plane_xyzrgb(
            self.inner.pin_mut(),
            &sample_vec,
            &mut coefficients,
        ) {
            Ok(coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to compute plane model coefficients".to_string(),
            ))
        }
    }

    /// Get distances from all points in the cloud to the model
    pub fn get_distances_to_model(&mut self, coefficients: &[f32]) -> Vec<f64> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut distances = Vec::new();
        ffi::get_distances_to_model_plane_xyzrgb(self.inner.pin_mut(), &coeff_vec, &mut distances);
        distances
    }

    /// Select points within a distance threshold from the model
    pub fn select_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> Vec<i32> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::select_within_distance_plane_xyzrgb(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Count points within a distance threshold from the model
    pub fn count_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> i32 {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::count_within_distance_plane_xyzrgb(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Optimize model coefficients using a set of inlier points
    pub fn optimize_model_coefficients(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
    ) -> PclResult<Vec<f32>> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut optimized_coefficients = Vec::new();
        if ffi::optimize_model_coefficients_plane_xyzrgb(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            &mut optimized_coefficients,
        ) {
            Ok(optimized_coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to optimize plane model coefficients".to_string(),
            ))
        }
    }

    /// Project points onto the plane surface
    pub fn project_points(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
        copy_data_fields: bool,
    ) -> PclResult<PointCloudXYZRGB> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let projected = ffi::project_points_plane_xyzrgb(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            copy_data_fields,
        );
        if projected.is_null() {
            return Err(PclError::ModelError(
                "Failed to project points onto plane".to_string(),
            ));
        }
        Ok(PointCloudXYZRGB::from_unique_ptr(projected))
    }
}

/// Sphere model for PointXYZRGB clouds
///
/// Represents a 3D sphere with center (x, y, z) and radius r
pub struct SphereModelXYZRGB {
    inner: UniquePtr<ffi::SampleConsensusModelSphere_PointXYZRGB>,
}

impl SphereModelXYZRGB {
    /// Create a new sphere model for the given point cloud
    pub fn new(cloud: &PointCloudXYZRGB) -> PclResult<Self> {
        let inner = ffi::new_sac_model_sphere_xyzrgb(cloud.inner());
        if inner.is_null() {
            return Err(PclError::ModelError(
                "Failed to create sphere model".to_string(),
            ));
        }
        Ok(Self { inner })
    }

    /// Compute model coefficients from a set of sample points
    pub fn compute_model_coefficients(&mut self, samples: &[i32]) -> PclResult<Vec<f32>> {
        let sample_vec: Vec<i32> = samples.to_vec();
        let mut coefficients = Vec::new();
        if ffi::compute_model_coefficients_sphere_xyzrgb(
            self.inner.pin_mut(),
            &sample_vec,
            &mut coefficients,
        ) {
            Ok(coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to compute sphere model coefficients".to_string(),
            ))
        }
    }

    /// Get distances from all points in the cloud to the model surface
    pub fn get_distances_to_model(&mut self, coefficients: &[f32]) -> Vec<f64> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut distances = Vec::new();
        ffi::get_distances_to_model_sphere_xyzrgb(self.inner.pin_mut(), &coeff_vec, &mut distances);
        distances
    }

    /// Select points within a distance threshold from the model surface
    pub fn select_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> Vec<i32> {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::select_within_distance_sphere_xyzrgb(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Count points within a distance threshold from the model surface
    pub fn count_within_distance(&mut self, coefficients: &[f32], threshold: f64) -> i32 {
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        ffi::count_within_distance_sphere_xyzrgb(self.inner.pin_mut(), &coeff_vec, threshold)
    }

    /// Optimize model coefficients using a set of inlier points
    pub fn optimize_model_coefficients(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
    ) -> PclResult<Vec<f32>> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let mut optimized_coefficients = Vec::new();
        if ffi::optimize_model_coefficients_sphere_xyzrgb(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            &mut optimized_coefficients,
        ) {
            Ok(optimized_coefficients)
        } else {
            Err(PclError::ModelError(
                "Failed to optimize sphere model coefficients".to_string(),
            ))
        }
    }

    /// Set radius limits for sphere fitting
    pub fn set_radius_limits(&mut self, min_radius: f64, max_radius: f64) {
        ffi::set_radius_limits_sphere_xyzrgb(self.inner.pin_mut(), min_radius, max_radius);
    }

    /// Get radius limits for sphere fitting
    pub fn get_radius_limits(&mut self) -> (f64, f64) {
        let mut min_radius = 0.0;
        let mut max_radius = 0.0;
        ffi::get_radius_limits_sphere_xyzrgb(
            self.inner.pin_mut(),
            &mut min_radius,
            &mut max_radius,
        );
        (min_radius, max_radius)
    }

    /// Project points onto the sphere surface
    pub fn project_points(
        &mut self,
        inliers: &[i32],
        coefficients: &[f32],
        copy_data_fields: bool,
    ) -> PclResult<PointCloudXYZRGB> {
        let inliers_vec: Vec<i32> = inliers.to_vec();
        let coeff_vec: Vec<f32> = coefficients.to_vec();
        let projected = ffi::project_points_sphere_xyzrgb(
            self.inner.pin_mut(),
            &inliers_vec,
            &coeff_vec,
            copy_data_fields,
        );
        if projected.is_null() {
            return Err(PclError::ModelError(
                "Failed to project points onto sphere".to_string(),
            ));
        }
        Ok(PointCloudXYZRGB::from_unique_ptr(projected))
    }
}
