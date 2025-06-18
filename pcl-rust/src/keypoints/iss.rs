//! ISS (Intrinsic Shape Signatures) 3D keypoint detector implementation
//!
//! The ISS 3D algorithm detects keypoints by analyzing the eigenvalues of the
//! covariance matrix computed from the local neighborhood of each point.
//! It identifies distinctive features that are stable across different views.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::keypoints::{KeypointBuilder, KeypointDetector};
use crate::search::KdTreeXYZ;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// ISS 3D keypoint detector for PointXYZ clouds
///
/// The ISS (Intrinsic Shape Signatures) detector identifies keypoints by
/// analyzing the local geometric structure around each point. It computes
/// eigenvalues of the covariance matrix and uses them to determine distinctive points.
pub struct Iss3D {
    inner: UniquePtr<ffi::ISSKeypoint3D_PointXYZ_PointXYZ>,
}

impl Iss3D {
    /// Create a new ISS 3D keypoint detector
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_iss_3d_xyz();
        if inner.is_null() {
            Err(PclError::InvalidState {
                message: "Failed to create ISS 3D detector".to_string(),
                expected_state: "valid ISS 3D object".to_string(),
                actual_state: "null pointer".to_string(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the search method for finding neighbors
    pub fn set_search_method(&mut self, kdtree: &KdTreeXYZ) -> PclResult<()> {
        ffi::set_search_method_iss_3d_xyz(self.inner.pin_mut(), kdtree.inner());
        Ok(())
    }

    /// Set the radius for salient feature detection
    ///
    /// This radius is used to compute the covariance matrix for each point
    pub fn set_salient_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid salient radius",
                "radius",
                "positive value",
                radius.to_string(),
            ));
        }
        ffi::set_salient_radius_iss_3d_xyz(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Set the radius for non-maximum suppression
    ///
    /// Keypoints within this radius of a stronger keypoint will be suppressed
    pub fn set_non_max_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid non-max radius",
                "radius",
                "positive value",
                radius.to_string(),
            ));
        }
        ffi::set_non_max_radius_iss_3d_xyz(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Set the threshold for the ratio between the second and first eigenvalues
    ///
    /// Points are considered salient if λ2/λ1 < threshold21
    pub fn set_threshold21(&mut self, threshold: f64) -> PclResult<()> {
        if threshold <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid threshold21",
                "threshold",
                "positive value",
                threshold.to_string(),
            ));
        }
        ffi::set_threshold21_iss_3d_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Set the threshold for the ratio between the third and second eigenvalues
    ///
    /// Points are considered salient if λ3/λ2 < threshold32
    pub fn set_threshold32(&mut self, threshold: f64) -> PclResult<()> {
        if threshold <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid threshold32",
                "threshold",
                "positive value",
                threshold.to_string(),
            ));
        }
        ffi::set_threshold32_iss_3d_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Set the minimum number of neighbors required for keypoint detection
    pub fn set_min_neighbors(&mut self, min_neighbors: i32) -> PclResult<()> {
        if min_neighbors < 1 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum neighbors count",
                "min_neighbors",
                "positive integer",
                min_neighbors.to_string(),
            ));
        }
        ffi::set_min_neighbors_iss_3d_xyz(self.inner.pin_mut(), min_neighbors);
        Ok(())
    }
}

impl Default for Iss3D {
    fn default() -> Self {
        Self::new().expect("Failed to create default ISS 3D detector")
    }
}

impl KeypointDetector<PointCloudXYZ, PointCloudXYZ> for Iss3D {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_iss_3d_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn compute(&mut self) -> PclResult<PointCloudXYZ> {
        let result = ffi::compute_iss_3d_xyz(self.inner.pin_mut());
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "ISS 3D keypoint detection failed".to_string(),
                expected_state: "successful keypoint detection".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }
}

/// Builder for ISS 3D configuration
pub struct Iss3DBuilder {
    salient_radius: Option<f64>,
    non_max_radius: Option<f64>,
    threshold21: Option<f64>,
    threshold32: Option<f64>,
    min_neighbors: Option<i32>,
}

impl Iss3DBuilder {
    /// Create a new ISS 3D builder
    pub fn new() -> Self {
        Self {
            salient_radius: None,
            non_max_radius: None,
            threshold21: None,
            threshold32: None,
            min_neighbors: None,
        }
    }

    /// Set the radius for salient feature detection
    pub fn salient_radius(mut self, radius: f64) -> Self {
        self.salient_radius = Some(radius);
        self
    }

    /// Set the radius for non-maximum suppression
    pub fn non_max_radius(mut self, radius: f64) -> Self {
        self.non_max_radius = Some(radius);
        self
    }

    /// Set the threshold for eigenvalue ratio λ2/λ1
    pub fn threshold21(mut self, threshold: f64) -> Self {
        self.threshold21 = Some(threshold);
        self
    }

    /// Set the threshold for eigenvalue ratio λ3/λ2
    pub fn threshold32(mut self, threshold: f64) -> Self {
        self.threshold32 = Some(threshold);
        self
    }

    /// Set the minimum number of neighbors
    pub fn min_neighbors(mut self, min_neighbors: i32) -> Self {
        self.min_neighbors = Some(min_neighbors);
        self
    }
}

impl Default for Iss3DBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl KeypointBuilder<Iss3D> for Iss3DBuilder {
    fn build(self) -> PclResult<Iss3D> {
        let mut iss = Iss3D::new()?;

        if let Some(radius) = self.salient_radius {
            iss.set_salient_radius(radius)?;
        }

        if let Some(radius) = self.non_max_radius {
            iss.set_non_max_radius(radius)?;
        }

        if let Some(threshold) = self.threshold21 {
            iss.set_threshold21(threshold)?;
        }

        if let Some(threshold) = self.threshold32 {
            iss.set_threshold32(threshold)?;
        }

        if let Some(min_neighbors) = self.min_neighbors {
            iss.set_min_neighbors(min_neighbors)?;
        }

        Ok(iss)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iss_creation() {
        let iss = Iss3D::new();
        assert!(iss.is_ok());
    }

    #[test]
    fn test_iss_builder() {
        let iss = Iss3DBuilder::new()
            .salient_radius(0.05)
            .non_max_radius(0.02)
            .threshold21(0.975)
            .threshold32(0.975)
            .min_neighbors(5)
            .build();

        assert!(iss.is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut iss = Iss3D::new().unwrap();

        assert!(iss.set_salient_radius(-1.0).is_err());
        assert!(iss.set_non_max_radius(-1.0).is_err());
        assert!(iss.set_threshold21(-1.0).is_err());
        assert!(iss.set_threshold32(-1.0).is_err());
        assert!(iss.set_min_neighbors(0).is_err());
    }
}
