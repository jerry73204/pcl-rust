//! Harris 3D keypoint detector implementation
//!
//! The Harris 3D algorithm is an extension of the 2D Harris corner detector
//! to 3D point clouds. It detects keypoints that represent corners and edges
//! in the 3D structure.

use crate::common::{PointCloudXYZ, PointCloudXYZI};
use crate::error::{PclError, PclResult};
use crate::keypoints::{KeypointBuilder, KeypointDetector};
use crate::search::KdTreeXYZ;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Harris 3D keypoint detector for PointXYZ clouds
///
/// The Harris 3D detector identifies corner-like features in 3D point clouds
/// by analyzing the local surface variation. It outputs keypoints as PointXYZI
/// where the intensity represents the Harris response value.
pub struct Harris3D {
    inner: UniquePtr<ffi::HarrisKeypoint3D_PointXYZ_PointXYZI>,
}

impl Harris3D {
    /// Create a new Harris 3D keypoint detector
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_harris_3d_xyz();
        if inner.is_null() {
            Err(PclError::InvalidState {
                message: "Failed to create Harris 3D detector".to_string(),
                expected_state: "valid Harris 3D object".to_string(),
                actual_state: "null pointer".to_string(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the search method for finding neighbors
    pub fn set_search_method(&mut self, kdtree: &KdTreeXYZ) -> PclResult<()> {
        ffi::set_search_method_harris_3d_xyz(self.inner.pin_mut(), kdtree.inner());
        Ok(())
    }

    /// Set the radius for local surface analysis
    pub fn set_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid radius",
                "radius",
                "positive value",
                radius.to_string(),
            ));
        }
        ffi::set_radius_harris_3d_xyz(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Set the Harris response threshold
    ///
    /// Only keypoints with Harris response above this threshold will be detected
    pub fn set_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid threshold",
                "threshold",
                "non-negative value",
                threshold.to_string(),
            ));
        }
        ffi::set_threshold_harris_3d_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Enable or disable non-maximum suppression
    ///
    /// When enabled, suppresses keypoints that are not local maxima
    pub fn set_non_max_suppression(&mut self, suppress: bool) -> PclResult<()> {
        ffi::set_non_max_suppression_harris_3d_xyz(self.inner.pin_mut(), suppress);
        Ok(())
    }

    /// Enable or disable keypoint refinement
    ///
    /// When enabled, refines keypoint positions to sub-point accuracy
    pub fn set_refine(&mut self, refine: bool) -> PclResult<()> {
        ffi::set_refine_harris_3d_xyz(self.inner.pin_mut(), refine);
        Ok(())
    }
}

impl Default for Harris3D {
    fn default() -> Self {
        Self::new().expect("Failed to create default Harris 3D detector")
    }
}

impl KeypointDetector<PointCloudXYZ, PointCloudXYZI> for Harris3D {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_harris_3d_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn compute(&mut self) -> PclResult<PointCloudXYZI> {
        let result = ffi::compute_harris_3d_xyz(self.inner.pin_mut());
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "Harris 3D keypoint detection failed".to_string(),
                expected_state: "successful keypoint detection".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZI::from_unique_ptr(result))
        }
    }
}

/// Builder for Harris 3D configuration
pub struct Harris3DBuilder {
    radius: Option<f64>,
    threshold: Option<f32>,
    non_max_suppression: Option<bool>,
    refine: Option<bool>,
}

impl Harris3DBuilder {
    /// Create a new Harris 3D builder
    pub fn new() -> Self {
        Self {
            radius: None,
            threshold: None,
            non_max_suppression: None,
            refine: None,
        }
    }

    /// Set the radius for local surface analysis
    pub fn radius(mut self, radius: f64) -> Self {
        self.radius = Some(radius);
        self
    }

    /// Set the Harris response threshold
    pub fn threshold(mut self, threshold: f32) -> Self {
        self.threshold = Some(threshold);
        self
    }

    /// Enable or disable non-maximum suppression
    pub fn non_max_suppression(mut self, suppress: bool) -> Self {
        self.non_max_suppression = Some(suppress);
        self
    }

    /// Enable or disable keypoint refinement
    pub fn refine(mut self, refine: bool) -> Self {
        self.refine = Some(refine);
        self
    }
}

impl Default for Harris3DBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl KeypointBuilder<Harris3D> for Harris3DBuilder {
    fn build(self) -> PclResult<Harris3D> {
        let mut harris = Harris3D::new()?;

        if let Some(radius) = self.radius {
            harris.set_radius(radius)?;
        }

        if let Some(threshold) = self.threshold {
            harris.set_threshold(threshold)?;
        }

        if let Some(suppress) = self.non_max_suppression {
            harris.set_non_max_suppression(suppress)?;
        }

        if let Some(refine) = self.refine {
            harris.set_refine(refine)?;
        }

        Ok(harris)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_harris_creation() {
        let harris = Harris3D::new();
        assert!(harris.is_ok());
    }

    #[test]
    fn test_harris_builder() {
        let harris = Harris3DBuilder::new()
            .radius(0.05)
            .threshold(0.01)
            .non_max_suppression(true)
            .refine(false)
            .build();

        assert!(harris.is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut harris = Harris3D::new().unwrap();

        assert!(harris.set_radius(-1.0).is_err());
        assert!(harris.set_threshold(-1.0).is_err());
    }
}
