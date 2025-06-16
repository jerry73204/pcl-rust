//! SIFT (Scale Invariant Feature Transform) keypoint detector implementation
//!
//! The SIFT algorithm detects distinctive keypoints that are invariant to scaling,
//! rotation, and illumination changes. It requires intensity information and
//! outputs keypoints with scale information.

use crate::common::PointCloudXYZI;
use crate::error::{PclError, PclResult};
use crate::keypoints::{KeypointBuilder, KeypointDetector, PointWithScale};
use crate::search::KdTreeXYZI;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Point cloud type for PointWithScale
/// Note: This is a simplified wrapper since PointWithScale clouds
/// require special handling in the FFI layer
pub struct PointCloudWithScale {
    inner: UniquePtr<ffi::PointCloud_PointWithScale>,
}

impl PointCloudWithScale {
    /// Create a new point cloud from a unique pointer
    pub fn from_unique_ptr(inner: UniquePtr<ffi::PointCloud_PointWithScale>) -> Self {
        Self { inner }
    }

    /// Get access to the raw FFI object
    pub fn as_raw(&self) -> &ffi::PointCloud_PointWithScale {
        &self.inner
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        // For now, assume non-null means non-empty
        // This would need proper FFI implementation for PointWithScale clouds
        false
    }
}

/// SIFT keypoint detector for PointXYZI clouds
///
/// The SIFT (Scale Invariant Feature Transform) detector identifies keypoints
/// that are stable across different scales. It requires intensity information
/// and outputs keypoints with associated scale values.
pub struct SiftKeypoint {
    inner: UniquePtr<ffi::SIFTKeypoint_PointXYZI_PointWithScale>,
}

impl SiftKeypoint {
    /// Create a new SIFT keypoint detector
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_sift_keypoint_xyzi();
        if inner.is_null() {
            Err(PclError::InvalidState {
                message: "Failed to create SIFT detector".to_string(),
                expected_state: "valid SIFT object".to_string(),
                actual_state: "null pointer".to_string(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the search method for finding neighbors
    pub fn set_search_method(&mut self, kdtree: &KdTreeXYZI) -> PclResult<()> {
        ffi::set_search_method_sift_xyzi(self.inner.pin_mut(), kdtree.as_raw());
        Ok(())
    }

    /// Set the scale parameters for SIFT detection
    ///
    /// # Arguments
    /// * `min_scale` - Minimum scale to detect features at
    /// * `nr_octaves` - Number of octaves to compute
    /// * `nr_scales_per_octave` - Number of scales per octave
    pub fn set_scales(
        &mut self,
        min_scale: f32,
        nr_octaves: f32,
        nr_scales_per_octave: i32,
    ) -> PclResult<()> {
        if min_scale <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum scale",
                "min_scale",
                "positive value",
                min_scale.to_string(),
            ));
        }

        if nr_octaves <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid number of octaves",
                "nr_octaves",
                "positive value",
                nr_octaves.to_string(),
            ));
        }

        if nr_scales_per_octave <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid scales per octave",
                "nr_scales_per_octave",
                "positive integer",
                nr_scales_per_octave.to_string(),
            ));
        }

        ffi::set_scales_sift_xyzi(
            self.inner.pin_mut(),
            min_scale,
            nr_octaves,
            nr_scales_per_octave,
        );
        Ok(())
    }

    /// Set the minimum contrast threshold for keypoint detection
    ///
    /// Keypoints with contrast below this threshold will be rejected
    pub fn set_minimum_contrast(&mut self, min_contrast: f32) -> PclResult<()> {
        if min_contrast < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum contrast",
                "min_contrast",
                "non-negative value",
                min_contrast.to_string(),
            ));
        }
        ffi::set_minimum_contrast_sift_xyzi(self.inner.pin_mut(), min_contrast);
        Ok(())
    }
}

impl Default for SiftKeypoint {
    fn default() -> Self {
        Self::new().expect("Failed to create default SIFT detector")
    }
}

impl KeypointDetector<PointCloudXYZI, PointCloudWithScale> for SiftKeypoint {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZI) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_sift_xyzi(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn compute(&mut self) -> PclResult<PointCloudWithScale> {
        let result = ffi::compute_sift_xyzi(self.inner.pin_mut());
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "SIFT keypoint detection failed".to_string(),
                expected_state: "successful keypoint detection".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudWithScale::from_unique_ptr(result))
        }
    }
}

/// Builder for SIFT configuration
pub struct SiftKeypointBuilder {
    min_scale: Option<f32>,
    nr_octaves: Option<f32>,
    nr_scales_per_octave: Option<i32>,
    minimum_contrast: Option<f32>,
}

impl SiftKeypointBuilder {
    /// Create a new SIFT builder
    pub fn new() -> Self {
        Self {
            min_scale: None,
            nr_octaves: None,
            nr_scales_per_octave: None,
            minimum_contrast: None,
        }
    }

    /// Set the scale parameters
    pub fn scales(mut self, min_scale: f32, nr_octaves: f32, nr_scales_per_octave: i32) -> Self {
        self.min_scale = Some(min_scale);
        self.nr_octaves = Some(nr_octaves);
        self.nr_scales_per_octave = Some(nr_scales_per_octave);
        self
    }

    /// Set the minimum contrast threshold
    pub fn minimum_contrast(mut self, min_contrast: f32) -> Self {
        self.minimum_contrast = Some(min_contrast);
        self
    }
}

impl Default for SiftKeypointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl KeypointBuilder<SiftKeypoint> for SiftKeypointBuilder {
    fn build(self) -> PclResult<SiftKeypoint> {
        let mut sift = SiftKeypoint::new()?;

        if let (Some(min_scale), Some(nr_octaves), Some(nr_scales_per_octave)) =
            (self.min_scale, self.nr_octaves, self.nr_scales_per_octave)
        {
            sift.set_scales(min_scale, nr_octaves, nr_scales_per_octave)?;
        }

        if let Some(min_contrast) = self.minimum_contrast {
            sift.set_minimum_contrast(min_contrast)?;
        }

        Ok(sift)
    }
}

/// Helper functions for working with keypoint data
impl PointWithScale {
    /// Get the coordinates as a vector [x, y, z, scale]
    pub fn coords(&self) -> Vec<f32> {
        vec![self.x, self.y, self.z, self.scale]
    }

    /// Get the spatial coordinates as a vector [x, y, z]
    pub fn spatial_coords(&self) -> Vec<f32> {
        vec![self.x, self.y, self.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sift_creation() {
        let sift = SiftKeypoint::new();
        assert!(sift.is_ok());
    }

    #[test]
    fn test_sift_builder() {
        let sift = SiftKeypointBuilder::new()
            .scales(0.01, 3.0, 4)
            .minimum_contrast(0.03)
            .build();

        assert!(sift.is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut sift = SiftKeypoint::new().unwrap();

        assert!(sift.set_scales(-1.0, 3.0, 4).is_err());
        assert!(sift.set_scales(0.01, -1.0, 4).is_err());
        assert!(sift.set_scales(0.01, 3.0, -1).is_err());
        assert!(sift.set_minimum_contrast(-1.0).is_err());
    }

    #[test]
    fn test_point_with_scale() {
        let point = PointWithScale {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            scale: 0.5,
        };

        assert_eq!(point.coords(), vec![1.0, 2.0, 3.0, 0.5]);
        assert_eq!(point.spatial_coords(), vec![1.0, 2.0, 3.0]);

        let point_from_vec = PointWithScale::from(vec![1.0, 2.0, 3.0, 0.5]);
        assert_eq!(point_from_vec.x, 1.0);
        assert_eq!(point_from_vec.scale, 0.5);
    }
}
