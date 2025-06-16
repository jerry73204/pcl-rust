//! Progressive morphological filter for ground extraction
//!
//! This module provides the progressive morphological filter algorithm
//! for extracting ground points from point clouds.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Progressive morphological filter for PointXYZ clouds
///
/// This algorithm is particularly useful for extracting ground points from
/// airborne LiDAR data by applying morphological operations at progressively
/// larger window sizes.
pub struct ProgressiveMorphologicalFilterXYZ {
    inner: UniquePtr<ffi::ProgressiveMorphologicalFilter_PointXYZ>,
}

impl ProgressiveMorphologicalFilterXYZ {
    /// Create a new progressive morphological filter instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_progressive_morphological_filter_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "ProgressiveMorphologicalFilter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_pmf_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Set the maximum window size
    pub fn set_max_window_size(&mut self, size: i32) -> PclResult<()> {
        if size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum window size",
                "size",
                "positive value",
                &size.to_string(),
            ));
        }
        ffi::set_max_window_size_pmf_xyz(self.inner.pin_mut(), size);
        Ok(())
    }

    /// Get the maximum window size
    pub fn max_window_size(&self) -> i32 {
        ffi::get_max_window_size_pmf_xyz(&self.inner)
    }

    /// Set the slope value
    pub fn set_slope(&mut self, slope: f32) -> PclResult<()> {
        if slope < 0.0 || slope > 1.0 {
            return Err(PclError::invalid_parameters(
                "Invalid slope",
                "slope",
                "value between 0 and 1",
                &slope.to_string(),
            ));
        }
        ffi::set_slope_pmf_xyz(self.inner.pin_mut(), slope);
        Ok(())
    }

    /// Get the slope value
    pub fn slope(&self) -> f32 {
        ffi::get_slope_pmf_xyz(&self.inner)
    }

    /// Set the maximum distance
    pub fn set_max_distance(&mut self, distance: f32) -> PclResult<()> {
        if distance <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum distance",
                "distance",
                "positive value",
                &distance.to_string(),
            ));
        }
        ffi::set_max_distance_pmf_xyz(self.inner.pin_mut(), distance);
        Ok(())
    }

    /// Get the maximum distance
    pub fn max_distance(&self) -> f32 {
        ffi::get_max_distance_pmf_xyz(&self.inner)
    }

    /// Set the initial distance
    pub fn set_initial_distance(&mut self, distance: f32) -> PclResult<()> {
        if distance < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid initial distance",
                "distance",
                "non-negative value",
                &distance.to_string(),
            ));
        }
        ffi::set_initial_distance_pmf_xyz(self.inner.pin_mut(), distance);
        Ok(())
    }

    /// Get the initial distance
    pub fn initial_distance(&self) -> f32 {
        ffi::get_initial_distance_pmf_xyz(&self.inner)
    }

    /// Set the cell size
    pub fn set_cell_size(&mut self, size: f32) -> PclResult<()> {
        if size <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid cell size",
                "size",
                "positive value",
                &size.to_string(),
            ));
        }
        ffi::set_cell_size_pmf_xyz(self.inner.pin_mut(), size);
        Ok(())
    }

    /// Get the cell size
    pub fn cell_size(&self) -> f32 {
        ffi::get_cell_size_pmf_xyz(&self.inner)
    }

    /// Set the base value
    pub fn set_base(&mut self, base: f32) -> PclResult<()> {
        if base <= 1.0 {
            return Err(PclError::invalid_parameters(
                "Invalid base",
                "base",
                "value greater than 1",
                &base.to_string(),
            ));
        }
        ffi::set_base_pmf_xyz(self.inner.pin_mut(), base);
        Ok(())
    }

    /// Get the base value
    pub fn base(&self) -> f32 {
        ffi::get_base_pmf_xyz(&self.inner)
    }

    /// Set whether to use exponential growth
    pub fn set_exponential(&mut self, exponential: bool) {
        ffi::set_exponential_pmf_xyz(self.inner.pin_mut(), exponential);
    }

    /// Get whether exponential growth is used
    pub fn exponential(&self) -> bool {
        ffi::get_exponential_pmf_xyz(&self.inner)
    }

    /// Extract ground points
    pub fn extract_ground(&mut self) -> PclResult<Vec<i32>> {
        let result = ffi::extract_pmf_xyz(self.inner.pin_mut());
        Ok(result)
    }
}

impl Default for ProgressiveMorphologicalFilterXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default ProgressiveMorphologicalFilterXYZ")
    }
}

/// Builder for ProgressiveMorphologicalFilterXYZ configuration
pub struct ProgressiveMorphologicalFilterXYZBuilder {
    max_window_size: Option<i32>,
    slope: Option<f32>,
    max_distance: Option<f32>,
    initial_distance: Option<f32>,
    cell_size: Option<f32>,
    base: Option<f32>,
    exponential: Option<bool>,
}

impl ProgressiveMorphologicalFilterXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            max_window_size: None,
            slope: None,
            max_distance: None,
            initial_distance: None,
            cell_size: None,
            base: None,
            exponential: None,
        }
    }

    /// Set the maximum window size
    pub fn max_window_size(mut self, size: i32) -> Self {
        self.max_window_size = Some(size);
        self
    }

    /// Set the slope
    pub fn slope(mut self, slope: f32) -> Self {
        self.slope = Some(slope);
        self
    }

    /// Set the maximum distance
    pub fn max_distance(mut self, distance: f32) -> Self {
        self.max_distance = Some(distance);
        self
    }

    /// Set the initial distance
    pub fn initial_distance(mut self, distance: f32) -> Self {
        self.initial_distance = Some(distance);
        self
    }

    /// Set the cell size
    pub fn cell_size(mut self, size: f32) -> Self {
        self.cell_size = Some(size);
        self
    }

    /// Set the base
    pub fn base(mut self, base: f32) -> Self {
        self.base = Some(base);
        self
    }

    /// Set whether to use exponential growth
    pub fn exponential(mut self, exponential: bool) -> Self {
        self.exponential = Some(exponential);
        self
    }

    /// Build the ProgressiveMorphologicalFilterXYZ instance
    pub fn build(self) -> PclResult<ProgressiveMorphologicalFilterXYZ> {
        let mut pmf = ProgressiveMorphologicalFilterXYZ::new()?;

        if let Some(size) = self.max_window_size {
            pmf.set_max_window_size(size)?;
        }
        if let Some(slope) = self.slope {
            pmf.set_slope(slope)?;
        }
        if let Some(distance) = self.max_distance {
            pmf.set_max_distance(distance)?;
        }
        if let Some(distance) = self.initial_distance {
            pmf.set_initial_distance(distance)?;
        }
        if let Some(size) = self.cell_size {
            pmf.set_cell_size(size)?;
        }
        if let Some(base) = self.base {
            pmf.set_base(base)?;
        }
        if let Some(exponential) = self.exponential {
            pmf.set_exponential(exponential);
        }

        Ok(pmf)
    }
}

impl Default for ProgressiveMorphologicalFilterXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pmf_creation() {
        let pmf = ProgressiveMorphologicalFilterXYZ::new();
        assert!(pmf.is_ok());
    }

    #[test]
    fn test_pmf_builder() {
        let pmf = ProgressiveMorphologicalFilterXYZBuilder::new()
            .max_window_size(33)
            .slope(0.7)
            .max_distance(10.0)
            .initial_distance(0.15)
            .cell_size(1.0)
            .base(2.0)
            .exponential(true)
            .build();

        assert!(pmf.is_ok());

        let pmf = pmf.unwrap();
        assert_eq!(pmf.max_window_size(), 33);
        assert_eq!(pmf.slope(), 0.7);
        assert_eq!(pmf.max_distance(), 10.0);
        assert_eq!(pmf.initial_distance(), 0.15);
        assert_eq!(pmf.cell_size(), 1.0);
        assert_eq!(pmf.base(), 2.0);
        assert_eq!(pmf.exponential(), true);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut pmf = ProgressiveMorphologicalFilterXYZ::new().unwrap();

        assert!(pmf.set_max_window_size(0).is_err());
        assert!(pmf.set_slope(-0.1).is_err());
        assert!(pmf.set_slope(1.1).is_err());
        assert!(pmf.set_max_distance(-1.0).is_err());
        assert!(pmf.set_cell_size(0.0).is_err());
        assert!(pmf.set_base(0.5).is_err());
    }
}
