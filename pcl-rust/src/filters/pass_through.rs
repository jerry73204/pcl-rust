//! PassThrough filter for point cloud filtering
//!
//! The PassThrough filter removes points outside a specified range along a given field.
//! This is useful for cropping point clouds or removing outliers in specific dimensions.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use crate::filters::{FilterXYZ, FilterXYZRGB};
use pcl_sys::{UniquePtr, ffi};

/// PassThrough filter for PointXYZ clouds
pub struct PassThroughXYZ {
    filter: UniquePtr<ffi::PassThrough_PointXYZ>,
}

impl PassThroughXYZ {
    /// Create a new PassThrough filter for PointXYZ
    pub fn new() -> PclResult<Self> {
        let filter = ffi::new_pass_through_xyz();
        Ok(Self { filter })
    }

    /// Set the field name to filter on (e.g., "x", "y", "z")
    pub fn set_filter_field_name(&mut self, field_name: &str) -> PclResult<()> {
        ffi::set_filter_field_name_xyz(self.filter.pin_mut(), field_name);
        Ok(())
    }

    /// Get the current filter field name
    pub fn get_filter_field_name(&self) -> PclResult<String> {
        Ok(ffi::get_filter_field_name_xyz(&self.filter))
    }

    /// Set the filter limits (min and max values)
    pub fn set_filter_limits(&mut self, min: f32, max: f32) -> PclResult<()> {
        if min > max {
            return Err(PclError::invalid_parameters(
                "Invalid filter limits",
                "min/max",
                "min <= max",
                format!("min={}, max={}", min, max),
            ));
        }

        ffi::set_filter_limits_xyz(self.filter.pin_mut(), min, max);
        Ok(())
    }

    /// Set whether to return points inside (false) or outside (true) the limits
    pub fn set_negative(&mut self, negative: bool) -> PclResult<()> {
        ffi::set_filter_limits_negative_xyz(self.filter.pin_mut(), negative);
        Ok(())
    }

    /// Get whether the filter returns points outside the limits
    pub fn get_negative(&self) -> PclResult<bool> {
        Ok(ffi::get_filter_limits_negative_xyz(&self.filter))
    }

    /// Set whether to keep the point cloud organized (maintain NaN points)
    pub fn set_keep_organized(&mut self, keep_organized: bool) -> PclResult<()> {
        ffi::set_keep_organized_xyz(self.filter.pin_mut(), keep_organized);
        Ok(())
    }

    /// Get whether the filter keeps the point cloud organized
    pub fn get_keep_organized(&self) -> PclResult<bool> {
        Ok(ffi::get_keep_organized_xyz(&self.filter))
    }
}

impl Default for PassThroughXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create PassThrough filter")
    }
}

impl FilterXYZ for PassThroughXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_pass_xyz(self.filter.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZ> {
        let result = ffi::filter_pass_xyz(self.filter.pin_mut());
        if result.is_null() {
            Err(PclError::FilterError("Filter operation failed".to_string()))
        } else {
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }
}

/// PassThrough filter for PointXYZRGB clouds
pub struct PassThroughXYZRGB {
    filter: UniquePtr<ffi::PassThrough_PointXYZRGB>,
}

impl PassThroughXYZRGB {
    /// Create a new PassThrough filter for PointXYZRGB
    pub fn new() -> PclResult<Self> {
        let filter = ffi::new_pass_through_xyzrgb();
        Ok(Self { filter })
    }

    /// Set the field name to filter on (e.g., "x", "y", "z", "rgb")
    pub fn set_filter_field_name(&mut self, field_name: &str) -> PclResult<()> {
        ffi::set_filter_field_name_xyzrgb(self.filter.pin_mut(), field_name);
        Ok(())
    }

    /// Get the current filter field name
    pub fn get_filter_field_name(&self) -> PclResult<String> {
        Ok(ffi::get_filter_field_name_xyzrgb(&self.filter))
    }

    /// Set the filter limits (min and max values)
    pub fn set_filter_limits(&mut self, min: f32, max: f32) -> PclResult<()> {
        if min > max {
            return Err(PclError::invalid_parameters(
                "Invalid filter limits",
                "min/max",
                "min <= max",
                format!("min={}, max={}", min, max),
            ));
        }

        ffi::set_filter_limits_xyzrgb(self.filter.pin_mut(), min, max);
        Ok(())
    }

    /// Set whether to return points inside (false) or outside (true) the limits
    pub fn set_negative(&mut self, negative: bool) -> PclResult<()> {
        ffi::set_filter_limits_negative_xyzrgb(self.filter.pin_mut(), negative);
        Ok(())
    }

    /// Get whether the filter returns points outside the limits
    pub fn get_negative(&self) -> PclResult<bool> {
        Ok(ffi::get_filter_limits_negative_xyzrgb(&self.filter))
    }

    /// Set whether to keep the point cloud organized (maintain NaN points)
    pub fn set_keep_organized(&mut self, keep_organized: bool) -> PclResult<()> {
        ffi::set_keep_organized_xyzrgb(self.filter.pin_mut(), keep_organized);
        Ok(())
    }

    /// Get whether the filter keeps the point cloud organized
    pub fn get_keep_organized(&self) -> PclResult<bool> {
        Ok(ffi::get_keep_organized_xyzrgb(&self.filter))
    }
}

impl Default for PassThroughXYZRGB {
    fn default() -> Self {
        Self::new().expect("Failed to create PassThrough filter")
    }
}

impl FilterXYZRGB for PassThroughXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_cloud_pass_xyzrgb(self.filter.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZRGB> {
        let result = ffi::filter_pass_xyzrgb(self.filter.pin_mut());
        if result.is_null() {
            Err(PclError::FilterError("Filter operation failed".to_string()))
        } else {
            Ok(PointCloudXYZRGB::from_unique_ptr(result))
        }
    }
}

/// Builder for PassThroughXYZ filter configuration
pub struct PassThroughXYZBuilder {
    field_name: String,
    min_limit: Option<f32>,
    max_limit: Option<f32>,
    negative: bool,
    keep_organized: bool,
}

impl PassThroughXYZBuilder {
    /// Create a new PassThrough filter builder
    pub fn new() -> Self {
        Self {
            field_name: "z".to_string(),
            min_limit: None,
            max_limit: None,
            negative: false,
            keep_organized: false,
        }
    }

    /// Set the field name to filter on
    pub fn field_name(mut self, field_name: &str) -> Self {
        self.field_name = field_name.to_string();
        self
    }

    /// Set the minimum limit
    pub fn min_limit(mut self, min: f32) -> Self {
        self.min_limit = Some(min);
        self
    }

    /// Set the maximum limit
    pub fn max_limit(mut self, max: f32) -> Self {
        self.max_limit = Some(max);
        self
    }

    /// Set both min and max limits
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.min_limit = Some(min);
        self.max_limit = Some(max);
        self
    }

    /// Set whether to return points outside the limits
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Set whether to keep the point cloud organized
    pub fn keep_organized(mut self, keep_organized: bool) -> Self {
        self.keep_organized = keep_organized;
        self
    }

    /// Build the PassThrough filter
    pub fn build(self) -> PclResult<PassThroughXYZ> {
        let mut filter = PassThroughXYZ::new()?;

        filter.set_filter_field_name(&self.field_name)?;

        if let (Some(min), Some(max)) = (self.min_limit, self.max_limit) {
            filter.set_filter_limits(min, max)?;
        }

        filter.set_negative(self.negative)?;
        filter.set_keep_organized(self.keep_organized)?;

        Ok(filter)
    }
}

impl Default for PassThroughXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for PassThroughXYZRGB filter configuration
pub struct PassThroughXYZRGBBuilder {
    field_name: String,
    min_limit: Option<f32>,
    max_limit: Option<f32>,
    negative: bool,
    keep_organized: bool,
}

impl PassThroughXYZRGBBuilder {
    /// Create a new PassThrough filter builder
    pub fn new() -> Self {
        Self {
            field_name: "z".to_string(),
            min_limit: None,
            max_limit: None,
            negative: false,
            keep_organized: false,
        }
    }

    /// Set the field name to filter on
    pub fn field_name(mut self, field_name: &str) -> Self {
        self.field_name = field_name.to_string();
        self
    }

    /// Set the minimum limit
    pub fn min_limit(mut self, min: f32) -> Self {
        self.min_limit = Some(min);
        self
    }

    /// Set the maximum limit
    pub fn max_limit(mut self, max: f32) -> Self {
        self.max_limit = Some(max);
        self
    }

    /// Set both min and max limits
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.min_limit = Some(min);
        self.max_limit = Some(max);
        self
    }

    /// Set whether to return points outside the limits
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Set whether to keep the point cloud organized
    pub fn keep_organized(mut self, keep_organized: bool) -> Self {
        self.keep_organized = keep_organized;
        self
    }

    /// Build the PassThrough filter
    pub fn build(self) -> PclResult<PassThroughXYZRGB> {
        let mut filter = PassThroughXYZRGB::new()?;

        filter.set_filter_field_name(&self.field_name)?;

        if let (Some(min), Some(max)) = (self.min_limit, self.max_limit) {
            filter.set_filter_limits(min, max)?;
        }

        filter.set_negative(self.negative)?;
        filter.set_keep_organized(self.keep_organized)?;

        Ok(filter)
    }
}

impl Default for PassThroughXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pass_through_xyz_creation() {
        let filter = PassThroughXYZ::new();
        assert!(filter.is_ok());
    }

    #[test]
    fn test_pass_through_xyzrgb_creation() {
        let filter = PassThroughXYZRGB::new();
        assert!(filter.is_ok());
    }

    #[test]
    fn test_pass_through_xyz_builder() {
        let filter = PassThroughXYZBuilder::new()
            .field_name("z")
            .limits(0.0, 2.0)
            .negative(false)
            .keep_organized(true)
            .build();

        assert!(filter.is_ok());
        let filter = filter.unwrap();
        assert_eq!(filter.get_filter_field_name().unwrap(), "z");
        assert!(!filter.get_negative().unwrap());
        assert!(filter.get_keep_organized().unwrap());
    }

    #[test]
    fn test_pass_through_xyzrgb_builder() {
        let filter = PassThroughXYZRGBBuilder::new()
            .field_name("x")
            .limits(-1.0, 1.0)
            .negative(true)
            .keep_organized(false)
            .build();

        assert!(filter.is_ok());
        let filter = filter.unwrap();
        assert_eq!(filter.get_filter_field_name().unwrap(), "x");
        assert!(filter.get_negative().unwrap());
        assert!(!filter.get_keep_organized().unwrap());
    }
}
