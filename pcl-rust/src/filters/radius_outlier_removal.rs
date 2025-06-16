//! RadiusOutlierRemoval filter for removing outliers from point clouds
//!
//! This filter removes points that have fewer than a specified number of neighbors
//! within a given search radius. It's useful for removing isolated points or small
//! clusters of points that are likely to be noise.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::PclResult;
use crate::filters::{FilterXYZ, FilterXYZRGB};
use pcl_sys::{UniquePtr, ffi};

/// RadiusOutlierRemoval filter for PointXYZ clouds
pub struct RadiusOutlierRemovalXYZ {
    inner: UniquePtr<ffi::RadiusOutlierRemoval_PointXYZ>,
}

impl RadiusOutlierRemovalXYZ {
    /// Create a new RadiusOutlierRemoval filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_radius_outlier_removal_xyz();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "RadiusOutlierRemoval filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the radius to search for neighbors
    pub fn set_radius_search(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "radius".to_string(),
                message: "must be positive".to_string(),
            });
        }
        unsafe {
            ffi::set_radius_search_xyz(self.inner.pin_mut(), radius);
        }
        Ok(())
    }

    /// Set the minimum number of neighbors required within the search radius
    ///
    /// Points with fewer neighbors than this threshold will be removed
    pub fn set_min_neighbors_in_radius(&mut self, min_neighbors: i32) -> PclResult<()> {
        if min_neighbors < 0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "min_neighbors".to_string(),
                message: "must be non-negative".to_string(),
            });
        }
        unsafe {
            ffi::set_min_neighbors_in_radius_xyz(self.inner.pin_mut(), min_neighbors);
        }
        Ok(())
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn set_negative(&mut self, negative: bool) {
        unsafe {
            ffi::set_negative_radius_xyz(self.inner.pin_mut(), negative);
        }
    }
}

impl FilterXYZ for RadiusOutlierRemovalXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        unsafe {
            ffi::set_input_cloud_radius_xyz(self.inner.pin_mut(), cloud.as_raw());
        }
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZ> {
        unsafe {
            let result = ffi::filter_radius_xyz(self.inner.pin_mut());
            if result.is_null() {
                return Err(crate::error::PclError::ProcessingFailed {
                    message: "RadiusOutlierRemoval filter failed".to_string(),
                });
            }
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }
}

/// Builder for RadiusOutlierRemovalXYZ filter
pub struct RadiusOutlierRemovalXYZBuilder {
    radius_search: Option<f64>,
    min_neighbors: Option<i32>,
    negative: bool,
}

impl RadiusOutlierRemovalXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            radius_search: None,
            min_neighbors: None,
            negative: false,
        }
    }

    /// Set the radius to search for neighbors
    pub fn radius_search(mut self, radius: f64) -> Self {
        self.radius_search = Some(radius);
        self
    }

    /// Set the minimum number of neighbors required within the search radius
    pub fn min_neighbors_in_radius(mut self, min_neighbors: i32) -> Self {
        self.min_neighbors = Some(min_neighbors);
        self
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Build the RadiusOutlierRemoval filter
    pub fn build(self) -> PclResult<RadiusOutlierRemovalXYZ> {
        let mut filter = RadiusOutlierRemovalXYZ::new()?;

        if let Some(radius) = self.radius_search {
            filter.set_radius_search(radius)?;
        }

        if let Some(min_neighbors) = self.min_neighbors {
            filter.set_min_neighbors_in_radius(min_neighbors)?;
        }

        filter.set_negative(self.negative);

        Ok(filter)
    }
}

impl Default for RadiusOutlierRemovalXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// RadiusOutlierRemoval filter for PointXYZRGB clouds
pub struct RadiusOutlierRemovalXYZRGB {
    inner: UniquePtr<ffi::RadiusOutlierRemoval_PointXYZRGB>,
}

impl RadiusOutlierRemovalXYZRGB {
    /// Create a new RadiusOutlierRemoval filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_radius_outlier_removal_xyzrgb();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "RadiusOutlierRemoval filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the radius to search for neighbors
    pub fn set_radius_search(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "radius".to_string(),
                message: "must be positive".to_string(),
            });
        }
        unsafe {
            ffi::set_radius_search_xyzrgb(self.inner.pin_mut(), radius);
        }
        Ok(())
    }

    /// Set the minimum number of neighbors required within the search radius
    pub fn set_min_neighbors_in_radius(&mut self, min_neighbors: i32) -> PclResult<()> {
        if min_neighbors < 0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "min_neighbors".to_string(),
                message: "must be non-negative".to_string(),
            });
        }
        unsafe {
            ffi::set_min_neighbors_in_radius_xyzrgb(self.inner.pin_mut(), min_neighbors);
        }
        Ok(())
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn set_negative(&mut self, negative: bool) {
        unsafe {
            ffi::set_negative_radius_xyzrgb(self.inner.pin_mut(), negative);
        }
    }
}

impl FilterXYZRGB for RadiusOutlierRemovalXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        unsafe {
            ffi::set_input_cloud_radius_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        }
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZRGB> {
        unsafe {
            let result = ffi::filter_radius_xyzrgb(self.inner.pin_mut());
            if result.is_null() {
                return Err(crate::error::PclError::ProcessingFailed {
                    message: "RadiusOutlierRemoval filter failed".to_string(),
                });
            }
            Ok(PointCloudXYZRGB::from_unique_ptr(result))
        }
    }
}

/// Builder for RadiusOutlierRemovalXYZRGB filter
pub struct RadiusOutlierRemovalXYZRGBBuilder {
    radius_search: Option<f64>,
    min_neighbors: Option<i32>,
    negative: bool,
}

impl RadiusOutlierRemovalXYZRGBBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            radius_search: None,
            min_neighbors: None,
            negative: false,
        }
    }

    /// Set the radius to search for neighbors
    pub fn radius_search(mut self, radius: f64) -> Self {
        self.radius_search = Some(radius);
        self
    }

    /// Set the minimum number of neighbors required within the search radius
    pub fn min_neighbors_in_radius(mut self, min_neighbors: i32) -> Self {
        self.min_neighbors = Some(min_neighbors);
        self
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Build the RadiusOutlierRemoval filter
    pub fn build(self) -> PclResult<RadiusOutlierRemovalXYZRGB> {
        let mut filter = RadiusOutlierRemovalXYZRGB::new()?;

        if let Some(radius) = self.radius_search {
            filter.set_radius_search(radius)?;
        }

        if let Some(min_neighbors) = self.min_neighbors {
            filter.set_min_neighbors_in_radius(min_neighbors)?;
        }

        filter.set_negative(self.negative);

        Ok(filter)
    }
}

impl Default for RadiusOutlierRemovalXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}
