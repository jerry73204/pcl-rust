//! StatisticalOutlierRemoval filter for removing outliers from point clouds
//!
//! This filter performs statistical analysis on each point's neighborhood and removes
//! those points which do not meet a certain criteria. The filter iterates through
//! the entire input once, and for each point, computes the average distance to its
//! k nearest neighbors. The point will be considered an outlier if its average distance
//! is above a threshold defined by the global mean distance plus stddev_mult times the
//! global standard deviation.

use crate::common::{PointCloud, PointXYZ, PointXYZRGB};
use crate::error::PclResult;
use crate::filters::Filter;
use pcl_sys::{UniquePtr, ffi};

/// StatisticalOutlierRemoval filter for PointXYZ clouds
pub struct StatisticalOutlierRemovalXYZ {
    inner: UniquePtr<ffi::StatisticalOutlierRemoval_PointXYZ>,
}

impl StatisticalOutlierRemovalXYZ {
    /// Create a new StatisticalOutlierRemoval filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_statistical_outlier_removal_xyz();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "StatisticalOutlierRemoval filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the number of neighbors to analyze for each point
    pub fn set_mean_k(&mut self, k: i32) -> PclResult<()> {
        if k <= 0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "mean_k".to_string(),
                message: "must be positive".to_string(),
            });
        }
        ffi::set_mean_k_statistical_xyz(self.inner.pin_mut(), k);
        Ok(())
    }

    /// Set the standard deviation multiplier threshold
    ///
    /// Points will be considered outliers if their average neighbor distance is greater than:
    /// global_mean + stddev_mult * global_std_dev
    pub fn set_std_dev_mul_thresh(&mut self, stddev_mult: f64) -> PclResult<()> {
        if stddev_mult <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "stddev_mult".to_string(),
                message: "must be positive".to_string(),
            });
        }
        ffi::set_std_dev_mul_thresh_statistical_xyz(self.inner.pin_mut(), stddev_mult);
        Ok(())
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn set_negative(&mut self, negative: bool) {
        ffi::set_negative_statistical_xyz(self.inner.pin_mut(), negative);
    }
}

impl Filter<PointXYZ> for StatisticalOutlierRemovalXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZ>) -> PclResult<()> {
        ffi::set_input_cloud_statistical_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloud<PointXYZ>> {
        let result = ffi::filter_statistical_xyz(self.inner.pin_mut());
        if result.is_null() {
            return Err(crate::error::PclError::ProcessingFailed {
                message: "StatisticalOutlierRemoval filter failed".to_string(),
            });
        }
        Ok(PointCloud::from_unique_ptr(result))
    }
}

/// Builder for StatisticalOutlierRemovalXYZ filter
pub struct StatisticalOutlierRemovalXYZBuilder {
    mean_k: Option<i32>,
    stddev_mult: Option<f64>,
    negative: bool,
}

impl StatisticalOutlierRemovalXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            mean_k: None,
            stddev_mult: None,
            negative: false,
        }
    }

    /// Set the number of neighbors to analyze for each point
    pub fn mean_k(mut self, k: i32) -> Self {
        self.mean_k = Some(k);
        self
    }

    /// Set the standard deviation multiplier threshold
    pub fn std_dev_mul_thresh(mut self, stddev_mult: f64) -> Self {
        self.stddev_mult = Some(stddev_mult);
        self
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Build the StatisticalOutlierRemoval filter
    pub fn build(self) -> PclResult<StatisticalOutlierRemovalXYZ> {
        let mut filter = StatisticalOutlierRemovalXYZ::new()?;

        if let Some(k) = self.mean_k {
            filter.set_mean_k(k)?;
        }

        if let Some(stddev_mult) = self.stddev_mult {
            filter.set_std_dev_mul_thresh(stddev_mult)?;
        }

        filter.set_negative(self.negative);

        Ok(filter)
    }
}

impl Default for StatisticalOutlierRemovalXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// StatisticalOutlierRemoval filter for PointXYZRGB clouds
pub struct StatisticalOutlierRemovalXYZRGB {
    inner: UniquePtr<ffi::StatisticalOutlierRemoval_PointXYZRGB>,
}

impl StatisticalOutlierRemovalXYZRGB {
    /// Create a new StatisticalOutlierRemoval filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_statistical_outlier_removal_xyzrgb();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "StatisticalOutlierRemoval filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the number of neighbors to analyze for each point
    pub fn set_mean_k(&mut self, k: i32) -> PclResult<()> {
        if k <= 0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "mean_k".to_string(),
                message: "must be positive".to_string(),
            });
        }
        ffi::set_mean_k_statistical_xyzrgb(self.inner.pin_mut(), k);
        Ok(())
    }

    /// Set the standard deviation multiplier threshold
    pub fn set_std_dev_mul_thresh(&mut self, stddev_mult: f64) -> PclResult<()> {
        if stddev_mult <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "stddev_mult".to_string(),
                message: "must be positive".to_string(),
            });
        }
        ffi::set_std_dev_mul_thresh_statistical_xyzrgb(self.inner.pin_mut(), stddev_mult);
        Ok(())
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn set_negative(&mut self, negative: bool) {
        ffi::set_negative_statistical_xyzrgb(self.inner.pin_mut(), negative);
    }
}

impl Filter<PointXYZRGB> for StatisticalOutlierRemovalXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZRGB>) -> PclResult<()> {
        ffi::set_input_cloud_statistical_xyzrgb(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloud<PointXYZRGB>> {
        let result = ffi::filter_statistical_xyzrgb(self.inner.pin_mut());
        if result.is_null() {
            return Err(crate::error::PclError::ProcessingFailed {
                message: "StatisticalOutlierRemoval filter failed".to_string(),
            });
        }
        Ok(PointCloud::from_unique_ptr(result))
    }
}

/// Builder for StatisticalOutlierRemovalXYZRGB filter
pub struct StatisticalOutlierRemovalXYZRGBBuilder {
    mean_k: Option<i32>,
    stddev_mult: Option<f64>,
    negative: bool,
}

impl StatisticalOutlierRemovalXYZRGBBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            mean_k: None,
            stddev_mult: None,
            negative: false,
        }
    }

    /// Set the number of neighbors to analyze for each point
    pub fn mean_k(mut self, k: i32) -> Self {
        self.mean_k = Some(k);
        self
    }

    /// Set the standard deviation multiplier threshold
    pub fn std_dev_mul_thresh(mut self, stddev_mult: f64) -> Self {
        self.stddev_mult = Some(stddev_mult);
        self
    }

    /// Set whether to return the outliers instead of the inliers
    pub fn negative(mut self, negative: bool) -> Self {
        self.negative = negative;
        self
    }

    /// Build the StatisticalOutlierRemoval filter
    pub fn build(self) -> PclResult<StatisticalOutlierRemovalXYZRGB> {
        let mut filter = StatisticalOutlierRemovalXYZRGB::new()?;

        if let Some(k) = self.mean_k {
            filter.set_mean_k(k)?;
        }

        if let Some(stddev_mult) = self.stddev_mult {
            filter.set_std_dev_mul_thresh(stddev_mult)?;
        }

        filter.set_negative(self.negative);

        Ok(filter)
    }
}

impl Default for StatisticalOutlierRemovalXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}
