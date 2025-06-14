//! Search algorithms and spatial queries
//!
//! This module provides safe wrappers around PCL's search interfaces
//! for nearest neighbor and radius-based spatial queries.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB, PointXYZ, PointXYZRGB};
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// KdTree search for PointXYZ clouds
pub struct KdTreeXYZ {
    inner: UniquePtr<ffi::KdTree_PointXYZ>,
}

impl KdTreeXYZ {
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_kdtree_xyz();
        Ok(Self { inner })
    }

    /// Set the input point cloud for search operations
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
        if k <= 0 {
            return Err(PclError::InvalidParameters(
                "k must be positive".to_string(),
            ));
        }

        let indices = ffi::nearest_k_search_xyz(&self.inner, &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
        if radius <= 0.0 {
            return Err(PclError::InvalidParameters(
                "radius must be positive".to_string(),
            ));
        }

        let indices = ffi::radius_search_xyz(&self.inner, &point.inner, radius);
        Ok(indices)
    }
}

impl Default for KdTreeXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default KdTreeXYZ")
    }
}

impl std::fmt::Debug for KdTreeXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KdTreeXYZ").finish()
    }
}

/// KdTree search for PointXYZRGB clouds
pub struct KdTreeXYZRGB {
    inner: UniquePtr<ffi::KdTree_PointXYZRGB>,
}

impl KdTreeXYZRGB {
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_kdtree_xyzrgb();
        Ok(Self { inner })
    }

    /// Set the input point cloud for search operations
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_cloud_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }
}

impl Default for KdTreeXYZRGB {
    fn default() -> Self {
        Self::new().expect("Failed to create default KdTreeXYZRGB")
    }
}

impl std::fmt::Debug for KdTreeXYZRGB {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KdTreeXYZRGB").finish()
    }
}
