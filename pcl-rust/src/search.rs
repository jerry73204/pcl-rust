//! Search algorithms and spatial queries
//!
//! This module provides safe wrappers around PCL's search interfaces
//! for nearest neighbor and radius-based spatial queries.

pub mod traits;
pub mod unified;

use crate::common::{
    PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB, PointXYZ, PointXYZI, PointXYZRGB,
};
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

pub use traits::{NearestNeighborSearch, SearchConfiguration, SearchInputCloud, SearchMethod};
pub use unified::{SearchXYZ, SearchXYZRGB};

/// KdTree search for PointXYZ clouds
pub struct KdTreeXYZ {
    inner: UniquePtr<ffi::KdTree_PointXYZ>,
    has_cloud: bool,
}

impl KdTreeXYZ {
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_kdtree_xyz();
        Ok(Self {
            inner,
            has_cloud: false,
        })
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "k must be positive",
                "k",
                "positive integer",
                format!("{}", k),
            ));
        }

        let indices = ffi::nearest_k_search_xyz(&self.inner, &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "radius must be positive",
                "radius",
                "positive value",
                format!("{}", radius),
            ));
        }

        let indices = ffi::radius_search_xyz(&self.inner, &point.inner, radius);
        Ok(indices)
    }

    /// Get a reference to the underlying pcl-sys KdTree
    pub fn as_raw(&self) -> &ffi::KdTree_PointXYZ {
        &self.inner
    }

    /// Get reference to inner KdTree for FFI operations
    pub(crate) fn inner(&self) -> &ffi::KdTree_PointXYZ {
        &self.inner
    }
}

impl NearestNeighborSearch<PointXYZ> for KdTreeXYZ {
    fn nearest_k_search(&self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
        self.nearest_k_search(point, k)
    }

    fn nearest_k_search_with_distances(
        &self,
        point: &PointXYZ,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.nearest_k_search(point, k)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }

    fn radius_search(&self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
        self.radius_search(point, radius)
    }

    fn radius_search_with_distances(
        &self,
        point: &PointXYZ,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.radius_search(point, radius)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }
}

impl SearchConfiguration for KdTreeXYZ {
    fn epsilon(&self) -> f32 {
        ffi::get_epsilon_xyz(&self.inner)
    }

    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "epsilon must be non-negative",
                "epsilon",
                "non-negative value",
                format!("{}", epsilon),
            ));
        }
        ffi::set_epsilon_xyz(self.inner.pin_mut(), epsilon);
        Ok(())
    }
}

impl SearchInputCloud<PointCloudXYZ> for KdTreeXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_xyz(self.inner.pin_mut(), cloud.inner());
        self.has_cloud = true;
        Ok(())
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
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

/// KdTree search for PointXYZI clouds
pub struct KdTreeXYZI {
    inner: UniquePtr<ffi::KdTree_PointXYZI>,
    has_cloud: bool,
}

impl KdTreeXYZI {
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_kdtree_xyzi();
        Ok(Self {
            inner,
            has_cloud: false,
        })
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&self, point: &PointXYZI, k: i32) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "k must be positive",
                "k",
                "positive integer",
                format!("{}", k),
            ));
        }

        let indices = ffi::nearest_k_search_xyzi(&self.inner, &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&self, point: &PointXYZI, radius: f64) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "radius must be positive",
                "radius",
                "positive value",
                format!("{}", radius),
            ));
        }

        let indices = ffi::radius_search_xyzi(&self.inner, &point.inner, radius);
        Ok(indices)
    }

    /// Get access to the raw FFI object
    pub fn as_raw(&self) -> &ffi::KdTree_PointXYZI {
        &self.inner
    }

    /// Get reference to inner KdTree for FFI operations
    pub(crate) fn inner(&self) -> &ffi::KdTree_PointXYZI {
        &self.inner
    }
}

impl NearestNeighborSearch<PointXYZI> for KdTreeXYZI {
    fn nearest_k_search(&self, point: &PointXYZI, k: i32) -> PclResult<Vec<i32>> {
        self.nearest_k_search(point, k)
    }

    fn nearest_k_search_with_distances(
        &self,
        point: &PointXYZI,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.nearest_k_search(point, k)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }

    fn radius_search(&self, point: &PointXYZI, radius: f64) -> PclResult<Vec<i32>> {
        self.radius_search(point, radius)
    }

    fn radius_search_with_distances(
        &self,
        point: &PointXYZI,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.radius_search(point, radius)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }
}

impl SearchConfiguration for KdTreeXYZI {
    fn epsilon(&self) -> f32 {
        ffi::get_epsilon_xyzi(&self.inner)
    }

    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "epsilon must be non-negative",
                "epsilon",
                "non-negative value",
                format!("{}", epsilon),
            ));
        }
        ffi::set_epsilon_xyzi(self.inner.pin_mut(), epsilon);
        Ok(())
    }
}

impl SearchInputCloud<PointCloudXYZI> for KdTreeXYZI {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZI) -> PclResult<()> {
        ffi::set_input_cloud_xyzi(self.inner.pin_mut(), cloud.inner());
        self.has_cloud = true;
        Ok(())
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

impl Default for KdTreeXYZI {
    fn default() -> Self {
        Self::new().expect("Failed to create default KdTreeXYZI")
    }
}

impl std::fmt::Debug for KdTreeXYZI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KdTreeXYZI").finish()
    }
}

/// KdTree search for PointXYZRGB clouds
pub struct KdTreeXYZRGB {
    inner: UniquePtr<ffi::KdTree_PointXYZRGB>,
    has_cloud: bool,
}

impl KdTreeXYZRGB {
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_kdtree_xyzrgb();
        Ok(Self {
            inner,
            has_cloud: false,
        })
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&self, point: &PointXYZRGB, k: i32) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "k must be positive",
                "k",
                "positive integer",
                format!("{}", k),
            ));
        }

        let indices = ffi::nearest_k_search_xyzrgb(&self.inner, &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&self, point: &PointXYZRGB, radius: f64) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "radius must be positive",
                "radius",
                "positive value",
                format!("{}", radius),
            ));
        }

        let indices = ffi::radius_search_xyzrgb(&self.inner, &point.inner, radius);
        Ok(indices)
    }

    /// Get reference to inner KdTree for FFI operations
    pub(crate) fn inner(&self) -> &ffi::KdTree_PointXYZRGB {
        &self.inner
    }
}

impl NearestNeighborSearch<PointXYZRGB> for KdTreeXYZRGB {
    fn nearest_k_search(&self, point: &PointXYZRGB, k: i32) -> PclResult<Vec<i32>> {
        self.nearest_k_search(point, k)
    }

    fn nearest_k_search_with_distances(
        &self,
        point: &PointXYZRGB,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.nearest_k_search(point, k)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }

    fn radius_search(&self, point: &PointXYZRGB, radius: f64) -> PclResult<Vec<i32>> {
        self.radius_search(point, radius)
    }

    fn radius_search_with_distances(
        &self,
        point: &PointXYZRGB,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        // For now, return indices only
        // TODO: Extend FFI to return distances as well
        let indices = self.radius_search(point, radius)?;
        let distances = vec![0.0; indices.len()];
        Ok((indices, distances))
    }
}

impl SearchConfiguration for KdTreeXYZRGB {
    fn epsilon(&self) -> f32 {
        ffi::get_epsilon_xyzrgb(&self.inner)
    }

    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "epsilon must be non-negative",
                "epsilon",
                "non-negative value",
                format!("{}", epsilon),
            ));
        }
        ffi::set_epsilon_xyzrgb(self.inner.pin_mut(), epsilon);
        Ok(())
    }
}

impl SearchInputCloud<PointCloudXYZRGB> for KdTreeXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_cloud_xyzrgb(self.inner.pin_mut(), cloud.inner());
        self.has_cloud = true;
        Ok(())
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
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
