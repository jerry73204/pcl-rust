//! Search algorithms and spatial queries
//!
//! This module provides safe wrappers around PCL's search interfaces
//! for nearest neighbor and radius-based spatial queries.

pub mod traits;
pub mod unified;

use crate::common::{PointCloudXYZ, PointCloudXYZRGB, PointXYZ, PointXYZRGB};
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
            return Err(PclError::InvalidParameters(
                "epsilon must be non-negative".to_string(),
            ));
        }
        ffi::set_epsilon_xyz(self.inner.pin_mut(), epsilon);
        Ok(())
    }
}

impl SearchInputCloud<PointCloudXYZ> for KdTreeXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_xyz(self.inner.pin_mut(), cloud.as_raw());
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
        if k <= 0 {
            return Err(PclError::InvalidParameters(
                "k must be positive".to_string(),
            ));
        }

        let indices = ffi::nearest_k_search_xyzrgb(&self.inner, &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&self, point: &PointXYZRGB, radius: f64) -> PclResult<Vec<i32>> {
        if radius <= 0.0 {
            return Err(PclError::InvalidParameters(
                "radius must be positive".to_string(),
            ));
        }

        let indices = ffi::radius_search_xyzrgb(&self.inner, &point.inner, radius);
        Ok(indices)
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
            return Err(PclError::InvalidParameters(
                "epsilon must be non-negative".to_string(),
            ));
        }
        ffi::set_epsilon_xyzrgb(self.inner.pin_mut(), epsilon);
        Ok(())
    }
}

impl SearchInputCloud<PointCloudXYZRGB> for KdTreeXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        ffi::set_input_cloud_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
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
