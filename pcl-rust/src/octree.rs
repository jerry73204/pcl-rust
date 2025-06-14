//! Octree spatial data structures
//!
//! This module provides safe wrappers around PCL's octree implementations
//! for efficient spatial organization and queries.

use crate::common::{PointCloudXYZ, PointXYZ};
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Octree for spatial search operations on PointXYZ clouds
pub struct OctreeSearchXYZ {
    inner: UniquePtr<ffi::OctreePointCloudSearch_PointXYZ>,
}

impl OctreeSearchXYZ {
    /// Create a new octree with the specified resolution
    pub fn new(resolution: f64) -> PclResult<Self> {
        if resolution <= 0.0 {
            return Err(PclError::InvalidParameters(
                "resolution must be positive".to_string(),
            ));
        }

        let inner = ffi::new_octree_search_xyz(resolution);
        Ok(Self { inner })
    }

    /// Set the input point cloud and build the octree
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_octree_xyz(self.inner.pin_mut(), cloud.as_raw());
        ffi::add_points_from_input_cloud_xyz(self.inner.pin_mut());
        Ok(())
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&mut self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
        if k <= 0 {
            return Err(PclError::InvalidParameters(
                "k must be positive".to_string(),
            ));
        }

        let indices = ffi::nearest_k_search_octree_xyz(self.inner.pin_mut(), &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&mut self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
        if radius <= 0.0 {
            return Err(PclError::InvalidParameters(
                "radius must be positive".to_string(),
            ));
        }

        let indices = ffi::radius_search_octree_xyz(self.inner.pin_mut(), &point.inner, radius);
        Ok(indices)
    }

    /// Find all points in the same voxel as the query point
    pub fn voxel_search(&mut self, point: &PointXYZ) -> PclResult<Vec<i32>> {
        let indices = ffi::voxel_search_octree_xyz(self.inner.pin_mut(), &point.inner);
        Ok(indices)
    }

    /// Get the octree resolution
    pub fn resolution(&mut self) -> f64 {
        ffi::get_resolution(self.inner.pin_mut())
    }

    /// Get the depth of the octree
    pub fn tree_depth(&mut self) -> u32 {
        ffi::get_tree_depth(self.inner.pin_mut())
    }

    /// Get the number of leaf nodes
    pub fn leaf_count(&mut self) -> usize {
        ffi::get_leaf_count(self.inner.pin_mut())
    }

    /// Get the number of branch nodes
    pub fn branch_count(&mut self) -> usize {
        ffi::get_branch_count(self.inner.pin_mut())
    }
}

impl std::fmt::Debug for OctreeSearchXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OctreeSearchXYZ").finish()
    }
}

/// Octree for voxel centroid operations on PointXYZ clouds
pub struct OctreeVoxelCentroidXYZ {
    inner: UniquePtr<ffi::OctreePointCloudVoxelCentroid_PointXYZ>,
}

impl OctreeVoxelCentroidXYZ {
    /// Create a new voxel centroid octree with the specified resolution
    pub fn new(resolution: f64) -> PclResult<Self> {
        if resolution <= 0.0 {
            return Err(PclError::InvalidParameters(
                "resolution must be positive".to_string(),
            ));
        }

        let inner = ffi::new_octree_voxel_centroid_xyz(resolution);
        Ok(Self { inner })
    }

    /// Set the input point cloud
    /// Note: Full voxel centroid functionality would require additional FFI functions
    pub fn set_input_cloud(&mut self, _cloud: &PointCloudXYZ) -> PclResult<()> {
        Err(PclError::NotImplemented(
            "OctreeVoxelCentroid operations not yet fully implemented".to_string(),
        ))
    }
}

impl std::fmt::Debug for OctreeVoxelCentroidXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OctreeVoxelCentroidXYZ").finish()
    }
}
