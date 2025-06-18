//! Octree spatial data structures
//!
//! This module provides safe wrappers around PCL's octree implementations
//! for efficient spatial organization and queries.

pub mod builders;
pub mod traits;

use crate::common::{PointCloudXYZ, PointXYZ};
use crate::error::{PclError, PclResult};
use crate::search::{NearestNeighborSearch, SearchInputCloud};
use cxx::UniquePtr;
use pcl_sys::ffi;

pub use builders::{OctreeSearchBuilder, OctreeVoxelCentroidBuilder};
pub use traits::{OctreeConfiguration, OctreeManagement, OctreeType, VoxelOperations};

/// Octree for spatial search operations on PointXYZ clouds
pub struct OctreeSearchXYZ {
    inner: UniquePtr<ffi::OctreePointCloudSearch_PointXYZ>,
    has_cloud: bool,
}

impl OctreeSearchXYZ {
    /// Create a new octree with the specified resolution
    pub fn new(resolution: f64) -> PclResult<Self> {
        if resolution <= 0.0 {
            return Err(PclError::invalid_parameters(
                "resolution must be positive",
                "resolution",
                "positive value",
                format!("{}", resolution),
            ));
        }

        let inner = ffi::new_octree_search_xyz(resolution);
        Ok(Self {
            inner,
            has_cloud: false,
        })
    }

    /// Set the input point cloud and build the octree
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_octree_xyz(self.inner.pin_mut(), cloud.inner());
        ffi::add_points_from_input_cloud_xyz(self.inner.pin_mut());
        self.has_cloud = true;
        Ok(())
    }

    /// Find the k nearest neighbors to a query point
    pub fn nearest_k_search(&mut self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
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

        let indices = ffi::nearest_k_search_octree_xyz(self.inner.pin_mut(), &point.inner, k);
        Ok(indices)
    }

    /// Find all neighbors within a radius of a query point
    pub fn radius_search(&mut self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
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

        let indices = ffi::radius_search_octree_xyz(self.inner.pin_mut(), &point.inner, radius);
        Ok(indices)
    }

    /// Find all points in the same voxel as the query point
    pub fn voxel_search(&mut self, point: &PointXYZ) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
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

impl NearestNeighborSearch<PointXYZ> for OctreeSearchXYZ {
    fn nearest_k_search(&self, _point: &PointXYZ, _k: i32) -> PclResult<Vec<i32>> {
        // Octree requires mutable reference, so we need to handle this differently
        // For now, return not implemented
        Err(PclError::not_implemented(
            "Const nearest_k_search for octree",
            Some("Use the mutable nearest_k_search method directly on OctreeSearchXYZ".to_string()),
        ))
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &PointXYZ,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        Err(PclError::not_implemented(
            "Const nearest_k_search_with_distances for octree",
            Some("Use the mutable nearest_k_search method directly on OctreeSearchXYZ".to_string()),
        ))
    }

    fn radius_search(&self, _point: &PointXYZ, _radius: f64) -> PclResult<Vec<i32>> {
        Err(PclError::not_implemented(
            "Const radius_search for octree",
            Some("Use the mutable radius_search method directly on OctreeSearchXYZ".to_string()),
        ))
    }

    fn radius_search_with_distances(
        &self,
        _point: &PointXYZ,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        Err(PclError::not_implemented(
            "Const radius_search_with_distances for octree",
            Some("Use the mutable radius_search method directly on OctreeSearchXYZ".to_string()),
        ))
    }
}

impl SearchInputCloud<PointCloudXYZ> for OctreeSearchXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        self.set_input_cloud(cloud)
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

impl OctreeConfiguration for OctreeSearchXYZ {
    fn resolution(&mut self) -> f64 {
        self.resolution()
    }

    fn tree_depth(&mut self) -> u32 {
        self.tree_depth()
    }

    fn leaf_count(&mut self) -> usize {
        self.leaf_count()
    }

    fn branch_count(&mut self) -> usize {
        self.branch_count()
    }
}

impl VoxelOperations for OctreeSearchXYZ {
    fn voxel_search(&mut self, point: &PointXYZ) -> PclResult<Vec<i32>> {
        self.voxel_search(point)
    }
}

impl OctreeManagement for OctreeSearchXYZ {
    fn delete_tree(&mut self) -> PclResult<()> {
        // Note: PCL doesn't expose delete_tree for OctreePointCloudSearch
        // We would need to recreate the octree
        self.has_cloud = false;
        Ok(())
    }

    fn is_built(&self) -> bool {
        self.has_cloud
    }
}

impl std::fmt::Debug for OctreeSearchXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OctreeSearchXYZ")
            .field("has_cloud", &self.has_cloud)
            .finish()
    }
}

/// Octree for voxel centroid operations on PointXYZ clouds
///
/// This octree computes centroids for all occupied voxels, which is useful
/// for downsampling point clouds while preserving their structure.
pub struct OctreeVoxelCentroidXYZ {
    inner: UniquePtr<ffi::OctreePointCloudVoxelCentroid_PointXYZ>,
    has_cloud: bool,
}

impl OctreeVoxelCentroidXYZ {
    /// Create a new voxel centroid octree with the specified resolution
    pub fn new(resolution: f64) -> PclResult<Self> {
        if resolution <= 0.0 {
            return Err(PclError::invalid_parameters(
                "resolution must be positive",
                "resolution",
                "positive value",
                format!("{}", resolution),
            ));
        }

        let inner = ffi::new_octree_voxel_centroid_xyz(resolution);
        Ok(Self {
            inner,
            has_cloud: false,
        })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        ffi::set_input_cloud_voxel_centroid_xyz(self.inner.pin_mut(), cloud.inner());
        self.has_cloud = true;
        Ok(())
    }

    /// Add points from the input cloud to the octree
    pub fn add_points_from_input_cloud(&mut self) -> PclResult<()> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        ffi::add_points_from_input_cloud_voxel_xyz(self.inner.pin_mut());
        Ok(())
    }

    /// Get the centroids of all occupied voxels as a point cloud
    pub fn get_voxel_centroids(&mut self) -> PclResult<PointCloudXYZ> {
        let inner = ffi::get_voxel_centroids_xyz(self.inner.pin_mut());
        Ok(PointCloudXYZ::from_unique_ptr(inner))
    }

    /// Get the octree resolution
    pub fn resolution(&mut self) -> f64 {
        ffi::get_resolution_voxel_xyz(self.inner.pin_mut())
    }

    /// Get the depth of the octree
    pub fn tree_depth(&mut self) -> u32 {
        ffi::get_tree_depth_voxel_xyz(self.inner.pin_mut())
    }

    /// Delete the octree structure
    pub fn delete_tree(&mut self) -> PclResult<()> {
        ffi::delete_tree_voxel_xyz(self.inner.pin_mut());
        self.has_cloud = false;
        Ok(())
    }

    /// Check if an input cloud has been set
    pub fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

impl OctreeConfiguration for OctreeVoxelCentroidXYZ {
    fn resolution(&mut self) -> f64 {
        self.resolution()
    }

    fn tree_depth(&mut self) -> u32 {
        self.tree_depth()
    }

    fn leaf_count(&mut self) -> usize {
        // Not available for voxel centroid octree
        0
    }

    fn branch_count(&mut self) -> usize {
        // Not available for voxel centroid octree
        0
    }
}

impl OctreeManagement for OctreeVoxelCentroidXYZ {
    fn delete_tree(&mut self) -> PclResult<()> {
        self.delete_tree()
    }

    fn is_built(&self) -> bool {
        self.has_cloud
    }
}

impl std::fmt::Debug for OctreeVoxelCentroidXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OctreeVoxelCentroidXYZ")
            .field("has_cloud", &self.has_cloud)
            .finish()
    }
}
