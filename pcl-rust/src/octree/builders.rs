//! Builder patterns for octree structures
//!
//! This module provides builder patterns to simplify octree construction
//! and configuration.

use super::{OctreeSearchXYZ, OctreeVoxelCentroidXYZ};
use crate::common::{PointCloud, XYZ};
use crate::error::PclResult;

/// Builder for OctreeSearchXYZ
pub struct OctreeSearchBuilder {
    resolution: f64,
    input_cloud: Option<PointCloud<XYZ>>,
}

impl OctreeSearchBuilder {
    /// Create a new builder with the specified resolution
    pub fn new(resolution: f64) -> Self {
        Self {
            resolution,
            input_cloud: None,
        }
    }

    /// Set the resolution (voxel size)
    pub fn resolution(mut self, resolution: f64) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set the input cloud (will be moved)
    pub fn input_cloud(mut self, cloud: PointCloud<XYZ>) -> Self {
        self.input_cloud = Some(cloud);
        self
    }

    /// Build the octree
    pub fn build(self) -> PclResult<OctreeSearchXYZ> {
        let mut octree = OctreeSearchXYZ::new(self.resolution)?;

        if let Some(cloud) = self.input_cloud.as_ref() {
            octree.set_input_cloud(cloud)?;
        }

        Ok(octree)
    }
}

/// Builder for OctreeVoxelCentroidXYZ
pub struct OctreeVoxelCentroidBuilder {
    resolution: f64,
    input_cloud: Option<PointCloud<XYZ>>,
    auto_build: bool,
}

impl OctreeVoxelCentroidBuilder {
    /// Create a new builder with the specified resolution
    pub fn new(resolution: f64) -> Self {
        Self {
            resolution,
            input_cloud: None,
            auto_build: true,
        }
    }

    /// Set the resolution (voxel size)
    pub fn resolution(mut self, resolution: f64) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set the input cloud (will be moved)
    pub fn input_cloud(mut self, cloud: PointCloud<XYZ>) -> Self {
        self.input_cloud = Some(cloud);
        self
    }

    /// Set whether to automatically build the octree after setting the cloud
    pub fn auto_build(mut self, auto_build: bool) -> Self {
        self.auto_build = auto_build;
        self
    }

    /// Build the octree
    pub fn build(self) -> PclResult<OctreeVoxelCentroidXYZ> {
        let mut octree = OctreeVoxelCentroidXYZ::new(self.resolution)?;

        if let Some(cloud) = self.input_cloud.as_ref() {
            octree.set_input_cloud(cloud)?;

            if self.auto_build {
                octree.add_points_from_input_cloud()?;
            }
        }

        Ok(octree)
    }
}

/// Utility functions for working with octrees
pub mod utils {
    use super::*;
    use crate::common::{PointCloud, XYZ};
    use crate::error::PclResult;

    /// Create an octree search structure with optimal resolution
    ///
    /// The resolution is computed as a fraction of the cloud's bounding box diagonal
    pub fn create_octree_with_auto_resolution(
        cloud: &PointCloud<XYZ>,
        resolution_factor: f64,
    ) -> PclResult<OctreeSearchXYZ> {
        // For now, use a default resolution
        // TODO: Compute bounding box when we have point access
        let default_resolution = 0.1;
        let resolution = default_resolution * resolution_factor;

        let mut octree = OctreeSearchXYZ::new(resolution)?;
        octree.set_input_cloud(cloud)?;
        Ok(octree)
    }

    /// Downsample a point cloud using voxel centroids
    pub fn downsample_cloud(
        cloud: &PointCloud<XYZ>,
        voxel_size: f64,
    ) -> PclResult<PointCloud<XYZ>> {
        let mut octree = OctreeVoxelCentroidXYZ::new(voxel_size)?;
        octree.set_input_cloud(cloud)?;
        octree.add_points_from_input_cloud()?;
        octree.get_voxel_centroids()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_octree_search_builder() {
        let mut octree = OctreeSearchBuilder::new(0.1)
            .resolution(0.05)
            .build()
            .unwrap();

        assert_eq!(octree.resolution(), 0.05);
    }

    #[test]
    fn test_octree_voxel_centroid_builder() {
        let octree = OctreeVoxelCentroidBuilder::new(0.1)
            .auto_build(false)
            .build()
            .unwrap();

        assert!(!octree.has_input_cloud());
    }
}
