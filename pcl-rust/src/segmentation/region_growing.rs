//! Region growing segmentation algorithms
//!
//! This module provides region growing segmentation for both geometric and color-based
//! region growing algorithms.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Trait for segmentation algorithms that work with PointXYZ clouds
pub trait SegmentationXYZ {
    /// Set the input point cloud
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()>;

    /// Perform segmentation and return cluster indices
    fn segment(&mut self) -> PclResult<Vec<Vec<i32>>>;
}

/// Trait for segmentation algorithms that work with PointXYZRGB clouds
pub trait SegmentationXYZRGB {
    /// Set the input point cloud
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()>;

    /// Perform segmentation and return cluster indices
    fn segment(&mut self) -> PclResult<Vec<Vec<i32>>>;
}

/// Region growing segmentation for PointXYZ clouds using surface normals
pub struct RegionGrowingXYZ {
    inner: UniquePtr<pcl_sys::RegionGrowingXYZ>,
}

impl RegionGrowingXYZ {
    /// Create a new region growing segmentation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_region_growing_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "RegionGrowing".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input normals for region growing
    pub fn set_input_normals(&mut self, normals: &PointCloudNormal) -> PclResult<()> {
        ffi::set_input_normals_region_growing_xyz(self.inner.pin_mut(), &normals.inner);
        Ok(())
    }

    /// Set the minimum cluster size
    pub fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::InvalidParameter {
                param: "min_cluster_size".to_string(),
                message: "Must be positive".to_string(),
            });
        }
        ffi::set_min_cluster_size_region_growing_xyz(self.inner.pin_mut(), min_size);
        Ok(())
    }

    /// Get the minimum cluster size
    pub fn min_cluster_size(&mut self) -> i32 {
        ffi::get_min_cluster_size_region_growing_xyz(self.inner.pin_mut())
    }

    /// Set the maximum cluster size
    pub fn set_max_cluster_size(&mut self, max_size: i32) -> PclResult<()> {
        if max_size <= 0 {
            return Err(PclError::InvalidParameter {
                param: "max_cluster_size".to_string(),
                message: "Must be positive".to_string(),
            });
        }
        ffi::set_max_cluster_size_region_growing_xyz(self.inner.pin_mut(), max_size);
        Ok(())
    }

    /// Get the maximum cluster size
    pub fn max_cluster_size(&mut self) -> i32 {
        ffi::get_max_cluster_size_region_growing_xyz(self.inner.pin_mut())
    }

    /// Set the smoothness threshold (in radians)
    pub fn set_smoothness_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::InvalidParameter {
                param: "smoothness_threshold".to_string(),
                message: "Must be non-negative".to_string(),
            });
        }
        ffi::set_smoothness_threshold_region_growing_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the smoothness threshold
    pub fn smoothness_threshold(&mut self) -> f32 {
        ffi::get_smoothness_threshold_region_growing_xyz(self.inner.pin_mut())
    }

    /// Set the curvature threshold
    pub fn set_curvature_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::InvalidParameter {
                param: "curvature_threshold".to_string(),
                message: "Must be non-negative".to_string(),
            });
        }
        ffi::set_curvature_threshold_region_growing_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the curvature threshold
    pub fn curvature_threshold(&mut self) -> f32 {
        ffi::get_curvature_threshold_region_growing_xyz(self.inner.pin_mut())
    }

    /// Set the number of neighbours for region growing
    pub fn set_number_of_neighbours(&mut self, k: i32) -> PclResult<()> {
        if k <= 0 {
            return Err(PclError::InvalidParameter {
                param: "number_of_neighbours".to_string(),
                message: "Must be positive".to_string(),
            });
        }
        ffi::set_number_of_neighbours_region_growing_xyz(self.inner.pin_mut(), k);
        Ok(())
    }

    /// Get the number of neighbours
    pub fn number_of_neighbours(&mut self) -> i32 {
        ffi::get_number_of_neighbours_region_growing_xyz(self.inner.pin_mut())
    }

    /// Estimate normals for the input cloud
    pub fn estimate_normals(cloud: &PointCloudXYZ, radius: f64) -> PclResult<PointCloudNormal> {
        if radius <= 0.0 {
            return Err(PclError::InvalidParameter {
                param: "radius".to_string(),
                message: "Must be positive".to_string(),
            });
        }

        let normals = ffi::estimate_normals_xyz(&cloud.inner, radius);
        if normals.is_null() {
            return Err(PclError::ProcessingFailed {
                message: "Failed to estimate normals".to_string(),
            });
        }

        Ok(PointCloudNormal { inner: normals })
    }

    /// Parse flattened cluster data into Vec<Vec<i32>>
    fn parse_cluster_data(flat_data: Vec<i32>) -> PclResult<Vec<Vec<i32>>> {
        if flat_data.is_empty() {
            return Ok(Vec::new());
        }

        let cluster_count = flat_data[0] as usize;
        let mut clusters = Vec::with_capacity(cluster_count);
        let mut index = 1;

        for _ in 0..cluster_count {
            if index >= flat_data.len() {
                return Err(PclError::ProcessingFailed {
                    message: "Invalid cluster data format".to_string(),
                });
            }

            let cluster_size = flat_data[index] as usize;
            index += 1;

            if index + cluster_size > flat_data.len() {
                return Err(PclError::ProcessingFailed {
                    message: "Invalid cluster data format".to_string(),
                });
            }

            let cluster: Vec<i32> = flat_data[index..index + cluster_size].to_vec();
            clusters.push(cluster);
            index += cluster_size;
        }

        Ok(clusters)
    }
}

impl SegmentationXYZ for RegionGrowingXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::InvalidPointCloud {
                message: "Input cloud is empty".to_string(),
                source: None,
            });
        }
        ffi::set_input_cloud_region_growing_xyz(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn segment(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let flat_data = ffi::extract_region_growing_xyz(self.inner.pin_mut());
        Self::parse_cluster_data(flat_data)
    }
}

/// Region growing RGB segmentation for PointXYZRGB clouds
pub struct RegionGrowingRgbXYZRGB {
    inner: UniquePtr<pcl_sys::RegionGrowingRgbXYZRGB>,
}

impl RegionGrowingRgbXYZRGB {
    /// Create a new RGB region growing segmentation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_region_growing_rgb_xyzrgb();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "RegionGrowingRGB".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the distance threshold for neighbor search
    pub fn set_distance_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold <= 0.0 {
            return Err(PclError::InvalidParameter {
                param: "distance_threshold".to_string(),
                message: "Must be positive".to_string(),
            });
        }
        ffi::set_distance_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the distance threshold
    pub fn distance_threshold(&mut self) -> f32 {
        ffi::get_distance_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut())
    }

    /// Set the point color threshold
    pub fn set_point_color_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::InvalidParameter {
                param: "point_color_threshold".to_string(),
                message: "Must be non-negative".to_string(),
            });
        }
        ffi::set_point_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the point color threshold
    pub fn point_color_threshold(&mut self) -> f32 {
        ffi::get_point_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut())
    }

    /// Set the region color threshold
    pub fn set_region_color_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::InvalidParameter {
                param: "region_color_threshold".to_string(),
                message: "Must be non-negative".to_string(),
            });
        }
        ffi::set_region_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the region color threshold
    pub fn region_color_threshold(&mut self) -> f32 {
        ffi::get_region_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut())
    }

    /// Set the minimum cluster size
    pub fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::InvalidParameter {
                param: "min_cluster_size".to_string(),
                message: "Must be positive".to_string(),
            });
        }
        ffi::set_min_cluster_size_region_growing_rgb_xyzrgb(self.inner.pin_mut(), min_size);
        Ok(())
    }

    /// Get the minimum cluster size
    pub fn min_cluster_size(&mut self) -> i32 {
        ffi::get_min_cluster_size_region_growing_rgb_xyzrgb(self.inner.pin_mut())
    }
}

impl SegmentationXYZRGB for RegionGrowingRgbXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::InvalidPointCloud {
                message: "Input cloud is empty".to_string(),
                source: None,
            });
        }
        ffi::set_input_cloud_region_growing_rgb_xyzrgb(self.inner.pin_mut(), &cloud.inner);
        Ok(())
    }

    fn segment(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let flat_data = ffi::extract_region_growing_rgb_xyzrgb(self.inner.pin_mut());
        RegionGrowingXYZ::parse_cluster_data(flat_data)
    }
}

/// Wrapper for PointCloud<Normal> from pcl-sys
pub struct PointCloudNormal {
    pub(crate) inner: UniquePtr<pcl_sys::PointCloudNormal>,
}

impl PointCloudNormal {
    /// Check if the normal cloud is empty
    pub fn empty(&self) -> bool {
        // We can't directly check this through FFI, assume non-null means not empty
        self.inner.is_null()
    }
}
