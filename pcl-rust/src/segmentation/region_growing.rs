//! Region growing segmentation algorithms
//!
//! This module provides region growing segmentation for both geometric and color-based
//! region growing algorithms.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
#[cfg(feature = "features")]
use crate::features::NormalCloud;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Region growing segmentation for PointXYZ clouds using surface normals
pub struct RegionGrowingXYZ {
    inner: UniquePtr<ffi::RegionGrowing_PointXYZ_Normal>,
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

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_region_growing_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Set the input normals for region growing
    #[cfg(feature = "features")]
    pub fn set_input_normals(&mut self, normals: &NormalCloud) -> PclResult<()> {
        if normals.is_empty() {
            return Err(PclError::invalid_point_cloud(
                "Input normals cloud is empty",
            ));
        }
        // Note: This requires NormalCloud to expose a method to get the raw pointer
        // For now, we'll skip this method since NormalCloud doesn't expose its inner pointer
        todo!("NormalCloud needs to expose a method to get the raw pointer")
    }

    /// Set the minimum cluster size
    pub fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum cluster size",
                "min_size",
                "positive value",
                min_size.to_string(),
            ));
        }
        ffi::set_min_cluster_size_region_growing_xyz(self.inner.pin_mut(), min_size);
        Ok(())
    }

    /// Get the minimum cluster size
    pub fn min_cluster_size(&self) -> i32 {
        ffi::get_min_cluster_size_region_growing_xyz(&self.inner)
    }

    /// Set the maximum cluster size
    pub fn set_max_cluster_size(&mut self, max_size: i32) -> PclResult<()> {
        if max_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum cluster size",
                "max_size",
                "positive value",
                max_size.to_string(),
            ));
        }
        ffi::set_max_cluster_size_region_growing_xyz(self.inner.pin_mut(), max_size);
        Ok(())
    }

    /// Get the maximum cluster size
    pub fn max_cluster_size(&self) -> i32 {
        ffi::get_max_cluster_size_region_growing_xyz(&self.inner)
    }

    /// Set the smoothness threshold in radians
    pub fn set_smoothness_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid smoothness threshold",
                "threshold",
                "non-negative value",
                threshold.to_string(),
            ));
        }
        ffi::set_smoothness_threshold_region_growing_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the smoothness threshold
    pub fn smoothness_threshold(&self) -> f32 {
        ffi::get_smoothness_threshold_region_growing_xyz(&self.inner)
    }

    /// Set the curvature threshold
    pub fn set_curvature_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid curvature threshold",
                "threshold",
                "non-negative value",
                threshold.to_string(),
            ));
        }
        ffi::set_curvature_threshold_region_growing_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the curvature threshold
    pub fn curvature_threshold(&self) -> f32 {
        ffi::get_curvature_threshold_region_growing_xyz(&self.inner)
    }

    /// Set the number of neighbors to analyze
    pub fn set_number_of_neighbours(&mut self, k: i32) -> PclResult<()> {
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid number of neighbors",
                "k",
                "positive value",
                k.to_string(),
            ));
        }
        ffi::set_number_of_neighbours_region_growing_xyz(self.inner.pin_mut(), k);
        Ok(())
    }

    /// Get the number of neighbors
    pub fn number_of_neighbours(&self) -> i32 {
        ffi::get_number_of_neighbours_region_growing_xyz(&self.inner)
    }

    /// Perform region growing segmentation
    pub fn extract(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let result = ffi::extract_region_growing_xyz(self.inner.pin_mut());
        if result.is_empty() {
            return Err(PclError::ProcessingFailed {
                message: "Region growing segmentation failed".to_string(),
            });
        }

        // Parse the result vector
        // Format: [cluster_count, cluster1_size, cluster1_indices..., cluster2_size, cluster2_indices..., ...]
        let mut clusters = Vec::new();
        let mut idx = 0;

        if idx >= result.len() {
            return Ok(clusters);
        }

        let cluster_count = result[idx] as usize;
        idx += 1;

        for _ in 0..cluster_count {
            if idx >= result.len() {
                break;
            }
            let cluster_size = result[idx] as usize;
            idx += 1;

            let mut cluster = Vec::with_capacity(cluster_size);
            for _ in 0..cluster_size {
                if idx >= result.len() {
                    break;
                }
                cluster.push(result[idx]);
                idx += 1;
            }
            clusters.push(cluster);
        }

        Ok(clusters)
    }
}

impl Default for RegionGrowingXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default RegionGrowingXYZ")
    }
}

/// Region growing segmentation for PointXYZRGB clouds based on color similarity
pub struct RegionGrowingRgbXYZRGB {
    inner: UniquePtr<ffi::RegionGrowingRGB_PointXYZRGB>,
}

impl RegionGrowingRgbXYZRGB {
    /// Create a new region growing RGB segmentation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_region_growing_rgb_xyzrgb();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "RegionGrowingRGB".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_region_growing_rgb_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Set the distance threshold
    pub fn set_distance_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid distance threshold",
                "threshold",
                "positive value",
                threshold.to_string(),
            ));
        }
        ffi::set_distance_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the distance threshold
    pub fn distance_threshold(&self) -> f32 {
        ffi::get_distance_threshold_region_growing_rgb_xyzrgb(&self.inner)
    }

    /// Set the point color threshold
    pub fn set_point_color_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid point color threshold",
                "threshold",
                "non-negative value",
                threshold.to_string(),
            ));
        }
        ffi::set_point_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the point color threshold
    pub fn point_color_threshold(&self) -> f32 {
        ffi::get_point_color_threshold_region_growing_rgb_xyzrgb(&self.inner)
    }

    /// Set the region color threshold
    pub fn set_region_color_threshold(&mut self, threshold: f32) -> PclResult<()> {
        if threshold < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid region color threshold",
                "threshold",
                "non-negative value",
                threshold.to_string(),
            ));
        }
        ffi::set_region_color_threshold_region_growing_rgb_xyzrgb(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the region color threshold
    pub fn region_color_threshold(&self) -> f32 {
        ffi::get_region_color_threshold_region_growing_rgb_xyzrgb(&self.inner)
    }

    /// Set the minimum cluster size
    pub fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum cluster size",
                "min_size",
                "positive value",
                min_size.to_string(),
            ));
        }
        ffi::set_min_cluster_size_region_growing_rgb_xyzrgb(self.inner.pin_mut(), min_size);
        Ok(())
    }

    /// Get the minimum cluster size
    pub fn min_cluster_size(&self) -> i32 {
        ffi::get_min_cluster_size_region_growing_rgb_xyzrgb(&self.inner)
    }

    /// Perform region growing RGB segmentation
    pub fn extract(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let result = ffi::extract_region_growing_rgb_xyzrgb(self.inner.pin_mut());
        if result.is_empty() {
            return Err(PclError::ProcessingFailed {
                message: "Region growing RGB segmentation failed".to_string(),
            });
        }

        // Parse the result vector (same format as RegionGrowingXYZ)
        let mut clusters = Vec::new();
        let mut idx = 0;

        if idx >= result.len() {
            return Ok(clusters);
        }

        let cluster_count = result[idx] as usize;
        idx += 1;

        for _ in 0..cluster_count {
            if idx >= result.len() {
                break;
            }
            let cluster_size = result[idx] as usize;
            idx += 1;

            let mut cluster = Vec::with_capacity(cluster_size);
            for _ in 0..cluster_size {
                if idx >= result.len() {
                    break;
                }
                cluster.push(result[idx]);
                idx += 1;
            }
            clusters.push(cluster);
        }

        Ok(clusters)
    }
}

impl Default for RegionGrowingRgbXYZRGB {
    fn default() -> Self {
        Self::new().expect("Failed to create default RegionGrowingRgbXYZRGB")
    }
}

/// Builder for RegionGrowingXYZ configuration
pub struct RegionGrowingXYZBuilder {
    min_cluster_size: Option<i32>,
    max_cluster_size: Option<i32>,
    smoothness_threshold: Option<f32>,
    curvature_threshold: Option<f32>,
    number_of_neighbours: Option<i32>,
}

impl RegionGrowingXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            min_cluster_size: None,
            max_cluster_size: None,
            smoothness_threshold: None,
            curvature_threshold: None,
            number_of_neighbours: None,
        }
    }

    /// Set the minimum cluster size
    pub fn min_cluster_size(mut self, size: i32) -> Self {
        self.min_cluster_size = Some(size);
        self
    }

    /// Set the maximum cluster size
    pub fn max_cluster_size(mut self, size: i32) -> Self {
        self.max_cluster_size = Some(size);
        self
    }

    /// Set the smoothness threshold
    pub fn smoothness_threshold(mut self, threshold: f32) -> Self {
        self.smoothness_threshold = Some(threshold);
        self
    }

    /// Set the curvature threshold
    pub fn curvature_threshold(mut self, threshold: f32) -> Self {
        self.curvature_threshold = Some(threshold);
        self
    }

    /// Set the number of neighbors
    pub fn number_of_neighbours(mut self, k: i32) -> Self {
        self.number_of_neighbours = Some(k);
        self
    }

    /// Build the RegionGrowingXYZ instance
    pub fn build(self) -> PclResult<RegionGrowingXYZ> {
        let mut rg = RegionGrowingXYZ::new()?;

        if let Some(size) = self.min_cluster_size {
            rg.set_min_cluster_size(size)?;
        }
        if let Some(size) = self.max_cluster_size {
            rg.set_max_cluster_size(size)?;
        }
        if let Some(threshold) = self.smoothness_threshold {
            rg.set_smoothness_threshold(threshold)?;
        }
        if let Some(threshold) = self.curvature_threshold {
            rg.set_curvature_threshold(threshold)?;
        }
        if let Some(k) = self.number_of_neighbours {
            rg.set_number_of_neighbours(k)?;
        }

        Ok(rg)
    }
}

impl Default for RegionGrowingXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_region_growing_creation() {
        let rg = RegionGrowingXYZ::new();
        assert!(rg.is_ok());
    }

    #[test]
    fn test_region_growing_rgb_creation() {
        let rg = RegionGrowingRgbXYZRGB::new();
        assert!(rg.is_ok());
    }

    #[test]
    fn test_region_growing_builder() {
        let rg = RegionGrowingXYZBuilder::new()
            .min_cluster_size(50)
            .max_cluster_size(10000)
            .smoothness_threshold(3.0 * std::f32::consts::PI / 180.0) // 3 degrees
            .curvature_threshold(1.0)
            .number_of_neighbours(30)
            .build();

        assert!(rg.is_ok());

        let rg = rg.unwrap();
        assert_eq!(rg.min_cluster_size(), 50);
        assert_eq!(rg.max_cluster_size(), 10000);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut rg = RegionGrowingXYZ::new().unwrap();

        assert!(rg.set_min_cluster_size(-1).is_err());
        assert!(rg.set_smoothness_threshold(-1.0).is_err());
        assert!(rg.set_number_of_neighbours(0).is_err());
    }
}
