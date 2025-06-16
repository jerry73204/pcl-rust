//! Min-cut segmentation algorithm
//!
//! This module provides min-cut segmentation for foreground/background
//! separation in point clouds.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Min-cut segmentation for PointXYZ clouds
///
/// This algorithm uses graph-based min-cut to separate foreground from background
/// points based on user-provided foreground points.
pub struct MinCutSegmentationXYZ {
    inner: UniquePtr<ffi::MinCutSegmentation_PointXYZ>,
}

impl MinCutSegmentationXYZ {
    /// Create a new min-cut segmentation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_min_cut_segmentation_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "MinCutSegmentation".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_min_cut_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Set the foreground points
    pub fn set_foreground_points(&mut self, foreground: &PointCloudXYZ) -> PclResult<()> {
        if foreground.empty() {
            return Err(PclError::invalid_point_cloud("Foreground points are empty"));
        }
        ffi::set_foreground_points_min_cut_xyz(self.inner.pin_mut(), foreground.as_raw());
        Ok(())
    }

    /// Set the sigma value (edge weight = exp(-dist²/sigma²))
    pub fn set_sigma(&mut self, sigma: f64) -> PclResult<()> {
        if sigma <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid sigma value",
                "sigma",
                "positive value",
                &sigma.to_string(),
            ));
        }
        ffi::set_sigma_min_cut_xyz(self.inner.pin_mut(), sigma);
        Ok(())
    }

    /// Get the sigma value
    pub fn sigma(&self) -> f64 {
        ffi::get_sigma_min_cut_xyz(&self.inner)
    }

    /// Set the radius for building the graph
    pub fn set_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid radius",
                "radius",
                "positive value",
                &radius.to_string(),
            ));
        }
        ffi::set_radius_min_cut_xyz(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Get the radius
    pub fn radius(&self) -> f64 {
        ffi::get_radius_min_cut_xyz(&self.inner)
    }

    /// Set the number of neighbors for graph construction
    pub fn set_number_of_neighbours(&mut self, k: i32) -> PclResult<()> {
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid number of neighbors",
                "k",
                "positive value",
                &k.to_string(),
            ));
        }
        ffi::set_number_of_neighbours_min_cut_xyz(self.inner.pin_mut(), k);
        Ok(())
    }

    /// Get the number of neighbors
    pub fn number_of_neighbours(&self) -> i32 {
        ffi::get_number_of_neighbours_min_cut_xyz(&self.inner)
    }

    /// Set the source weight
    pub fn set_source_weight(&mut self, weight: f64) -> PclResult<()> {
        if weight < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid source weight",
                "weight",
                "non-negative value",
                &weight.to_string(),
            ));
        }
        ffi::set_source_weight_min_cut_xyz(self.inner.pin_mut(), weight);
        Ok(())
    }

    /// Get the source weight
    pub fn source_weight(&self) -> f64 {
        ffi::get_source_weight_min_cut_xyz(&self.inner)
    }

    /// Extract foreground and background clusters
    pub fn extract(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let result = ffi::extract_min_cut_xyz(self.inner.pin_mut());
        if result.is_empty() {
            return Err(PclError::ProcessingFailed {
                message: "Min-cut segmentation failed".to_string(),
            });
        }

        // Parse the result vector
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

impl Default for MinCutSegmentationXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default MinCutSegmentationXYZ")
    }
}

/// Builder for MinCutSegmentationXYZ configuration
pub struct MinCutSegmentationXYZBuilder {
    sigma: Option<f64>,
    radius: Option<f64>,
    number_of_neighbours: Option<i32>,
    source_weight: Option<f64>,
}

impl MinCutSegmentationXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            sigma: None,
            radius: None,
            number_of_neighbours: None,
            source_weight: None,
        }
    }

    /// Set the sigma value
    pub fn sigma(mut self, sigma: f64) -> Self {
        self.sigma = Some(sigma);
        self
    }

    /// Set the radius
    pub fn radius(mut self, radius: f64) -> Self {
        self.radius = Some(radius);
        self
    }

    /// Set the number of neighbors
    pub fn number_of_neighbours(mut self, k: i32) -> Self {
        self.number_of_neighbours = Some(k);
        self
    }

    /// Set the source weight
    pub fn source_weight(mut self, weight: f64) -> Self {
        self.source_weight = Some(weight);
        self
    }

    /// Build the MinCutSegmentationXYZ instance
    pub fn build(self) -> PclResult<MinCutSegmentationXYZ> {
        let mut mc = MinCutSegmentationXYZ::new()?;

        if let Some(sigma) = self.sigma {
            mc.set_sigma(sigma)?;
        }
        if let Some(radius) = self.radius {
            mc.set_radius(radius)?;
        }
        if let Some(k) = self.number_of_neighbours {
            mc.set_number_of_neighbours(k)?;
        }
        if let Some(weight) = self.source_weight {
            mc.set_source_weight(weight)?;
        }

        Ok(mc)
    }
}

impl Default for MinCutSegmentationXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min_cut_creation() {
        let mc = MinCutSegmentationXYZ::new();
        assert!(mc.is_ok());
    }

    #[test]
    fn test_min_cut_builder() {
        let mc = MinCutSegmentationXYZBuilder::new()
            .sigma(0.25)
            .radius(3.0)
            .number_of_neighbours(14)
            .source_weight(0.8)
            .build();

        assert!(mc.is_ok());

        let mc = mc.unwrap();
        assert_eq!(mc.sigma(), 0.25);
        assert_eq!(mc.radius(), 3.0);
        assert_eq!(mc.number_of_neighbours(), 14);
        assert_eq!(mc.source_weight(), 0.8);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut mc = MinCutSegmentationXYZ::new().unwrap();

        assert!(mc.set_sigma(-1.0).is_err());
        assert!(mc.set_radius(0.0).is_err());
        assert!(mc.set_number_of_neighbours(-1).is_err());
        assert!(mc.set_source_weight(-0.1).is_err());
    }
}
