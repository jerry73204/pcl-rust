//! Clustering algorithms for point cloud segmentation
//!
//! This module provides clustering algorithms such as Euclidean clustering
//! for grouping nearby points into clusters.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Trait for clustering algorithms
pub trait ClusteringXYZ {
    /// Set the cluster tolerance (maximum distance between points in a cluster)
    fn set_cluster_tolerance(&mut self, tolerance: f64) -> PclResult<()>;

    /// Get the cluster tolerance
    fn cluster_tolerance(&mut self) -> f64;

    /// Set the minimum cluster size
    fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()>;

    /// Get the minimum cluster size
    fn min_cluster_size(&mut self) -> i32;

    /// Set the maximum cluster size
    fn set_max_cluster_size(&mut self, max_size: i32) -> PclResult<()>;

    /// Get the maximum cluster size
    fn max_cluster_size(&mut self) -> i32;
}

/// Euclidean cluster extraction for PointXYZ clouds
///
/// This algorithm groups points that are close to each other based on Euclidean distance.
/// It's useful for separating different objects in a point cloud.
pub struct EuclideanClusterExtractionXYZ {
    inner: UniquePtr<ffi::EuclideanClusterExtraction_PointXYZ>,
}

impl EuclideanClusterExtractionXYZ {
    /// Create a new Euclidean cluster extraction instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_euclidean_cluster_extraction_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "EuclideanClusterExtraction".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Create a new instance with builder pattern
    pub fn builder() -> EuclideanClusterExtractionBuilder {
        EuclideanClusterExtractionBuilder::new()
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_euclidean_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Extract clusters from the point cloud
    pub fn extract(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let flat_data = ffi::extract_euclidean_clusters_xyz(self.inner.pin_mut());
        Self::parse_cluster_data(flat_data)
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

impl ClusteringXYZ for EuclideanClusterExtractionXYZ {
    fn set_cluster_tolerance(&mut self, tolerance: f64) -> PclResult<()> {
        if tolerance <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid cluster tolerance",
                "cluster_tolerance",
                "positive value",
                &tolerance.to_string(),
            ));
        }
        ffi::set_cluster_tolerance_euclidean_xyz(self.inner.pin_mut(), tolerance);
        Ok(())
    }

    fn cluster_tolerance(&mut self) -> f64 {
        ffi::get_cluster_tolerance_euclidean_xyz(self.inner.pin_mut())
    }

    fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum cluster size",
                "min_cluster_size",
                "positive value",
                &min_size.to_string(),
            ));
        }
        ffi::set_min_cluster_size_euclidean_xyz(self.inner.pin_mut(), min_size);
        Ok(())
    }

    fn min_cluster_size(&mut self) -> i32 {
        ffi::get_min_cluster_size_euclidean_xyz(&self.inner)
    }

    fn set_max_cluster_size(&mut self, max_size: i32) -> PclResult<()> {
        if max_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum cluster size",
                "max_cluster_size",
                "positive value",
                &max_size.to_string(),
            ));
        }
        ffi::set_max_cluster_size_euclidean_xyz(self.inner.pin_mut(), max_size);
        Ok(())
    }

    fn max_cluster_size(&mut self) -> i32 {
        ffi::get_max_cluster_size_euclidean_xyz(&self.inner)
    }
}

/// Builder for EuclideanClusterExtractionXYZ
pub struct EuclideanClusterExtractionBuilder {
    cluster_tolerance: Option<f64>,
    min_cluster_size: Option<i32>,
    max_cluster_size: Option<i32>,
}

impl EuclideanClusterExtractionBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            cluster_tolerance: None,
            min_cluster_size: None,
            max_cluster_size: None,
        }
    }

    /// Set the cluster tolerance
    pub fn cluster_tolerance(mut self, tolerance: f64) -> Self {
        self.cluster_tolerance = Some(tolerance);
        self
    }

    /// Set the minimum cluster size
    pub fn min_cluster_size(mut self, min_size: i32) -> Self {
        self.min_cluster_size = Some(min_size);
        self
    }

    /// Set the maximum cluster size
    pub fn max_cluster_size(mut self, max_size: i32) -> Self {
        self.max_cluster_size = Some(max_size);
        self
    }

    /// Build the EuclideanClusterExtraction instance
    pub fn build(self) -> PclResult<EuclideanClusterExtractionXYZ> {
        let mut clustering = EuclideanClusterExtractionXYZ::new()?;

        if let Some(tolerance) = self.cluster_tolerance {
            clustering.set_cluster_tolerance(tolerance)?;
        }

        if let Some(min_size) = self.min_cluster_size {
            clustering.set_min_cluster_size(min_size)?;
        }

        if let Some(max_size) = self.max_cluster_size {
            clustering.set_max_cluster_size(max_size)?;
        }

        Ok(clustering)
    }
}

impl Default for EuclideanClusterExtractionBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for EuclideanClusterExtractionXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default EuclideanClusterExtractionXYZ")
    }
}

/// Conditional Euclidean clustering for PointXYZ clouds
///
/// This performs clustering based on both spatial distance and custom conditions.
/// Currently uses a simplified distance-based condition.
pub struct ConditionalEuclideanClusteringXYZ {
    inner: UniquePtr<ffi::ConditionalEuclideanClustering_PointXYZ>,
}

impl ConditionalEuclideanClusteringXYZ {
    /// Create a new conditional Euclidean clustering instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_conditional_euclidean_clustering_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "ConditionalEuclideanClustering".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_conditional_euclidean_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Set the cluster tolerance
    pub fn set_cluster_tolerance(&mut self, tolerance: f32) -> PclResult<()> {
        if tolerance <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid cluster tolerance",
                "tolerance",
                "positive value",
                &tolerance.to_string(),
            ));
        }
        ffi::set_cluster_tolerance_conditional_euclidean_xyz(self.inner.pin_mut(), tolerance);
        Ok(())
    }

    /// Get the cluster tolerance
    pub fn cluster_tolerance(&self) -> f32 {
        ffi::get_cluster_tolerance_conditional_euclidean_xyz(&self.inner)
    }

    /// Set the minimum cluster size
    pub fn set_min_cluster_size(&mut self, min_size: i32) -> PclResult<()> {
        if min_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum cluster size",
                "min_size",
                "positive value",
                &min_size.to_string(),
            ));
        }
        ffi::set_min_cluster_size_conditional_euclidean_xyz(self.inner.pin_mut(), min_size);
        Ok(())
    }

    /// Get the minimum cluster size
    pub fn min_cluster_size(&mut self) -> i32 {
        ffi::get_min_cluster_size_conditional_euclidean_xyz(self.inner.pin_mut())
    }

    /// Set the maximum cluster size
    pub fn set_max_cluster_size(&mut self, max_size: i32) -> PclResult<()> {
        if max_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum cluster size",
                "max_size",
                "positive value",
                &max_size.to_string(),
            ));
        }
        ffi::set_max_cluster_size_conditional_euclidean_xyz(self.inner.pin_mut(), max_size);
        Ok(())
    }

    /// Get the maximum cluster size
    pub fn max_cluster_size(&mut self) -> i32 {
        ffi::get_max_cluster_size_conditional_euclidean_xyz(self.inner.pin_mut())
    }

    /// Extract clusters with conditional constraints
    pub fn extract(&mut self) -> PclResult<Vec<Vec<i32>>> {
        let result = ffi::segment_conditional_euclidean_xyz(self.inner.pin_mut());
        if result.is_empty() {
            return Err(PclError::ProcessingFailed {
                message: "Conditional Euclidean clustering failed".to_string(),
            });
        }

        // Parse the result vector (same format as EuclideanClusterExtraction)
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

impl Default for ConditionalEuclideanClusteringXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default ConditionalEuclideanClusteringXYZ")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_euclidean_cluster_creation() {
        let ece = EuclideanClusterExtractionXYZ::new();
        assert!(ece.is_ok());
    }

    #[test]
    fn test_conditional_euclidean_creation() {
        let cec = ConditionalEuclideanClusteringXYZ::new();
        assert!(cec.is_ok());
    }

    #[test]
    fn test_euclidean_cluster_builder() {
        let ece = EuclideanClusterExtractionBuilder::new()
            .cluster_tolerance(0.02)
            .min_cluster_size(100)
            .max_cluster_size(25000)
            .build();

        assert!(ece.is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut ece = EuclideanClusterExtractionXYZ::new().unwrap();

        assert!(ece.set_cluster_tolerance(-0.1).is_err());
        assert!(ece.set_min_cluster_size(0).is_err());
        assert!(ece.set_max_cluster_size(-10).is_err());
    }
}
