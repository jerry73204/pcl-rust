//! Polygonal prism data extraction
//!
//! This module provides functionality to extract points that lie within
//! a polygonal prism defined by a planar hull and height limits.

use crate::common::{PointCloud, XYZ};
use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Extract polygonal prism data for PointXYZ clouds
///
/// This algorithm extracts points that lie within a 3D polygonal prism.
/// The prism is defined by a 2D polygon (planar hull) and height limits
/// perpendicular to the polygon's plane.
pub struct ExtractPolygonalPrismDataXYZ {
    inner: UniquePtr<ffi::ExtractPolygonalPrismData_PointXYZ>,
}

impl ExtractPolygonalPrismDataXYZ {
    /// Create a new extract polygonal prism data instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_extract_polygonal_prism_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "ExtractPolygonalPrismData".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloud<XYZ>) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_polygonal_prism_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the planar hull that defines the polygon
    pub fn set_input_planar_hull(&mut self, hull: &PointCloud<XYZ>) -> PclResult<()> {
        if hull.empty() {
            return Err(PclError::invalid_point_cloud("Planar hull is empty"));
        }
        if hull.size() < 3 {
            return Err(PclError::invalid_point_cloud(
                "Planar hull must have at least 3 points to form a polygon",
            ));
        }
        ffi::set_input_planar_hull_polygonal_prism_xyz(self.inner.pin_mut(), hull.inner());
        Ok(())
    }

    /// Set the height limits perpendicular to the polygon plane
    pub fn set_height_limits(&mut self, min_height: f64, max_height: f64) -> PclResult<()> {
        if min_height >= max_height {
            return Err(PclError::invalid_parameters(
                "Invalid height limits",
                "min_height/max_height",
                "min_height < max_height",
                format!("min: {}, max: {}", min_height, max_height),
            ));
        }
        ffi::set_height_limits_polygonal_prism_xyz(self.inner.pin_mut(), min_height, max_height);
        Ok(())
    }

    /// Get the height limits
    pub fn height_limits(&self) -> (f64, f64) {
        let mut min_height = 0.0;
        let mut max_height = 0.0;
        ffi::get_height_limits_polygonal_prism_xyz(&self.inner, &mut min_height, &mut max_height);
        (min_height, max_height)
    }

    /// Extract points within the polygonal prism
    pub fn segment(&mut self) -> PclResult<Vec<i32>> {
        let result = ffi::segment_polygonal_prism_xyz(self.inner.pin_mut());
        Ok(result)
    }
}

impl Default for ExtractPolygonalPrismDataXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default ExtractPolygonalPrismDataXYZ")
    }
}

/// Builder for ExtractPolygonalPrismDataXYZ configuration
pub struct ExtractPolygonalPrismDataXYZBuilder {
    min_height: Option<f64>,
    max_height: Option<f64>,
}

impl ExtractPolygonalPrismDataXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            min_height: None,
            max_height: None,
        }
    }

    /// Set the height limits
    pub fn height_limits(mut self, min_height: f64, max_height: f64) -> Self {
        self.min_height = Some(min_height);
        self.max_height = Some(max_height);
        self
    }

    /// Build the ExtractPolygonalPrismDataXYZ instance
    pub fn build(self) -> PclResult<ExtractPolygonalPrismDataXYZ> {
        let mut prism = ExtractPolygonalPrismDataXYZ::new()?;

        if let (Some(min), Some(max)) = (self.min_height, self.max_height) {
            prism.set_height_limits(min, max)?;
        }

        Ok(prism)
    }
}

impl Default for ExtractPolygonalPrismDataXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polygonal_prism_creation() {
        let prism = ExtractPolygonalPrismDataXYZ::new();
        assert!(prism.is_ok());
    }

    #[test]
    fn test_polygonal_prism_builder() {
        let prism = ExtractPolygonalPrismDataXYZBuilder::new()
            .height_limits(0.1, 2.0)
            .build();

        assert!(prism.is_ok());

        let prism = prism.unwrap();
        let (min, max) = prism.height_limits();
        assert_eq!(min, 0.1);
        assert_eq!(max, 2.0);
    }

    #[test]
    fn test_invalid_height_limits() {
        let mut prism = ExtractPolygonalPrismDataXYZ::new().unwrap();

        // min >= max should fail
        assert!(prism.set_height_limits(2.0, 1.0).is_err());
        assert!(prism.set_height_limits(1.0, 1.0).is_err());
    }
}
