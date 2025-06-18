//! Builder patterns for working with PCL data structures
//!
//! Since direct point creation is not supported due to cxx limitations,
//! this module provides builder patterns and utility functions to work
//! around these limitations.

use crate::error::PclResult;
use crate::{PointCloudNormal, PointCloudXYZ, PointCloudXYZRGB};

/// Builder for creating and populating PointCloudXYZ
pub struct PointCloudXYZBuilder {
    points: Vec<(f32, f32, f32)>,
    width: Option<u32>,
    height: Option<u32>,
}

impl PointCloudXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            width: None,
            height: None,
        }
    }

    /// Add a point to the cloud
    pub fn add_point(mut self, x: f32, y: f32, z: f32) -> Self {
        self.points.push((x, y, z));
        self
    }

    /// Add multiple points to the cloud
    pub fn add_points(mut self, points: impl IntoIterator<Item = (f32, f32, f32)>) -> Self {
        self.points.extend(points);
        self
    }

    /// Set the width (for organized clouds)
    pub fn width(mut self, width: u32) -> Self {
        self.width = Some(width);
        self
    }

    /// Set the height (for organized clouds)
    pub fn height(mut self, height: u32) -> Self {
        self.height = Some(height);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloudXYZ> {
        let mut cloud = PointCloudXYZ::new()?;

        // Reserve space for efficiency
        if !self.points.is_empty() {
            cloud.reserve(self.points.len())?;
        }

        // Add all points using the push method
        for (x, y, z) in self.points {
            cloud.push(x, y, z)?;
        }

        Ok(cloud)
    }
}

impl Default for PointCloudXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for creating and populating PointCloudXYZRGB
pub struct PointCloudXYZRGBBuilder {
    points: Vec<(f32, f32, f32, u8, u8, u8)>,
    width: Option<u32>,
    height: Option<u32>,
}

impl PointCloudXYZRGBBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            width: None,
            height: None,
        }
    }

    /// Add a point to the cloud with RGB color
    pub fn add_point_rgb(mut self, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        self.points.push((x, y, z, r, g, b));
        self
    }

    /// Add a point to the cloud (alias for add_point_rgb)
    pub fn add_point(mut self, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        self.points.push((x, y, z, r, g, b));
        self
    }

    /// Add multiple points to the cloud
    pub fn add_points(
        mut self,
        points: impl IntoIterator<Item = (f32, f32, f32, u8, u8, u8)>,
    ) -> Self {
        self.points.extend(points);
        self
    }

    /// Set the width (for organized clouds)
    pub fn width(mut self, width: u32) -> Self {
        self.width = Some(width);
        self
    }

    /// Set the height (for organized clouds)
    pub fn height(mut self, height: u32) -> Self {
        self.height = Some(height);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloudXYZRGB> {
        let mut cloud = PointCloudXYZRGB::new()?;

        // Reserve space for efficiency
        if !self.points.is_empty() {
            cloud.reserve(self.points.len())?;
        }

        // Add all points using the push method
        for (x, y, z, r, g, b) in self.points {
            cloud.push(x, y, z, r, g, b)?;
        }

        Ok(cloud)
    }
}

impl Default for PointCloudXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for creating and populating PointCloudNormal
pub struct PointCloudNormalBuilder {
    points: Vec<(f32, f32, f32, f32, f32, f32)>,
    width: Option<u32>,
    height: Option<u32>,
}

impl PointCloudNormalBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            width: None,
            height: None,
        }
    }

    /// Add a point with normal to the cloud
    pub fn add_point_normal(mut self, x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        self.points.push((x, y, z, nx, ny, nz));
        self
    }

    /// Add a point to the cloud (alias for add_point_normal)
    pub fn add_point(mut self, x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        self.points.push((x, y, z, nx, ny, nz));
        self
    }

    /// Add multiple points to the cloud
    pub fn add_points(
        mut self,
        points: impl IntoIterator<Item = (f32, f32, f32, f32, f32, f32)>,
    ) -> Self {
        self.points.extend(points);
        self
    }

    /// Set the width (for organized clouds)
    pub fn width(mut self, width: u32) -> Self {
        self.width = Some(width);
        self
    }

    /// Set the height (for organized clouds)
    pub fn height(mut self, height: u32) -> Self {
        self.height = Some(height);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloudNormal> {
        let mut cloud = PointCloudNormal::new()?;

        // Reserve space for efficiency
        if !self.points.is_empty() {
            cloud.reserve(self.points.len())?;
        }

        // Add all points using the push method
        for (x, y, z, nx, ny, nz) in self.points {
            cloud.push_with_normal(x, y, z, nx, ny, nz)?;
        }

        Ok(cloud)
    }
}

impl Default for PointCloudNormalBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Utility functions for working with point clouds
pub mod utils {
    use super::*;

    /// Create a point cloud with pre-allocated capacity
    pub fn create_cloud_with_capacity(capacity: usize) -> PclResult<PointCloudXYZ> {
        let mut cloud = PointCloudXYZ::new()?;
        cloud.reserve(capacity)?;
        Ok(cloud)
    }

    /// Create a point cloud with specific dimensions
    pub fn create_organized_cloud(width: u32, height: u32) -> PclResult<PointCloudXYZ> {
        let mut cloud = PointCloudXYZ::new()?;
        let size = (width * height) as usize;
        cloud.resize(size)?;
        // Note: We cannot set width/height from Rust currently
        Ok(cloud)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_cloud_builder() {
        let cloud = PointCloudXYZBuilder::new()
            .add_point(1.0, 2.0, 3.0)
            .add_point(4.0, 5.0, 6.0)
            .add_points(vec![(7.0, 8.0, 9.0), (10.0, 11.0, 12.0)])
            .build()
            .unwrap();

        // The cloud will have the right size but default values
        assert_eq!(cloud.size(), 4);
    }

    #[test]
    fn test_rgb_cloud_builder() {
        let cloud = PointCloudXYZRGBBuilder::new()
            .add_point(1.0, 2.0, 3.0, 255, 0, 0)
            .add_point(4.0, 5.0, 6.0, 0, 255, 0)
            .build()
            .unwrap();

        assert_eq!(cloud.size(), 2);
    }

    #[test]
    fn test_normal_cloud_builder() {
        let cloud = PointCloudNormalBuilder::new()
            .add_point_normal(1.0, 2.0, 3.0, 0.0, 0.0, 1.0)
            .add_point_normal(4.0, 5.0, 6.0, 0.0, 1.0, 0.0)
            .build()
            .unwrap();

        assert_eq!(cloud.size(), 2);
    }
}
