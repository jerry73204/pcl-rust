//! Builders for PointCloud using the new API
//!
//! These builders provide a convenient way to create point clouds.

use crate::common::point_cloud::PointCloud;
use crate::common::point_types::{Normal, PointNormal, PointXYZ, PointXYZRGB, XYZ, XYZRGB};
use crate::error::PclResult;

/// Builder for XYZ point clouds using the new API
pub struct PointCloudXYZBuilder {
    points: Vec<PointXYZ>,
}

impl PointCloudXYZBuilder {
    /// Create a new XYZ builder
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a point from coordinates
    pub fn point(mut self, x: f32, y: f32, z: f32) -> Self {
        self.points.push(PointXYZ::new(x, y, z));
        self
    }

    /// Add a point from coordinates (alias for backwards compatibility)
    pub fn add_point(self, x: f32, y: f32, z: f32) -> Self {
        self.point(x, y, z)
    }

    /// Add multiple points from coordinates tuples
    pub fn add_points(mut self, points: Vec<(f32, f32, f32)>) -> Self {
        for (x, y, z) in points {
            self.points.push(PointXYZ::new(x, y, z));
        }
        self
    }

    /// Add a point from struct
    pub fn point_struct(mut self, point: PointXYZ) -> Self {
        self.points.push(point);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloud<XYZ>> {
        let mut cloud = PointCloud::new()?;
        for point in self.points {
            cloud.push(point)?;
        }
        Ok(cloud)
    }
}

impl Default for PointCloudXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for XYZRGB point clouds using the new API
pub struct PointCloudXYZRGBBuilder {
    points: Vec<PointXYZRGB>,
}

impl PointCloudXYZRGBBuilder {
    /// Create a new XYZRGB builder
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a colored point
    pub fn point(mut self, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        self.points.push(PointXYZRGB::new(x, y, z, r, g, b));
        self
    }

    /// Add a point from struct
    pub fn point_struct(mut self, point: PointXYZRGB) -> Self {
        self.points.push(point);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloud<XYZRGB>> {
        let mut cloud = PointCloud::new()?;
        for point in self.points {
            cloud.push(point)?;
        }
        Ok(cloud)
    }
}

impl Default for PointCloudXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for Normal point clouds using the new API
pub struct PointCloudNormalBuilder {
    points: Vec<PointNormal>,
}

impl PointCloudNormalBuilder {
    /// Create a new Normal builder
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a point with normal
    pub fn point(mut self, x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        self.points.push(PointNormal::new(x, y, z, nx, ny, nz));
        self
    }

    /// Add a point from struct
    pub fn point_struct(mut self, point: PointNormal) -> Self {
        self.points.push(point);
        self
    }

    /// Build the point cloud
    pub fn build(self) -> PclResult<PointCloud<Normal>> {
        let mut cloud = PointCloud::new()?;
        for point in self.points {
            cloud.push(point)?;
        }
        Ok(cloud)
    }
}

impl Default for PointCloudNormalBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_xyz_builder() {
        let cloud = PointCloudXYZBuilder::new()
            .point(1.0, 2.0, 3.0)
            .point(4.0, 5.0, 6.0)
            .build()
            .unwrap();

        assert_eq!(cloud.size(), 2);
    }

    #[test]
    fn test_xyzrgb_builder() {
        let cloud = PointCloudXYZRGBBuilder::new()
            .point(1.0, 2.0, 3.0, 255, 0, 0)
            .point(4.0, 5.0, 6.0, 0, 255, 0)
            .build()
            .unwrap();

        assert_eq!(cloud.size(), 2);
    }

    #[test]
    fn test_normal_builder() {
        let cloud = PointCloudNormalBuilder::new()
            .point(1.0, 2.0, 3.0, 0.0, 0.0, 1.0)
            .point(4.0, 5.0, 6.0, 0.0, 1.0, 0.0)
            .build()
            .unwrap();

        assert_eq!(cloud.size(), 2);
    }
}
