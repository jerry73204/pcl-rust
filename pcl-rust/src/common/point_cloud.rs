//! Safe wrappers for PCL PointCloud container
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's PointCloud template.

use crate::error::PclResult;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// A container for 3D points with XYZ coordinates
pub struct PointCloudXYZ {
    inner: UniquePtr<ffi::PointCloud_PointXYZ>,
}

impl PointCloudXYZ {
    /// Create a new empty point cloud
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_point_cloud_xyz();
        Ok(Self { inner })
    }

    /// Get the number of points in the cloud
    pub fn size(&self) -> usize {
        ffi::size(&self.inner)
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        ffi::empty(&self.inner)
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        ffi::clear(self.inner.pin_mut());
        Ok(())
    }

    /// Get a reference to the underlying pcl-sys point cloud
    pub fn as_raw(&self) -> &ffi::PointCloud_PointXYZ {
        &self.inner
    }
}

impl Default for PointCloudXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloudXYZ")
    }
}

impl std::fmt::Debug for PointCloudXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloudXYZ")
            .field("size", &self.size())
            .field("empty", &self.empty())
            .finish()
    }
}

/// A container for 3D points with XYZ coordinates and RGB color
pub struct PointCloudXYZRGB {
    inner: UniquePtr<ffi::PointCloud_PointXYZRGB>,
}

impl PointCloudXYZRGB {
    /// Create a new empty point cloud
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_point_cloud_xyzrgb();
        Ok(Self { inner })
    }

    /// Get the number of points in the cloud
    pub fn size(&self) -> usize {
        ffi::size_xyzrgb(&self.inner)
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        ffi::empty_xyzrgb(&self.inner)
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        ffi::clear_xyzrgb(self.inner.pin_mut());
        Ok(())
    }

    /// Get a reference to the underlying pcl-sys point cloud
    pub fn as_raw(&self) -> &ffi::PointCloud_PointXYZRGB {
        &self.inner
    }
}

impl Default for PointCloudXYZRGB {
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloudXYZRGB")
    }
}

impl std::fmt::Debug for PointCloudXYZRGB {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloudXYZRGB")
            .field("size", &self.size())
            .field("empty", &self.empty())
            .finish()
    }
}
