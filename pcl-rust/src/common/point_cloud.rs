//! Safe wrappers for PCL PointCloud container
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's PointCloud template.

use crate::error::PclResult;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// A container for 3D points with XYZ coordinates
pub struct PointCloudXYZ {
    pub(crate) inner: UniquePtr<ffi::PointCloud_PointXYZ>,
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

    /// Reserve capacity for at least n points
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        ffi::reserve_xyz(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Resize the point cloud to contain n points
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        ffi::resize_xyz(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Get the width of the point cloud (for organized clouds)
    pub fn width(&self) -> u32 {
        ffi::width(&self.inner)
    }

    /// Get the height of the point cloud (for organized clouds)
    pub fn height(&self) -> u32 {
        ffi::height(&self.inner)
    }

    /// Check if the point cloud is dense (no invalid points)
    pub fn is_dense(&self) -> bool {
        ffi::is_dense(&self.inner)
    }

    /// Check if the point cloud is organized (2D structure)
    pub fn is_organized(&self) -> bool {
        self.height() > 1
    }

    /// Add a point to the cloud
    pub fn push(&mut self, x: f32, y: f32, z: f32) -> PclResult<()> {
        let coords = [x, y, z];
        ffi::push_back_xyz(self.inner.pin_mut(), &coords);
        Ok(())
    }

    /// Get a reference to the underlying pcl-sys point cloud
    pub fn as_raw(&self) -> &ffi::PointCloud_PointXYZ {
        &self.inner
    }

    /// Create from a UniquePtr (internal use)
    pub(crate) fn from_unique_ptr(inner: UniquePtr<ffi::PointCloud_PointXYZ>) -> Self {
        Self { inner }
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
    pub(crate) inner: UniquePtr<ffi::PointCloud_PointXYZRGB>,
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

    /// Reserve capacity for at least n points
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        ffi::reserve_xyzrgb(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Resize the point cloud to contain n points
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        ffi::resize_xyzrgb(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Get the width of the point cloud (for organized clouds)
    pub fn width(&self) -> u32 {
        ffi::width_xyzrgb(&self.inner)
    }

    /// Get the height of the point cloud (for organized clouds)
    pub fn height(&self) -> u32 {
        ffi::height_xyzrgb(&self.inner)
    }

    /// Check if the point cloud is dense (no invalid points)
    pub fn is_dense(&self) -> bool {
        ffi::is_dense_xyzrgb(&self.inner)
    }

    /// Check if the point cloud is organized (2D structure)
    pub fn is_organized(&self) -> bool {
        self.height() > 1
    }

    /// Get a reference to the underlying pcl-sys point cloud
    pub fn as_raw(&self) -> &ffi::PointCloud_PointXYZRGB {
        &self.inner
    }

    /// Create from a UniquePtr (internal use)
    pub(crate) fn from_unique_ptr(inner: UniquePtr<ffi::PointCloud_PointXYZRGB>) -> Self {
        Self { inner }
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

/// A container for 3D points with XYZ coordinates and intensity
pub struct PointCloudXYZI {
    pub(crate) inner: UniquePtr<ffi::PointCloud_PointXYZI>,
}

impl PointCloudXYZI {
    /// Create a new empty point cloud
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_point_cloud_xyzi();
        Ok(Self { inner })
    }

    /// Get the number of points in the cloud
    pub fn size(&self) -> usize {
        ffi::size_xyzi(&self.inner)
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        ffi::empty_xyzi(&self.inner)
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        ffi::clear_xyzi(self.inner.pin_mut());
        Ok(())
    }

    /// Reserve capacity for at least n points
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        ffi::reserve_xyzi(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Resize the point cloud to contain n points
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        ffi::resize_xyzi(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Get the width of the point cloud (for organized clouds)
    pub fn width(&self) -> u32 {
        ffi::width_xyzi(&self.inner)
    }

    /// Get the height of the point cloud (for organized clouds)
    pub fn height(&self) -> u32 {
        ffi::height_xyzi(&self.inner)
    }

    /// Check if the point cloud is dense (no invalid points)
    pub fn is_dense(&self) -> bool {
        ffi::is_dense_xyzi(&self.inner)
    }

    /// Check if the point cloud is organized (2D structure)
    pub fn is_organized(&self) -> bool {
        self.height() > 1
    }

    /// Get a reference to the underlying pcl-sys point cloud
    pub fn as_raw(&self) -> &ffi::PointCloud_PointXYZI {
        &self.inner
    }

    /// Create from a UniquePtr (internal use)
    pub(crate) fn from_unique_ptr(inner: UniquePtr<ffi::PointCloud_PointXYZI>) -> Self {
        Self { inner }
    }
}

impl Default for PointCloudXYZI {
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloudXYZI")
    }
}

impl std::fmt::Debug for PointCloudXYZI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloudXYZI")
            .field("size", &self.size())
            .field("empty", &self.empty())
            .finish()
    }
}
