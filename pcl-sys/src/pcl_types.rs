//! PCL type implementations with proper memory management
//!
//! This module provides implementations of the PclDrop trait for various PCL types,
//! ensuring proper cleanup using PCL's aligned deallocation functions.

use crate::ptr::{PclDrop, PclPtr};
use crate::raw_ffi::{self, PointCloudNormal, PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB};

// Implement PclDrop for PointCloud types
impl PclDrop for PointCloudXYZ {
    unsafe fn pcl_drop(ptr: *mut Self) {
        unsafe {
            raw_ffi::pcl_pointcloud_xyz_delete(ptr);
        }
    }
}

impl PclDrop for PointCloudXYZI {
    unsafe fn pcl_drop(ptr: *mut Self) {
        unsafe {
            raw_ffi::pcl_pointcloud_xyzi_delete(ptr);
        }
    }
}

impl PclDrop for PointCloudXYZRGB {
    unsafe fn pcl_drop(ptr: *mut Self) {
        unsafe {
            raw_ffi::pcl_pointcloud_xyzrgb_delete(ptr);
        }
    }
}

impl PclDrop for PointCloudNormal {
    unsafe fn pcl_drop(ptr: *mut Self) {
        unsafe {
            raw_ffi::pcl_pointcloud_normal_delete(ptr);
        }
    }
}

// Type aliases for cleaner usage
pub type PointCloudXYZPtr = PclPtr<PointCloudXYZ>;
pub type PointCloudXYZIPtr = PclPtr<PointCloudXYZI>;
pub type PointCloudXYZRGBPtr = PclPtr<PointCloudXYZRGB>;
pub type PointCloudNormalPtr = PclPtr<PointCloudNormal>;

// Factory functions for creating PCL objects
impl PointCloudXYZPtr {
    /// Creates a new PointCloud<PointXYZ> using PCL's aligned allocation.
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = raw_ffi::pcl_pointcloud_xyz_new();
            PclPtr::from_raw(ptr)
        }
    }
}

impl PointCloudXYZIPtr {
    /// Creates a new PointCloud<PointXYZI> using PCL's aligned allocation.
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = raw_ffi::pcl_pointcloud_xyzi_new();
            PclPtr::from_raw(ptr)
        }
    }
}

impl PointCloudXYZRGBPtr {
    /// Creates a new PointCloud<PointXYZRGB> using PCL's aligned allocation.
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = raw_ffi::pcl_pointcloud_xyzrgb_new();
            PclPtr::from_raw(ptr)
        }
    }
}

impl PointCloudNormalPtr {
    /// Creates a new PointCloud<PointNormal> using PCL's aligned allocation.
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = raw_ffi::pcl_pointcloud_normal_new();
            PclPtr::from_raw(ptr)
        }
    }
}

// Safe wrapper functions for common PointCloud operations
impl PointCloudXYZPtr {
    /// Returns the number of points in the cloud.
    pub fn size(&self) -> usize {
        unsafe { raw_ffi::pcl_pointcloud_xyz_size(self.as_ptr()) }
    }

    /// Returns true if the cloud is empty.
    pub fn empty(&self) -> bool {
        unsafe { raw_ffi::pcl_pointcloud_xyz_empty(self.as_ptr()) }
    }

    /// Clears all points from the cloud.
    pub fn clear(&mut self) {
        unsafe { raw_ffi::pcl_pointcloud_xyz_clear(self.as_mut_ptr()) }
    }

    /// Reserves space for at least `n` points.
    pub fn reserve(&mut self, n: usize) {
        unsafe { raw_ffi::pcl_pointcloud_xyz_reserve(self.as_mut_ptr(), n) }
    }

    /// Resizes the cloud to contain `n` points.
    pub fn resize(&mut self, n: usize) {
        unsafe { raw_ffi::pcl_pointcloud_xyz_resize(self.as_mut_ptr(), n) }
    }

    /// Adds a new point to the cloud.
    pub fn push_back(&mut self, x: f32, y: f32, z: f32) {
        unsafe { raw_ffi::pcl_pointcloud_xyz_push_back(self.as_mut_ptr(), x, y, z) }
    }

    /// Gets the coordinates of a point at the given index.
    /// Returns None if the index is out of bounds.
    pub fn get_point(&self, index: usize) -> Option<(f32, f32, f32)> {
        if index >= self.size() {
            return None;
        }

        let mut x = 0.0f32;
        let mut y = 0.0f32;
        let mut z = 0.0f32;

        unsafe {
            raw_ffi::pcl_pointcloud_xyz_get_point(self.as_ptr(), index, &mut x, &mut y, &mut z);
        }

        Some((x, y, z))
    }

    /// Sets the coordinates of a point at the given index.
    /// Does nothing if the index is out of bounds.
    pub fn set_point(&mut self, index: usize, x: f32, y: f32, z: f32) {
        if index < self.size() {
            unsafe {
                raw_ffi::pcl_pointcloud_xyz_set_point(self.as_mut_ptr(), index, x, y, z);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pointcloud_xyz_creation() {
        let cloud = PointCloudXYZPtr::new();
        assert!(cloud.is_some(), "Should be able to create PointCloud");

        let cloud = cloud.unwrap();
        assert_eq!(cloud.size(), 0);
        assert!(cloud.empty());
    }

    #[test]
    fn test_pointcloud_xyz_operations() {
        let mut cloud = PointCloudXYZPtr::new().unwrap();

        // Test basic operations
        cloud.push_back(1.0, 2.0, 3.0);
        assert_eq!(cloud.size(), 1);
        assert!(!cloud.empty());

        // Test point access
        let point = cloud.get_point(0);
        assert!(point.is_some());
        let (x, y, z) = point.unwrap();
        assert_eq!(x, 1.0);
        assert_eq!(y, 2.0);
        assert_eq!(z, 3.0);

        // Test point modification
        cloud.set_point(0, 4.0, 5.0, 6.0);
        let point = cloud.get_point(0).unwrap();
        assert_eq!(point, (4.0, 5.0, 6.0));

        // Test bounds checking
        assert!(cloud.get_point(10).is_none());

        // Test clear
        cloud.clear();
        assert_eq!(cloud.size(), 0);
        assert!(cloud.empty());
    }

    #[test]
    fn test_pointcloud_xyz_resize_reserve() {
        let mut cloud = PointCloudXYZPtr::new().unwrap();

        // Test reserve
        cloud.reserve(100);
        assert_eq!(cloud.size(), 0); // Reserve doesn't change size

        // Test resize
        cloud.resize(5);
        assert_eq!(cloud.size(), 5);

        // Test that we can access all resized points
        for i in 0..5 {
            cloud.set_point(i, i as f32, (i + 1) as f32, (i + 2) as f32);
        }

        for i in 0..5 {
            let point = cloud.get_point(i).unwrap();
            assert_eq!(point, (i as f32, (i + 1) as f32, (i + 2) as f32));
        }
    }
}
