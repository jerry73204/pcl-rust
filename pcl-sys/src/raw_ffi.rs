//! Raw FFI bindings for PCL objects using explicit memory management
//!
//! This module provides FFI bindings that work with raw pointers instead of
//! cxx::UniquePtr, allowing proper handling of PCL's aligned memory requirements.

use std::os::raw::c_char;

// Re-export the main ffi module types we need
pub use crate::ffi::{
    PointCloud_PointNormal, PointCloud_PointXYZ, PointCloud_PointXYZI, PointCloud_PointXYZRGB,
    PointNormal, PointXYZ, PointXYZI, PointXYZRGB,
};

// Raw pointer type aliases for cleaner FFI declarations
pub type PointCloudXYZ = PointCloud_PointXYZ;
pub type PointCloudXYZI = PointCloud_PointXYZI;
pub type PointCloudXYZRGB = PointCloud_PointXYZRGB;
pub type PointCloudNormal = PointCloud_PointNormal;

unsafe extern "C" {
    // ============================================================================
    // PointCloud creation and destruction functions
    // ============================================================================

    pub fn pcl_pointcloud_xyz_new() -> *mut PointCloudXYZ;
    pub fn pcl_pointcloud_xyz_delete(cloud: *mut PointCloudXYZ);

    pub fn pcl_pointcloud_xyzi_new() -> *mut PointCloudXYZI;
    pub fn pcl_pointcloud_xyzi_delete(cloud: *mut PointCloudXYZI);

    pub fn pcl_pointcloud_xyzrgb_new() -> *mut PointCloudXYZRGB;
    pub fn pcl_pointcloud_xyzrgb_delete(cloud: *mut PointCloudXYZRGB);

    pub fn pcl_pointcloud_normal_new() -> *mut PointCloudNormal;
    pub fn pcl_pointcloud_normal_delete(cloud: *mut PointCloudNormal);

    // ============================================================================
    // PointCloud basic operations
    // ============================================================================

    pub fn pcl_pointcloud_xyz_size(cloud: *const PointCloudXYZ) -> usize;
    pub fn pcl_pointcloud_xyz_empty(cloud: *const PointCloudXYZ) -> bool;
    pub fn pcl_pointcloud_xyz_clear(cloud: *mut PointCloudXYZ);
    pub fn pcl_pointcloud_xyz_reserve(cloud: *mut PointCloudXYZ, n: usize);
    pub fn pcl_pointcloud_xyz_resize(cloud: *mut PointCloudXYZ, n: usize);

    pub fn pcl_pointcloud_xyz_push_back(cloud: *mut PointCloudXYZ, x: f32, y: f32, z: f32);
    pub fn pcl_pointcloud_xyz_get_point(
        cloud: *const PointCloudXYZ,
        index: usize,
        x: *mut f32,
        y: *mut f32,
        z: *mut f32,
    );
    pub fn pcl_pointcloud_xyz_set_point(
        cloud: *mut PointCloudXYZ,
        index: usize,
        x: f32,
        y: f32,
        z: f32,
    );

    // ============================================================================
    // Filter creation and destruction functions
    // ============================================================================

    // These will be defined when we create the filter types
    // For now, we'll use opaque types since filters don't have direct cxx bindings

    // PassThrough filters
    pub fn pcl_passthrough_xyz_new() -> *mut std::ffi::c_void;
    pub fn pcl_passthrough_xyz_delete(filter: *mut std::ffi::c_void);

    pub fn pcl_passthrough_xyzrgb_new() -> *mut std::ffi::c_void;
    pub fn pcl_passthrough_xyzrgb_delete(filter: *mut std::ffi::c_void);

    // VoxelGrid filters
    pub fn pcl_voxelgrid_xyz_new() -> *mut std::ffi::c_void;
    pub fn pcl_voxelgrid_xyz_delete(filter: *mut std::ffi::c_void);

    pub fn pcl_voxelgrid_xyzrgb_new() -> *mut std::ffi::c_void;
    pub fn pcl_voxelgrid_xyzrgb_delete(filter: *mut std::ffi::c_void);

    // StatisticalOutlierRemoval filters
    pub fn pcl_statistical_xyz_new() -> *mut std::ffi::c_void;
    pub fn pcl_statistical_xyz_delete(filter: *mut std::ffi::c_void);

    pub fn pcl_statistical_xyzrgb_new() -> *mut std::ffi::c_void;
    pub fn pcl_statistical_xyzrgb_delete(filter: *mut std::ffi::c_void);

    // RadiusOutlierRemoval filters
    pub fn pcl_radius_xyz_new() -> *mut std::ffi::c_void;
    pub fn pcl_radius_xyz_delete(filter: *mut std::ffi::c_void);

    pub fn pcl_radius_xyzrgb_new() -> *mut std::ffi::c_void;
    pub fn pcl_radius_xyzrgb_delete(filter: *mut std::ffi::c_void);

    // ============================================================================
    // Filter operation functions
    // ============================================================================

    // PassThrough filter operations
    pub fn pcl_passthrough_xyz_set_input_cloud(
        filter: *mut std::ffi::c_void,
        cloud: *const PointCloudXYZ,
    );

    pub fn pcl_passthrough_xyz_set_filter_field_name(
        filter: *mut std::ffi::c_void,
        field_name: *const c_char,
    );

    pub fn pcl_passthrough_xyz_set_filter_limits(filter: *mut std::ffi::c_void, min: f32, max: f32);

    pub fn pcl_passthrough_xyz_set_negative(filter: *mut std::ffi::c_void, negative: bool);

    pub fn pcl_passthrough_xyz_filter(filter: *mut std::ffi::c_void) -> *mut PointCloudXYZ;

    // VoxelGrid filter operations
    pub fn pcl_voxelgrid_xyz_set_input_cloud(
        filter: *mut std::ffi::c_void,
        cloud: *const PointCloudXYZ,
    );

    pub fn pcl_voxelgrid_xyz_set_leaf_size(
        filter: *mut std::ffi::c_void,
        lx: f32,
        ly: f32,
        lz: f32,
    );

    pub fn pcl_voxelgrid_xyz_filter(filter: *mut std::ffi::c_void) -> *mut PointCloudXYZ;

    // Statistical filter operations
    pub fn pcl_statistical_xyz_set_input_cloud(
        filter: *mut std::ffi::c_void,
        cloud: *const PointCloudXYZ,
    );

    pub fn pcl_statistical_xyz_set_mean_k(filter: *mut std::ffi::c_void, mean_k: i32);

    pub fn pcl_statistical_xyz_set_stddev_mul_thresh(
        filter: *mut std::ffi::c_void,
        stddev_mult: f64,
    );

    pub fn pcl_statistical_xyz_filter(filter: *mut std::ffi::c_void) -> *mut PointCloudXYZ;

    // Radius filter operations
    pub fn pcl_radius_xyz_set_input_cloud(
        filter: *mut std::ffi::c_void,
        cloud: *const PointCloudXYZ,
    );

    pub fn pcl_radius_xyz_set_radius_search(filter: *mut std::ffi::c_void, radius: f64);

    pub fn pcl_radius_xyz_set_min_neighbors_in_radius(
        filter: *mut std::ffi::c_void,
        min_neighbors: i32,
    );

    pub fn pcl_radius_xyz_filter(filter: *mut std::ffi::c_void) -> *mut PointCloudXYZ;
}

// Type-safe wrappers for filter types
pub struct PassThroughXYZ(*mut std::ffi::c_void);
pub struct VoxelGridXYZ(*mut std::ffi::c_void);
pub struct StatisticalOutlierRemovalXYZ(*mut std::ffi::c_void);
pub struct RadiusOutlierRemovalXYZ(*mut std::ffi::c_void);

impl PassThroughXYZ {
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = pcl_passthrough_xyz_new();
            if ptr.is_null() { None } else { Some(Self(ptr)) }
        }
    }

    pub fn as_ptr(&self) -> *mut std::ffi::c_void {
        self.0
    }
}

impl Drop for PassThroughXYZ {
    fn drop(&mut self) {
        unsafe {
            pcl_passthrough_xyz_delete(self.0);
        }
    }
}

impl VoxelGridXYZ {
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = pcl_voxelgrid_xyz_new();
            if ptr.is_null() { None } else { Some(Self(ptr)) }
        }
    }

    pub fn as_ptr(&self) -> *mut std::ffi::c_void {
        self.0
    }
}

impl Drop for VoxelGridXYZ {
    fn drop(&mut self) {
        unsafe {
            pcl_voxelgrid_xyz_delete(self.0);
        }
    }
}

impl StatisticalOutlierRemovalXYZ {
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = pcl_statistical_xyz_new();
            if ptr.is_null() { None } else { Some(Self(ptr)) }
        }
    }

    pub fn as_ptr(&self) -> *mut std::ffi::c_void {
        self.0
    }
}

impl Drop for StatisticalOutlierRemovalXYZ {
    fn drop(&mut self) {
        unsafe {
            pcl_statistical_xyz_delete(self.0);
        }
    }
}

impl RadiusOutlierRemovalXYZ {
    pub fn new() -> Option<Self> {
        unsafe {
            let ptr = pcl_radius_xyz_new();
            if ptr.is_null() { None } else { Some(Self(ptr)) }
        }
    }

    pub fn as_ptr(&self) -> *mut std::ffi::c_void {
        self.0
    }
}

impl Drop for RadiusOutlierRemovalXYZ {
    fn drop(&mut self) {
        unsafe {
            pcl_radius_xyz_delete(self.0);
        }
    }
}
