//! PointCloud container implementation
//!
//! This module provides FFI bindings for PCL's PointCloud template.

use crate::ffi;
use crate::{PointCloudXYZ, PointCloudXYZRGB};
use cxx::UniquePtr;
use std::pin::Pin;

impl PointCloudXYZ {
    pub fn new() -> UniquePtr<Self> {
        ffi::new_point_cloud_xyz()
    }

    pub fn size(&self) -> usize {
        ffi::size(self)
    }

    pub fn clear(self: Pin<&mut Self>) {
        ffi::clear(self)
    }

    pub fn empty(&self) -> bool {
        ffi::empty(self)
    }
}

impl PointCloudXYZRGB {
    pub fn new() -> UniquePtr<Self> {
        ffi::new_point_cloud_xyzrgb()
    }

    pub fn size(&self) -> usize {
        ffi::size_xyzrgb(self)
    }

    pub fn clear(self: Pin<&mut Self>) {
        ffi::clear_xyzrgb(self)
    }

    pub fn empty(&self) -> bool {
        ffi::empty_xyzrgb(self)
    }
}
