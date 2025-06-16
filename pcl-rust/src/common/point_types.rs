//! Safe wrappers for PCL point types
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's point types.

use crate::error::{PclError, PclResult};
use crate::traits::point_cloud::PointCloudImpl;
use crate::traits::{Intensity, Point, PointCloud, PointFfi, Rgb, Xyz};
use pcl_sys::ffi;
use std::ffi::c_void;
use std::fmt::Debug;

/// A 3D point with x, y, z coordinates
pub struct PointXYZ {
    pub(crate) inner: ffi::PointXYZ,
}

// Clone is implemented but panics because points cannot be created directly
impl Clone for PointXYZ {
    fn clone(&self) -> Self {
        panic!("Cannot clone PointXYZ - FFI type is opaque")
    }
}

impl Debug for PointXYZ {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZ")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .finish()
    }
}

// Note: Coordinate access methods are provided by the PointXyz trait implementation

// Implement Point trait for PointXYZ
impl Point for PointXYZ {
    fn type_name() -> &'static str {
        "PointXYZ"
    }

    fn default_point() -> Self {
        // This should not be called as points cannot be created directly
        panic!("Cannot create PointXYZ directly - use PointCloud methods instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        let inner = Box::new(PointCloudXYZImpl::new()?);
        Ok(PointCloud::from_impl(inner))
    }
}

// Implement internal FFI trait
impl PointFfi for PointXYZ {
    type FfiType = ffi::PointXYZ;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

// Implement PointXyz trait for PointXYZ
impl Xyz for PointXYZ {
    fn x(&self) -> f32 {
        ffi::get_x(&self.inner)
    }

    fn y(&self) -> f32 {
        ffi::get_y(&self.inner)
    }

    fn z(&self) -> f32 {
        ffi::get_z(&self.inner)
    }

    fn set_x(&mut self, _x: f32) {
        // Point modification not supported through FFI yet
        // Points are typically created and managed by PCL algorithms
    }

    fn set_y(&mut self, _y: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_z(&mut self, _z: f32) {
        // Point modification not supported through FFI yet
    }
}

/// A 3D point with x, y, z coordinates and intensity
pub struct PointXYZI {
    pub(crate) inner: ffi::PointXYZI,
}

// Clone is implemented but panics because points cannot be created directly
impl Clone for PointXYZI {
    fn clone(&self) -> Self {
        panic!("Cannot clone PointXYZI - FFI type is opaque")
    }
}

impl Debug for PointXYZI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZI")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("intensity", &self.intensity())
            .finish()
    }
}

// Note: Coordinate and intensity access methods are provided by the PointXyz and PointIntensity trait implementations

// Implement Point trait for PointXYZI
impl Point for PointXYZI {
    fn type_name() -> &'static str {
        "PointXYZI"
    }

    fn default_point() -> Self {
        // This should not be called as points cannot be created directly
        panic!("Cannot create PointXYZI directly - use PointCloud methods instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        let inner = Box::new(PointCloudXYZIImpl::new()?);
        Ok(PointCloud::from_impl(inner))
    }
}

// Implement internal FFI trait
impl PointFfi for PointXYZI {
    type FfiType = ffi::PointXYZI;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

// Implement PointXyz trait for PointXYZI
impl Xyz for PointXYZI {
    fn x(&self) -> f32 {
        ffi::get_x_xyzi(&self.inner)
    }

    fn y(&self) -> f32 {
        ffi::get_y_xyzi(&self.inner)
    }

    fn z(&self) -> f32 {
        ffi::get_z_xyzi(&self.inner)
    }

    fn set_x(&mut self, _x: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_y(&mut self, _y: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_z(&mut self, _z: f32) {
        // Point modification not supported through FFI yet
    }
}

// Implement PointIntensity trait for PointXYZI
impl Intensity for PointXYZI {
    fn intensity(&self) -> f32 {
        ffi::get_intensity(&self.inner)
    }

    fn set_intensity(&mut self, _intensity: f32) {
        // Point modification not supported through FFI yet
    }
}

// Note: Point creation is not currently supported due to cxx limitations
// Points must be created and managed by PCL C++ code

/// A 3D point with x, y, z coordinates and RGB color
pub struct PointXYZRGB {
    pub(crate) inner: ffi::PointXYZRGB,
}

// Clone is implemented but panics because points cannot be created directly
impl Clone for PointXYZRGB {
    fn clone(&self) -> Self {
        panic!("Cannot clone PointXYZRGB - FFI type is opaque")
    }
}

impl Debug for PointXYZRGB {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZRGB")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("r", &self.r())
            .field("g", &self.g())
            .field("b", &self.b())
            .finish()
    }
}

// Note: Coordinate and color access methods are provided by the PointXyz and PointRgb trait implementations

// Implement Point trait for PointXYZRGB
impl Point for PointXYZRGB {
    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

    fn default_point() -> Self {
        // This should not be called as points cannot be created directly
        panic!("Cannot create PointXYZRGB directly - use PointCloud methods instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        let inner = Box::new(PointCloudXYZRGBImpl::new()?);
        Ok(PointCloud::from_impl(inner))
    }
}

// Implement internal FFI trait
impl PointFfi for PointXYZRGB {
    type FfiType = ffi::PointXYZRGB;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

// Implement PointXyz trait for PointXYZRGB
impl Xyz for PointXYZRGB {
    fn x(&self) -> f32 {
        ffi::get_x_xyzrgb(&self.inner)
    }

    fn y(&self) -> f32 {
        ffi::get_y_xyzrgb(&self.inner)
    }

    fn z(&self) -> f32 {
        ffi::get_z_xyzrgb(&self.inner)
    }

    fn set_x(&mut self, _x: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_y(&mut self, _y: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_z(&mut self, _z: f32) {
        // Point modification not supported through FFI yet
    }
}

// Implement PointRgb trait for PointXYZRGB
impl Rgb for PointXYZRGB {
    fn r(&self) -> u8 {
        ffi::get_r(&self.inner)
    }

    fn g(&self) -> u8 {
        ffi::get_g(&self.inner)
    }

    fn b(&self) -> u8 {
        ffi::get_b(&self.inner)
    }

    fn set_r(&mut self, _r: u8) {
        // Point modification not supported through FFI yet
    }

    fn set_g(&mut self, _g: u8) {
        // Point modification not supported through FFI yet
    }

    fn set_b(&mut self, _b: u8) {
        // Point modification not supported through FFI yet
    }
}

// Concrete PointCloudImpl implementations

/// PointCloudImpl implementation for PointXYZ
struct PointCloudXYZImpl {
    cloud: crate::common::PointCloudXYZ,
}

impl PointCloudXYZImpl {
    fn new() -> PclResult<Self> {
        Ok(Self {
            cloud: crate::common::PointCloudXYZ::new()?,
        })
    }
}

impl PointCloudImpl<PointXYZ> for PointCloudXYZImpl {
    fn size(&self) -> usize {
        self.cloud.size()
    }

    fn empty(&self) -> bool {
        self.cloud.empty()
    }

    fn clear(&mut self) -> PclResult<()> {
        self.cloud.clear()
    }

    fn reserve(&mut self, n: usize) -> PclResult<()> {
        self.cloud.reserve(n)
    }

    fn resize(&mut self, n: usize) -> PclResult<()> {
        self.cloud.resize(n)
    }

    fn width(&self) -> u32 {
        self.cloud.width()
    }

    fn height(&self) -> u32 {
        self.cloud.height()
    }

    fn set_width(&mut self, _width: u32) {
        // Not implemented in current FFI
    }

    fn set_height(&mut self, _height: u32) {
        // Not implemented in current FFI
    }

    fn is_dense(&self) -> bool {
        self.cloud.is_dense()
    }

    fn set_is_dense(&mut self, _is_dense: bool) {
        // Not implemented in current FFI
    }

    fn at(&self, index: usize) -> PclResult<PointXYZ> {
        let coords = ffi::get_point_coords(self.cloud.as_raw(), index);
        if coords.len() >= 3 {
            // Points cannot be created directly, return error
            Err(PclError::NotImplemented {
                feature: "Direct point access".to_string(),
                workaround: Some("Use point cloud methods to access coordinates".to_string()),
            })
        } else {
            Err(PclError::InvalidState {
                message: "Failed to get point coordinates".to_string(),
                expected_state: "valid point data".to_string(),
                actual_state: "insufficient coordinates".to_string(),
            })
        }
    }

    fn set_at(&mut self, _index: usize, _point: PointXYZ) -> PclResult<()> {
        // Point modification not supported through FFI yet
        Ok(())
    }

    fn push(&mut self, _point: PointXYZ) -> PclResult<()> {
        // Point creation not supported through FFI yet
        // Use cloud.push(x, y, z) instead
        Err(PclError::NotImplemented {
            feature: "Point push".to_string(),
            workaround: Some("Use PointCloudXYZ::push(x, y, z) directly".to_string()),
        })
    }

    fn clone_impl(&self) -> Box<dyn PointCloudImpl<PointXYZ>> {
        Box::new(PointCloudXYZImpl {
            cloud: crate::common::PointCloudXYZ::new().unwrap(),
        })
    }

    fn as_ffi_ptr(&self) -> *const c_void {
        self.cloud.as_raw() as *const _ as *const c_void
    }

    fn as_ffi_ptr_mut(&mut self) -> *mut c_void {
        self.cloud.as_raw() as *const _ as *mut c_void
    }
}

/// PointCloudImpl implementation for PointXYZI
struct PointCloudXYZIImpl {
    cloud: crate::common::PointCloudXYZI,
}

impl PointCloudXYZIImpl {
    fn new() -> PclResult<Self> {
        Ok(Self {
            cloud: crate::common::PointCloudXYZI::new()?,
        })
    }
}

impl PointCloudImpl<PointXYZI> for PointCloudXYZIImpl {
    fn size(&self) -> usize {
        self.cloud.size()
    }

    fn empty(&self) -> bool {
        self.cloud.empty()
    }

    fn clear(&mut self) -> PclResult<()> {
        self.cloud.clear()
    }

    fn reserve(&mut self, n: usize) -> PclResult<()> {
        self.cloud.reserve(n)
    }

    fn resize(&mut self, n: usize) -> PclResult<()> {
        self.cloud.resize(n)
    }

    fn width(&self) -> u32 {
        self.cloud.width()
    }

    fn height(&self) -> u32 {
        self.cloud.height()
    }

    fn set_width(&mut self, _width: u32) {
        // Not implemented in current FFI
    }

    fn set_height(&mut self, _height: u32) {
        // Not implemented in current FFI
    }

    fn is_dense(&self) -> bool {
        self.cloud.is_dense()
    }

    fn set_is_dense(&mut self, _is_dense: bool) {
        // Not implemented in current FFI
    }

    fn at(&self, index: usize) -> PclResult<PointXYZI> {
        let coords = ffi::get_point_coords_xyzi(self.cloud.as_raw(), index);
        if coords.len() >= 4 {
            // Points cannot be created directly, return error
            Err(PclError::NotImplemented {
                feature: "Direct point access".to_string(),
                workaround: Some("Use point cloud methods to access coordinates".to_string()),
            })
        } else {
            Err(PclError::InvalidState {
                message: "Failed to get point coordinates".to_string(),
                expected_state: "valid point data".to_string(),
                actual_state: "insufficient coordinates".to_string(),
            })
        }
    }

    fn set_at(&mut self, _index: usize, _point: PointXYZI) -> PclResult<()> {
        Ok(())
    }

    fn push(&mut self, _point: PointXYZI) -> PclResult<()> {
        Err(PclError::NotImplemented {
            feature: "Point push".to_string(),
            workaround: Some("Use direct coordinate manipulation".to_string()),
        })
    }

    fn clone_impl(&self) -> Box<dyn PointCloudImpl<PointXYZI>> {
        Box::new(PointCloudXYZIImpl {
            cloud: crate::common::PointCloudXYZI::new().unwrap(),
        })
    }

    fn as_ffi_ptr(&self) -> *const c_void {
        self.cloud.as_raw() as *const _ as *const c_void
    }

    fn as_ffi_ptr_mut(&mut self) -> *mut c_void {
        self.cloud.as_raw() as *const _ as *mut c_void
    }
}

/// PointCloudImpl implementation for PointXYZRGB
struct PointCloudXYZRGBImpl {
    cloud: crate::common::PointCloudXYZRGB,
}

impl PointCloudXYZRGBImpl {
    fn new() -> PclResult<Self> {
        Ok(Self {
            cloud: crate::common::PointCloudXYZRGB::new()?,
        })
    }
}

impl PointCloudImpl<PointXYZRGB> for PointCloudXYZRGBImpl {
    fn size(&self) -> usize {
        self.cloud.size()
    }

    fn empty(&self) -> bool {
        self.cloud.empty()
    }

    fn clear(&mut self) -> PclResult<()> {
        self.cloud.clear()
    }

    fn reserve(&mut self, n: usize) -> PclResult<()> {
        self.cloud.reserve(n)
    }

    fn resize(&mut self, n: usize) -> PclResult<()> {
        self.cloud.resize(n)
    }

    fn width(&self) -> u32 {
        self.cloud.width()
    }

    fn height(&self) -> u32 {
        self.cloud.height()
    }

    fn set_width(&mut self, _width: u32) {
        // Not implemented in current FFI
    }

    fn set_height(&mut self, _height: u32) {
        // Not implemented in current FFI
    }

    fn is_dense(&self) -> bool {
        self.cloud.is_dense()
    }

    fn set_is_dense(&mut self, _is_dense: bool) {
        // Not implemented in current FFI
    }

    fn at(&self, index: usize) -> PclResult<PointXYZRGB> {
        let coords = ffi::get_point_coords_xyzrgb(self.cloud.as_raw(), index);
        if coords.len() >= 6 {
            // Points cannot be created directly, return error
            Err(PclError::NotImplemented {
                feature: "Direct point access".to_string(),
                workaround: Some("Use point cloud methods to access coordinates".to_string()),
            })
        } else {
            Err(PclError::InvalidState {
                message: "Failed to get point coordinates".to_string(),
                expected_state: "valid point data".to_string(),
                actual_state: "insufficient coordinates".to_string(),
            })
        }
    }

    fn set_at(&mut self, _index: usize, _point: PointXYZRGB) -> PclResult<()> {
        Ok(())
    }

    fn push(&mut self, _point: PointXYZRGB) -> PclResult<()> {
        Err(PclError::NotImplemented {
            feature: "Point push".to_string(),
            workaround: Some("Use direct coordinate manipulation".to_string()),
        })
    }

    fn clone_impl(&self) -> Box<dyn PointCloudImpl<PointXYZRGB>> {
        Box::new(PointCloudXYZRGBImpl {
            cloud: crate::common::PointCloudXYZRGB::new().unwrap(),
        })
    }

    fn as_ffi_ptr(&self) -> *const c_void {
        self.cloud.as_raw() as *const _ as *const c_void
    }

    fn as_ffi_ptr_mut(&mut self) -> *mut c_void {
        self.cloud.as_raw() as *const _ as *mut c_void
    }
}
