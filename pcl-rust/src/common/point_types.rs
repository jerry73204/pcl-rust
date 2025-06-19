//! Safe wrappers for PCL point types
//!
//! This module provides safe, idiomatic Rust interfaces for PCL's point types.

use crate::common::point_cloud_generic::PointCloud;
use crate::error::PclResult;
use crate::traits::{Intensity, Point, PointFfi, Rgb, Xyz};
use pcl_sys::ffi;
use std::fmt::Debug;
use std::pin::Pin;

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
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }

    fn default_point() -> Self {
        // FFI types cannot be created directly in safe Rust
        // This is a placeholder that should not be called
        panic!("PointXYZ cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        PointCloud::new()
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyz()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::empty(cloud)
    }

    fn cloud_clear(cloud: Pin<&mut Self::CloudType>) {
        ffi::clear(cloud);
    }

    fn cloud_reserve(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::reserve_xyz(cloud, n);
    }

    fn cloud_resize(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::resize_xyz(cloud, n);
    }

    fn cloud_width(cloud: &Self::CloudType) -> u32 {
        ffi::width(cloud)
    }

    fn cloud_height(cloud: &Self::CloudType) -> u32 {
        ffi::height(cloud)
    }

    fn cloud_is_dense(cloud: &Self::CloudType) -> bool {
        ffi::is_dense(cloud)
    }

    fn as_raw_cloud(cloud: &Self::CloudType) -> *const Self::CloudType {
        cloud as *const _
    }

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType> {
        ffi::get_point_at_xyz(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyz(cloud, index, &point.inner);
    }

    fn from_unique_ptr(_ptr: cxx::UniquePtr<Self::FfiPointType>) -> PclResult<Self> {
        // FFI points cannot be moved out of UniquePtr in safe Rust
        // This is a limitation of the cxx bridge
        Err(crate::error::PclError::NotImplemented {
            feature: "Direct point access".to_string(),
            workaround: Some("Use point cloud methods to access point data".to_string()),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height(cloud, height);
    }

    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType> {
        ffi::clone_point_cloud_xyz(cloud)
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

    fn set_x(&mut self, x: f32) {
        unsafe {
            ffi::set_x(Pin::new_unchecked(&mut self.inner), x);
        }
    }

    fn set_y(&mut self, y: f32) {
        unsafe {
            ffi::set_y(Pin::new_unchecked(&mut self.inner), y);
        }
    }

    fn set_z(&mut self, z: f32) {
        unsafe {
            ffi::set_z(Pin::new_unchecked(&mut self.inner), z);
        }
    }
}

// Implement PointXyzOps trait for PointXYZ
impl crate::traits::PointXyzOps for PointXYZ {
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32) {
        let coords = [x, y, z];
        ffi::push_back_xyz(cloud, &coords);
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
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }

    fn default_point() -> Self {
        panic!("PointXYZI cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        PointCloud::new()
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyzi()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_xyzi(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::empty_xyzi(cloud)
    }

    fn cloud_clear(cloud: Pin<&mut Self::CloudType>) {
        ffi::clear_xyzi(cloud);
    }

    fn cloud_reserve(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::reserve_xyzi(cloud, n);
    }

    fn cloud_resize(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::resize_xyzi(cloud, n);
    }

    fn cloud_width(cloud: &Self::CloudType) -> u32 {
        ffi::width_xyzi(cloud)
    }

    fn cloud_height(cloud: &Self::CloudType) -> u32 {
        ffi::height_xyzi(cloud)
    }

    fn cloud_is_dense(cloud: &Self::CloudType) -> bool {
        ffi::is_dense_xyzi(cloud)
    }

    fn as_raw_cloud(cloud: &Self::CloudType) -> *const Self::CloudType {
        cloud as *const _
    }

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType> {
        ffi::get_point_at_xyzi(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyzi(cloud, index, &point.inner);
    }

    fn from_unique_ptr(_ptr: cxx::UniquePtr<Self::FfiPointType>) -> PclResult<Self> {
        // FFI points cannot be moved out of UniquePtr in safe Rust
        // This is a limitation of the cxx bridge
        Err(crate::error::PclError::NotImplemented {
            feature: "Direct point access".to_string(),
            workaround: Some("Use point cloud methods to access point data".to_string()),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzi(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzi(cloud, height);
    }

    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType> {
        ffi::clone_point_cloud_xyzi(cloud)
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

    fn set_x(&mut self, x: f32) {
        unsafe {
            ffi::set_x_xyzi(Pin::new_unchecked(&mut self.inner), x);
        }
    }

    fn set_y(&mut self, y: f32) {
        unsafe {
            ffi::set_y_xyzi(Pin::new_unchecked(&mut self.inner), y);
        }
    }

    fn set_z(&mut self, z: f32) {
        unsafe {
            ffi::set_z_xyzi(Pin::new_unchecked(&mut self.inner), z);
        }
    }
}

// Implement PointIntensity trait for PointXYZI
impl Intensity for PointXYZI {
    fn intensity(&self) -> f32 {
        ffi::get_intensity(&self.inner)
    }

    fn set_intensity(&mut self, intensity: f32) {
        unsafe {
            ffi::set_intensity(Pin::new_unchecked(&mut self.inner), intensity);
        }
    }
}

// Implement PointIntensityOps trait for PointXYZI
impl crate::traits::PointIntensityOps for PointXYZI {
    fn push_xyzi(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, intensity: f32) {
        let coords = [x, y, z, intensity];
        ffi::push_back_xyzi(cloud, &coords);
    }
}

// Note: Point creation is not currently supported due to cxx limitations
// Points must be created and managed by PCL C++ code

/// A 3D point with surface normal information
pub struct PointNormal {
    pub(crate) inner: ffi::PointNormal,
}

// Clone is implemented but panics because points cannot be created directly
impl Clone for PointNormal {
    fn clone(&self) -> Self {
        panic!("Cannot clone PointNormal - FFI type is opaque")
    }
}

impl Debug for PointNormal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointNormal").finish()
    }
}

// Implement Point trait for PointNormal
impl Point for PointNormal {
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }

    fn default_point() -> Self {
        panic!("PointNormal cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        PointCloud::new()
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_point_normal()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_point_normal(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::empty_point_normal(cloud)
    }

    fn cloud_clear(cloud: Pin<&mut Self::CloudType>) {
        ffi::clear_point_normal(cloud);
    }

    fn cloud_reserve(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::reserve_point_normal(cloud, n);
    }

    fn cloud_resize(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::resize_point_normal(cloud, n);
    }

    fn cloud_width(cloud: &Self::CloudType) -> u32 {
        ffi::width_point_normal(cloud)
    }

    fn cloud_height(cloud: &Self::CloudType) -> u32 {
        ffi::height_point_normal(cloud)
    }

    fn cloud_is_dense(cloud: &Self::CloudType) -> bool {
        ffi::is_dense_point_normal(cloud)
    }

    fn as_raw_cloud(cloud: &Self::CloudType) -> *const Self::CloudType {
        cloud as *const _
    }

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType> {
        ffi::get_point_at_point_normal(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_point_normal(cloud, index, &point.inner);
    }

    fn from_unique_ptr(_ptr: cxx::UniquePtr<Self::FfiPointType>) -> PclResult<Self> {
        // FFI points cannot be moved out of UniquePtr in safe Rust
        // This is a limitation of the cxx bridge
        Err(crate::error::PclError::NotImplemented {
            feature: "Direct point access".to_string(),
            workaround: Some("Use point cloud methods to access point data".to_string()),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_point_normal(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_point_normal(cloud, height);
    }

    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType> {
        ffi::clone_point_cloud_point_normal(cloud)
    }
}

// Implement internal FFI trait
impl PointFfi for PointNormal {
    type FfiType = pcl_sys::ffi::PointNormal;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

// Implement Xyz trait for PointNormal
impl Xyz for PointNormal {
    fn x(&self) -> f32 {
        ffi::get_x_point_normal(&self.inner)
    }

    fn y(&self) -> f32 {
        ffi::get_y_point_normal(&self.inner)
    }

    fn z(&self) -> f32 {
        ffi::get_z_point_normal(&self.inner)
    }

    fn set_x(&mut self, x: f32) {
        unsafe {
            ffi::set_x_point_normal(Pin::new_unchecked(&mut self.inner), x);
        }
    }

    fn set_y(&mut self, y: f32) {
        unsafe {
            ffi::set_y_point_normal(Pin::new_unchecked(&mut self.inner), y);
        }
    }

    fn set_z(&mut self, z: f32) {
        unsafe {
            ffi::set_z_point_normal(Pin::new_unchecked(&mut self.inner), z);
        }
    }
}

// Implement NormalXyz trait for PointNormal
impl crate::traits::NormalXyz for PointNormal {
    fn normal_x(&self) -> f32 {
        ffi::get_normal_x_point_normal(&self.inner)
    }

    fn normal_y(&self) -> f32 {
        ffi::get_normal_y_point_normal(&self.inner)
    }

    fn normal_z(&self) -> f32 {
        ffi::get_normal_z_point_normal(&self.inner)
    }

    fn set_normal_x(&mut self, nx: f32) {
        unsafe {
            ffi::set_normal_x_point_normal(Pin::new_unchecked(&mut self.inner), nx);
        }
    }

    fn set_normal_y(&mut self, ny: f32) {
        unsafe {
            ffi::set_normal_y_point_normal(Pin::new_unchecked(&mut self.inner), ny);
        }
    }

    fn set_normal_z(&mut self, nz: f32) {
        unsafe {
            ffi::set_normal_z_point_normal(Pin::new_unchecked(&mut self.inner), nz);
        }
    }
}

// Implement PointNormalOps trait for push operations
impl PointNormalOps for PointNormal {
    fn push_point_normal(
        cloud: Pin<&mut Self::CloudType>,
        x: f32,
        y: f32,
        z: f32,
        nx: f32,
        ny: f32,
        nz: f32,
    ) {
        let coords = [x, y, z, nx, ny, nz];
        ffi::push_back_point_normal(cloud, &coords);
    }
}

/// Trait for point types that support normal push operations
pub trait PointNormalOps: Point {
    /// Push a point with x, y, z coordinates and normal to the cloud
    fn push_point_normal(
        cloud: Pin<&mut Self::CloudType>,
        x: f32,
        y: f32,
        z: f32,
        nx: f32,
        ny: f32,
        nz: f32,
    );
}

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
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

    fn default_point() -> Self {
        panic!("PointXYZRGB cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> PclResult<PointCloud<Self>>
    where
        Self: Sized,
    {
        PointCloud::new()
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyzrgb()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_xyzrgb(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::empty_xyzrgb(cloud)
    }

    fn cloud_clear(cloud: Pin<&mut Self::CloudType>) {
        ffi::clear_xyzrgb(cloud);
    }

    fn cloud_reserve(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::reserve_xyzrgb(cloud, n);
    }

    fn cloud_resize(cloud: Pin<&mut Self::CloudType>, n: usize) {
        ffi::resize_xyzrgb(cloud, n);
    }

    fn cloud_width(cloud: &Self::CloudType) -> u32 {
        ffi::width_xyzrgb(cloud)
    }

    fn cloud_height(cloud: &Self::CloudType) -> u32 {
        ffi::height_xyzrgb(cloud)
    }

    fn cloud_is_dense(cloud: &Self::CloudType) -> bool {
        ffi::is_dense_xyzrgb(cloud)
    }

    fn as_raw_cloud(cloud: &Self::CloudType) -> *const Self::CloudType {
        cloud as *const _
    }

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType> {
        ffi::get_point_at_xyzrgb(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyzrgb(cloud, index, &point.inner);
    }

    fn from_unique_ptr(_ptr: cxx::UniquePtr<Self::FfiPointType>) -> PclResult<Self> {
        // FFI points cannot be moved out of UniquePtr in safe Rust
        // This is a limitation of the cxx bridge
        Err(crate::error::PclError::NotImplemented {
            feature: "Direct point access".to_string(),
            workaround: Some("Use point cloud methods to access point data".to_string()),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzrgb(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzrgb(cloud, height);
    }

    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType> {
        ffi::clone_point_cloud_xyzrgb(cloud)
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

    fn set_x(&mut self, x: f32) {
        unsafe {
            ffi::set_x_xyzrgb(Pin::new_unchecked(&mut self.inner), x);
        }
    }

    fn set_y(&mut self, y: f32) {
        unsafe {
            ffi::set_y_xyzrgb(Pin::new_unchecked(&mut self.inner), y);
        }
    }

    fn set_z(&mut self, z: f32) {
        unsafe {
            ffi::set_z_xyzrgb(Pin::new_unchecked(&mut self.inner), z);
        }
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

    fn set_r(&mut self, r: u8) {
        unsafe {
            ffi::set_r(Pin::new_unchecked(&mut self.inner), r);
        }
    }

    fn set_g(&mut self, g: u8) {
        unsafe {
            ffi::set_g(Pin::new_unchecked(&mut self.inner), g);
        }
    }

    fn set_b(&mut self, b: u8) {
        unsafe {
            ffi::set_b(Pin::new_unchecked(&mut self.inner), b);
        }
    }
}

// Implement PointRgbOps trait for PointXYZRGB
impl crate::traits::PointRgbOps for PointXYZRGB {
    fn push_xyzrgb(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) {
        let coords = [x, y, z, r as f32, g as f32, b as f32];
        ffi::push_back_xyzrgb(cloud, &coords);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::PointCloudNormalBuilder;

    #[test]
    fn test_point_normal_cloud_creation() {
        let cloud = PointCloud::<PointNormal>::new();
        assert!(cloud.is_ok());
        let cloud = cloud.unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
    }

    #[test]
    fn test_point_normal_cloud_builder() {
        let cloud = PointCloudNormalBuilder::new()
            .add_point_normal(1.0, 2.0, 3.0, 0.0, 0.0, 1.0)
            .add_point_normal(4.0, 5.0, 6.0, 0.0, 1.0, 0.0)
            .build();

        assert!(cloud.is_ok());
        let cloud = cloud.unwrap();
        assert_eq!(cloud.size(), 2);
        assert!(!cloud.empty());
    }

    #[test]
    fn test_point_normal_type_name() {
        assert_eq!(PointNormal::type_name(), "PointNormal");
    }
}
