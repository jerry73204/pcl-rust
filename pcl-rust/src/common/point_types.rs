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

    fn set_normal_x(&mut self, _nx: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_normal_y(&mut self, _ny: f32) {
        // Point modification not supported through FFI yet
    }

    fn set_normal_z(&mut self, _nz: f32) {
        // Point modification not supported through FFI yet
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
