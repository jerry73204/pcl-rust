//! New point type design with owned/reference split
//!
//! This module implements the new design where:
//! - Owned types (PointXYZ, etc.) are simple Rust structs
//! - Reference types (PointXYZRef, etc.) wrap FFI types
//! - Marker types (XYZ, etc.) connect them via associated types

use cxx::memory::UniquePtrTarget;
use pcl_sys::ffi;
use std::fmt::Debug;
use std::pin::Pin;

// ============================================================================
// Marker Types
// ============================================================================

/// Marker type for XYZ point types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZ;

/// Marker type for XYZI point types  
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZI;

/// Marker type for XYZRGB point types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XYZRGB;

/// Marker type for PointNormal types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Normal;

// ============================================================================
// Core Trait
// ============================================================================

/// Trait for converting reference types to owned types
pub trait ToPointOwned {
    /// The owned type
    type Owned;

    /// Convert to owned type
    fn to_owned(&self) -> Self::Owned;
}

/// Trait that connects marker types to their owned and reference types
pub trait PointType: Debug + Clone + Copy + 'static {
    /// The owned point type (e.g., PointXYZ)
    type Owned: Clone + Copy + Debug + PartialEq + 'static;

    /// The reference point type (e.g., PointXYZRef)
    type Ref: ?Sized + ToPointOwned<Owned = Self::Owned>;

    /// The FFI cloud type
    type CloudType: UniquePtrTarget;

    /// The FFI point type
    type FfiPointType: UniquePtrTarget;

    /// Get the type name for debugging
    fn type_name() -> &'static str;
}

// ============================================================================
// Owned Point Types
// ============================================================================

/// A 3D point with x, y, z coordinates (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl PointXYZ {
    /// Create a new point
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Calculate squared distance to another point
    pub fn distance_squared_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance to another point
    pub fn distance_to(&self, other: &Self) -> f32 {
        self.distance_squared_to(other).sqrt()
    }
}

/// A 3D point with x, y, z coordinates and intensity (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl PointXYZI {
    /// Create a new point with intensity
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }
}

/// A 3D point with x, y, z coordinates and RGB color (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    _padding: u8, // To match PCL's 32-bit alignment
}

impl PointXYZRGB {
    /// Create a new colored point
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self {
            x,
            y,
            z,
            r,
            g,
            b,
            _padding: 0,
        }
    }
}

/// A 3D point with normal information (owned)
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[repr(C)]
pub struct PointNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
    pub curvature: f32,
}

impl PointNormal {
    /// Create a new point with normal
    pub fn new(x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        Self {
            x,
            y,
            z,
            normal_x: nx,
            normal_y: ny,
            normal_z: nz,
            curvature: 0.0,
        }
    }
}

// ============================================================================
// Reference Types (wrapping FFI types)
// ============================================================================

/// Reference to a PCL PointXYZ
#[repr(transparent)]
pub struct PointXYZRef {
    pub(crate) inner: ffi::PointXYZ,
}

impl PointXYZRef {
    /// Get x coordinate
    pub fn x(&self) -> f32 {
        ffi::get_x(&self.inner)
    }

    /// Get y coordinate
    pub fn y(&self) -> f32 {
        ffi::get_y(&self.inner)
    }

    /// Get z coordinate
    pub fn z(&self) -> f32 {
        ffi::get_z(&self.inner)
    }
}

impl ToPointOwned for PointXYZRef {
    type Owned = PointXYZ;

    fn to_owned(&self) -> Self::Owned {
        PointXYZ {
            x: self.x(),
            y: self.y(),
            z: self.z(),
        }
    }
}

impl Debug for PointXYZRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZRef")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .finish()
    }
}

/// Reference to a PCL PointXYZI
#[repr(transparent)]
pub struct PointXYZIRef {
    pub(crate) inner: ffi::PointXYZI,
}

impl PointXYZIRef {
    pub fn x(&self) -> f32 {
        ffi::get_x_xyzi(&self.inner)
    }

    pub fn y(&self) -> f32 {
        ffi::get_y_xyzi(&self.inner)
    }

    pub fn z(&self) -> f32 {
        ffi::get_z_xyzi(&self.inner)
    }

    pub fn intensity(&self) -> f32 {
        ffi::get_intensity(&self.inner)
    }
}

impl ToPointOwned for PointXYZIRef {
    type Owned = PointXYZI;

    fn to_owned(&self) -> Self::Owned {
        PointXYZI {
            x: self.x(),
            y: self.y(),
            z: self.z(),
            intensity: self.intensity(),
        }
    }
}

impl Debug for PointXYZIRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZIRef")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("intensity", &self.intensity())
            .finish()
    }
}

/// Reference to a PCL PointXYZRGB
#[repr(transparent)]
pub struct PointXYZRGBRef {
    pub(crate) inner: ffi::PointXYZRGB,
}

impl PointXYZRGBRef {
    pub fn x(&self) -> f32 {
        ffi::get_x_xyzrgb(&self.inner)
    }

    pub fn y(&self) -> f32 {
        ffi::get_y_xyzrgb(&self.inner)
    }

    pub fn z(&self) -> f32 {
        ffi::get_z_xyzrgb(&self.inner)
    }

    pub fn r(&self) -> u8 {
        ffi::get_r(&self.inner)
    }

    pub fn g(&self) -> u8 {
        ffi::get_g(&self.inner)
    }

    pub fn b(&self) -> u8 {
        ffi::get_b(&self.inner)
    }
}

impl ToPointOwned for PointXYZRGBRef {
    type Owned = PointXYZRGB;

    fn to_owned(&self) -> Self::Owned {
        PointXYZRGB::new(self.x(), self.y(), self.z(), self.r(), self.g(), self.b())
    }
}

impl Debug for PointXYZRGBRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointXYZRGBRef")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("r", &self.r())
            .field("g", &self.g())
            .field("b", &self.b())
            .finish()
    }
}

/// Reference to a PCL PointNormal
#[repr(transparent)]
pub struct PointNormalRef {
    pub(crate) inner: ffi::PointNormal,
}

impl PointNormalRef {
    pub fn x(&self) -> f32 {
        ffi::get_x_point_normal(&self.inner)
    }

    pub fn y(&self) -> f32 {
        ffi::get_y_point_normal(&self.inner)
    }

    pub fn z(&self) -> f32 {
        ffi::get_z_point_normal(&self.inner)
    }

    pub fn normal_x(&self) -> f32 {
        ffi::get_normal_x_point_normal(&self.inner)
    }

    pub fn normal_y(&self) -> f32 {
        ffi::get_normal_y_point_normal(&self.inner)
    }

    pub fn normal_z(&self) -> f32 {
        ffi::get_normal_z_point_normal(&self.inner)
    }
}

impl ToPointOwned for PointNormalRef {
    type Owned = PointNormal;

    fn to_owned(&self) -> Self::Owned {
        PointNormal {
            x: self.x(),
            y: self.y(),
            z: self.z(),
            normal_x: self.normal_x(),
            normal_y: self.normal_y(),
            normal_z: self.normal_z(),
            curvature: 0.0, // TODO: Add curvature getter
        }
    }
}

impl Debug for PointNormalRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointNormalRef")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("normal_x", &self.normal_x())
            .field("normal_y", &self.normal_y())
            .field("normal_z", &self.normal_z())
            .finish()
    }
}

// ============================================================================
// PointType Implementations
// ============================================================================

impl PointType for XYZ {
    type Owned = PointXYZ;
    type Ref = PointXYZRef;
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }
}

impl PointType for XYZI {
    type Owned = PointXYZI;
    type Ref = PointXYZIRef;
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }
}

impl PointType for XYZRGB {
    type Owned = PointXYZRGB;
    type Ref = PointXYZRGBRef;
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }
}

impl PointType for Normal {
    type Owned = PointNormal;
    type Ref = PointNormalRef;
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }
}

// ============================================================================
// Compatibility with Old Point Trait System
// ============================================================================

// These implementations are for compatibility with the old trait system
// They will be removed in a future version

use crate::traits::{
    Intensity, NormalXyz, Point, PointCloudClone, PointFfi, PointIntensityOps, PointRgbOps,
    PointXyzOps, Rgb, Xyz,
};

impl Point for PointXYZRef {
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }

    fn default_point() -> Self {
        panic!("PointXYZRef cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        // This method is deprecated - use the new PointCloud<T> API instead
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyz(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyz(cloud, index, &point.inner);
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        // This is unsafe because we're trusting PCL to give us a valid pointer
        let inner = unsafe { std::ptr::read(ptr.as_ref().unwrap()) };
        Ok(Self { inner })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height(cloud, height);
    }

    fn is_finite(&self) -> bool {
        ffi::is_finite_xyz(&self.inner)
    }
}

impl Clone for PointXYZRef {
    fn clone(&self) -> Self {
        panic!(
            "Cannot clone individual PointXYZRef - points can only be cloned as part of a PointCloud. This is a limitation of the FFI bridge."
        )
    }
}

impl PointFfi for PointXYZRef {
    type FfiType = ffi::PointXYZ;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

impl Xyz for PointXYZRef {
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

impl PointCloudClone for PointXYZRef {
    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget,
    {
        ffi::clone_point_cloud_xyz(cloud)
    }
}

impl PointXyzOps for PointXYZRef {
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32) {
        let coords = [x, y, z];
        ffi::push_back_xyz(cloud, &coords);
    }
}

// Compatibility trait for PointNormal operations
pub trait PointNormalOps: Point {
    /// Push a point with normal to the cloud
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

// Implementation is provided later with the full Point trait implementation

// Implement Point trait for PointXYZIRef
impl Point for PointXYZIRef {
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }

    fn default_point() -> Self {
        panic!("PointXYZIRef cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        // This method is deprecated - use the new PointCloud<T> API instead
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyzi(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyzi(cloud, index, &point.inner);
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let inner = unsafe { std::ptr::read(ptr.as_ref().unwrap()) };
        Ok(Self { inner })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzi(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzi(cloud, height);
    }

    fn is_finite(&self) -> bool {
        ffi::is_finite_xyzi(&self.inner)
    }
}

impl Clone for PointXYZIRef {
    fn clone(&self) -> Self {
        panic!(
            "Cannot clone individual PointXYZIRef - points can only be cloned as part of a PointCloud. This is a limitation of the FFI bridge."
        )
    }
}

impl PointFfi for PointXYZIRef {
    type FfiType = ffi::PointXYZI;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

impl Xyz for PointXYZIRef {
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

impl Intensity for PointXYZIRef {
    fn intensity(&self) -> f32 {
        ffi::get_intensity(&self.inner)
    }

    fn set_intensity(&mut self, intensity: f32) {
        unsafe {
            ffi::set_intensity(Pin::new_unchecked(&mut self.inner), intensity);
        }
    }
}

impl PointCloudClone for PointXYZIRef {
    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget,
    {
        ffi::clone_point_cloud_xyzi(cloud)
    }
}

impl PointXyzOps for PointXYZIRef {
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32) {
        let coords = [x, y, z, 0.0]; // Default intensity
        ffi::push_back_xyzi(cloud, &coords);
    }
}

impl PointIntensityOps for PointXYZIRef {
    fn push_xyzi(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, intensity: f32) {
        let coords = [x, y, z, intensity];
        ffi::push_back_xyzi(cloud, &coords);
    }
}

// Implement Point trait for PointXYZRGBRef
impl Point for PointXYZRGBRef {
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

    fn default_point() -> Self {
        panic!("PointXYZRGBRef cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        // This method is deprecated - use the new PointCloud<T> API instead
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyzrgb(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_xyzrgb(cloud, index, &point.inner);
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let inner = unsafe { std::ptr::read(ptr.as_ref().unwrap()) };
        Ok(Self { inner })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzrgb(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzrgb(cloud, height);
    }

    fn is_finite(&self) -> bool {
        ffi::is_finite_xyzrgb(&self.inner)
    }
}

impl Clone for PointXYZRGBRef {
    fn clone(&self) -> Self {
        panic!(
            "Cannot clone individual PointXYZRGBRef - points can only be cloned as part of a PointCloud. This is a limitation of the FFI bridge."
        )
    }
}

impl PointFfi for PointXYZRGBRef {
    type FfiType = ffi::PointXYZRGB;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

impl Xyz for PointXYZRGBRef {
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

impl Rgb for PointXYZRGBRef {
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

impl PointCloudClone for PointXYZRGBRef {
    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget,
    {
        ffi::clone_point_cloud_xyzrgb(cloud)
    }
}

impl PointXyzOps for PointXYZRGBRef {
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32) {
        let coords = [x, y, z, 0.0, 0.0, 0.0]; // Default color black
        ffi::push_back_xyzrgb(cloud, &coords);
    }
}

impl PointRgbOps for PointXYZRGBRef {
    fn push_xyzrgb(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) {
        let coords = [x, y, z, r as f32, g as f32, b as f32];
        ffi::push_back_xyzrgb(cloud, &coords);
    }
}

// Implement Point trait for PointNormalRef
impl Point for PointNormalRef {
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }

    fn default_point() -> Self {
        panic!("PointNormalRef cannot be created directly - use point cloud operations instead")
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        // This method is deprecated - use the new PointCloud<T> API instead
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_point_normal(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        ffi::set_point_at_point_normal(cloud, index, &point.inner);
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let inner = unsafe { std::ptr::read(ptr.as_ref().unwrap()) };
        Ok(Self { inner })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_point_normal(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_point_normal(cloud, height);
    }

    fn is_finite(&self) -> bool {
        ffi::is_finite_point_normal(&self.inner)
    }
}

impl Clone for PointNormalRef {
    fn clone(&self) -> Self {
        panic!(
            "Cannot clone individual PointNormalRef - points can only be cloned as part of a PointCloud. This is a limitation of the FFI bridge."
        )
    }
}

impl PointFfi for PointNormalRef {
    type FfiType = ffi::PointNormal;

    fn as_ffi(&self) -> &Self::FfiType {
        &self.inner
    }

    fn as_ffi_mut(&mut self) -> &mut Self::FfiType {
        &mut self.inner
    }
}

impl Xyz for PointNormalRef {
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

impl NormalXyz for PointNormalRef {
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

impl PointCloudClone for PointNormalRef {
    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget,
    {
        ffi::clone_point_cloud_point_normal(cloud)
    }
}

impl PointXyzOps for PointNormalRef {
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32) {
        let coords = [x, y, z, 0.0, 0.0, 1.0]; // Default normal pointing up
        ffi::push_back_point_normal(cloud, &coords);
    }
}

impl PointNormalOps for PointNormalRef {
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

// ============================================================================
// Point Trait Implementations for Owned Types
// ============================================================================

// These implementations are needed for compatibility with filter modules
// that expect owned types to implement Point and coordinate traits

impl crate::traits::Point for PointXYZ {
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }

    fn default_point() -> Self {
        Self::default()
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        // This method is deprecated - use the new PointCloud<T> API instead
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyz()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::size(cloud) == 0
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyz(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        // Convert owned point to FFI for setting
        let ffi_point = owned_to_ffi_xyz(*point);
        ffi::set_point_at_xyz(cloud, index, ffi_point.as_ref().unwrap());
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let ffi_point = ptr.as_ref().unwrap();
        Ok(Self {
            x: ffi::get_x(ffi_point),
            y: ffi::get_y(ffi_point),
            z: ffi::get_z(ffi_point),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height(cloud, height);
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl crate::traits::Xyz for PointXYZ {
    fn x(&self) -> f32 {
        self.x
    }

    fn y(&self) -> f32 {
        self.y
    }

    fn z(&self) -> f32 {
        self.z
    }

    fn set_x(&mut self, x: f32) {
        self.x = x;
    }

    fn set_y(&mut self, y: f32) {
        self.y = y;
    }

    fn set_z(&mut self, z: f32) {
        self.z = z;
    }
}

impl crate::traits::Point for PointXYZRGB {
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

    fn default_point() -> Self {
        Self::default()
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyzrgb()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_xyzrgb(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::size_xyzrgb(cloud) == 0
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyzrgb(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        let ffi_point = owned_to_ffi_xyzrgb(*point);
        ffi::set_point_at_xyzrgb(cloud, index, ffi_point.as_ref().unwrap());
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let ffi_point = ptr.as_ref().unwrap();
        Ok(Self {
            x: ffi::get_x_xyzrgb(ffi_point),
            y: ffi::get_y_xyzrgb(ffi_point),
            z: ffi::get_z_xyzrgb(ffi_point),
            r: ffi::get_r(ffi_point),
            g: ffi::get_g(ffi_point),
            b: ffi::get_b(ffi_point),
            _padding: 0,
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzrgb(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzrgb(cloud, height);
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl crate::traits::Xyz for PointXYZRGB {
    fn x(&self) -> f32 {
        self.x
    }

    fn y(&self) -> f32 {
        self.y
    }

    fn z(&self) -> f32 {
        self.z
    }

    fn set_x(&mut self, x: f32) {
        self.x = x;
    }

    fn set_y(&mut self, y: f32) {
        self.y = y;
    }

    fn set_z(&mut self, z: f32) {
        self.z = z;
    }
}

impl crate::traits::Rgb for PointXYZRGB {
    fn r(&self) -> u8 {
        self.r
    }

    fn g(&self) -> u8 {
        self.g
    }

    fn b(&self) -> u8 {
        self.b
    }

    fn set_r(&mut self, r: u8) {
        self.r = r;
    }

    fn set_g(&mut self, g: u8) {
        self.g = g;
    }

    fn set_b(&mut self, b: u8) {
        self.b = b;
    }

    // The trait provides default implementations for rgb(), set_rgb(), and rgb_as_float()
    // that work correctly with our basic getters and setters above
}

impl crate::traits::Point for PointXYZI {
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }

    fn default_point() -> Self {
        Self::default()
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_xyzi()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_xyzi(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::size_xyzi(cloud) == 0
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_xyzi(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        let ffi_point = owned_to_ffi_xyzi(*point);
        ffi::set_point_at_xyzi(cloud, index, ffi_point.as_ref().unwrap());
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let ffi_point = ptr.as_ref().unwrap();
        Ok(Self {
            x: ffi::get_x_xyzi(ffi_point),
            y: ffi::get_y_xyzi(ffi_point),
            z: ffi::get_z_xyzi(ffi_point),
            intensity: ffi::get_intensity(ffi_point),
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_xyzi(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_xyzi(cloud, height);
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.intensity.is_finite()
    }
}

impl crate::traits::Xyz for PointXYZI {
    fn x(&self) -> f32 {
        self.x
    }

    fn y(&self) -> f32 {
        self.y
    }

    fn z(&self) -> f32 {
        self.z
    }

    fn set_x(&mut self, x: f32) {
        self.x = x;
    }

    fn set_y(&mut self, y: f32) {
        self.y = y;
    }

    fn set_z(&mut self, z: f32) {
        self.z = z;
    }
}

impl crate::traits::Intensity for PointXYZI {
    fn intensity(&self) -> f32 {
        self.intensity
    }

    fn set_intensity(&mut self, intensity: f32) {
        self.intensity = intensity;
    }
}

impl crate::traits::Point for PointNormal {
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }

    fn default_point() -> Self {
        Self::default()
    }

    fn create_cloud() -> crate::error::PclResult<Self>
    where
        Self: Sized,
    {
        panic!("create_cloud is deprecated - use PointCloud<T>::new() instead")
    }

    fn new_cloud() -> cxx::UniquePtr<Self::CloudType> {
        ffi::new_point_cloud_point_normal()
    }

    fn cloud_size(cloud: &Self::CloudType) -> usize {
        ffi::size_point_normal(cloud)
    }

    fn cloud_empty(cloud: &Self::CloudType) -> bool {
        ffi::size_point_normal(cloud) == 0
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

    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        ffi::get_point_at_point_normal(cloud, index)
    }

    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self) {
        let ffi_point = owned_to_ffi_normal(*point);
        ffi::set_point_at_point_normal(cloud, index, ffi_point.as_ref().unwrap());
    }

    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if ptr.is_null() {
            return Err(crate::error::PclError::InvalidPointCloud {
                message: "Cannot create point from null pointer".to_string(),
                source: None,
            });
        }
        let ffi_point = ptr.as_ref().unwrap();
        Ok(Self {
            x: ffi::get_x_point_normal(ffi_point),
            y: ffi::get_y_point_normal(ffi_point),
            z: ffi::get_z_point_normal(ffi_point),
            normal_x: ffi::get_normal_x_point_normal(ffi_point),
            normal_y: ffi::get_normal_y_point_normal(ffi_point),
            normal_z: ffi::get_normal_z_point_normal(ffi_point),
            curvature: 0.0, // Default value for curvature
        })
    }

    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32) {
        ffi::set_width_point_normal(cloud, width);
    }

    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32) {
        ffi::set_height_point_normal(cloud, height);
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite()
            && self.y.is_finite()
            && self.z.is_finite()
            && self.normal_x.is_finite()
            && self.normal_y.is_finite()
            && self.normal_z.is_finite()
    }
}

impl crate::traits::Xyz for PointNormal {
    fn x(&self) -> f32 {
        self.x
    }

    fn y(&self) -> f32 {
        self.y
    }

    fn z(&self) -> f32 {
        self.z
    }

    fn set_x(&mut self, x: f32) {
        self.x = x;
    }

    fn set_y(&mut self, y: f32) {
        self.y = y;
    }

    fn set_z(&mut self, z: f32) {
        self.z = z;
    }
}

impl crate::traits::NormalXyz for PointNormal {
    fn normal_x(&self) -> f32 {
        self.normal_x
    }

    fn normal_y(&self) -> f32 {
        self.normal_y
    }

    fn normal_z(&self) -> f32 {
        self.normal_z
    }

    fn set_normal_x(&mut self, nx: f32) {
        self.normal_x = nx;
    }

    fn set_normal_y(&mut self, ny: f32) {
        self.normal_y = ny;
    }

    fn set_normal_z(&mut self, nz: f32) {
        self.normal_z = nz;
    }
}

// ============================================================================
// PointType Implementations for Owned Types (for compatibility)
// ============================================================================

// These allow owned types to work with new PointCloud<T> API
// For example: PointCloud<PointXYZ> instead of just PointCloud<XYZ>

impl PointType for PointXYZ {
    type Owned = PointXYZ;
    type Ref = PointXYZRef;
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }
}

impl PointType for PointXYZRGB {
    type Owned = PointXYZRGB;
    type Ref = PointXYZRGBRef;
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }
}

impl PointType for PointXYZI {
    type Owned = PointXYZI;
    type Ref = PointXYZIRef;
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }
}

impl PointType for PointNormal {
    type Owned = PointNormal;
    type Ref = PointNormalRef;
    type CloudType = ffi::PointCloud_PointNormal;
    type FfiPointType = ffi::PointNormal;

    fn type_name() -> &'static str {
        "PointNormal"
    }
}

// ============================================================================
// Conversion Utilities
// ============================================================================

/// Convert owned point to FFI point for pushing to cloud
pub fn owned_to_ffi_xyz(point: PointXYZ) -> cxx::UniquePtr<ffi::PointXYZ> {
    ffi::create_point_xyz(point.x, point.y, point.z)
}

pub fn owned_to_ffi_xyzi(point: PointXYZI) -> cxx::UniquePtr<ffi::PointXYZI> {
    ffi::create_point_xyzi(point.x, point.y, point.z, point.intensity)
}

pub fn owned_to_ffi_xyzrgb(point: PointXYZRGB) -> cxx::UniquePtr<ffi::PointXYZRGB> {
    ffi::create_point_xyzrgb(point.x, point.y, point.z, point.r, point.g, point.b)
}

pub fn owned_to_ffi_normal(point: PointNormal) -> cxx::UniquePtr<ffi::PointNormal> {
    ffi::create_point_normal(
        point.x,
        point.y,
        point.z,
        point.normal_x,
        point.normal_y,
        point.normal_z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_owned_point_creation() {
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.z, 3.0);
    }

    #[test]
    fn test_point_distance() {
        let p1 = PointXYZ::new(0.0, 0.0, 0.0);
        let p2 = PointXYZ::new(3.0, 4.0, 0.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
    }

    #[test]
    fn test_marker_types() {
        // Verify marker types work correctly
        assert_eq!(XYZ::type_name(), "PointXYZ");
        assert_eq!(XYZI::type_name(), "PointXYZI");
    }

    #[test]
    fn test_conversions() {
        // Test owned to FFI conversion
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        let ffi_point = owned_to_ffi_xyz(p);
        assert!(!ffi_point.is_null());
    }
}
