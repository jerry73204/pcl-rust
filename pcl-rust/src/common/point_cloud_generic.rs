//! Generic PointCloud<T> implementation using associated types
//!
//! This module provides the unified generic PointCloud<T> API that works with any
//! point type implementing the Point trait with associated types.

use crate::common::point_types::{PointNormal, PointNormalOps, PointXYZ, PointXYZI, PointXYZRGB};
use crate::error::{PclError, PclResult};
use crate::traits::{Point, PointIntensityOps, PointRgbOps, PointXyzOps, Xyz};
use cxx::{self, memory::UniquePtrTarget};
use std::fmt;
use std::marker::PhantomData;

/// A generic point cloud container that works with any point type
///
/// This struct provides a unified interface for working with point clouds
/// of different types (PointXYZ, PointXYZRGB, PointXYZI, etc.) through
/// the use of associated types.
///
/// # Type Parameters
///
/// * `T` - The point type, which must implement the `Point` trait
///
/// # Examples
///
/// ```rust
/// use pcl::{PointCloud, PointXYZ, PointXYZRGB};
///
/// // Create a point cloud of XYZ points
/// let mut xyz_cloud: PointCloud<PointXYZ> = PointCloud::new()?;
///
/// // Create a point cloud of colored points
/// let mut rgb_cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;
/// ```
pub struct PointCloud<T: Point>
where
    T::CloudType: UniquePtrTarget,
{
    inner: cxx::UniquePtr<T::CloudType>,
    _phantom: PhantomData<T>,
}

impl<T: Point> Clone for PointCloud<T>
where
    T::CloudType: UniquePtrTarget,
{
    fn clone(&self) -> Self {
        // Use our custom deep_clone method which returns Result
        // and unwrap since Clone trait doesn't allow Result
        self.deep_clone().expect("Failed to clone PointCloud")
    }
}

impl<T: Point> PointCloud<T>
where
    T::CloudType: UniquePtrTarget,
{
    /// Create a new empty point cloud
    pub fn new() -> PclResult<Self> {
        let inner = T::new_cloud();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: T::type_name().to_string(),
            });
        }

        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    /// Create from a UniquePtr (internal use only)
    pub(crate) fn from_unique_ptr(inner: cxx::UniquePtr<T::CloudType>) -> Self {
        Self {
            inner,
            _phantom: PhantomData,
        }
    }

    /// Get the number of points in the cloud
    pub fn len(&self) -> usize {
        T::cloud_size(&*self.inner)
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        T::cloud_empty(&*self.inner)
    }

    /// Alias for size() for backward compatibility
    pub fn size(&self) -> usize {
        self.len()
    }

    /// Alias for empty() for backward compatibility
    pub fn is_empty(&self) -> bool {
        self.empty()
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        T::cloud_clear(self.inner.pin_mut());
        Ok(())
    }

    /// Reserve capacity for at least n points
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        T::cloud_reserve(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Resize the point cloud to contain n points
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        T::cloud_resize(self.inner.pin_mut(), n);
        Ok(())
    }

    /// Get the width of the cloud (for organized clouds)
    pub fn width(&self) -> u32 {
        T::cloud_width(&*self.inner)
    }

    /// Get the height of the cloud (for organized clouds)
    pub fn height(&self) -> u32 {
        T::cloud_height(&*self.inner)
    }

    /// Check if the cloud is organized (2D structure)
    pub fn is_organized(&self) -> bool {
        self.height() > 1
    }

    /// Check if the cloud is dense (no invalid points)
    pub fn is_dense(&self) -> bool {
        T::cloud_is_dense(&*self.inner)
    }

    /// Get raw pointer for FFI operations
    pub(crate) fn as_raw(&self) -> *const T::CloudType {
        T::as_raw_cloud(&*self.inner)
    }

    /// Get reference to inner cloud for FFI operations
    pub(crate) fn inner(&self) -> &T::CloudType {
        &*self.inner
    }

    /// Get mutable pinned reference to inner cloud for FFI operations
    pub(crate) fn inner_mut(&mut self) -> std::pin::Pin<&mut T::CloudType> {
        self.inner.pin_mut()
    }

    /// Access a point at the given index
    ///
    /// Returns the point at the specified index, or an error if the index is out of bounds.
    pub fn at(&self, index: usize) -> PclResult<T>
    where
        T::FfiPointType: cxx::memory::UniquePtrTarget,
    {
        if index >= self.size() {
            return Err(PclError::IndexOutOfBounds {
                index,
                size: self.size(),
            });
        }

        let point_ptr = T::get_point_at(&*self.inner, index);
        if point_ptr.is_null() {
            return Err(PclError::IndexOutOfBounds {
                index,
                size: self.size(),
            });
        }

        T::from_unique_ptr(point_ptr)
    }

    /// Set a point at the given index
    ///
    /// Replaces the point at the specified index with a new point.
    pub fn set_at(&mut self, index: usize, point: &T) -> PclResult<()> {
        if index >= self.size() {
            return Err(PclError::IndexOutOfBounds {
                index,
                size: self.size(),
            });
        }

        T::set_point_at(self.inner.pin_mut(), index, point);
        Ok(())
    }

    /// Set the width of the cloud (for organized clouds)
    pub fn set_width(&mut self, width: u32) {
        T::cloud_set_width(self.inner.pin_mut(), width);
    }

    /// Set the height of the cloud (for organized clouds)
    pub fn set_height(&mut self, height: u32) {
        T::cloud_set_height(self.inner.pin_mut(), height);
    }

    /// Deep clone the entire point cloud
    pub fn deep_clone(&self) -> PclResult<Self> {
        let cloned = T::cloud_clone(&*self.inner);
        if cloned.is_null() {
            return Err(PclError::CloneFailed {
                typename: T::type_name().to_string(),
            });
        }
        Ok(Self::from_unique_ptr(cloned))
    }
}

// Extension methods for XYZ points
impl<T: Point + Xyz + PointXyzOps> PointCloud<T>
where
    T::CloudType: UniquePtrTarget,
{
    /// Add a point by coordinates
    pub fn push(&mut self, x: f32, y: f32, z: f32) -> PclResult<()> {
        T::push_xyz(self.inner.pin_mut(), x, y, z);
        Ok(())
    }

    /// Add multiple points from coordinates
    pub fn extend_from_slice(&mut self, coords: &[[f32; 3]]) -> PclResult<()> {
        for &[x, y, z] in coords {
            self.push(x, y, z)?;
        }
        Ok(())
    }
}

// Specialized push method for PointXYZRGB
impl PointCloud<PointXYZRGB>
where
    <PointXYZRGB as Point>::CloudType: UniquePtrTarget,
{
    /// Add a colored point (alias for backward compatibility)
    pub fn push(&mut self, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> PclResult<()> {
        self.push_colored(x, y, z, r, g, b)
    }

    /// Add a colored point
    pub fn push_colored(&mut self, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> PclResult<()> {
        PointXYZRGB::push_xyzrgb(self.inner.pin_mut(), x, y, z, r, g, b);
        Ok(())
    }
}

// Specialized push method for PointXYZI
impl PointCloud<PointXYZI>
where
    <PointXYZI as Point>::CloudType: UniquePtrTarget,
{
    /// Add a point with intensity
    pub fn push_with_intensity(&mut self, x: f32, y: f32, z: f32, intensity: f32) -> PclResult<()> {
        PointXYZI::push_xyzi(self.inner.pin_mut(), x, y, z, intensity);
        Ok(())
    }
}

// Specialized push method for PointNormal
impl PointCloud<PointNormal>
where
    <PointNormal as Point>::CloudType: UniquePtrTarget,
{
    /// Add a point with normal
    pub fn push_with_normal(
        &mut self,
        x: f32,
        y: f32,
        z: f32,
        nx: f32,
        ny: f32,
        nz: f32,
    ) -> PclResult<()> {
        PointNormal::push_point_normal(self.inner.pin_mut(), x, y, z, nx, ny, nz);
        Ok(())
    }
}

// Implement Default for PointCloud
impl<T: Point> Default for PointCloud<T>
where
    T::CloudType: UniquePtrTarget,
{
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloud")
    }
}

// Implement Debug for PointCloud
impl<T: Point> fmt::Debug for PointCloud<T>
where
    T::CloudType: UniquePtrTarget,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PointCloud")
            .field("type", &T::type_name())
            .field("size", &self.size())
            .field("organized", &self.is_organized())
            .field("dense", &self.is_dense())
            .finish()
    }
}

// Type aliases for backward compatibility
pub type PointCloudXYZ = PointCloud<PointXYZ>;
pub type PointCloudXYZRGB = PointCloud<PointXYZRGB>;
pub type PointCloudXYZI = PointCloud<PointXYZI>;
pub type PointCloudNormal = PointCloud<PointNormal>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_cloud_creation() {
        let xyz_cloud: PointCloud<PointXYZ> = PointCloud::new().unwrap();
        assert!(xyz_cloud.empty());
        assert_eq!(xyz_cloud.len(), 0);

        let rgb_cloud: PointCloud<PointXYZRGB> = PointCloud::new().unwrap();
        assert!(rgb_cloud.empty());
        assert_eq!(rgb_cloud.len(), 0);

        let xyzi_cloud: PointCloud<PointXYZI> = PointCloud::new().unwrap();
        assert!(xyzi_cloud.empty());
        assert_eq!(xyzi_cloud.len(), 0);
    }

    #[test]
    fn test_type_aliases() {
        let _xyz: PointCloudXYZ = PointCloud::new().unwrap();
        let _rgb: PointCloudXYZRGB = PointCloud::new().unwrap();
        let _xyzi: PointCloudXYZI = PointCloud::new().unwrap();
    }
}
