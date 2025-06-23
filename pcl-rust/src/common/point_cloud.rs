//! PointCloud implementation using marker types
//!
//! This module provides the main PointCloud<T> API using marker types for clean generics.

use crate::common::point_types::{PointType, ToPointOwned};
use crate::error::{PclError, PclResult};
use pcl_sys::ffi;
use std::fmt;
use std::marker::PhantomData;
use std::pin::Pin;

/// A point cloud container using marker types for clean generic APIs
pub struct PointCloud<T: PointType> {
    inner: cxx::UniquePtr<T::CloudType>,
    _phantom: PhantomData<T>,
}

impl<T: PointType> PointCloud<T> {
    /// Create a new empty point cloud
    pub fn new() -> PclResult<Self> {
        let inner: cxx::UniquePtr<T::CloudType> = match T::type_name() {
            "PointXYZ" => {
                let cloud = ffi::new_point_cloud_xyz();
                // SAFETY: We know the type matches based on type_name
                unsafe {
                    std::mem::transmute::<
                        pcl_sys::UniquePtr<pcl_sys::common::PointCloudXYZ>,
                        pcl_sys::UniquePtr<T::CloudType>,
                    >(cloud)
                }
            }
            "PointXYZI" => {
                let cloud = ffi::new_point_cloud_xyzi();
                unsafe {
                    std::mem::transmute::<
                        pcl_sys::UniquePtr<pcl_sys::common::PointCloudXYZI>,
                        pcl_sys::UniquePtr<T::CloudType>,
                    >(cloud)
                }
            }
            "PointXYZRGB" => {
                let cloud = ffi::new_point_cloud_xyzrgb();
                unsafe {
                    std::mem::transmute::<
                        pcl_sys::UniquePtr<pcl_sys::common::PointCloudXYZRGB>,
                        pcl_sys::UniquePtr<T::CloudType>,
                    >(cloud)
                }
            }
            "PointNormal" => {
                let cloud = ffi::new_point_cloud_point_normal();
                unsafe {
                    std::mem::transmute::<
                        pcl_sys::UniquePtr<pcl_sys::raw_ffi::PointCloud_PointNormal>,
                        pcl_sys::UniquePtr<T::CloudType>,
                    >(cloud)
                }
            }
            _ => {
                return Err(PclError::InvalidParameter {
                    param: "point type".to_string(),
                    message: format!("Unsupported point type: {}", T::type_name()),
                });
            }
        };

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

    /// Get the number of points in the cloud
    pub fn size(&self) -> usize {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZ)
                };
                ffi::size(cloud)
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZI)
                };
                ffi::size_xyzi(cloud)
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZRGB)
                };
                ffi::size_xyzrgb(cloud)
            }
            "PointNormal" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointNormal)
                };
                ffi::size_point_normal(cloud)
            }
            _ => 0,
        }
    }

    /// Check if the cloud is empty
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Get a reference to a point at the given index
    pub fn get(&self, index: usize) -> PclResult<&T::Ref> {
        if index >= self.size() {
            return Err(PclError::IndexOutOfBounds {
                index,
                size: self.size(),
            });
        }

        // This will be implemented when we have proper point access
        todo!("Point access not yet implemented")
    }

    /// Get an owned copy of a point at the given index
    pub fn at(&self, index: usize) -> PclResult<T::Owned> {
        self.get(index).map(|p| p.to_owned())
    }

    /// Create an iterator over owned copies of points
    pub fn iter(&self) -> PointCloudIter<T> {
        PointCloudIter {
            cloud: self,
            index: 0,
        }
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZ),
                    )
                };
                ffi::clear(cloud);
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZI),
                    )
                };
                ffi::clear_xyzi(cloud);
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZRGB),
                    )
                };
                ffi::clear_xyzrgb(cloud);
            }
            "PointNormal" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointNormal),
                    )
                };
                ffi::clear_point_normal(cloud);
            }
            _ => {
                return Err(PclError::InvalidParameter {
                    param: "point type".to_string(),
                    message: format!("Clear not implemented for point type: {}", T::type_name()),
                });
            }
        }
        Ok(())
    }

    /// Reserve space for n points
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZ),
                    )
                };
                ffi::reserve_xyz(cloud, n);
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZI),
                    )
                };
                ffi::reserve_xyzi(cloud, n);
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZRGB),
                    )
                };
                ffi::reserve_xyzrgb(cloud, n);
            }
            "PointNormal" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointNormal),
                    )
                };
                ffi::reserve_point_normal(cloud, n);
            }
            _ => {
                return Err(PclError::InvalidParameter {
                    param: "point type".to_string(),
                    message: format!("Reserve not implemented for point type: {}", T::type_name()),
                });
            }
        }
        Ok(())
    }

    /// Resize the cloud to contain n points
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZ),
                    )
                };
                ffi::resize_xyz(cloud, n);
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZI),
                    )
                };
                ffi::resize_xyzi(cloud, n);
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZRGB),
                    )
                };
                ffi::resize_xyzrgb(cloud, n);
            }
            "PointNormal" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointNormal),
                    )
                };
                ffi::resize_point_normal(cloud, n);
            }
            _ => {
                return Err(PclError::InvalidParameter {
                    param: "point type".to_string(),
                    message: format!("Resize not implemented for point type: {}", T::type_name()),
                });
            }
        }
        Ok(())
    }

    /// Check if the cloud is empty (alias for backwards compatibility)
    pub fn empty(&self) -> bool {
        self.is_empty()
    }

    /// Create from a UniquePtr (internal use only)
    pub(crate) fn from_unique_ptr(inner: cxx::UniquePtr<T::CloudType>) -> Self {
        Self {
            inner,
            _phantom: PhantomData,
        }
    }

    /// Get access to the inner cloud pointer (for compatibility with filters)
    pub(crate) fn inner(&self) -> &T::CloudType {
        self.inner
            .as_ref()
            .expect("PointCloud inner pointer is null")
    }

    /// Get mutable access to the inner cloud pointer (for compatibility)
    pub(crate) fn inner_mut(&mut self) -> Pin<&mut T::CloudType> {
        self.inner.pin_mut()
    }

    /// Get the width of the cloud (for organized clouds)
    pub fn width(&self) -> u32 {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZ)
                };
                ffi::width(cloud)
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZI)
                };
                ffi::width_xyzi(cloud)
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZRGB)
                };
                ffi::width_xyzrgb(cloud)
            }
            "PointNormal" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointNormal)
                };
                ffi::width_point_normal(cloud)
            }
            _ => 0,
        }
    }

    /// Get the height of the cloud (for organized clouds)
    pub fn height(&self) -> u32 {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZ)
                };
                ffi::height(cloud)
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZI)
                };
                ffi::height_xyzi(cloud)
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointXYZRGB)
                };
                ffi::height_xyzrgb(cloud)
            }
            "PointNormal" => {
                let cloud = unsafe {
                    &*(self.inner.as_ref().unwrap() as *const T::CloudType
                        as *const ffi::PointCloud_PointNormal)
                };
                ffi::height_point_normal(cloud)
            }
            _ => 0,
        }
    }

    /// Check if the cloud is organized (width > 1 and height > 1)
    pub fn is_organized(&self) -> bool {
        self.width() > 1 && self.height() > 1
    }
}

// Generic push method using marker types
impl<T: PointType> PointCloud<T> {
    /// Push a point to the cloud
    pub fn push(&mut self, point: T::Owned) -> PclResult<()> {
        match T::type_name() {
            "PointXYZ" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZ),
                    )
                };
                // SAFETY: We know T::Owned is PointXYZ based on type_name
                let point: &crate::common::point_types::PointXYZ =
                    unsafe { std::mem::transmute(&point) };
                let coords = [point.x, point.y, point.z];
                ffi::push_back_xyz(cloud, &coords);
            }
            "PointXYZI" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZI),
                    )
                };
                let point: &crate::common::point_types::PointXYZI =
                    unsafe { std::mem::transmute(&point) };
                let coords = [point.x, point.y, point.z, point.intensity];
                ffi::push_back_xyzi(cloud, &coords);
            }
            "PointXYZRGB" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointXYZRGB),
                    )
                };
                let point: &crate::common::point_types::PointXYZRGB =
                    unsafe { std::mem::transmute(&point) };
                let coords = [
                    point.x,
                    point.y,
                    point.z,
                    point.r as f32,
                    point.g as f32,
                    point.b as f32,
                ];
                ffi::push_back_xyzrgb(cloud, &coords);
            }
            "PointNormal" => {
                let cloud = unsafe {
                    Pin::new_unchecked(
                        &mut *(self.inner.pin_mut().get_unchecked_mut() as *mut T::CloudType
                            as *mut ffi::PointCloud_PointNormal),
                    )
                };
                let point: &crate::common::point_types::PointNormal =
                    unsafe { std::mem::transmute(&point) };
                let coords = [
                    point.x,
                    point.y,
                    point.z,
                    point.normal_x,
                    point.normal_y,
                    point.normal_z,
                ];
                ffi::push_back_point_normal(cloud, &coords);
            }
            _ => {
                return Err(PclError::InvalidParameter {
                    param: "point type".to_string(),
                    message: format!("Unsupported point type for push: {}", T::type_name()),
                });
            }
        }
        Ok(())
    }
}

impl<T: PointType> Default for PointCloud<T> {
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloud")
    }
}

impl<T: PointType> fmt::Debug for PointCloud<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PointCloud")
            .field("type", &T::type_name())
            .field("size", &self.size())
            .finish()
    }
}

// ============================================================================
// Iterator Implementation
// ============================================================================

/// Iterator over owned copies of points in a PointCloud
pub struct PointCloudIter<'a, T: PointType> {
    cloud: &'a PointCloud<T>,
    index: usize,
}

impl<'a, T: PointType> Iterator for PointCloudIter<'a, T> {
    type Item = T::Owned;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.cloud.size() {
            return None;
        }

        // For now, we don't have point access implemented
        // When implemented, this would be:
        // let point = self.cloud.at(self.index).ok()?;
        // self.index += 1;
        // Some(point)

        None // TODO: Implement when point access is available
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.cloud.size().saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a, T: PointType> ExactSizeIterator for PointCloudIter<'a, T> {
    fn len(&self) -> usize {
        self.cloud.size().saturating_sub(self.index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::point_types::{Normal, PointXYZ, XYZ, XYZI, XYZRGB};

    #[test]
    fn test_point_cloud2_creation() {
        let cloud: PointCloud<XYZ> = PointCloud::new().unwrap();
        assert_eq!(cloud.size(), 0);
        assert!(cloud.is_empty());
    }

    #[test]
    fn test_point_cloud2_push() {
        let mut cloud: PointCloud<XYZ> = PointCloud::new().unwrap();
        let point = PointXYZ::new(1.0, 2.0, 3.0);
        cloud.push(point).unwrap();
        assert_eq!(cloud.size(), 1);
        assert!(!cloud.is_empty());
    }

    #[test]
    fn test_different_types() {
        let _xyz: PointCloud<XYZ> = PointCloud::new().unwrap();
        let _xyzi: PointCloud<XYZI> = PointCloud::new().unwrap();
        let _xyzrgb: PointCloud<XYZRGB> = PointCloud::new().unwrap();
        let _normal: PointCloud<Normal> = PointCloud::new().unwrap();
    }

    #[test]
    fn test_iterator() {
        let cloud: PointCloud<XYZ> = PointCloud::new().unwrap();
        let iter = cloud.iter();

        // Iterator should report correct size hints
        assert_eq!(iter.size_hint(), (0, Some(0)));
        assert_eq!(iter.len(), 0);
    }
}
