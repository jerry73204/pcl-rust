//! VoxelGrid filter for downsampling point clouds
//!
//! The VoxelGrid filter creates a 3D voxel grid over the input point cloud.
//! All points within each voxel are approximated with their centroid, effectively
//! downsampling the cloud while preserving its shape characteristics.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::PclResult;
use crate::filters::{FilterXYZ, FilterXYZRGB};
use pcl_sys::{UniquePtr, ffi};
use std::pin::Pin;

/// VoxelGrid filter for PointXYZ clouds
pub struct VoxelGridXYZ {
    inner: UniquePtr<ffi::VoxelGrid_PointXYZ>,
}

impl VoxelGridXYZ {
    /// Create a new VoxelGrid filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_voxel_grid_xyz();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "VoxelGrid filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the voxel size (leaf size) for the grid
    ///
    /// # Arguments
    /// * `lx` - Leaf size in X dimension
    /// * `ly` - Leaf size in Y dimension
    /// * `lz` - Leaf size in Z dimension
    pub fn set_leaf_size(&mut self, lx: f32, ly: f32, lz: f32) -> PclResult<()> {
        if lx <= 0.0 || ly <= 0.0 || lz <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "leaf_size".to_string(),
                message: "must be positive".to_string(),
            });
        }
        unsafe {
            ffi::set_leaf_size_xyz(self.inner.pin_mut(), lx, ly, lz);
        }
        Ok(())
    }
}

impl FilterXYZ for VoxelGridXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        unsafe {
            ffi::set_input_cloud_voxel_xyz(self.inner.pin_mut(), cloud.as_raw());
        }
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZ> {
        unsafe {
            let result = ffi::filter_voxel_xyz(self.inner.pin_mut());
            if result.is_null() {
                return Err(crate::error::PclError::ProcessingFailed {
                    message: "VoxelGrid filter failed".to_string(),
                });
            }
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }
}

/// Builder for VoxelGridXYZ filter
pub struct VoxelGridXYZBuilder {
    leaf_size: Option<(f32, f32, f32)>,
}

impl VoxelGridXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self { leaf_size: None }
    }

    /// Set the voxel size (leaf size) for the grid
    pub fn leaf_size(mut self, lx: f32, ly: f32, lz: f32) -> Self {
        self.leaf_size = Some((lx, ly, lz));
        self
    }

    /// Build the VoxelGrid filter
    pub fn build(self) -> PclResult<VoxelGridXYZ> {
        let mut filter = VoxelGridXYZ::new()?;

        if let Some((lx, ly, lz)) = self.leaf_size {
            filter.set_leaf_size(lx, ly, lz)?;
        }

        Ok(filter)
    }
}

impl Default for VoxelGridXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// VoxelGrid filter for PointXYZRGB clouds
pub struct VoxelGridXYZRGB {
    inner: UniquePtr<ffi::VoxelGrid_PointXYZRGB>,
}

impl VoxelGridXYZRGB {
    /// Create a new VoxelGrid filter
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_voxel_grid_xyzrgb();
        if inner.is_null() {
            return Err(crate::error::PclError::CreationFailed {
                typename: "VoxelGrid filter".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the voxel size (leaf size) for the grid
    ///
    /// # Arguments
    /// * `lx` - Leaf size in X dimension
    /// * `ly` - Leaf size in Y dimension
    /// * `lz` - Leaf size in Z dimension
    pub fn set_leaf_size(&mut self, lx: f32, ly: f32, lz: f32) -> PclResult<()> {
        if lx <= 0.0 || ly <= 0.0 || lz <= 0.0 {
            return Err(crate::error::PclError::InvalidParameter {
                param: "leaf_size".to_string(),
                message: "must be positive".to_string(),
            });
        }
        unsafe {
            ffi::set_leaf_size_xyzrgb(self.inner.pin_mut(), lx, ly, lz);
        }
        Ok(())
    }
}

impl FilterXYZRGB for VoxelGridXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        unsafe {
            ffi::set_input_cloud_voxel_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        }
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloudXYZRGB> {
        unsafe {
            let result = ffi::filter_voxel_xyzrgb(self.inner.pin_mut());
            if result.is_null() {
                return Err(crate::error::PclError::ProcessingFailed {
                    message: "VoxelGrid filter failed".to_string(),
                });
            }
            Ok(PointCloudXYZRGB::from_unique_ptr(result))
        }
    }
}

/// Builder for VoxelGridXYZRGB filter
pub struct VoxelGridXYZRGBBuilder {
    leaf_size: Option<(f32, f32, f32)>,
}

impl VoxelGridXYZRGBBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self { leaf_size: None }
    }

    /// Set the voxel size (leaf size) for the grid
    pub fn leaf_size(mut self, lx: f32, ly: f32, lz: f32) -> Self {
        self.leaf_size = Some((lx, ly, lz));
        self
    }

    /// Build the VoxelGrid filter
    pub fn build(self) -> PclResult<VoxelGridXYZRGB> {
        let mut filter = VoxelGridXYZRGB::new()?;

        if let Some((lx, ly, lz)) = self.leaf_size {
            filter.set_leaf_size(lx, ly, lz)?;
        }

        Ok(filter)
    }
}

impl Default for VoxelGridXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}
