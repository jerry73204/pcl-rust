//! Generic filter implementations that work with any point type
//!
//! This module provides generic filter traits and implementations that can work
//! with any point type implementing the required capabilities.

use crate::common::PointCloud;
use crate::error::{PclError, PclResult};
use crate::traits::{Point, Xyz};
use cxx::{UniquePtr, memory::UniquePtrTarget};
use std::marker::PhantomData;

/// Generic filter trait for point cloud filtering operations
///
/// This trait defines the common interface for all filters that operate
/// on point clouds, transforming an input cloud to an output cloud.
pub trait Filter<T: Point>
where
    T::CloudType: UniquePtrTarget,
{
    /// Set the input point cloud for filtering
    fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()>;

    /// Apply the filter and return the filtered point cloud
    fn filter(&mut self) -> PclResult<PointCloud<T>>;
}

/// Trait for point types that support VoxelGrid filtering
pub trait VoxelGridPoint: Point + Xyz {
    /// The FFI VoxelGrid type for this point type
    type VoxelGridType: Send + Sync;

    /// Create a new VoxelGrid filter instance
    fn new_voxel_grid() -> UniquePtr<Self::VoxelGridType>
    where
        Self::VoxelGridType: UniquePtrTarget;

    /// Set the input cloud for the filter
    fn set_input_cloud_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        cloud: &Self::CloudType,
    );

    /// Set the leaf size for the voxel grid
    fn set_leaf_size_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        lx: f32,
        ly: f32,
        lz: f32,
    );

    /// Apply the filter and return the result
    fn filter_voxel(filter: std::pin::Pin<&mut Self::VoxelGridType>) -> UniquePtr<Self::CloudType>
    where
        Self::CloudType: UniquePtrTarget;
}

/// Generic VoxelGrid filter for downsampling point clouds
///
/// The VoxelGrid filter creates a 3D voxel grid over the input point cloud.
/// All points within each voxel are approximated with their centroid.
///
/// # Type Parameters
///
/// * `T` - The point type, which must implement `VoxelGridPoint`
///
/// # Examples
///
/// ```rust
/// use pcl::{PointCloud, PointXYZ, VoxelGrid, Filter};
///
/// let mut cloud = PointCloud::<PointXYZ>::new()?;
/// // ... add points to cloud ...
///
/// let mut filter = VoxelGrid::<PointXYZ>::new()?;
/// filter.set_leaf_size(0.01, 0.01, 0.01)?;
/// filter.set_input_cloud(&cloud)?;
/// let filtered_cloud = filter.filter()?;
/// ```
pub struct VoxelGrid<T: VoxelGridPoint>
where
    T::VoxelGridType: UniquePtrTarget,
{
    inner: UniquePtr<T::VoxelGridType>,
    _phantom: PhantomData<T>,
}

impl<T: VoxelGridPoint> VoxelGrid<T>
where
    T::VoxelGridType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    /// Create a new VoxelGrid filter
    pub fn new() -> PclResult<Self> {
        let inner = T::new_voxel_grid();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: format!("VoxelGrid<{}>", T::type_name()),
            });
        }

        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    /// Set the voxel size (leaf size) for the grid
    ///
    /// # Arguments
    /// * `lx` - Leaf size in X dimension
    /// * `ly` - Leaf size in Y dimension
    /// * `lz` - Leaf size in Z dimension
    pub fn set_leaf_size(&mut self, lx: f32, ly: f32, lz: f32) -> PclResult<()> {
        if lx <= 0.0 || ly <= 0.0 || lz <= 0.0 {
            return Err(PclError::InvalidParameter {
                param: "leaf_size".to_string(),
                message: "must be positive".to_string(),
            });
        }
        T::set_leaf_size_voxel(self.inner.pin_mut(), lx, ly, lz);
        Ok(())
    }
}

impl<T: VoxelGridPoint> Filter<T> for VoxelGrid<T>
where
    T::VoxelGridType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()> {
        T::set_input_cloud_voxel(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloud<T>> {
        let result = T::filter_voxel(self.inner.pin_mut());
        if result.is_null() {
            return Err(PclError::ProcessingFailed {
                message: "VoxelGrid filter failed".to_string(),
            });
        }
        Ok(PointCloud::from_unique_ptr(result))
    }
}

/// Trait for point types that support PassThrough filtering
pub trait PassThroughPoint: Point {
    /// The FFI PassThrough type for this point type
    type PassThroughType: Send + Sync;

    /// Create a new PassThrough filter instance
    fn new_pass_through() -> UniquePtr<Self::PassThroughType>
    where
        Self::PassThroughType: UniquePtrTarget;

    /// Set the input cloud for the filter
    fn set_input_cloud_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        cloud: &Self::CloudType,
    );

    /// Set the field name to filter on
    fn set_filter_field_name_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        field_name: &str,
    );

    /// Set the filter limits
    fn set_filter_limits_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        min: f32,
        max: f32,
    );

    /// Apply the filter and return the result
    fn filter_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
    ) -> UniquePtr<Self::CloudType>
    where
        Self::CloudType: UniquePtrTarget;
}

/// Generic PassThrough filter for field-based filtering
///
/// The PassThrough filter allows filtering points based on the values
/// of a specific field (e.g., x, y, z coordinates).
pub struct PassThrough<T: PassThroughPoint>
where
    T::PassThroughType: UniquePtrTarget,
{
    inner: UniquePtr<T::PassThroughType>,
    _phantom: PhantomData<T>,
}

impl<T: PassThroughPoint> PassThrough<T>
where
    T::PassThroughType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    /// Create a new PassThrough filter
    pub fn new() -> PclResult<Self> {
        let inner = T::new_pass_through();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: format!("PassThrough<{}>", T::type_name()),
            });
        }

        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    /// Set the field name to filter on (e.g., "x", "y", "z")
    pub fn set_filter_field_name(&mut self, field_name: &str) -> PclResult<()> {
        T::set_filter_field_name_pass_through(self.inner.pin_mut(), field_name);
        Ok(())
    }

    /// Set the filter limits
    ///
    /// Points with field values outside [min, max] will be removed
    pub fn set_filter_limits(&mut self, min: f32, max: f32) -> PclResult<()> {
        if min > max {
            return Err(PclError::InvalidParameter {
                param: "filter_limits".to_string(),
                message: "min must be <= max".to_string(),
            });
        }
        T::set_filter_limits_pass_through(self.inner.pin_mut(), min, max);
        Ok(())
    }
}

impl<T: PassThroughPoint> Filter<T> for PassThrough<T>
where
    T::PassThroughType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()> {
        T::set_input_cloud_pass_through(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    fn filter(&mut self) -> PclResult<PointCloud<T>> {
        let result = T::filter_pass_through(self.inner.pin_mut());
        if result.is_null() {
            return Err(PclError::ProcessingFailed {
                message: "PassThrough filter failed".to_string(),
            });
        }
        Ok(PointCloud::from_unique_ptr(result))
    }
}

// Implement the filter traits for concrete point types
use crate::common::{PointXYZ, PointXYZRGB};
use pcl_sys::ffi;

impl VoxelGridPoint for PointXYZ {
    type VoxelGridType = ffi::VoxelGrid_PointXYZ;

    fn new_voxel_grid() -> UniquePtr<Self::VoxelGridType> {
        ffi::new_voxel_grid_xyz()
    }

    fn set_input_cloud_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        cloud: &Self::CloudType,
    ) {
        ffi::set_input_cloud_voxel_xyz(filter, cloud);
    }

    fn set_leaf_size_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        lx: f32,
        ly: f32,
        lz: f32,
    ) {
        ffi::set_leaf_size_xyz(filter, lx, ly, lz);
    }

    fn filter_voxel(filter: std::pin::Pin<&mut Self::VoxelGridType>) -> UniquePtr<Self::CloudType> {
        ffi::filter_voxel_xyz(filter)
    }
}

impl VoxelGridPoint for PointXYZRGB {
    type VoxelGridType = ffi::VoxelGrid_PointXYZRGB;

    fn new_voxel_grid() -> UniquePtr<Self::VoxelGridType> {
        ffi::new_voxel_grid_xyzrgb()
    }

    fn set_input_cloud_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        cloud: &Self::CloudType,
    ) {
        ffi::set_input_cloud_voxel_xyzrgb(filter, cloud);
    }

    fn set_leaf_size_voxel(
        filter: std::pin::Pin<&mut Self::VoxelGridType>,
        lx: f32,
        ly: f32,
        lz: f32,
    ) {
        ffi::set_leaf_size_xyzrgb(filter, lx, ly, lz);
    }

    fn filter_voxel(filter: std::pin::Pin<&mut Self::VoxelGridType>) -> UniquePtr<Self::CloudType> {
        ffi::filter_voxel_xyzrgb(filter)
    }
}

impl PassThroughPoint for PointXYZ {
    type PassThroughType = ffi::PassThrough_PointXYZ;

    fn new_pass_through() -> UniquePtr<Self::PassThroughType> {
        ffi::new_pass_through_xyz()
    }

    fn set_input_cloud_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        cloud: &Self::CloudType,
    ) {
        ffi::set_input_cloud_pass_xyz(filter, cloud);
    }

    fn set_filter_field_name_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        field_name: &str,
    ) {
        ffi::set_filter_field_name_xyz(filter, field_name);
    }

    fn set_filter_limits_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        min: f32,
        max: f32,
    ) {
        ffi::set_filter_limits_xyz(filter, min, max);
    }

    fn filter_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
    ) -> UniquePtr<Self::CloudType> {
        ffi::filter_pass_xyz(filter)
    }
}

impl PassThroughPoint for PointXYZRGB {
    type PassThroughType = ffi::PassThrough_PointXYZRGB;

    fn new_pass_through() -> UniquePtr<Self::PassThroughType> {
        ffi::new_pass_through_xyzrgb()
    }

    fn set_input_cloud_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        cloud: &Self::CloudType,
    ) {
        ffi::set_input_cloud_pass_xyzrgb(filter, cloud);
    }

    fn set_filter_field_name_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        field_name: &str,
    ) {
        ffi::set_filter_field_name_xyzrgb(filter, field_name);
    }

    fn set_filter_limits_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
        min: f32,
        max: f32,
    ) {
        ffi::set_filter_limits_xyzrgb(filter, min, max);
    }

    fn filter_pass_through(
        filter: std::pin::Pin<&mut Self::PassThroughType>,
    ) -> UniquePtr<Self::CloudType> {
        ffi::filter_pass_xyzrgb(filter)
    }
}

// Type aliases for backward compatibility
pub type VoxelGridXYZ = VoxelGrid<PointXYZ>;
pub type VoxelGridXYZRGB = VoxelGrid<PointXYZRGB>;
pub type PassThroughXYZ = PassThrough<PointXYZ>;
pub type PassThroughXYZRGB = PassThrough<PointXYZRGB>;

/// Builder for generic VoxelGrid filter
pub struct VoxelGridBuilder<T: VoxelGridPoint> {
    leaf_size: Option<(f32, f32, f32)>,
    _phantom: PhantomData<T>,
}

impl<T: VoxelGridPoint> VoxelGridBuilder<T> {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            leaf_size: None,
            _phantom: PhantomData,
        }
    }

    /// Set the voxel size (leaf size) for the grid
    pub fn leaf_size(mut self, lx: f32, ly: f32, lz: f32) -> Self {
        self.leaf_size = Some((lx, ly, lz));
        self
    }

    /// Build the VoxelGrid filter
    pub fn build(self) -> PclResult<VoxelGrid<T>>
    where
        T::VoxelGridType: UniquePtrTarget,
        T::CloudType: UniquePtrTarget,
    {
        let mut filter = VoxelGrid::new()?;

        if let Some((lx, ly, lz)) = self.leaf_size {
            filter.set_leaf_size(lx, ly, lz)?;
        }

        Ok(filter)
    }
}

impl<T: VoxelGridPoint> Default for VoxelGridBuilder<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voxel_grid_creation() {
        let filter = VoxelGrid::<PointXYZ>::new();
        assert!(filter.is_ok());

        let filter = VoxelGrid::<PointXYZRGB>::new();
        assert!(filter.is_ok());
    }

    #[test]
    fn test_pass_through_creation() {
        let filter = PassThrough::<PointXYZ>::new();
        assert!(filter.is_ok());

        let filter = PassThrough::<PointXYZRGB>::new();
        assert!(filter.is_ok());
    }

    #[test]
    fn test_type_aliases() {
        let _filter: VoxelGridXYZ = VoxelGrid::new().unwrap();
        let _filter: VoxelGridXYZRGB = VoxelGrid::new().unwrap();
        let _filter: PassThroughXYZ = PassThrough::new().unwrap();
        let _filter: PassThroughXYZRGB = PassThrough::new().unwrap();
    }

    #[test]
    fn test_builder() {
        let filter = VoxelGridBuilder::<PointXYZ>::new()
            .leaf_size(0.01, 0.01, 0.01)
            .build();
        assert!(filter.is_ok());
    }
}
