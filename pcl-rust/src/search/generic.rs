//! Generic KdTree implementation that works with any point type
//!
//! This module provides a generic KdTree<T> that can work with any point type
//! implementing the required traits, providing a unified API for spatial search.

use crate::common::PointCloud;
use crate::error::{PclError, PclResult};
use crate::traits::{Point, Xyz};
use cxx::{UniquePtr, memory::UniquePtrTarget};
use std::marker::PhantomData;

/// Associated type trait for KdTree FFI types
pub trait KdTreePoint: Point + Xyz {
    /// The FFI KdTree type for this point type
    type KdTreeType: Send + Sync;

    /// Create a new KdTree instance
    fn new_kdtree() -> UniquePtr<Self::KdTreeType>
    where
        Self::KdTreeType: UniquePtrTarget;

    /// Set the input cloud for the KdTree
    fn set_input_cloud_kdtree(tree: std::pin::Pin<&mut Self::KdTreeType>, cloud: &Self::CloudType);

    /// Perform k-nearest neighbor search
    fn nearest_k_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        k: i32,
    ) -> Vec<i32>;

    /// Perform radius search
    fn radius_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        radius: f64,
    ) -> Vec<i32>;
}

/// Generic KdTree implementation for spatial search
///
/// This struct provides a unified interface for KdTree operations
/// that works with any point type implementing the KdTreePoint trait.
///
/// # Type Parameters
///
/// * `T` - The point type, which must implement `KdTreePoint`
///
/// # Examples
///
/// ```rust
/// use pcl::{PointCloud, PointXYZ, KdTree};
///
/// let mut cloud = PointCloud::<PointXYZ>::new()?;
/// cloud.push(1.0, 2.0, 3.0)?;
/// cloud.push(4.0, 5.0, 6.0)?;
///
/// let mut kdtree = KdTree::<PointXYZ>::new()?;
/// kdtree.set_input_cloud(&cloud)?;
///
/// // Find 5 nearest neighbors
/// let query = cloud.at(0)?;
/// let indices = kdtree.nearest_k_search(&query, 5)?;
/// ```
pub struct KdTree<T: KdTreePoint>
where
    T::KdTreeType: UniquePtrTarget,
{
    inner: UniquePtr<T::KdTreeType>,
    has_cloud: bool,
    _phantom: PhantomData<T>,
}

impl<T: KdTreePoint> KdTree<T>
where
    T::KdTreeType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = T::new_kdtree();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: format!("KdTree<{}>", T::type_name()),
            });
        }

        Ok(Self {
            inner,
            has_cloud: false,
            _phantom: PhantomData,
        })
    }

    /// Set the input point cloud for the search
    pub fn set_input_cloud(&mut self, cloud: &PointCloud<T>) -> PclResult<()> {
        T::set_input_cloud_kdtree(self.inner.pin_mut(), cloud.inner());
        self.has_cloud = true;
        Ok(())
    }

    /// Find the k nearest neighbors to a query point
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `k` - Number of nearest neighbors to find
    ///
    /// # Returns
    /// A vector of indices into the input cloud
    pub fn nearest_k_search(&self, point: &T, k: i32) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "k must be positive",
                "k",
                "positive integer",
                format!("{}", k),
            ));
        }

        // Note: This requires access to the FFI point representation
        // In practice, we need a way to get the FFI point from the Rust point
        // For now, this is a limitation of the design
        todo!("Need access to FFI point representation")
    }

    /// Find all neighbors within a radius of a query point
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `radius` - Search radius
    ///
    /// # Returns
    /// A vector of indices into the input cloud
    pub fn radius_search(&self, point: &T, radius: f64) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "radius must be positive",
                "radius",
                "positive value",
                format!("{}", radius),
            ));
        }

        todo!("Need access to FFI point representation")
    }

    /// Find k nearest neighbors using point coordinates
    ///
    /// This method allows searching without having a point instance
    pub fn nearest_k_search_coords(&self, x: f32, y: f32, z: f32, k: i32) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if k <= 0 {
            return Err(PclError::invalid_parameters(
                "k must be positive",
                "k",
                "positive integer",
                format!("{}", k),
            ));
        }

        // This would require a coordinate-based search function in FFI
        todo!("Implement coordinate-based search in FFI layer")
    }

    /// Find all neighbors within a radius using point coordinates
    pub fn radius_search_coords(&self, x: f32, y: f32, z: f32, radius: f64) -> PclResult<Vec<i32>> {
        if !self.has_cloud {
            return Err(PclError::invalid_state(
                "No input cloud set",
                "input cloud set",
                "no input cloud",
            ));
        }
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "radius must be positive",
                "radius",
                "positive value",
                format!("{}", radius),
            ));
        }

        todo!("Implement coordinate-based search in FFI layer")
    }
}

impl<T: KdTreePoint> Default for KdTree<T>
where
    T::KdTreeType: UniquePtrTarget,
    T::CloudType: UniquePtrTarget,
{
    fn default() -> Self {
        Self::new().expect("Failed to create default KdTree")
    }
}

// Implement KdTreePoint for concrete point types
use crate::common::{PointXYZ, PointXYZI, PointXYZRGB};
use pcl_sys::ffi;

impl KdTreePoint for PointXYZ {
    type KdTreeType = ffi::KdTree_PointXYZ;

    fn new_kdtree() -> UniquePtr<Self::KdTreeType> {
        ffi::new_kdtree_xyz()
    }

    fn set_input_cloud_kdtree(tree: std::pin::Pin<&mut Self::KdTreeType>, cloud: &Self::CloudType) {
        ffi::set_input_cloud_xyz(tree, cloud);
    }

    fn nearest_k_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        k: i32,
    ) -> Vec<i32> {
        ffi::nearest_k_search_xyz(tree, point, k)
    }

    fn radius_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        radius: f64,
    ) -> Vec<i32> {
        ffi::radius_search_xyz(tree, point, radius)
    }
}

impl KdTreePoint for PointXYZRGB {
    type KdTreeType = ffi::KdTree_PointXYZRGB;

    fn new_kdtree() -> UniquePtr<Self::KdTreeType> {
        ffi::new_kdtree_xyzrgb()
    }

    fn set_input_cloud_kdtree(tree: std::pin::Pin<&mut Self::KdTreeType>, cloud: &Self::CloudType) {
        ffi::set_input_cloud_xyzrgb(tree, cloud);
    }

    fn nearest_k_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        k: i32,
    ) -> Vec<i32> {
        ffi::nearest_k_search_xyzrgb(tree, point, k)
    }

    fn radius_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        radius: f64,
    ) -> Vec<i32> {
        ffi::radius_search_xyzrgb(tree, point, radius)
    }
}

impl KdTreePoint for PointXYZI {
    type KdTreeType = ffi::KdTree_PointXYZI;

    fn new_kdtree() -> UniquePtr<Self::KdTreeType> {
        ffi::new_kdtree_xyzi()
    }

    fn set_input_cloud_kdtree(tree: std::pin::Pin<&mut Self::KdTreeType>, cloud: &Self::CloudType) {
        ffi::set_input_cloud_xyzi(tree, cloud);
    }

    fn nearest_k_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        k: i32,
    ) -> Vec<i32> {
        ffi::nearest_k_search_xyzi(tree, point, k)
    }

    fn radius_search_kdtree(
        tree: &Self::KdTreeType,
        point: &Self::FfiPointType,
        radius: f64,
    ) -> Vec<i32> {
        ffi::radius_search_xyzi(tree, point, radius)
    }
}

// Type aliases for backward compatibility
pub type KdTreeXYZ = KdTree<PointXYZ>;
pub type KdTreeXYZRGB = KdTree<PointXYZRGB>;
pub type KdTreeXYZI = KdTree<PointXYZI>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kdtree_creation() {
        let kdtree = KdTree::<PointXYZ>::new();
        assert!(kdtree.is_ok());

        let kdtree = KdTree::<PointXYZRGB>::new();
        assert!(kdtree.is_ok());

        let kdtree = KdTree::<PointXYZI>::new();
        assert!(kdtree.is_ok());
    }

    #[test]
    fn test_type_aliases() {
        let _kdtree: KdTreeXYZ = KdTree::new().unwrap();
        let _kdtree: KdTreeXYZRGB = KdTree::new().unwrap();
        let _kdtree: KdTreeXYZI = KdTree::new().unwrap();
    }
}
