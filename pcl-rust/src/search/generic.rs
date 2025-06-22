//! Generic KdTree implementation that works with any point type
//!
//! This module provides a generic KdTree<T> that can work with any point type
//! implementing the required traits, providing a unified API for spatial search.

use crate::common::PointCloud;
use crate::error::{PclError, PclResult};
// Removed trait imports - using new marker type system
use cxx::{UniquePtr, memory::UniquePtrTarget};
use std::marker::PhantomData;

/// Associated type trait for KdTree FFI types
pub trait KdTreePoint {
    /// The FFI KdTree type for this point type
    type KdTreeType: Send + Sync;

    /// The FFI cloud type
    type CloudType: UniquePtrTarget;

    /// The FFI point type
    type FfiPointType: UniquePtrTarget;

    /// Get the type name for debugging
    fn type_name() -> &'static str;

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
/// use pcl::{PointCloud, PointXYZ, XYZ, KdTree};
///
/// let mut cloud = PointCloud::<XYZ>::new()?;
/// cloud.push(PointXYZ::new(1.0, 2.0, 3.0))?;
/// cloud.push(PointXYZ::new(4.0, 5.0, 6.0))?;
///
/// let mut kdtree = KdTree::<PointXYZ>::new()?;
/// kdtree.set_input_cloud(&cloud)?;
///
/// // Find 5 nearest neighbors - NOTE: need FFI point access
/// // let query = cloud.at(0)?;
/// // let indices = kdtree.nearest_k_search(&query, 5)?;
/// ```
pub struct KdTree<T: KdTreePoint>
where
    T::KdTreeType: UniquePtrTarget,
{
    inner: UniquePtr<T::KdTreeType>,
    has_cloud: bool,
    _phantom: PhantomData<T>,
}

impl<T: KdTreePoint + crate::common::point_types::PointType> KdTree<T>
where
    T::KdTreeType: UniquePtrTarget,
    <T as KdTreePoint>::CloudType: UniquePtrTarget,
    <T as crate::common::point_types::PointType>::CloudType: UniquePtrTarget,
{
    /// Create a new KdTree
    pub fn new() -> PclResult<Self> {
        let inner = T::new_kdtree();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: format!(
                    "KdTree<{}>",
                    <T as crate::common::point_types::PointType>::type_name()
                ),
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
        // SAFETY: KdTreePoint::CloudType and PointType::CloudType are the same type
        let inner = unsafe {
            &*(cloud.inner() as *const <T as crate::common::point_types::PointType>::CloudType
                as *const <T as KdTreePoint>::CloudType)
        };
        T::set_input_cloud_kdtree(self.inner.pin_mut(), inner);
        self.has_cloud = true;
        Ok(())
    }

    /// Get access to the inner FFI object for use in other modules
    pub fn inner(&self) -> &T::KdTreeType {
        &self.inner
    }

    /// Find the k nearest neighbors to a query point
    ///
    /// # Arguments
    /// * `point` - The query point
    /// * `k` - Number of nearest neighbors to find
    ///
    /// # Returns
    /// A vector of indices into the input cloud
    pub fn nearest_k_search(&self, _point: &T, k: i32) -> PclResult<Vec<i32>> {
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
    pub fn radius_search(&self, _point: &T, radius: f64) -> PclResult<Vec<i32>> {
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
    pub fn nearest_k_search_coords(
        &self,
        _x: f32,
        _y: f32,
        _z: f32,
        k: i32,
    ) -> PclResult<Vec<i32>> {
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
    pub fn radius_search_coords(
        &self,
        _x: f32,
        _y: f32,
        _z: f32,
        radius: f64,
    ) -> PclResult<Vec<i32>> {
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

impl<T: KdTreePoint + crate::common::point_types::PointType> Default for KdTree<T>
where
    T::KdTreeType: UniquePtrTarget,
    <T as KdTreePoint>::CloudType: UniquePtrTarget,
    <T as crate::common::point_types::PointType>::CloudType: UniquePtrTarget,
{
    fn default() -> Self {
        Self::new().expect("Failed to create default KdTree")
    }
}

// Implement KdTreePoint for marker types and concrete point types
use crate::common::{PointXYZ, PointXYZI, PointXYZRGB, XYZ, XYZI, XYZRGB};
use pcl_sys::ffi;

impl KdTreePoint for PointXYZ {
    type KdTreeType = ffi::KdTree_PointXYZ;
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }

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
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

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
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }

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

// ============================================================================
// KdTreePoint implementations for marker types
// ============================================================================

impl KdTreePoint for XYZ {
    type KdTreeType = ffi::KdTree_PointXYZ;
    type CloudType = ffi::PointCloud_PointXYZ;
    type FfiPointType = ffi::PointXYZ;

    fn type_name() -> &'static str {
        "PointXYZ"
    }

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

impl KdTreePoint for XYZRGB {
    type KdTreeType = ffi::KdTree_PointXYZRGB;
    type CloudType = ffi::PointCloud_PointXYZRGB;
    type FfiPointType = ffi::PointXYZRGB;

    fn type_name() -> &'static str {
        "PointXYZRGB"
    }

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

impl KdTreePoint for XYZI {
    type KdTreeType = ffi::KdTree_PointXYZI;
    type CloudType = ffi::PointCloud_PointXYZI;
    type FfiPointType = ffi::PointXYZI;

    fn type_name() -> &'static str {
        "PointXYZI"
    }

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

// ============================================================================
// Trait Implementations for Generic KdTree
// ============================================================================

use super::traits::{NearestNeighborSearch, SearchConfiguration, SearchInputCloud};

// Specific implementations for concrete types to avoid generic complexity
impl NearestNeighborSearch<PointXYZ> for KdTree<PointXYZ> {
    fn nearest_k_search(&self, _point: &PointXYZ, _k: i32) -> PclResult<Vec<i32>> {
        todo!("nearest_k_search not yet implemented")
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &PointXYZ,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("nearest_k_search_with_distances not yet implemented")
    }

    fn radius_search(&self, _point: &PointXYZ, _radius: f64) -> PclResult<Vec<i32>> {
        todo!("radius_search not yet implemented")
    }

    fn radius_search_with_distances(
        &self,
        _point: &PointXYZ,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("radius_search_with_distances not yet implemented")
    }
}

impl SearchConfiguration for KdTree<PointXYZ> {
    fn epsilon(&self) -> f32 {
        todo!("epsilon getter not yet implemented")
    }

    fn set_epsilon(&mut self, _epsilon: f32) -> PclResult<()> {
        todo!("epsilon setter not yet implemented")
    }
}

impl SearchInputCloud<PointCloud<PointXYZ>> for KdTree<PointXYZ> {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZ>) -> PclResult<()> {
        // The generic set_input_cloud expects PointCloud<PointXYZ>, not PointCloud<XYZ>
        // This is a limitation of the current design where marker and owned types don't align
        todo!("Type system mismatch between marker and owned types")
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

// Similar implementations for PointXYZRGB
impl NearestNeighborSearch<PointXYZRGB> for KdTree<PointXYZRGB> {
    fn nearest_k_search(&self, _point: &PointXYZRGB, _k: i32) -> PclResult<Vec<i32>> {
        todo!("nearest_k_search not yet implemented")
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &PointXYZRGB,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("nearest_k_search_with_distances not yet implemented")
    }

    fn radius_search(&self, _point: &PointXYZRGB, _radius: f64) -> PclResult<Vec<i32>> {
        todo!("radius_search not yet implemented")
    }

    fn radius_search_with_distances(
        &self,
        _point: &PointXYZRGB,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("radius_search_with_distances not yet implemented")
    }
}

impl SearchConfiguration for KdTree<PointXYZRGB> {
    fn epsilon(&self) -> f32 {
        todo!("epsilon getter not yet implemented")
    }

    fn set_epsilon(&mut self, _epsilon: f32) -> PclResult<()> {
        todo!("epsilon setter not yet implemented")
    }
}

impl SearchInputCloud<PointCloud<PointXYZRGB>> for KdTree<PointXYZRGB> {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZRGB>) -> PclResult<()> {
        // The generic set_input_cloud expects PointCloud<PointXYZRGB>, not PointCloud<XYZRGB>
        // This is a limitation of the current design where marker and owned types don't align
        todo!("Type system mismatch between marker and owned types")
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

// ============================================================================
// Trait Implementations for Marker Types
// ============================================================================

// Implementations for XYZ marker type
impl NearestNeighborSearch<XYZ> for KdTree<XYZ> {
    fn nearest_k_search(&self, _point: &XYZ, _k: i32) -> PclResult<Vec<i32>> {
        todo!("nearest_k_search not yet implemented")
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &XYZ,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("nearest_k_search_with_distances not yet implemented")
    }

    fn radius_search(&self, _point: &XYZ, _radius: f64) -> PclResult<Vec<i32>> {
        todo!("radius_search not yet implemented")
    }

    fn radius_search_with_distances(
        &self,
        _point: &XYZ,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("radius_search_with_distances not yet implemented")
    }
}

impl SearchConfiguration for KdTree<XYZ> {
    fn epsilon(&self) -> f32 {
        todo!("epsilon getter not yet implemented")
    }

    fn set_epsilon(&mut self, _epsilon: f32) -> PclResult<()> {
        todo!("epsilon setter not yet implemented")
    }
}

impl SearchInputCloud<PointCloud<XYZ>> for KdTree<XYZ> {
    fn set_input_cloud(&mut self, cloud: &PointCloud<XYZ>) -> PclResult<()> {
        // Forward to the inherent method with manual disambiguation
        KdTree::set_input_cloud(self, cloud)
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

// Implementations for XYZRGB marker type
impl NearestNeighborSearch<XYZRGB> for KdTree<XYZRGB> {
    fn nearest_k_search(&self, _point: &XYZRGB, _k: i32) -> PclResult<Vec<i32>> {
        todo!("nearest_k_search not yet implemented")
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &XYZRGB,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("nearest_k_search_with_distances not yet implemented")
    }

    fn radius_search(&self, _point: &XYZRGB, _radius: f64) -> PclResult<Vec<i32>> {
        todo!("radius_search not yet implemented")
    }

    fn radius_search_with_distances(
        &self,
        _point: &XYZRGB,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("radius_search_with_distances not yet implemented")
    }
}

impl SearchConfiguration for KdTree<XYZRGB> {
    fn epsilon(&self) -> f32 {
        todo!("epsilon getter not yet implemented")
    }

    fn set_epsilon(&mut self, _epsilon: f32) -> PclResult<()> {
        todo!("epsilon setter not yet implemented")
    }
}

impl SearchInputCloud<PointCloud<XYZRGB>> for KdTree<XYZRGB> {
    fn set_input_cloud(&mut self, cloud: &PointCloud<XYZRGB>) -> PclResult<()> {
        // Forward to the inherent method with manual disambiguation
        KdTree::set_input_cloud(self, cloud)
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

// Implementations for XYZI marker type
impl NearestNeighborSearch<XYZI> for KdTree<XYZI> {
    fn nearest_k_search(&self, _point: &XYZI, _k: i32) -> PclResult<Vec<i32>> {
        todo!("nearest_k_search not yet implemented")
    }

    fn nearest_k_search_with_distances(
        &self,
        _point: &XYZI,
        _k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("nearest_k_search_with_distances not yet implemented")
    }

    fn radius_search(&self, _point: &XYZI, _radius: f64) -> PclResult<Vec<i32>> {
        todo!("radius_search not yet implemented")
    }

    fn radius_search_with_distances(
        &self,
        _point: &XYZI,
        _radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        todo!("radius_search_with_distances not yet implemented")
    }
}

impl SearchConfiguration for KdTree<XYZI> {
    fn epsilon(&self) -> f32 {
        todo!("epsilon getter not yet implemented")
    }

    fn set_epsilon(&mut self, _epsilon: f32) -> PclResult<()> {
        todo!("epsilon setter not yet implemented")
    }
}

impl SearchInputCloud<PointCloud<XYZI>> for KdTree<XYZI> {
    fn set_input_cloud(&mut self, cloud: &PointCloud<XYZI>) -> PclResult<()> {
        // Forward to the inherent method with manual disambiguation
        KdTree::set_input_cloud(self, cloud)
    }

    fn has_input_cloud(&self) -> bool {
        self.has_cloud
    }
}

// Deprecated type aliases have been removed - use KdTree<PointType> directly

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
    fn test_explicit_types() {
        let _kdtree: KdTree<PointXYZ> = KdTree::new().unwrap();
        let _kdtree: KdTree<PointXYZRGB> = KdTree::new().unwrap();
        let _kdtree: KdTree<PointXYZI> = KdTree::new().unwrap();
    }
}
