//! Generic PointCloud container for the trait-based type system
//!
//! This module provides `PointCloud<T>` - a generic point cloud container that works
//! with any point type implementing the `Point` trait. This enables type-safe,
//! generic algorithms while maintaining performance through zero-cost abstractions.

use crate::error::{PclError, PclResult};
use crate::traits::Point;
use std::marker::PhantomData;

/// Generic point cloud container that works with any point type
///
/// `PointCloud<T>` provides a unified interface for point cloud operations
/// regardless of the underlying point type. The implementation uses trait objects
/// for type erasure while maintaining type safety at the API level.
///
/// ## Type Safety
///
/// The generic parameter `T` ensures that:
/// - All points in the cloud have the same type
/// - Operations are type-checked at compile time
/// - No mixing of incompatible point types
///
/// ## Performance
///
/// Despite using trait objects internally, performance remains high because:
/// - Trait methods can be inlined by the compiler
/// - FFI operations work directly with native PCL types
/// - No unnecessary copying or conversion
///
/// ## Example Usage
///
/// ```rust
/// use pcl::traits::{Point, PointXyz, PointCloud};
/// use pcl::error::PclResult;
///
/// // Generic function that works with any 3D point type
/// fn process_cloud<T: PointXyz>(cloud: &mut PointCloud<T>) -> PclResult<()> {
///     // Resize cloud
///     cloud.resize(100)?;
///     
///     // Access points generically
///     for i in 0..cloud.size() {
///         let point = cloud.at(i)?;
///         let (x, y, z) = point.xyz();
///         println!("Point {}: ({}, {}, {})", i, x, y, z);
///     }
///     
///     Ok(())
/// }
/// ```
pub struct PointCloud<T: Point> {
    inner: Box<dyn PointCloudImpl<T>>,
    _phantom: PhantomData<T>,
}

/// Internal implementation trait for type erasure
///
/// This trait allows `PointCloud<T>` to work with different underlying
/// PCL point cloud types while maintaining a uniform interface.
pub trait PointCloudImpl<T: Point> {
    /// Get the number of points in the cloud
    fn size(&self) -> usize;

    /// Check if the cloud is empty
    fn empty(&self) -> bool;

    /// Clear all points from the cloud
    fn clear(&mut self) -> PclResult<()>;

    /// Reserve space for at least `n` points
    fn reserve(&mut self, n: usize) -> PclResult<()>;

    /// Resize the cloud to contain exactly `n` points
    fn resize(&mut self, n: usize) -> PclResult<()>;

    /// Get the width of the cloud (for organized clouds)
    fn width(&self) -> u32;

    /// Get the height of the cloud (for organized clouds)
    fn height(&self) -> u32;

    /// Set the width of the cloud
    fn set_width(&mut self, width: u32);

    /// Set the height of the cloud
    fn set_height(&mut self, height: u32);

    /// Check if all points are valid (no NaN/Inf values)
    fn is_dense(&self) -> bool;

    /// Set the dense flag
    fn set_is_dense(&mut self, is_dense: bool);

    /// Get a point at the specified index
    fn at(&self, index: usize) -> PclResult<T>;

    /// Set a point at the specified index
    fn set_at(&mut self, index: usize, point: T) -> PclResult<()>;

    /// Add a point to the end of the cloud
    fn push(&mut self, point: T) -> PclResult<()>;

    /// Clone the implementation (for Clone trait)
    fn clone_impl(&self) -> Box<dyn PointCloudImpl<T>>;

    /// Get a reference to the underlying FFI cloud (for interop)
    fn as_ffi_ptr(&self) -> *const std::ffi::c_void;

    /// Get a mutable reference to the underlying FFI cloud (for interop)
    fn as_ffi_ptr_mut(&mut self) -> *mut std::ffi::c_void;
}

impl<T: Point> PointCloud<T> {
    /// Create a new empty point cloud
    ///
    /// This is a placeholder implementation. In the full system, this would
    /// call a trait method on `T` to create the appropriate FFI cloud type.
    pub fn new() -> PclResult<Self> {
        // TODO: This should call T::create_cloud() when that's implemented
        Err(PclError::NotImplemented {
            feature: "Generic PointCloud creation".to_string(),
            workaround: Some("Use concrete types like PointCloudXYZ for now".to_string()),
        })
    }

    /// Create a PointCloud from an implementation (internal use)
    pub(crate) fn from_impl(inner: Box<dyn PointCloudImpl<T>>) -> Self {
        Self {
            inner,
            _phantom: PhantomData,
        }
    }

    /// Create a point cloud with specified initial capacity
    pub fn with_capacity(capacity: usize) -> PclResult<Self> {
        let mut cloud = Self::new()?;
        cloud.reserve(capacity)?;
        Ok(cloud)
    }

    /// Get the number of points in the cloud
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Check if the cloud is empty
    pub fn empty(&self) -> bool {
        self.inner.empty()
    }

    /// Check if the cloud is empty (alternative name for consistency)
    pub fn is_empty(&self) -> bool {
        self.empty()
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) -> PclResult<()> {
        self.inner.clear()
    }

    /// Reserve space for at least `n` points
    ///
    /// This can improve performance when you know how many points
    /// you'll be adding, as it reduces memory reallocations.
    pub fn reserve(&mut self, n: usize) -> PclResult<()> {
        self.inner.reserve(n)
    }

    /// Resize the cloud to contain exactly `n` points
    ///
    /// If `n` is larger than the current size, new points are added
    /// with default values. If smaller, the cloud is truncated.
    pub fn resize(&mut self, n: usize) -> PclResult<()> {
        self.inner.resize(n)
    }

    /// Get the width of the cloud
    ///
    /// For unorganized clouds, this is typically equal to the number of points.
    /// For organized clouds (like those from depth cameras), this represents
    /// the image width.
    pub fn width(&self) -> u32 {
        self.inner.width()
    }

    /// Get the height of the cloud
    ///
    /// For unorganized clouds, this is typically 1.
    /// For organized clouds, this represents the image height.
    pub fn height(&self) -> u32 {
        self.inner.height()
    }

    /// Set the cloud dimensions (width and height)
    ///
    /// This is primarily used for organized point clouds where the
    /// spatial relationship between points follows a 2D grid structure.
    pub fn set_dimensions(&mut self, width: u32, height: u32) {
        self.inner.set_width(width);
        self.inner.set_height(height);
    }

    /// Check if this is an organized point cloud
    ///
    /// Organized clouds have height > 1 and maintain spatial relationships
    /// between neighboring points, like pixels in an image.
    pub fn is_organized(&self) -> bool {
        self.height() > 1
    }

    /// Check if all points in the cloud are valid
    ///
    /// A dense cloud contains no NaN or infinite values in any coordinate.
    /// This flag is often used for optimization in algorithms.
    pub fn is_dense(&self) -> bool {
        self.inner.is_dense()
    }

    /// Set the dense flag
    ///
    /// This should be set to `true` only if you're certain that all points
    /// in the cloud have valid (finite) coordinates.
    pub fn set_is_dense(&mut self, is_dense: bool) {
        self.inner.set_is_dense(is_dense);
    }

    /// Get a point at the specified index
    ///
    /// Returns an error if the index is out of bounds.
    pub fn at(&self, index: usize) -> PclResult<T> {
        if index >= self.size() {
            return Err(PclError::invalid_parameters(
                "Point index out of bounds",
                "index",
                &format!("0..{}", self.size()),
                &index.to_string(),
            ));
        }
        self.inner.at(index)
    }

    /// Set a point at the specified index
    ///
    /// Returns an error if the index is out of bounds.
    pub fn set_at(&mut self, index: usize, point: T) -> PclResult<()> {
        if index >= self.size() {
            return Err(PclError::invalid_parameters(
                "Point index out of bounds",
                "index",
                &format!("0..{}", self.size()),
                &index.to_string(),
            ));
        }
        self.inner.set_at(index, point)
    }

    /// Add a point to the end of the cloud
    ///
    /// This increases the cloud size by 1.
    pub fn push(&mut self, point: T) -> PclResult<()> {
        self.inner.push(point)
    }

    /// Create an iterator over the points in the cloud
    ///
    /// Note: This creates a new point object for each iteration,
    /// which may be less efficient than indexed access for simple operations.
    pub fn iter(&self) -> PointCloudIter<T> {
        PointCloudIter::new(self)
    }

    /// Create a mutable iterator over point indices
    ///
    /// This allows you to modify points in place while iterating.
    pub fn iter_mut(&mut self) -> PointCloudIterMut<T> {
        PointCloudIterMut::new(self)
    }

    /// Get the capacity of the underlying storage
    ///
    /// This is the number of points that can be stored without reallocation.
    /// Note: Not all PCL cloud types support querying capacity.
    pub fn capacity(&self) -> usize {
        // TODO: This could be implemented if PCL exposes capacity information
        self.size()
    }

    /// Shrink the cloud's capacity to fit the current size
    ///
    /// This can free unused memory but may require reallocation on future growth.
    pub fn shrink_to_fit(&mut self) -> PclResult<()> {
        // TODO: This could be implemented if PCL supports shrinking
        Ok(())
    }

    /// Get the first point in the cloud
    pub fn first(&self) -> PclResult<T> {
        if self.empty() {
            Err(PclError::InvalidState {
                message: "Cannot get first point of empty cloud".to_string(),
                expected_state: "non-empty cloud".to_string(),
                actual_state: "empty cloud".to_string(),
            })
        } else {
            self.at(0)
        }
    }

    /// Get the last point in the cloud
    pub fn last(&self) -> PclResult<T> {
        if self.empty() {
            Err(PclError::InvalidState {
                message: "Cannot get last point of empty cloud".to_string(),
                expected_state: "non-empty cloud".to_string(),
                actual_state: "empty cloud".to_string(),
            })
        } else {
            self.at(self.size() - 1)
        }
    }

    /// Remove and return the last point in the cloud
    pub fn pop(&mut self) -> PclResult<T> {
        if self.empty() {
            return Err(PclError::InvalidState {
                message: "Cannot pop from empty cloud".to_string(),
                expected_state: "non-empty cloud".to_string(),
                actual_state: "empty cloud".to_string(),
            });
        }

        let last_point = self.last()?;
        self.resize(self.size() - 1)?;
        Ok(last_point)
    }

    /// Extend the cloud with points from an iterator
    pub fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) -> PclResult<()> {
        for point in iter {
            self.push(point)?;
        }
        Ok(())
    }

    /// Append another cloud to this one
    pub fn append(&mut self, other: &PointCloud<T>) -> PclResult<()> {
        let original_size = self.size();
        self.resize(original_size + other.size())?;

        for i in 0..other.size() {
            let point = other.at(i)?;
            self.set_at(original_size + i, point)?;
        }

        Ok(())
    }

    /// Get a reference to the underlying FFI pointer for interop
    ///
    /// This is primarily used by algorithms that need to pass the cloud
    /// to PCL C++ functions directly.
    pub fn as_ffi_ptr(&self) -> *const std::ffi::c_void {
        self.inner.as_ffi_ptr()
    }

    /// Get a mutable reference to the underlying FFI pointer for interop
    pub fn as_ffi_ptr_mut(&mut self) -> *mut std::ffi::c_void {
        self.inner.as_ffi_ptr_mut()
    }
}

impl<T: Point> Clone for PointCloud<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone_impl(),
            _phantom: PhantomData,
        }
    }
}

impl<T: Point> Default for PointCloud<T> {
    fn default() -> Self {
        Self::new().expect("Failed to create default PointCloud")
    }
}

/// Iterator over points in a point cloud
pub struct PointCloudIter<'a, T: Point> {
    cloud: &'a PointCloud<T>,
    index: usize,
}

impl<'a, T: Point> PointCloudIter<'a, T> {
    fn new(cloud: &'a PointCloud<T>) -> Self {
        Self { cloud, index: 0 }
    }
}

impl<'a, T: Point> Iterator for PointCloudIter<'a, T> {
    type Item = PclResult<T>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.cloud.size() {
            None
        } else {
            let result = self.cloud.at(self.index);
            self.index += 1;
            Some(result)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.cloud.size().saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a, T: Point> ExactSizeIterator for PointCloudIter<'a, T> {
    fn len(&self) -> usize {
        self.cloud.size().saturating_sub(self.index)
    }
}

/// Mutable iterator over point indices in a point cloud
///
/// This iterator yields indices rather than points, allowing you to
/// modify points in place using the cloud's `set_at` method.
pub struct PointCloudIterMut<'a, T: Point> {
    _phantom: PhantomData<&'a mut PointCloud<T>>,
    index: usize,
    size: usize, // Cached to avoid borrowing issues
}

impl<'a, T: Point> PointCloudIterMut<'a, T> {
    fn new(cloud: &'a mut PointCloud<T>) -> Self {
        let size = cloud.size();
        Self {
            _phantom: PhantomData,
            index: 0,
            size,
        }
    }
}

impl<'a, T: Point> Iterator for PointCloudIterMut<'a, T> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.size {
            None
        } else {
            let current = self.index;
            self.index += 1;
            Some(current)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.size.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a, T: Point> ExactSizeIterator for PointCloudIterMut<'a, T> {
    fn len(&self) -> usize {
        self.size.saturating_sub(self.index)
    }
}

/// Utilities for working with generic point clouds
pub mod utils {
    use super::*;
    use crate::traits::{Rgb, Xyz};

    /// Merge multiple point clouds into one
    pub fn merge_clouds<T: Point>(clouds: &[&PointCloud<T>]) -> PclResult<PointCloud<T>> {
        let total_size: usize = clouds.iter().map(|c| c.size()).sum();
        let mut result = PointCloud::new()?;
        result.reserve(total_size)?;

        for cloud in clouds {
            result.append(cloud)?;
        }

        Ok(result)
    }

    /// Filter points based on a predicate function
    pub fn filter_points<T: Point, F>(
        cloud: &PointCloud<T>,
        predicate: F,
    ) -> PclResult<PointCloud<T>>
    where
        F: Fn(&T) -> bool,
    {
        let mut result = PointCloud::new()?;

        for i in 0..cloud.size() {
            let point = cloud.at(i)?;
            if predicate(&point) {
                result.push(point)?;
            }
        }

        Ok(result)
    }

    /// Transform all points in a cloud using a function
    pub fn transform_points<T: Point, F>(cloud: &mut PointCloud<T>, transform: F) -> PclResult<()>
    where
        F: Fn(T) -> T,
    {
        for i in 0..cloud.size() {
            let point = cloud.at(i)?;
            let transformed = transform(point);
            cloud.set_at(i, transformed)?;
        }

        Ok(())
    }

    /// Extract coordinates from all points into separate vectors
    pub fn extract_coordinates<T: Xyz>(
        cloud: &PointCloud<T>,
    ) -> PclResult<(Vec<f32>, Vec<f32>, Vec<f32>)> {
        let size = cloud.size();
        let mut x_coords = Vec::with_capacity(size);
        let mut y_coords = Vec::with_capacity(size);
        let mut z_coords = Vec::with_capacity(size);

        for i in 0..size {
            let point = cloud.at(i)?;
            let (x, y, z) = point.xyz();
            x_coords.push(x);
            y_coords.push(y);
            z_coords.push(z);
        }

        Ok((x_coords, y_coords, z_coords))
    }

    /// Extract colors from all points into separate vectors
    pub fn extract_colors<T: Rgb>(cloud: &PointCloud<T>) -> PclResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
        let size = cloud.size();
        let mut r_values = Vec::with_capacity(size);
        let mut g_values = Vec::with_capacity(size);
        let mut b_values = Vec::with_capacity(size);

        for i in 0..size {
            let point = cloud.at(i)?;
            let (r, g, b) = point.rgb();
            r_values.push(r);
            g_values.push(g);
            b_values.push(b);
        }

        Ok((r_values, g_values, b_values))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock implementation for testing
    struct MockPointCloudImpl<T: Point> {
        points: Vec<T>,
        width: u32,
        height: u32,
        is_dense: bool,
    }

    impl<T: Point> MockPointCloudImpl<T> {
        fn new() -> Self {
            Self {
                points: Vec::new(),
                width: 0,
                height: 1,
                is_dense: true,
            }
        }
    }

    impl<T: Point> PointCloudImpl<T> for MockPointCloudImpl<T> {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn empty(&self) -> bool {
            self.points.is_empty()
        }

        fn clear(&mut self) -> PclResult<()> {
            self.points.clear();
            self.width = 0;
            Ok(())
        }

        fn reserve(&mut self, n: usize) -> PclResult<()> {
            self.points.reserve(n);
            Ok(())
        }

        fn resize(&mut self, n: usize) -> PclResult<()> {
            self.points.resize_with(n, || T::default_point());
            self.width = n as u32;
            Ok(())
        }

        fn width(&self) -> u32 {
            self.width
        }

        fn height(&self) -> u32 {
            self.height
        }

        fn set_width(&mut self, width: u32) {
            self.width = width;
        }

        fn set_height(&mut self, height: u32) {
            self.height = height;
        }

        fn is_dense(&self) -> bool {
            self.is_dense
        }

        fn set_is_dense(&mut self, is_dense: bool) {
            self.is_dense = is_dense;
        }

        fn at(&self, index: usize) -> PclResult<T> {
            self.points.get(index).cloned().ok_or_else(|| {
                PclError::invalid_parameters(
                    "Index out of bounds",
                    "index",
                    &format!("0..{}", self.points.len()),
                    &index.to_string(),
                )
            })
        }

        fn set_at(&mut self, index: usize, point: T) -> PclResult<()> {
            if index < self.points.len() {
                self.points[index] = point;
                Ok(())
            } else {
                Err(PclError::invalid_parameters(
                    "Index out of bounds",
                    "index",
                    &format!("0..{}", self.points.len()),
                    &index.to_string(),
                ))
            }
        }

        fn push(&mut self, point: T) -> PclResult<()> {
            self.points.push(point);
            self.width = self.points.len() as u32;
            Ok(())
        }

        fn clone_impl(&self) -> Box<dyn PointCloudImpl<T>> {
            Box::new(MockPointCloudImpl {
                points: self.points.clone(),
                width: self.width,
                height: self.height,
                is_dense: self.is_dense,
            })
        }

        fn as_ffi_ptr(&self) -> *const std::ffi::c_void {
            std::ptr::null()
        }

        fn as_ffi_ptr_mut(&mut self) -> *mut std::ffi::c_void {
            std::ptr::null_mut()
        }
    }

    // Mock point type for testing (same as in mod.rs)
    #[derive(Clone, Debug, PartialEq)]
    struct MockPoint {
        x: f32,
        y: f32,
        z: f32,
    }

    impl Point for MockPoint {
        fn type_name() -> &'static str {
            "MockPoint"
        }

        fn default_point() -> Self {
            Self {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        }

        fn create_cloud() -> PclResult<PointCloud<Self>> {
            // Mock implementation that returns an error
            Err(PclError::NotImplemented {
                feature: "MockPoint cloud creation".to_string(),
                workaround: None,
            })
        }
    }

    impl crate::traits::Xyz for MockPoint {
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

    // Helper to create a mock cloud for testing
    fn create_mock_cloud() -> PointCloud<MockPoint> {
        PointCloud {
            inner: Box::new(MockPointCloudImpl::new()),
            _phantom: PhantomData,
        }
    }

    #[test]
    fn test_point_cloud_basic_operations() {
        let mut cloud = create_mock_cloud();

        // Test empty cloud
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);

        // Test adding points
        let point1 = MockPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        cloud.push(point1.clone()).unwrap();

        assert!(!cloud.empty());
        assert_eq!(cloud.size(), 1);

        // Test accessing points
        let retrieved = cloud.at(0).unwrap();
        assert_eq!(retrieved, point1);

        // Test setting points
        let point2 = MockPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        cloud.set_at(0, point2.clone()).unwrap();

        let retrieved = cloud.at(0).unwrap();
        assert_eq!(retrieved, point2);
    }

    #[test]
    fn test_point_cloud_resize() {
        let mut cloud = create_mock_cloud();

        cloud.resize(5).unwrap();
        assert_eq!(cloud.size(), 5);
        assert_eq!(cloud.width(), 5);

        // All points should be default
        for i in 0..5 {
            let point = cloud.at(i).unwrap();
            assert_eq!(point, MockPoint::default_point());
        }
    }

    #[test]
    fn test_point_cloud_clear() {
        let mut cloud = create_mock_cloud();

        // Add some points
        for i in 0..3 {
            cloud
                .push(MockPoint {
                    x: i as f32,
                    y: 0.0,
                    z: 0.0,
                })
                .unwrap();
        }

        assert_eq!(cloud.size(), 3);

        cloud.clear().unwrap();
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
    }

    #[test]
    fn test_point_cloud_dimensions() {
        let mut cloud = create_mock_cloud();

        assert_eq!(cloud.height(), 1);
        assert!(!cloud.is_organized());

        cloud.set_dimensions(10, 5);
        assert_eq!(cloud.width(), 10);
        assert_eq!(cloud.height(), 5);
        assert!(cloud.is_organized());
    }

    #[test]
    fn test_point_cloud_dense_flag() {
        let mut cloud = create_mock_cloud();

        assert!(cloud.is_dense());

        cloud.set_is_dense(false);
        assert!(!cloud.is_dense());
    }

    #[test]
    fn test_point_cloud_bounds_checking() {
        let cloud = create_mock_cloud();

        // Test out of bounds access
        let result = cloud.at(0);
        assert!(result.is_err());

        let mut cloud = create_mock_cloud();
        cloud.resize(1).unwrap();

        // Test valid access
        let result = cloud.at(0);
        assert!(result.is_ok());

        // Test out of bounds access
        let result = cloud.at(1);
        assert!(result.is_err());
    }

    #[test]
    fn test_point_cloud_first_last() {
        let mut cloud = create_mock_cloud();

        // Test empty cloud
        assert!(cloud.first().is_err());
        assert!(cloud.last().is_err());

        // Add points
        let p1 = MockPoint {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = MockPoint {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        let p3 = MockPoint {
            x: 3.0,
            y: 0.0,
            z: 0.0,
        };

        cloud.push(p1.clone()).unwrap();
        cloud.push(p2).unwrap();
        cloud.push(p3.clone()).unwrap();

        assert_eq!(cloud.first().unwrap(), p1);
        assert_eq!(cloud.last().unwrap(), p3);
    }

    #[test]
    fn test_point_cloud_pop() {
        let mut cloud = create_mock_cloud();

        // Test empty cloud
        assert!(cloud.pop().is_err());

        // Add points
        let p1 = MockPoint {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = MockPoint {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };

        cloud.push(p1).unwrap();
        cloud.push(p2.clone()).unwrap();

        assert_eq!(cloud.size(), 2);

        let popped = cloud.pop().unwrap();
        assert_eq!(popped, p2);
        assert_eq!(cloud.size(), 1);
    }

    #[test]
    fn test_point_cloud_iterator() {
        let mut cloud = create_mock_cloud();

        // Add test points
        let points = vec![
            MockPoint {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            MockPoint {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            MockPoint {
                x: 3.0,
                y: 0.0,
                z: 0.0,
            },
        ];

        for point in &points {
            cloud.push(point.clone()).unwrap();
        }

        // Test iterator
        let mut iter_points = Vec::new();
        for point_result in cloud.iter() {
            iter_points.push(point_result.unwrap());
        }

        assert_eq!(iter_points, points);
    }
}
