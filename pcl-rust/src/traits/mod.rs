//! Core traits for generic point type system
//!
//! This module provides the foundational trait system that enables generic programming
//! with PCL point types. The design allows algorithms to work with any point type that
//! implements the required capabilities (3D coordinates, colors, normals, etc.).
//!
//! ## Design Philosophy
//!
//! The trait system is designed around **capability-based programming**:
//! - Algorithms specify exactly what point capabilities they need
//! - Point types implement only the traits they actually support
//! - Compile-time safety ensures algorithm-point type compatibility
//! - Zero-cost abstractions with trait method inlining
//!
//! ## Core Traits Hierarchy
//!
//! ```text
//! Point (base trait)
//! ├── PointXyz (3D coordinates)
//! ├── PointRgb (color information)
//! ├── PointNormal (surface normals)
//! ├── PointIntensity (intensity values)
//! └── PointCurvature (curvature information)
//! ```
//!
//! ## Usage Examples
//!
//! ### Generic Algorithm with Coordinate Requirements
//! ```rust
//! use pcl::traits::{Point, PointXyz};
//! use pcl::error::PclResult;
//!
//! // Algorithm that works with any point type having 3D coordinates
//! fn compute_centroid<T: PointXyz>(points: &[T]) -> PclResult<(f32, f32, f32)> {
//!     if points.is_empty() {
//!         return Ok((0.0, 0.0, 0.0));
//!     }
//!     
//!     let mut sum_x = 0.0;
//!     let mut sum_y = 0.0;
//!     let mut sum_z = 0.0;
//!     
//!     for point in points {
//!         sum_x += point.x();
//!         sum_y += point.y();
//!         sum_z += point.z();
//!     }
//!     
//!     let count = points.len() as f32;
//!     Ok((sum_x / count, sum_y / count, sum_z / count))
//! }
//! ```
//!
//! ### Algorithm with Multiple Capability Requirements
//! ```rust
//! use pcl::traits::{PointXyz, PointRgb};
//!
//! // Algorithm that requires both 3D coordinates and color
//! fn compute_color_distance<T: PointXyz + PointRgb>(p1: &T, p2: &T) -> f32 {
//!     let spatial_dist = p1.distance_to(p2);
//!     let color_dist = {
//!         let (r1, g1, b1) = p1.rgb_as_float();
//!         let (r2, g2, b2) = p2.rgb_as_float();
//!         ((r2-r1).powi(2) + (g2-g1).powi(2) + (b2-b1).powi(2)).sqrt()
//!     };
//!     spatial_dist + color_dist
//! }
//! ```

// Note: PclError and PclResult are used in trait definitions
use cxx;
use std::fmt::Debug;
use std::pin::Pin;

/// Internal trait for FFI type conversion (not exposed to users)
///
/// This trait is used internally to convert between Rust point types
/// and their underlying C++ FFI representations. It should not be
/// implemented by users or used in public APIs.
pub(crate) trait PointFfi: Point {
    /// The underlying FFI type that represents this point in PCL C++
    type FfiType;

    /// Get a reference to the underlying FFI representation
    fn as_ffi(&self) -> &Self::FfiType;

    /// Get a mutable reference to the underlying FFI representation
    fn as_ffi_mut(&mut self) -> &mut Self::FfiType;
}

/// Core trait for all point types in the PCL ecosystem
///
/// This trait establishes the basic requirements for any type that can be used
/// as a point in PCL algorithms. It provides:
/// - Type identification for debugging and error reporting
/// - Basic lifecycle management (Clone, Debug, Send, Sync)
/// - Factory methods for creating points and point clouds
/// - Associated types for FFI cloud operations
pub trait Point: Clone + Debug + 'static {
    /// The underlying FFI cloud type for this point type
    type CloudType: Send + Sync;

    /// The underlying FFI point type
    type FfiPointType: Send + Sync;

    /// Get the point type name for debugging and error messages
    fn type_name() -> &'static str;

    /// Create a new point with default/zero values
    fn default_point() -> Self
    where
        Self: Sized;

    /// Create a new point cloud for this point type (legacy method)
    fn create_cloud()
    -> crate::error::PclResult<crate::common::point_cloud_generic::PointCloud<Self>>
    where
        Self: Sized,
        Self::CloudType: cxx::memory::UniquePtrTarget;

    // Cloud operation methods using associated types

    /// Create a new empty cloud
    fn new_cloud() -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget;

    /// Get the number of points in the cloud
    fn cloud_size(cloud: &Self::CloudType) -> usize;

    /// Check if the cloud is empty
    fn cloud_empty(cloud: &Self::CloudType) -> bool;

    /// Clear all points from the cloud
    fn cloud_clear(cloud: Pin<&mut Self::CloudType>);

    /// Reserve capacity for at least n points
    fn cloud_reserve(cloud: Pin<&mut Self::CloudType>, n: usize);

    /// Resize the cloud to contain n points
    fn cloud_resize(cloud: Pin<&mut Self::CloudType>, n: usize);

    /// Get the width of the cloud (for organized clouds)
    fn cloud_width(cloud: &Self::CloudType) -> u32;

    /// Get the height of the cloud (for organized clouds)
    fn cloud_height(cloud: &Self::CloudType) -> u32;

    /// Check if the cloud is dense (no invalid points)
    fn cloud_is_dense(cloud: &Self::CloudType) -> bool;

    /// Get a raw pointer for FFI operations
    fn as_raw_cloud(cloud: &Self::CloudType) -> *const Self::CloudType;

    /// Get a point at the given index
    fn get_point_at(cloud: &Self::CloudType, index: usize) -> cxx::UniquePtr<Self::FfiPointType>
    where
        Self::FfiPointType: cxx::memory::UniquePtrTarget;

    /// Set a point at the given index
    fn set_point_at(cloud: Pin<&mut Self::CloudType>, index: usize, point: &Self);

    /// Create a point from a UniquePtr
    fn from_unique_ptr(ptr: cxx::UniquePtr<Self::FfiPointType>) -> crate::error::PclResult<Self>
    where
        Self: Sized,
        Self::FfiPointType: cxx::memory::UniquePtrTarget;

    /// Set the cloud width
    fn cloud_set_width(cloud: Pin<&mut Self::CloudType>, width: u32);

    /// Set the cloud height  
    fn cloud_set_height(cloud: Pin<&mut Self::CloudType>, height: u32);

    /// Clone the entire cloud
    fn cloud_clone(cloud: &Self::CloudType) -> cxx::UniquePtr<Self::CloudType>
    where
        Self::CloudType: cxx::memory::UniquePtrTarget;
}

/// Trait for points with 3D Cartesian coordinates (x, y, z)
///
/// This trait provides access to spatial coordinates and derived spatial operations.
/// Any point type that has a physical location in 3D space should implement this trait.
///
/// ## Default Implementations
///
/// The trait provides default implementations for:
/// - `xyz()` - tuple access to coordinates
/// - `set_xyz()` - bulk coordinate setting
/// - `distance_to()` - Euclidean distance calculation
/// - `distance_squared_to()` - squared distance (faster, avoids sqrt)
/// - `dot_product()` - treating points as vectors from origin
/// - `magnitude()` - distance from origin
/// - `normalize()` - create unit vector from origin
pub trait Xyz: Point {
    /// Get the X coordinate
    fn x(&self) -> f32;

    /// Get the Y coordinate
    fn y(&self) -> f32;

    /// Get the Z coordinate
    fn z(&self) -> f32;

    /// Set the X coordinate
    fn set_x(&mut self, x: f32);

    /// Set the Y coordinate
    fn set_y(&mut self, y: f32);

    /// Set the Z coordinate
    fn set_z(&mut self, z: f32);

    /// Get all coordinates as a tuple
    fn xyz(&self) -> (f32, f32, f32) {
        (self.x(), self.y(), self.z())
    }

    /// Set all coordinates at once
    fn set_xyz(&mut self, x: f32, y: f32, z: f32) {
        self.set_x(x);
        self.set_y(y);
        self.set_z(z);
    }

    /// Compute Euclidean distance to another point
    fn distance_to(&self, other: &Self) -> f32 {
        self.distance_squared_to(other).sqrt()
    }

    /// Compute squared Euclidean distance (faster than distance_to)
    fn distance_squared_to(&self, other: &Self) -> f32 {
        let (x1, y1, z1) = self.xyz();
        let (x2, y2, z2) = other.xyz();
        (x2 - x1).powi(2) + (y2 - y1).powi(2) + (z2 - z1).powi(2)
    }

    /// Compute dot product treating points as vectors from origin
    fn dot_product(&self, other: &Self) -> f32 {
        let (x1, y1, z1) = self.xyz();
        let (x2, y2, z2) = other.xyz();
        x1 * x2 + y1 * y2 + z1 * z2
    }

    /// Get magnitude (distance from origin)
    fn magnitude(&self) -> f32 {
        let (x, y, z) = self.xyz();
        (x * x + y * y + z * z).sqrt()
    }

    /// Get squared magnitude (faster than magnitude)
    fn magnitude_squared(&self) -> f32 {
        let (x, y, z) = self.xyz();
        x * x + y * y + z * z
    }

    /// Create a normalized version of this point (unit vector from origin)
    fn normalized(&self) -> Self
    where
        Self: Sized,
    {
        let mag = self.magnitude();
        let mut result = self.clone();
        if mag > 0.0 {
            let (x, y, z) = self.xyz();
            result.set_xyz(x / mag, y / mag, z / mag);
        }
        result
    }

    /// Normalize this point in place
    fn normalize(&mut self) {
        let mag = self.magnitude();
        if mag > 0.0 {
            let (x, y, z) = self.xyz();
            self.set_xyz(x / mag, y / mag, z / mag);
        }
    }

    /// Check if this point is at the origin (within epsilon)
    fn is_origin(&self, epsilon: f32) -> bool {
        self.magnitude_squared() < epsilon * epsilon
    }

    /// Create a point at the origin
    fn origin() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_xyz(0.0, 0.0, 0.0);
        point
    }
}

/// Trait for points with RGB color information
///
/// This trait provides access to color channels and color-space operations.
/// Color values are stored as 8-bit integers (0-255) but can be converted
/// to floating-point representation (0.0-1.0) for calculations.
///
/// ## Color Operations
///
/// Default implementations include:
/// - Tuple access to RGB values
/// - Bulk color setting
/// - Float conversion for calculations
/// - Color distance computation
/// - Grayscale conversion
/// - Color blending operations
pub trait Rgb: Point {
    /// Get the red component (0-255)
    fn r(&self) -> u8;

    /// Get the green component (0-255)
    fn g(&self) -> u8;

    /// Get the blue component (0-255)
    fn b(&self) -> u8;

    /// Set the red component (0-255)
    fn set_r(&mut self, r: u8);

    /// Set the green component (0-255)
    fn set_g(&mut self, g: u8);

    /// Set the blue component (0-255)
    fn set_b(&mut self, b: u8);

    /// Get all color channels as a tuple
    fn rgb(&self) -> (u8, u8, u8) {
        (self.r(), self.g(), self.b())
    }

    /// Set all color channels at once
    fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        self.set_r(r);
        self.set_g(g);
        self.set_b(b);
    }

    /// Get color as floating-point values (0.0-1.0)
    fn rgb_as_float(&self) -> (f32, f32, f32) {
        let (r, g, b) = self.rgb();
        (r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
    }

    /// Set color from floating-point values (0.0-1.0)
    fn set_rgb_from_float(&mut self, r: f32, g: f32, b: f32) {
        self.set_rgb(
            (r.clamp(0.0, 1.0) * 255.0) as u8,
            (g.clamp(0.0, 1.0) * 255.0) as u8,
            (b.clamp(0.0, 1.0) * 255.0) as u8,
        );
    }

    /// Compute color distance between two points (0.0-1.0 scale)
    fn color_distance_to(&self, other: &Self) -> f32 {
        let (r1, g1, b1) = self.rgb_as_float();
        let (r2, g2, b2) = other.rgb_as_float();
        ((r2 - r1).powi(2) + (g2 - g1).powi(2) + (b2 - b1).powi(2)).sqrt()
    }

    /// Convert to grayscale using standard luminance formula
    fn to_grayscale(&self) -> u8 {
        let (r, g, b) = self.rgb();
        (0.299 * r as f32 + 0.587 * g as f32 + 0.114 * b as f32) as u8
    }

    /// Blend this color with another using linear interpolation
    ///
    /// `alpha` should be between 0.0 and 1.0:
    /// - 0.0 returns this color
    /// - 1.0 returns other color
    /// - 0.5 returns 50/50 blend
    fn blend_with(&self, other: &Self, alpha: f32) -> (u8, u8, u8) {
        let alpha = alpha.clamp(0.0, 1.0);
        let (r1, g1, b1) = self.rgb_as_float();
        let (r2, g2, b2) = other.rgb_as_float();

        let r = r1 * (1.0 - alpha) + r2 * alpha;
        let g = g1 * (1.0 - alpha) + g2 * alpha;
        let b = b1 * (1.0 - alpha) + b2 * alpha;

        ((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
    }

    /// Create a white point
    fn white() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_rgb(255, 255, 255);
        point
    }

    /// Create a black point
    fn black() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_rgb(0, 0, 0);
        point
    }

    /// Create a red point
    fn red() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_rgb(255, 0, 0);
        point
    }

    /// Create a green point
    fn green() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_rgb(0, 255, 0);
        point
    }

    /// Create a blue point
    fn blue() -> Self
    where
        Self: Sized,
    {
        let mut point = Self::default_point();
        point.set_rgb(0, 0, 255);
        point
    }
}

/// Trait for points with surface normal information
///
/// Surface normals are unit vectors that indicate the direction perpendicular
/// to a surface at a given point. They are essential for lighting calculations,
/// surface reconstruction, and many geometric algorithms.
///
/// ## Normal Vector Operations
///
/// Default implementations include:
/// - Tuple access to normal components
/// - Normal magnitude and normalization
/// - Angle calculations between normals
/// - Normal validity checking
pub trait NormalXyz: Point {
    /// Get the X component of the surface normal
    fn normal_x(&self) -> f32;

    /// Get the Y component of the surface normal
    fn normal_y(&self) -> f32;

    /// Get the Z component of the surface normal
    fn normal_z(&self) -> f32;

    /// Set the X component of the surface normal
    fn set_normal_x(&mut self, nx: f32);

    /// Set the Y component of the surface normal
    fn set_normal_y(&mut self, ny: f32);

    /// Set the Z component of the surface normal
    fn set_normal_z(&mut self, nz: f32);

    /// Get all normal components as a tuple
    fn normal(&self) -> (f32, f32, f32) {
        (self.normal_x(), self.normal_y(), self.normal_z())
    }

    /// Set all normal components at once
    fn set_normal(&mut self, nx: f32, ny: f32, nz: f32) {
        self.set_normal_x(nx);
        self.set_normal_y(ny);
        self.set_normal_z(nz);
    }

    /// Get the magnitude of the normal vector
    fn normal_magnitude(&self) -> f32 {
        let (nx, ny, nz) = self.normal();
        (nx * nx + ny * ny + nz * nz).sqrt()
    }

    /// Get the squared magnitude of the normal vector (faster)
    fn normal_magnitude_squared(&self) -> f32 {
        let (nx, ny, nz) = self.normal();
        nx * nx + ny * ny + nz * nz
    }

    /// Check if the normal is approximately unit length
    fn is_normal_unit(&self, epsilon: f32) -> bool {
        let mag_sq = self.normal_magnitude_squared();
        (mag_sq - 1.0).abs() < epsilon
    }

    /// Normalize the normal vector to unit length
    fn normalize_normal(&mut self) {
        let mag = self.normal_magnitude();
        if mag > 0.0 {
            let (nx, ny, nz) = self.normal();
            self.set_normal(nx / mag, ny / mag, nz / mag);
        }
    }

    /// Get a normalized copy of the normal vector
    fn normalized_normal(&self) -> (f32, f32, f32) {
        let mag = self.normal_magnitude();
        if mag > 0.0 {
            let (nx, ny, nz) = self.normal();
            (nx / mag, ny / mag, nz / mag)
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Compute dot product between this normal and another
    fn normal_dot_product(&self, other: &Self) -> f32 {
        let (nx1, ny1, nz1) = self.normal();
        let (nx2, ny2, nz2) = other.normal();
        nx1 * nx2 + ny1 * ny2 + nz1 * nz2
    }

    /// Compute angle between this normal and another (in radians)
    fn normal_angle_to(&self, other: &Self) -> f32 {
        let dot = self.normal_dot_product(other);
        // Clamp to avoid floating point errors in acos
        dot.clamp(-1.0, 1.0).acos()
    }

    /// Check if two normals are approximately parallel
    fn is_parallel_to(&self, other: &Self, angle_threshold: f32) -> bool {
        let angle = self.normal_angle_to(other);
        angle < angle_threshold || (std::f32::consts::PI - angle) < angle_threshold
    }

    /// Flip the normal direction
    fn flip_normal(&mut self) {
        let (nx, ny, nz) = self.normal();
        self.set_normal(-nx, -ny, -nz);
    }

    /// Check if the normal is valid (not NaN or infinite)
    fn is_normal_valid(&self) -> bool {
        let (nx, ny, nz) = self.normal();
        nx.is_finite() && ny.is_finite() && nz.is_finite()
    }
}

/// Trait for points with intensity information
///
/// Intensity represents the strength of the reflected signal in laser scanning
/// or the brightness value in imaging. This is commonly used in LiDAR data.
pub trait Intensity: Point {
    /// Get the intensity value
    fn intensity(&self) -> f32;

    /// Set the intensity value
    fn set_intensity(&mut self, intensity: f32);

    /// Check if intensity is within a valid range
    fn is_intensity_valid(&self) -> bool {
        let intensity = self.intensity();
        intensity.is_finite() && intensity >= 0.0
    }

    /// Normalize intensity to 0.0-1.0 range given max intensity
    fn normalized_intensity(&self, max_intensity: f32) -> f32 {
        if max_intensity > 0.0 {
            (self.intensity() / max_intensity).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
}

/// Extension trait for points with both XYZ and Intensity
pub trait Xyzi: Xyz + Intensity {
    /// Get all components as a tuple (x, y, z, intensity)
    fn xyzi(&self) -> (f32, f32, f32, f32) {
        (self.x(), self.y(), self.z(), self.intensity())
    }
}

// Blanket implementation for all types that have both traits
impl<T: Xyz + Intensity> Xyzi for T {}

/// Extension trait for points with both XYZ and RGB
pub trait Xyzrgb: Xyz + Rgb {
    /// Get all components as a tuple (x, y, z, r, g, b)
    fn xyzrgb(&self) -> (f32, f32, f32, u8, u8, u8) {
        (self.x(), self.y(), self.z(), self.r(), self.g(), self.b())
    }
}

// Blanket implementation for all types that have both traits
impl<T: Xyz + Rgb> Xyzrgb for T {}

/// Trait for points with curvature information
///
/// Curvature represents how much a surface bends at a given point.
/// Higher values indicate sharper features like edges and corners.
pub trait Curvature: Point {
    /// Get the curvature value
    fn curvature(&self) -> f32;

    /// Set the curvature value
    fn set_curvature(&mut self, curvature: f32);

    /// Check if this point represents an edge (high curvature)
    fn is_edge(&self, threshold: f32) -> bool {
        self.curvature() > threshold
    }

    /// Check if this point is on a flat surface (low curvature)
    fn is_flat(&self, threshold: f32) -> bool {
        self.curvature() < threshold
    }

    /// Check if curvature is valid (not NaN or infinite)
    fn is_curvature_valid(&self) -> bool {
        self.curvature().is_finite()
    }
}

/// Utility trait for converting between different point types
///
/// This trait allows safe conversion between point types when they share
/// common capabilities. For example, converting PointXYZRGB to PointXYZ
/// preserves the spatial coordinates while discarding color information.
pub trait ConvertPoint<T: Point>: Point {
    /// Convert this point to another point type
    ///
    /// The conversion preserves all compatible fields and uses default
    /// values for fields that don't exist in the source type.
    fn convert_to(&self) -> T;
}

/// Marker trait for points that can be used in spatial indexing structures
///
/// This trait is automatically implemented for any point type that has
/// 3D coordinates, indicating it can be used with spatial acceleration
/// structures like KdTree and Octree.
pub trait SpatialPoint: Xyz {}

// Blanket implementation: any point with XYZ coordinates can be spatially indexed
impl<T: Xyz> SpatialPoint for T {}

/// Marker trait for points suitable for surface reconstruction
///
/// Surface reconstruction algorithms typically require both position
/// and normal information to create high-quality meshes.
pub trait SurfacePoint: Xyz + NormalXyz {}

// Blanket implementation: any point with both XYZ and normals can do surface reconstruction
impl<T: Xyz + NormalXyz> SurfacePoint for T {}

/// Trait for point types that support coordinate push operations
pub trait PointXyzOps: Point {
    /// Push a point with x, y, z coordinates to the cloud
    fn push_xyz(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32);
}

/// Trait for point types that support RGB push operations
pub trait PointRgbOps: Point {
    /// Push a point with x, y, z coordinates and RGB color to the cloud
    fn push_xyzrgb(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, r: u8, g: u8, b: u8);
}

/// Trait for point types that support intensity push operations
pub trait PointIntensityOps: Point {
    /// Push a point with x, y, z coordinates and intensity to the cloud
    fn push_xyzi(cloud: Pin<&mut Self::CloudType>, x: f32, y: f32, z: f32, intensity: f32);
}

/// Generic utilities for working with point traits
pub mod utils {
    use super::*;

    /// Compute the bounding box of a collection of points
    pub fn compute_bounding_box<T: Xyz>(
        points: &[T],
    ) -> Option<((f32, f32, f32), (f32, f32, f32))> {
        if points.is_empty() {
            return None;
        }

        let first = &points[0];
        let (mut min_x, mut min_y, mut min_z) = first.xyz();
        let (mut max_x, mut max_y, mut max_z) = (min_x, min_y, min_z);

        for point in points.iter().skip(1) {
            let (x, y, z) = point.xyz();
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            min_z = min_z.min(z);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
            max_z = max_z.max(z);
        }

        Some(((min_x, min_y, min_z), (max_x, max_y, max_z)))
    }

    /// Compute the centroid of a collection of points
    pub fn compute_centroid<T: Xyz>(points: &[T]) -> Option<(f32, f32, f32)> {
        if points.is_empty() {
            return None;
        }

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;

        for point in points {
            let (x, y, z) = point.xyz();
            sum_x += x;
            sum_y += y;
            sum_z += z;
        }

        let count = points.len() as f32;
        Some((sum_x / count, sum_y / count, sum_z / count))
    }

    /// Find the closest point to a query point
    /// Returns (index, distance) of the closest point
    pub fn find_closest<T: Xyz>(query: &T, points: &[T]) -> Option<(usize, f32)> {
        if points.is_empty() {
            return None;
        }

        let mut closest_idx = 0;
        let mut closest_distance = query.distance_to(&points[0]);

        for (idx, point) in points.iter().enumerate().skip(1) {
            let distance = query.distance_to(point);
            if distance < closest_distance {
                closest_idx = idx;
                closest_distance = distance;
            }
        }

        Some((closest_idx, closest_distance))
    }

    /// Filter points by distance from a center point
    pub fn filter_by_distance<T: Xyz>(center: &T, points: &[T], max_distance: f32) -> Vec<usize> {
        points
            .iter()
            .enumerate()
            .filter_map(|(idx, point)| {
                if center.distance_to(point) <= max_distance {
                    Some(idx)
                } else {
                    None
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    // use super::*;

    // Note: We can't easily create a mock that implements UniquePtrTarget
    // (it's a sealed trait from cxx), and all our traits require Point
    // as a supertrait which requires UniquePtrTarget for its associated
    // types. So these trait tests are temporarily disabled.
    // The traits are tested through the actual point type implementations
    // in the integration tests and examples.

    /*
    // Simple mock point type that only implements the trait methods we can test
    #[derive(Clone, Debug)]
    struct MockPoint {
        x: f32,
        y: f32,
        z: f32,
        r: u8,
        g: u8,
        b: u8,
    }

    impl MockPoint {
        fn default_point() -> Self {
            Self {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                r: 0,
                g: 0,
                b: 0,
            }
        }
    }

    impl Xyz for MockPoint {
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

    impl Rgb for MockPoint {
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
    }

    #[test]
    fn test_point_xyz_basics() {
        let mut point = MockPoint::default_point();
        assert_eq!(point.xyz(), (0.0, 0.0, 0.0));

        point.set_xyz(1.0, 2.0, 3.0);
        assert_eq!(point.xyz(), (1.0, 2.0, 3.0));
        assert_eq!(point.x(), 1.0);
        assert_eq!(point.y(), 2.0);
        assert_eq!(point.z(), 3.0);
    }

    #[test]
    fn test_point_distance() {
        let p1 = {
            let mut p = MockPoint::default_point();
            p.set_xyz(0.0, 0.0, 0.0);
            p
        };

        let p2 = {
            let mut p = MockPoint::default_point();
            p.set_xyz(3.0, 4.0, 0.0);
            p
        };

        assert!((p1.distance_to(&p2) - 5.0).abs() < 1e-6);
        assert!((p1.distance_squared_to(&p2) - 25.0).abs() < 1e-6);
    }

    #[test]
    fn test_point_rgb_basics() {
        let mut point = MockPoint::default_point();
        assert_eq!(point.rgb(), (0, 0, 0));

        point.set_rgb(255, 128, 64);
        assert_eq!(point.rgb(), (255, 128, 64));
        assert_eq!(point.r(), 255);
        assert_eq!(point.g(), 128);
        assert_eq!(point.b(), 64);
    }

    #[test]
    fn test_color_operations() {
        let mut point = MockPoint::default_point();
        point.set_rgb_from_float(1.0, 0.5, 0.25);

        let (r, g, b) = point.rgb();
        assert_eq!(r, 255);
        assert_eq!(g, 127); // 0.5 * 255 = 127.5 -> 127
        assert_eq!(b, 63); // 0.25 * 255 = 63.75 -> 63

        let (rf, gf, bf) = point.rgb_as_float();
        assert!((rf - 1.0).abs() < 0.01);
        assert!((gf - 0.498).abs() < 0.01); // 127/255 ≈ 0.498
        assert!((bf - 0.247).abs() < 0.01); // 63/255 ≈ 0.247
    }

    #[test]
    fn test_color_distance() {
        let p1 = {
            let mut p = MockPoint::default_point();
            p.set_rgb(255, 0, 0); // Red
            p
        };

        let p2 = {
            let mut p = MockPoint::default_point();
            p.set_rgb(0, 0, 255); // Blue
            p
        };

        let distance = p1.color_distance_to(&p2);
        // Should be sqrt(1^2 + 0^2 + 1^2) = sqrt(2) ≈ 1.414
        assert!((distance - std::f32::consts::SQRT_2).abs() < 0.01);
    }

    #[test]
    fn test_utils_bounding_box() {
        let points = vec![
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(1.0, 2.0, 3.0);
                p
            },
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(-1.0, 5.0, 0.0);
                p
            },
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(3.0, -1.0, 7.0);
                p
            },
        ];

        let bbox = utils::compute_bounding_box(&points).unwrap();
        assert_eq!(bbox, ((-1.0, -1.0, 0.0), (3.0, 5.0, 7.0)));
    }

    #[test]
    fn test_utils_centroid() {
        let points = vec![
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(0.0, 0.0, 0.0);
                p
            },
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(6.0, 0.0, 0.0);
                p
            },
            {
                let mut p = MockPoint::default_point();
                p.set_xyz(0.0, 9.0, 0.0);
                p
            },
        ];

        let centroid = utils::compute_centroid(&points).unwrap();
        assert_eq!(centroid, (2.0, 3.0, 0.0));
    }


    #[test]
    fn test_magnitude_and_normalization() {
        let mut point = MockPoint::default_point();
        point.set_xyz(3.0, 4.0, 0.0);

        assert!((point.magnitude() - 5.0).abs() < 1e-6);
        assert!((point.magnitude_squared() - 25.0).abs() < 1e-6);

        point.normalize();
        assert!((point.magnitude() - 1.0).abs() < 1e-6);
        assert!((point.x() - 0.6).abs() < 1e-6);
        assert!((point.y() - 0.8).abs() < 1e-6);
    }
    */
}
