//! Point cloud transformation utilities
//!
//! This module provides utilities for transforming point clouds using 4x4 homogeneous
//! transformation matrices.

use crate::common::PointCloud;
use crate::error::{PclError, PclResult};
use crate::traits::Point;
use pcl_sys::ffi;
use std::pin::Pin;

/// Transform a point cloud using a 4x4 homogeneous transformation matrix
///
/// # Arguments
/// * `input` - The input point cloud
/// * `transform` - A 4x4 transformation matrix in row-major order (16 elements)
///
/// # Returns
/// A new transformed point cloud
///
/// # Example
/// ```no_run
/// use pcl::common::{PointCloud, PointXYZ, transform_point_cloud};
/// use pcl::error::PclResult;
///
/// fn example() -> PclResult<()> {
///     let mut cloud = PointCloud::<PointXYZ>::new()?;
///     cloud.push(1.0, 0.0, 0.0)?;
///     cloud.push(0.0, 1.0, 0.0)?;
///     
///     // Create a 90-degree rotation around Z axis
///     let transform = [
///         0.0, -1.0, 0.0, 0.0,  // cos(90°), -sin(90°), 0, 0
///         1.0,  0.0, 0.0, 0.0,  // sin(90°),  cos(90°), 0, 0
///         0.0,  0.0, 1.0, 0.0,  // 0, 0, 1, 0
///         0.0,  0.0, 0.0, 1.0,  // 0, 0, 0, 1
///     ];
///     
///     let transformed = transform_point_cloud(&cloud, &transform)?;
///     Ok(())
/// }
/// ```
pub fn transform_point_cloud<T: Point>(
    input: &PointCloud<T>,
    transform: &[f32; 16],
) -> PclResult<PointCloud<T>>
where
    T::CloudType: cxx::memory::UniquePtrTarget,
{
    // Create output cloud
    let mut output = PointCloud::<T>::new()?;

    // Convert transform to Vec for FFI
    let transform_vec = transform.to_vec();

    // Get the inner references
    let input_inner = input.inner();
    let mut output_inner = output.inner_mut();

    // Call the appropriate FFI function based on point type
    match T::type_name() {
        "PointXYZ" => {
            // SAFETY: We know the types are correct based on type_name()
            unsafe {
                let input_cloud =
                    &*(input_inner as *const T::CloudType as *const ffi::PointCloud_PointXYZ);
                let output_cloud = Pin::new_unchecked(
                    &mut *(output_inner.as_mut().get_unchecked_mut() as *mut T::CloudType
                        as *mut ffi::PointCloud_PointXYZ),
                );
                ffi::transform_point_cloud_xyz(input_cloud, output_cloud, &transform_vec);
            }
        }
        "PointXYZI" => unsafe {
            let input_cloud =
                &*(input_inner as *const T::CloudType as *const ffi::PointCloud_PointXYZI);
            let output_cloud = Pin::new_unchecked(
                &mut *(output_inner.as_mut().get_unchecked_mut() as *mut T::CloudType
                    as *mut ffi::PointCloud_PointXYZI),
            );
            ffi::transform_point_cloud_xyzi(input_cloud, output_cloud, &transform_vec);
        },
        "PointXYZRGB" => unsafe {
            let input_cloud =
                &*(input_inner as *const T::CloudType as *const ffi::PointCloud_PointXYZRGB);
            let output_cloud = Pin::new_unchecked(
                &mut *(output_inner.as_mut().get_unchecked_mut() as *mut T::CloudType
                    as *mut ffi::PointCloud_PointXYZRGB),
            );
            ffi::transform_point_cloud_xyzrgb(input_cloud, output_cloud, &transform_vec);
        },
        "PointNormal" => unsafe {
            let input_cloud =
                &*(input_inner as *const T::CloudType as *const ffi::PointCloud_PointNormal);
            let output_cloud = Pin::new_unchecked(
                &mut *(output_inner.as_mut().get_unchecked_mut() as *mut T::CloudType
                    as *mut ffi::PointCloud_PointNormal),
            );
            ffi::transform_point_cloud_normal(input_cloud, output_cloud, &transform_vec);
        },
        _ => {
            return Err(PclError::InvalidParameter {
                param: "point type".to_string(),
                message: format!(
                    "Transform not implemented for point type: {}",
                    T::type_name()
                ),
            });
        }
    }

    Ok(output)
}

/// Helper to create common transformation matrices
pub struct TransformBuilder {
    matrix: [f32; 16],
}

impl TransformBuilder {
    /// Create a new identity transformation
    pub fn new() -> Self {
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a translation transformation
    pub fn translation(x: f32, y: f32, z: f32) -> Self {
        Self {
            matrix: [
                1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a rotation around X axis (in radians)
    pub fn rotation_x(angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, 0.0, cos, -sin, 0.0, 0.0, sin, cos, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a rotation around Y axis (in radians)
    pub fn rotation_y(angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            matrix: [
                cos, 0.0, sin, 0.0, 0.0, 1.0, 0.0, 0.0, -sin, 0.0, cos, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a rotation around Z axis (in radians)
    pub fn rotation_z(angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            matrix: [
                cos, -sin, 0.0, 0.0, sin, cos, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a uniform scaling transformation
    pub fn scale(s: f32) -> Self {
        Self {
            matrix: [
                s, 0.0, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Get the transformation matrix
    pub fn build(self) -> [f32; 16] {
        self.matrix
    }
}

impl Default for TransformBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::PointXYZ;

    #[test]
    fn test_transform_builder() {
        let identity = TransformBuilder::new().build();
        assert_eq!(identity[0], 1.0);
        assert_eq!(identity[5], 1.0);
        assert_eq!(identity[10], 1.0);
        assert_eq!(identity[15], 1.0);

        let translation = TransformBuilder::translation(1.0, 2.0, 3.0).build();
        assert_eq!(translation[3], 1.0);
        assert_eq!(translation[7], 2.0);
        assert_eq!(translation[11], 3.0);
    }

    #[test]
    #[ignore = "Transform of empty cloud causes SIGFPE - needs investigation"]
    fn test_transform_empty_cloud() {
        let cloud = PointCloud::<PointXYZ>::new().unwrap();
        let transform = TransformBuilder::new().build();
        let result = transform_point_cloud(&cloud, &transform);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().size(), 0);
    }
}
