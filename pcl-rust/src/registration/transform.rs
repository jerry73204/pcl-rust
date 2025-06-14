//! Transformation matrix utilities for point cloud registration
//!
//! This module provides utilities for working with 3D transformation matrices
//! commonly used in point cloud registration algorithms.

use std::ops::{Index, IndexMut, Mul};

/// A 4x4 homogeneous transformation matrix for 3D transformations
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformationMatrix {
    data: [[f32; 4]; 4],
}

impl TransformationMatrix {
    /// Create a new identity transformation matrix
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a transformation matrix from a flat array (row-major order)
    pub fn from_array(array: &[f32; 16]) -> Self {
        let mut data = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                data[i][j] = array[i * 4 + j];
            }
        }
        Self { data }
    }

    /// Create a transformation matrix from a Vec<f32> (row-major order)
    pub fn from_vec(vec: &Vec<f32>) -> Option<Self> {
        if vec.len() != 16 {
            return None;
        }

        let mut data = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                data[i][j] = vec[i * 4 + j];
            }
        }
        Some(Self { data })
    }

    /// Convert to a flat Vec<f32> (row-major order)
    pub fn to_vec(&self) -> Vec<f32> {
        let mut result = Vec::with_capacity(16);
        for i in 0..4 {
            for j in 0..4 {
                result.push(self.data[i][j]);
            }
        }
        result
    }

    /// Get the translation component
    pub fn translation(&self) -> [f32; 3] {
        [self.data[0][3], self.data[1][3], self.data[2][3]]
    }

    /// Set the translation component
    pub fn set_translation(&mut self, x: f32, y: f32, z: f32) {
        self.data[0][3] = x;
        self.data[1][3] = y;
        self.data[2][3] = z;
    }

    /// Get the rotation matrix (3x3 upper-left portion)
    pub fn rotation(&self) -> [[f32; 3]; 3] {
        [
            [self.data[0][0], self.data[0][1], self.data[0][2]],
            [self.data[1][0], self.data[1][1], self.data[1][2]],
            [self.data[2][0], self.data[2][1], self.data[2][2]],
        ]
    }

    /// Create a translation matrix
    pub fn translation_matrix(x: f32, y: f32, z: f32) -> Self {
        let mut result = Self::identity();
        result.set_translation(x, y, z);
        result
    }

    /// Create a rotation matrix around X axis
    pub fn rotation_x(angle_radians: f32) -> Self {
        let cos = angle_radians.cos();
        let sin = angle_radians.sin();

        Self {
            data: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, cos, -sin, 0.0],
                [0.0, sin, cos, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a rotation matrix around Y axis
    pub fn rotation_y(angle_radians: f32) -> Self {
        let cos = angle_radians.cos();
        let sin = angle_radians.sin();

        Self {
            data: [
                [cos, 0.0, sin, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [-sin, 0.0, cos, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a rotation matrix around Z axis
    pub fn rotation_z(angle_radians: f32) -> Self {
        let cos = angle_radians.cos();
        let sin = angle_radians.sin();

        Self {
            data: [
                [cos, -sin, 0.0, 0.0],
                [sin, cos, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Compute the inverse of this transformation matrix
    pub fn inverse(&self) -> Option<Self> {
        // For rigid transformations (rotation + translation),
        // the inverse can be computed efficiently
        let r = self.rotation();
        let t = self.translation();

        // Transpose of rotation matrix
        let mut result = Self::identity();
        for i in 0..3 {
            for j in 0..3 {
                result.data[i][j] = r[j][i];
            }
        }

        // Negative translation in rotated coordinates
        result.data[0][3] = -(r[0][0] * t[0] + r[1][0] * t[1] + r[2][0] * t[2]);
        result.data[1][3] = -(r[0][1] * t[0] + r[1][1] * t[1] + r[2][1] * t[2]);
        result.data[2][3] = -(r[0][2] * t[0] + r[1][2] * t[1] + r[2][2] * t[2]);

        Some(result)
    }

    /// Transform a 3D point
    pub fn transform_point(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        let x_new =
            self.data[0][0] * x + self.data[0][1] * y + self.data[0][2] * z + self.data[0][3];
        let y_new =
            self.data[1][0] * x + self.data[1][1] * y + self.data[1][2] * z + self.data[1][3];
        let z_new =
            self.data[2][0] * x + self.data[2][1] * y + self.data[2][2] * z + self.data[2][3];
        (x_new, y_new, z_new)
    }
}

impl Default for TransformationMatrix {
    fn default() -> Self {
        Self::identity()
    }
}

impl Index<(usize, usize)> for TransformationMatrix {
    type Output = f32;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0][index.1]
    }
}

impl IndexMut<(usize, usize)> for TransformationMatrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0][index.1]
    }
}

impl Mul for TransformationMatrix {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut result = Self::identity();

        for i in 0..4 {
            for j in 0..4 {
                result.data[i][j] = 0.0;
                for k in 0..4 {
                    result.data[i][j] += self.data[i][k] * rhs.data[k][j];
                }
            }
        }

        result
    }
}

/// Convenience type alias for 3D transformations
pub type Transform3D = TransformationMatrix;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_matrix() {
        let t = TransformationMatrix::identity();
        assert_eq!(t[(0, 0)], 1.0);
        assert_eq!(t[(1, 1)], 1.0);
        assert_eq!(t[(2, 2)], 1.0);
        assert_eq!(t[(3, 3)], 1.0);
        assert_eq!(t[(0, 1)], 0.0);
    }

    #[test]
    fn test_translation_matrix() {
        let t = TransformationMatrix::translation_matrix(1.0, 2.0, 3.0);
        assert_eq!(t.translation(), [1.0, 2.0, 3.0]);

        let (x, y, z) = t.transform_point(0.0, 0.0, 0.0);
        assert_eq!(x, 1.0);
        assert_eq!(y, 2.0);
        assert_eq!(z, 3.0);
    }

    #[test]
    fn test_matrix_multiplication() {
        let t1 = TransformationMatrix::translation_matrix(1.0, 0.0, 0.0);
        let t2 = TransformationMatrix::translation_matrix(0.0, 1.0, 0.0);
        let combined = t1 * t2;

        let (x, y, z) = combined.transform_point(0.0, 0.0, 0.0);
        assert_eq!(x, 1.0);
        assert_eq!(y, 1.0);
        assert_eq!(z, 0.0);
    }

    #[test]
    fn test_from_vec() {
        let vec: Vec<f32> = vec![
            1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, 1.0, 7.0, 0.0, 0.0, 0.0, 1.0,
        ];

        let t = TransformationMatrix::from_vec(&vec).unwrap();
        assert_eq!(t.translation(), [5.0, 6.0, 7.0]);
    }
}
