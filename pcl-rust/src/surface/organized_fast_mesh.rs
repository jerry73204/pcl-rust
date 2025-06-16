//! Organized Fast Mesh surface reconstruction
//!
//! This module provides a fast triangulation method for organized point clouds
//! (e.g., from RGB-D sensors) where the point cloud has a 2D structure.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::surface::{PolygonMesh, SurfaceReconstruction};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Triangulation type for organized fast mesh
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriangulationType {
    /// Triangle mesh (most common)
    TriangleMesh = 0,
    /// Triangle with adaptive threshold
    TriangleAdaptive = 1,
    /// Quad mesh
    QuadMesh = 2,
}

impl TriangulationType {
    /// Convert from integer value
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(TriangulationType::TriangleMesh),
            1 => Some(TriangulationType::TriangleAdaptive),
            2 => Some(TriangulationType::QuadMesh),
            _ => None,
        }
    }

    /// Convert to integer value
    pub fn to_i32(self) -> i32 {
        self as i32
    }
}

/// Fast triangulation for organized point clouds
pub struct OrganizedFastMeshXYZ {
    inner: UniquePtr<ffi::OrganizedFastMesh_PointXYZ>,
}

impl OrganizedFastMeshXYZ {
    /// Create a new OrganizedFastMesh instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_organized_fast_mesh_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "OrganizedFastMesh".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the maximum number of pixels that form an angle for triangulation
    pub fn set_triangle_pixel_size(&mut self, triangle_size: i32) -> PclResult<()> {
        if triangle_size <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid triangle pixel size",
                "triangle_size",
                "positive value",
                &triangle_size.to_string(),
            ));
        }
        ffi::set_triangle_pixel_size_xyz(self.inner.pin_mut(), triangle_size);
        Ok(())
    }

    /// Get the current triangle pixel size
    pub fn triangle_pixel_size(&self) -> i32 {
        ffi::get_triangle_pixel_size_xyz(&self.inner)
    }

    /// Set the triangulation type
    pub fn set_triangulation_type(&mut self, triangle_type: TriangulationType) -> PclResult<()> {
        ffi::set_triangulation_type_xyz(self.inner.pin_mut(), triangle_type.to_i32());
        Ok(())
    }

    /// Get the current triangulation type
    pub fn triangulation_type(&self) -> TriangulationType {
        let value = ffi::get_triangulation_type_xyz(&self.inner);
        TriangulationType::from_i32(value).unwrap_or(TriangulationType::TriangleMesh)
    }
}

impl SurfaceReconstruction<PointCloudXYZ, PolygonMesh> for OrganizedFastMeshXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }

        // Organized fast mesh requires organized point clouds
        if cloud.height() <= 1 {
            return Err(PclError::invalid_point_cloud(
                "OrganizedFastMesh requires organized point clouds (height > 1)",
            ));
        }

        ffi::set_input_cloud_ofm_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        let result = ffi::perform_reconstruction_ofm_xyz(self.inner.pin_mut(), mesh.as_raw_mut());
        if result < 0 {
            return Err(PclError::ProcessingFailed {
                message: "OrganizedFastMesh reconstruction failed".to_string(),
            });
        }
        Ok(())
    }
}

impl Default for OrganizedFastMeshXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default OrganizedFastMeshXYZ")
    }
}

/// Builder for OrganizedFastMeshXYZ configuration
pub struct OrganizedFastMeshXYZBuilder {
    triangle_pixel_size: Option<i32>,
    triangulation_type: Option<TriangulationType>,
}

impl OrganizedFastMeshXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            triangle_pixel_size: None,
            triangulation_type: None,
        }
    }

    /// Set the triangle pixel size
    pub fn triangle_pixel_size(mut self, size: i32) -> Self {
        self.triangle_pixel_size = Some(size);
        self
    }

    /// Set the triangulation type
    pub fn triangulation_type(mut self, triangle_type: TriangulationType) -> Self {
        self.triangulation_type = Some(triangle_type);
        self
    }

    /// Build the OrganizedFastMeshXYZ instance
    pub fn build(self) -> PclResult<OrganizedFastMeshXYZ> {
        let mut ofm = OrganizedFastMeshXYZ::new()?;

        if let Some(size) = self.triangle_pixel_size {
            ofm.set_triangle_pixel_size(size)?;
        }
        if let Some(triangle_type) = self.triangulation_type {
            ofm.set_triangulation_type(triangle_type)?;
        }

        Ok(ofm)
    }
}

impl Default for OrganizedFastMeshXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_organized_fast_mesh_creation() {
        let ofm = OrganizedFastMeshXYZ::new();
        assert!(ofm.is_ok());
    }

    #[test]
    fn test_triangulation_type() {
        assert_eq!(TriangulationType::TriangleMesh.to_i32(), 0);
        assert_eq!(TriangulationType::TriangleAdaptive.to_i32(), 1);
        assert_eq!(TriangulationType::QuadMesh.to_i32(), 2);

        assert_eq!(
            TriangulationType::from_i32(0),
            Some(TriangulationType::TriangleMesh)
        );
        assert_eq!(
            TriangulationType::from_i32(1),
            Some(TriangulationType::TriangleAdaptive)
        );
        assert_eq!(
            TriangulationType::from_i32(2),
            Some(TriangulationType::QuadMesh)
        );
        assert_eq!(TriangulationType::from_i32(99), None);
    }

    #[test]
    fn test_organized_fast_mesh_builder() {
        let ofm = OrganizedFastMeshXYZBuilder::new()
            .triangle_pixel_size(1)
            .triangulation_type(TriangulationType::TriangleMesh)
            .build();

        assert!(ofm.is_ok());
        let ofm = ofm.unwrap();
        assert_eq!(ofm.triangle_pixel_size(), 1);
        assert_eq!(ofm.triangulation_type(), TriangulationType::TriangleMesh);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut ofm = OrganizedFastMeshXYZ::new().unwrap();
        assert!(ofm.set_triangle_pixel_size(-1).is_err());
        assert!(ofm.set_triangle_pixel_size(0).is_err());
    }
}
