//! Polygon mesh data structure and operations
//!
//! This module provides a safe Rust wrapper around PCL's PolygonMesh type,
//! which represents triangulated surface meshes.

use crate::error::{PclError, PclResult};
use cxx::UniquePtr;
use pcl_sys::ffi;
use std::path::Path;

/// A polygon mesh containing vertices and face connectivity information
pub struct PolygonMesh {
    inner: UniquePtr<ffi::PolygonMesh>,
}

impl PolygonMesh {
    /// Create a new empty polygon mesh
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_polygon_mesh();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "PolygonMesh".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Get the number of polygons (faces) in the mesh
    pub fn polygon_count(&self) -> usize {
        ffi::get_polygon_count(&self.inner)
    }

    /// Get the number of vertices in the mesh
    pub fn vertex_count(&self) -> usize {
        ffi::get_vertex_count(&self.inner)
    }

    /// Check if the mesh is valid (has both vertices and polygons)
    pub fn is_valid(&self) -> bool {
        ffi::is_valid_mesh(&self.inner)
    }

    /// Check if the mesh is empty
    pub fn is_empty(&self) -> bool {
        self.polygon_count() == 0 || self.vertex_count() == 0
    }

    /// Save the mesh to a PLY file
    pub fn save_ply<P: AsRef<Path>>(&self, filename: P) -> PclResult<()> {
        let filename_str = filename.as_ref().to_string_lossy();
        let result = ffi::save_polygon_mesh_ply(&self.inner, &filename_str);
        if result < 0 {
            return Err(PclError::IoError {
                message: format!("Failed to save PLY file: {}", filename_str),
                path: Some(filename.as_ref().to_path_buf()),
                source: None,
            });
        }
        Ok(())
    }

    /// Save the mesh to an OBJ file
    pub fn save_obj<P: AsRef<Path>>(&self, filename: P) -> PclResult<()> {
        let filename_str = filename.as_ref().to_string_lossy();
        let result = ffi::save_polygon_mesh_obj(&self.inner, &filename_str);
        if result < 0 {
            return Err(PclError::IoError {
                message: format!("Failed to save OBJ file: {}", filename_str),
                path: Some(filename.as_ref().to_path_buf()),
                source: None,
            });
        }
        Ok(())
    }

    /// Save the mesh to a VTK file
    pub fn save_vtk<P: AsRef<Path>>(&self, filename: P) -> PclResult<()> {
        let filename_str = filename.as_ref().to_string_lossy();
        let result = ffi::save_polygon_mesh_vtk(&self.inner, &filename_str);
        if result < 0 {
            return Err(PclError::IoError {
                message: format!("Failed to save VTK file: {}", filename_str),
                path: Some(filename.as_ref().to_path_buf()),
                source: None,
            });
        }
        Ok(())
    }

    /// Get a mutable reference to the underlying PCL PolygonMesh
    pub(crate) fn as_raw_mut(&mut self) -> std::pin::Pin<&mut ffi::PolygonMesh> {
        self.inner.pin_mut()
    }

    /// Get a reference to the underlying PCL PolygonMesh
    pub(crate) fn as_raw(&self) -> &ffi::PolygonMesh {
        &self.inner
    }
}

impl Default for PolygonMesh {
    fn default() -> Self {
        Self::new().expect("Failed to create default PolygonMesh")
    }
}

impl std::fmt::Debug for PolygonMesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PolygonMesh")
            .field("vertices", &self.vertex_count())
            .field("polygons", &self.polygon_count())
            .field("is_valid", &self.is_valid())
            .finish()
    }
}

/// File format for mesh export
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshFileFormat {
    /// Stanford PLY format
    Ply,
    /// Wavefront OBJ format
    Obj,
    /// Visualization Toolkit (VTK) format
    Vtk,
}

impl MeshFileFormat {
    /// Get the typical file extension for this format
    pub fn extension(&self) -> &'static str {
        match self {
            MeshFileFormat::Ply => "ply",
            MeshFileFormat::Obj => "obj",
            MeshFileFormat::Vtk => "vtk",
        }
    }

    /// Detect format from file extension
    pub fn from_extension(extension: &str) -> Option<Self> {
        match extension.to_lowercase().as_str() {
            "ply" => Some(MeshFileFormat::Ply),
            "obj" => Some(MeshFileFormat::Obj),
            "vtk" => Some(MeshFileFormat::Vtk),
            _ => None,
        }
    }
}

impl PolygonMesh {
    /// Save the mesh with automatic format detection based on file extension
    pub fn save<P: AsRef<Path>>(&self, filename: P) -> PclResult<()> {
        let path = filename.as_ref();
        let extension = path
            .extension()
            .and_then(|ext| ext.to_str())
            .ok_or_else(|| {
                PclError::invalid_parameters(
                    "Cannot determine file format",
                    "filename",
                    "file with recognized extension (.ply, .obj, .vtk)",
                    path.to_string_lossy().as_ref(),
                )
            })?;

        let format = MeshFileFormat::from_extension(extension).ok_or_else(|| {
            PclError::invalid_parameters(
                "Unsupported file format",
                "extension",
                "supported format (.ply, .obj, .vtk)",
                extension,
            )
        })?;

        match format {
            MeshFileFormat::Ply => self.save_ply(filename),
            MeshFileFormat::Obj => self.save_obj(filename),
            MeshFileFormat::Vtk => self.save_vtk(filename),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polygon_mesh_creation() {
        let mesh = PolygonMesh::new();
        assert!(mesh.is_ok());

        let mesh = mesh.unwrap();
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.polygon_count(), 0);
        assert!(mesh.is_empty());
        assert!(!mesh.is_valid());
    }

    #[test]
    fn test_mesh_file_format() {
        assert_eq!(MeshFileFormat::Ply.extension(), "ply");
        assert_eq!(MeshFileFormat::Obj.extension(), "obj");
        assert_eq!(MeshFileFormat::Vtk.extension(), "vtk");

        assert_eq!(
            MeshFileFormat::from_extension("ply"),
            Some(MeshFileFormat::Ply)
        );
        assert_eq!(
            MeshFileFormat::from_extension("PLY"),
            Some(MeshFileFormat::Ply)
        );
        assert_eq!(
            MeshFileFormat::from_extension("obj"),
            Some(MeshFileFormat::Obj)
        );
        assert_eq!(
            MeshFileFormat::from_extension("vtk"),
            Some(MeshFileFormat::Vtk)
        );
        assert_eq!(MeshFileFormat::from_extension("unknown"), None);
    }

    #[test]
    fn test_debug_format() {
        let mesh = PolygonMesh::new().unwrap();
        let debug_str = format!("{:?}", mesh);
        assert!(debug_str.contains("PolygonMesh"));
        assert!(debug_str.contains("vertices"));
        assert!(debug_str.contains("polygons"));
    }
}
