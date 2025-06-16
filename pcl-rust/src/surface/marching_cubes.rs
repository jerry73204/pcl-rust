//! Marching Cubes surface reconstruction algorithms
//!
//! This module provides implementations of Marching Cubes surface reconstruction
//! using both Hoppe and RBF (Radial Basis Function) variants.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::surface::{PolygonMesh, SurfaceReconstruction};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Marching Cubes surface reconstruction using Hoppe's method
pub struct MarchingCubesHoppeXYZ {
    inner: UniquePtr<ffi::MarchingCubesHoppe_PointXYZ>,
}

impl MarchingCubesHoppeXYZ {
    /// Create a new Marching Cubes Hoppe reconstruction instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_marching_cubes_hoppe_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "MarchingCubesHoppe".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the iso level for surface extraction
    pub fn set_iso_level(&mut self, iso_level: f32) -> PclResult<()> {
        ffi::set_iso_level_hoppe_xyz(self.inner.pin_mut(), iso_level);
        Ok(())
    }

    /// Get the current iso level
    pub fn iso_level(&self) -> f32 {
        ffi::get_iso_level_hoppe_xyz(self.inner.pin_mut())
    }

    /// Set the grid resolution for voxelization
    pub fn set_grid_resolution(&mut self, res_x: i32, res_y: i32, res_z: i32) -> PclResult<()> {
        if res_x <= 0 || res_y <= 0 || res_z <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid grid resolution",
                "res_x, res_y, res_z",
                "positive values",
                &format!("({}, {}, {})", res_x, res_y, res_z),
            ));
        }
        ffi::set_grid_resolution_hoppe_xyz(self.inner.pin_mut(), res_x, res_y, res_z);
        Ok(())
    }

    /// Set the percentage to extend the grid beyond the bounding box
    pub fn set_percentage_extend_grid(&mut self, percentage: f32) -> PclResult<()> {
        if percentage < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid grid extension percentage",
                "percentage",
                "non-negative value",
                &percentage.to_string(),
            ));
        }
        ffi::set_percentage_extend_grid_hoppe_xyz(self.inner.pin_mut(), percentage);
        Ok(())
    }
}

impl SurfaceReconstruction<PointCloudXYZ, PolygonMesh> for MarchingCubesHoppeXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_hoppe_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        let result = ffi::perform_reconstruction_hoppe_xyz(self.inner.pin_mut(), mesh.as_raw_mut());
        if result < 0 {
            return Err(PclError::ProcessingFailed {
                message: "Marching Cubes Hoppe reconstruction failed".to_string(),
            });
        }
        Ok(())
    }
}

impl Default for MarchingCubesHoppeXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default MarchingCubesHoppeXYZ")
    }
}

/// Marching Cubes surface reconstruction using RBF (Radial Basis Function) method
pub struct MarchingCubesRbfXYZ {
    inner: UniquePtr<ffi::MarchingCubesRBF_PointXYZ>,
}

impl MarchingCubesRbfXYZ {
    /// Create a new Marching Cubes RBF reconstruction instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_marching_cubes_rbf_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "MarchingCubesRBF".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the iso level for surface extraction
    pub fn set_iso_level(&mut self, iso_level: f32) -> PclResult<()> {
        ffi::set_iso_level_rbf_xyz(self.inner.pin_mut(), iso_level);
        Ok(())
    }

    /// Get the current iso level
    pub fn iso_level(&self) -> f32 {
        ffi::get_iso_level_rbf_xyz(self.inner.pin_mut())
    }

    /// Set the grid resolution for voxelization
    pub fn set_grid_resolution(&mut self, res_x: i32, res_y: i32, res_z: i32) -> PclResult<()> {
        if res_x <= 0 || res_y <= 0 || res_z <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid grid resolution",
                "res_x, res_y, res_z",
                "positive values",
                &format!("({}, {}, {})", res_x, res_y, res_z),
            ));
        }
        ffi::set_grid_resolution_rbf_xyz(self.inner.pin_mut(), res_x, res_y, res_z);
        Ok(())
    }

    /// Set the percentage to extend the grid beyond the bounding box
    pub fn set_percentage_extend_grid(&mut self, percentage: f32) -> PclResult<()> {
        if percentage < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid grid extension percentage",
                "percentage",
                "non-negative value",
                &percentage.to_string(),
            ));
        }
        ffi::set_percentage_extend_grid_rbf_xyz(self.inner.pin_mut(), percentage);
        Ok(())
    }

    /// Set the off-surface displacement for RBF interpolation
    pub fn set_off_surface_displacement(&mut self, displacement: f32) -> PclResult<()> {
        ffi::set_off_surface_displacement_rbf_xyz(self.inner.pin_mut(), displacement);
        Ok(())
    }
}

impl SurfaceReconstruction<PointCloudXYZ, PolygonMesh> for MarchingCubesRbfXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_rbf_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        let result = ffi::perform_reconstruction_rbf_xyz(self.inner.pin_mut(), mesh.as_raw_mut());
        if result < 0 {
            return Err(PclError::ProcessingFailed {
                message: "Marching Cubes RBF reconstruction failed".to_string(),
            });
        }
        Ok(())
    }
}

impl Default for MarchingCubesRbfXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default MarchingCubesRbfXYZ")
    }
}

/// Builder for MarchingCubesHoppeXYZ configuration
pub struct MarchingCubesHoppeXYZBuilder {
    iso_level: Option<f32>,
    grid_resolution: Option<(i32, i32, i32)>,
    percentage_extend_grid: Option<f32>,
}

impl MarchingCubesHoppeXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            iso_level: None,
            grid_resolution: None,
            percentage_extend_grid: None,
        }
    }

    /// Set the iso level
    pub fn iso_level(mut self, iso_level: f32) -> Self {
        self.iso_level = Some(iso_level);
        self
    }

    /// Set the grid resolution
    pub fn grid_resolution(mut self, res_x: i32, res_y: i32, res_z: i32) -> Self {
        self.grid_resolution = Some((res_x, res_y, res_z));
        self
    }

    /// Set the percentage to extend grid
    pub fn percentage_extend_grid(mut self, percentage: f32) -> Self {
        self.percentage_extend_grid = Some(percentage);
        self
    }

    /// Build the MarchingCubesHoppeXYZ instance
    pub fn build(self) -> PclResult<MarchingCubesHoppeXYZ> {
        let mut mc = MarchingCubesHoppeXYZ::new()?;

        if let Some(iso_level) = self.iso_level {
            mc.set_iso_level(iso_level)?;
        }
        if let Some((res_x, res_y, res_z)) = self.grid_resolution {
            mc.set_grid_resolution(res_x, res_y, res_z)?;
        }
        if let Some(percentage) = self.percentage_extend_grid {
            mc.set_percentage_extend_grid(percentage)?;
        }

        Ok(mc)
    }
}

impl Default for MarchingCubesHoppeXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for MarchingCubesRbfXYZ configuration
pub struct MarchingCubesRbfXYZBuilder {
    iso_level: Option<f32>,
    grid_resolution: Option<(i32, i32, i32)>,
    percentage_extend_grid: Option<f32>,
    off_surface_displacement: Option<f32>,
}

impl MarchingCubesRbfXYZBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            iso_level: None,
            grid_resolution: None,
            percentage_extend_grid: None,
            off_surface_displacement: None,
        }
    }

    /// Set the iso level
    pub fn iso_level(mut self, iso_level: f32) -> Self {
        self.iso_level = Some(iso_level);
        self
    }

    /// Set the grid resolution
    pub fn grid_resolution(mut self, res_x: i32, res_y: i32, res_z: i32) -> Self {
        self.grid_resolution = Some((res_x, res_y, res_z));
        self
    }

    /// Set the percentage to extend grid
    pub fn percentage_extend_grid(mut self, percentage: f32) -> Self {
        self.percentage_extend_grid = Some(percentage);
        self
    }

    /// Set the off-surface displacement
    pub fn off_surface_displacement(mut self, displacement: f32) -> Self {
        self.off_surface_displacement = Some(displacement);
        self
    }

    /// Build the MarchingCubesRbfXYZ instance
    pub fn build(self) -> PclResult<MarchingCubesRbfXYZ> {
        let mut mc = MarchingCubesRbfXYZ::new()?;

        if let Some(iso_level) = self.iso_level {
            mc.set_iso_level(iso_level)?;
        }
        if let Some((res_x, res_y, res_z)) = self.grid_resolution {
            mc.set_grid_resolution(res_x, res_y, res_z)?;
        }
        if let Some(percentage) = self.percentage_extend_grid {
            mc.set_percentage_extend_grid(percentage)?;
        }
        if let Some(displacement) = self.off_surface_displacement {
            mc.set_off_surface_displacement(displacement)?;
        }

        Ok(mc)
    }
}

impl Default for MarchingCubesRbfXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_marching_cubes_hoppe_creation() {
        let mc = MarchingCubesHoppeXYZ::new();
        assert!(mc.is_ok());
    }

    #[test]
    fn test_marching_cubes_rbf_creation() {
        let mc = MarchingCubesRbfXYZ::new();
        assert!(mc.is_ok());
    }

    #[test]
    fn test_marching_cubes_hoppe_builder() {
        let mc = MarchingCubesHoppeXYZBuilder::new()
            .iso_level(0.0)
            .grid_resolution(50, 50, 50)
            .percentage_extend_grid(0.1)
            .build();

        assert!(mc.is_ok());
        let mc = mc.unwrap();
        assert_eq!(mc.iso_level(), 0.0);
    }

    #[test]
    fn test_marching_cubes_rbf_builder() {
        let mc = MarchingCubesRbfXYZBuilder::new()
            .iso_level(0.0)
            .grid_resolution(50, 50, 50)
            .percentage_extend_grid(0.1)
            .off_surface_displacement(0.01)
            .build();

        assert!(mc.is_ok());
        let mc = mc.unwrap();
        assert_eq!(mc.iso_level(), 0.0);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut mc = MarchingCubesHoppeXYZ::new().unwrap();

        assert!(mc.set_grid_resolution(-1, 50, 50).is_err());
        assert!(mc.set_percentage_extend_grid(-1.0).is_err());
    }
}
