//! Poisson surface reconstruction
//!
//! This module provides Poisson surface reconstruction, which is one of the most
//! robust methods for reconstructing surfaces from oriented point sets.

use crate::common::{Normal, PointCloud};
use crate::error::{PclError, PclResult};
use crate::surface::PolygonMesh;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Poisson surface reconstruction algorithm
pub struct PoissonReconstruction {
    inner: UniquePtr<ffi::Poisson_PointNormal>,
}

impl PoissonReconstruction {
    /// Create a new Poisson reconstruction instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_poisson();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "PoissonReconstruction".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the maximum depth of the tree used for reconstruction
    pub fn set_depth(&mut self, depth: i32) -> PclResult<()> {
        if depth <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid depth",
                "depth",
                "positive value",
                depth.to_string(),
            ));
        }
        ffi::set_depth_poisson(self.inner.pin_mut(), depth);
        Ok(())
    }

    /// Get the current depth
    pub fn depth(&mut self) -> i32 {
        ffi::get_depth_poisson(self.inner.pin_mut())
    }

    /// Set the minimum depth of the tree
    pub fn set_min_depth(&mut self, min_depth: i32) -> PclResult<()> {
        if min_depth < 0 {
            return Err(PclError::invalid_parameters(
                "Invalid minimum depth",
                "min_depth",
                "non-negative value",
                min_depth.to_string(),
            ));
        }
        ffi::set_min_depth_poisson(self.inner.pin_mut(), min_depth);
        Ok(())
    }

    /// Get the current minimum depth
    pub fn min_depth(&mut self) -> i32 {
        ffi::get_min_depth_poisson(self.inner.pin_mut())
    }

    /// Set the point weight for interpolation
    pub fn set_point_weight(&mut self, weight: f32) -> PclResult<()> {
        if weight < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid point weight",
                "weight",
                "non-negative value",
                weight.to_string(),
            ));
        }
        ffi::set_point_weight_poisson(self.inner.pin_mut(), weight);
        Ok(())
    }

    /// Get the current point weight
    pub fn point_weight(&mut self) -> f32 {
        ffi::get_point_weight_poisson(self.inner.pin_mut())
    }

    /// Set the scale factor
    pub fn set_scale(&mut self, scale: f32) -> PclResult<()> {
        if scale <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid scale",
                "scale",
                "positive value",
                scale.to_string(),
            ));
        }
        ffi::set_scale_poisson(self.inner.pin_mut(), scale);
        Ok(())
    }

    /// Get the current scale
    pub fn scale(&mut self) -> f32 {
        ffi::get_scale_poisson(self.inner.pin_mut())
    }

    /// Set the depth at which the solver should split the solution
    pub fn set_solver_divide(&mut self, solver_divide: i32) -> PclResult<()> {
        if solver_divide < 0 {
            return Err(PclError::invalid_parameters(
                "Invalid solver divide",
                "solver_divide",
                "non-negative value",
                solver_divide.to_string(),
            ));
        }
        ffi::set_solver_divide_poisson(self.inner.pin_mut(), solver_divide);
        Ok(())
    }

    /// Get the current solver divide
    pub fn solver_divide(&mut self) -> i32 {
        ffi::get_solver_divide_poisson(self.inner.pin_mut())
    }

    /// Set the depth at which the isosurface extraction should split
    pub fn set_iso_divide(&mut self, iso_divide: i32) -> PclResult<()> {
        if iso_divide < 0 {
            return Err(PclError::invalid_parameters(
                "Invalid iso divide",
                "iso_divide",
                "non-negative value",
                iso_divide.to_string(),
            ));
        }
        ffi::set_iso_divide_poisson(self.inner.pin_mut(), iso_divide);
        Ok(())
    }

    /// Get the current iso divide
    pub fn iso_divide(&mut self) -> i32 {
        ffi::get_iso_divide_poisson(self.inner.pin_mut())
    }

    /// Set the minimum number of sample points per octree node
    pub fn set_samples_per_node(&mut self, samples: f32) -> PclResult<()> {
        if samples <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid samples per node",
                "samples",
                "positive value",
                samples.to_string(),
            ));
        }
        ffi::set_samples_per_node_poisson(self.inner.pin_mut(), samples);
        Ok(())
    }

    /// Get the current samples per node
    pub fn samples_per_node(&mut self) -> f32 {
        ffi::get_samples_per_node_poisson(self.inner.pin_mut())
    }

    /// Set whether to use confidence weights for reconstruction
    pub fn set_confidence(&mut self, confidence: bool) {
        ffi::set_confidence_poisson(self.inner.pin_mut(), confidence);
    }

    /// Get whether confidence weights are used
    pub fn confidence(&mut self) -> bool {
        ffi::get_confidence_poisson(self.inner.pin_mut())
    }

    /// Set whether to output polygons (true) or a triangle mesh (false)
    pub fn set_output_polygons(&mut self, output_polygons: bool) {
        ffi::set_output_polygons_poisson(self.inner.pin_mut(), output_polygons);
    }

    /// Get whether polygons are output
    pub fn output_polygons(&mut self) -> bool {
        ffi::get_output_polygons_poisson(self.inner.pin_mut())
    }

    /// Set the B-spline degree
    pub fn set_degree(&mut self, degree: i32) -> PclResult<()> {
        if degree <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid degree",
                "degree",
                "positive value",
                degree.to_string(),
            ));
        }
        ffi::set_degree_poisson(self.inner.pin_mut(), degree);
        Ok(())
    }

    /// Get the current B-spline degree
    pub fn degree(&mut self) -> i32 {
        ffi::get_degree_poisson(self.inner.pin_mut())
    }

    /// Set whether to add manifold polygons to close boundaries
    pub fn set_manifold(&mut self, manifold: bool) {
        ffi::set_manifold_poisson(self.inner.pin_mut(), manifold);
    }

    /// Get whether manifold polygons are added
    pub fn manifold(&mut self) -> bool {
        ffi::get_manifold_poisson(self.inner.pin_mut())
    }

    /// Set the input point cloud with normals
    pub fn set_input_cloud(&mut self, cloud: &PointCloud<Normal>) -> PclResult<()> {
        ffi::set_input_cloud_poisson(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Perform Poisson surface reconstruction
    pub fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        let result = ffi::reconstruct_mesh_poisson(self.inner.pin_mut(), mesh.as_raw_mut());
        if result < 0 {
            return Err(PclError::ProcessingFailed {
                message: "Poisson reconstruction failed".to_string(),
            });
        }
        Ok(())
    }
}

impl Default for PoissonReconstruction {
    fn default() -> Self {
        Self::new().expect("Failed to create default PoissonReconstruction")
    }
}

impl crate::surface::SurfaceReconstruction<PointCloud<Normal>, PolygonMesh>
    for PoissonReconstruction
{
    fn set_input_cloud(&mut self, cloud: &PointCloud<Normal>) -> PclResult<()> {
        self.set_input_cloud(cloud)
    }

    fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        self.reconstruct(mesh)
    }
}

/// Builder for PoissonReconstruction configuration
pub struct PoissonReconstructionBuilder {
    depth: Option<i32>,
    min_depth: Option<i32>,
    point_weight: Option<f32>,
    scale: Option<f32>,
    solver_divide: Option<i32>,
    iso_divide: Option<i32>,
    samples_per_node: Option<f32>,
    confidence: Option<bool>,
    output_polygons: Option<bool>,
    degree: Option<i32>,
    manifold: Option<bool>,
}

impl PoissonReconstructionBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            depth: None,
            min_depth: None,
            point_weight: None,
            scale: None,
            solver_divide: None,
            iso_divide: None,
            samples_per_node: None,
            confidence: None,
            output_polygons: None,
            degree: None,
            manifold: None,
        }
    }

    /// Set the maximum depth
    pub fn depth(mut self, depth: i32) -> Self {
        self.depth = Some(depth);
        self
    }

    /// Set the minimum depth
    pub fn min_depth(mut self, min_depth: i32) -> Self {
        self.min_depth = Some(min_depth);
        self
    }

    /// Set the point weight
    pub fn point_weight(mut self, weight: f32) -> Self {
        self.point_weight = Some(weight);
        self
    }

    /// Set the scale
    pub fn scale(mut self, scale: f32) -> Self {
        self.scale = Some(scale);
        self
    }

    /// Set the solver divide
    pub fn solver_divide(mut self, solver_divide: i32) -> Self {
        self.solver_divide = Some(solver_divide);
        self
    }

    /// Set the iso divide
    pub fn iso_divide(mut self, iso_divide: i32) -> Self {
        self.iso_divide = Some(iso_divide);
        self
    }

    /// Set the samples per node
    pub fn samples_per_node(mut self, samples: f32) -> Self {
        self.samples_per_node = Some(samples);
        self
    }

    /// Set confidence usage
    pub fn confidence(mut self, confidence: bool) -> Self {
        self.confidence = Some(confidence);
        self
    }

    /// Set output polygons
    pub fn output_polygons(mut self, output_polygons: bool) -> Self {
        self.output_polygons = Some(output_polygons);
        self
    }

    /// Set the B-spline degree
    pub fn degree(mut self, degree: i32) -> Self {
        self.degree = Some(degree);
        self
    }

    /// Set manifold mode
    pub fn manifold(mut self, manifold: bool) -> Self {
        self.manifold = Some(manifold);
        self
    }

    /// Build the PoissonReconstruction instance
    pub fn build(self) -> PclResult<PoissonReconstruction> {
        let mut poisson = PoissonReconstruction::new()?;

        if let Some(depth) = self.depth {
            poisson.set_depth(depth)?;
        }
        if let Some(min_depth) = self.min_depth {
            poisson.set_min_depth(min_depth)?;
        }
        if let Some(weight) = self.point_weight {
            poisson.set_point_weight(weight)?;
        }
        if let Some(scale) = self.scale {
            poisson.set_scale(scale)?;
        }
        if let Some(solver_divide) = self.solver_divide {
            poisson.set_solver_divide(solver_divide)?;
        }
        if let Some(iso_divide) = self.iso_divide {
            poisson.set_iso_divide(iso_divide)?;
        }
        if let Some(samples) = self.samples_per_node {
            poisson.set_samples_per_node(samples)?;
        }
        if let Some(confidence) = self.confidence {
            poisson.set_confidence(confidence);
        }
        if let Some(output_polygons) = self.output_polygons {
            poisson.set_output_polygons(output_polygons);
        }
        if let Some(degree) = self.degree {
            poisson.set_degree(degree)?;
        }
        if let Some(manifold) = self.manifold {
            poisson.set_manifold(manifold);
        }

        Ok(poisson)
    }
}

impl Default for PoissonReconstructionBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_poisson_creation() {
        let poisson = PoissonReconstruction::new();
        assert!(poisson.is_ok());
    }

    #[test]
    fn test_poisson_builder() {
        let poisson = PoissonReconstructionBuilder::new()
            .depth(8)
            .min_depth(2)
            .point_weight(4.0)
            .scale(1.1)
            .confidence(true)
            .manifold(false)
            .build();

        assert!(poisson.is_ok());
        let mut poisson = poisson.unwrap();
        assert_eq!(poisson.depth(), 8);
        assert_eq!(poisson.min_depth(), 2);
        assert_eq!(poisson.point_weight(), 4.0);
        assert_eq!(poisson.scale(), 1.1);
        assert!(poisson.confidence());
        assert!(!poisson.manifold());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut poisson = PoissonReconstruction::new().unwrap();

        assert!(poisson.set_depth(-1).is_err());
        assert!(poisson.set_min_depth(-1).is_err());
        assert!(poisson.set_point_weight(-1.0).is_err());
        assert!(poisson.set_scale(-1.0).is_err());
        assert!(poisson.set_scale(0.0).is_err());
        assert!(poisson.set_solver_divide(-1).is_err());
        assert!(poisson.set_iso_divide(-1).is_err());
        assert!(poisson.set_samples_per_node(-1.0).is_err());
        assert!(poisson.set_samples_per_node(0.0).is_err());
        assert!(poisson.set_degree(-1).is_err());
        assert!(poisson.set_degree(0).is_err());
    }
}
