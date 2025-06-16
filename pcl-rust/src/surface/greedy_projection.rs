//! Greedy Projection Triangulation surface reconstruction
//!
//! This module provides the Greedy Projection Triangulation algorithm,
//! which performs triangulation on a set of points with normals.

use crate::error::{PclError, PclResult};
use crate::surface::PolygonMesh;
use crate::surface::poisson::PointCloudWithNormals;
use cxx::UniquePtr;
use pcl_sys::ffi;

/// Greedy Projection Triangulation surface reconstruction
pub struct GreedyProjectionTriangulation {
    inner: UniquePtr<ffi::GreedyProjectionTriangulation_PointNormal>,
}

impl GreedyProjectionTriangulation {
    /// Create a new Greedy Projection Triangulation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_greedy_projection_triangulation();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "GreedyProjectionTriangulation".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Set the multiplier of the nearest neighbor distance to obtain the final search radius
    pub fn set_mu(&mut self, mu: f64) -> PclResult<()> {
        if mu <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid mu parameter",
                "mu",
                "positive value",
                &mu.to_string(),
            ));
        }
        ffi::set_mu_greedy(self.inner.pin_mut(), mu);
        Ok(())
    }

    /// Get the current mu parameter
    pub fn mu(&mut self) -> f64 {
        ffi::get_mu_greedy(&self.inner)
    }

    /// Set the maximum distance between connected points
    pub fn set_search_radius(&mut self, radius: f64) -> PclResult<()> {
        if radius <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid search radius",
                "radius",
                "positive value",
                &radius.to_string(),
            ));
        }
        ffi::set_search_radius_greedy(self.inner.pin_mut(), radius);
        Ok(())
    }

    /// Get the current search radius
    pub fn search_radius(&mut self) -> f64 {
        ffi::get_search_radius_greedy(&self.inner)
    }

    /// Set the minimum angle for each triangle
    pub fn set_minimum_angle(&mut self, angle: f64) -> PclResult<()> {
        if angle < 0.0 || angle >= std::f64::consts::PI {
            return Err(PclError::invalid_parameters(
                "Invalid minimum angle",
                "angle",
                "value between 0 and PI",
                &angle.to_string(),
            ));
        }
        ffi::set_minimum_angle_greedy(self.inner.pin_mut(), angle);
        Ok(())
    }

    /// Get the current minimum angle
    pub fn minimum_angle(&mut self) -> f64 {
        ffi::get_minimum_angle_greedy(&self.inner)
    }

    /// Set the maximum angle for each triangle
    pub fn set_maximum_angle(&mut self, angle: f64) -> PclResult<()> {
        if angle <= 0.0 || angle > std::f64::consts::PI {
            return Err(PclError::invalid_parameters(
                "Invalid maximum angle",
                "angle",
                "value between 0 and PI",
                &angle.to_string(),
            ));
        }
        ffi::set_maximum_angle_greedy(self.inner.pin_mut(), angle);
        Ok(())
    }

    /// Get the current maximum angle
    pub fn maximum_angle(&mut self) -> f64 {
        ffi::get_maximum_angle_greedy(&self.inner)
    }

    /// Set the maximum number of nearest neighbors to be searched
    pub fn set_maximum_nearest_neighbors(&mut self, neighbors: i32) -> PclResult<()> {
        if neighbors <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum nearest neighbors",
                "neighbors",
                "positive value",
                &neighbors.to_string(),
            ));
        }
        ffi::set_maximum_nearest_neighbors_greedy(self.inner.pin_mut(), neighbors);
        Ok(())
    }

    /// Get the current maximum nearest neighbors
    pub fn maximum_nearest_neighbors(&mut self) -> i32 {
        ffi::get_maximum_nearest_neighbors_greedy(&self.inner)
    }

    /// Set the maximum surface angle
    pub fn set_maximum_surface_angle(&mut self, angle: f64) -> PclResult<()> {
        if angle < 0.0 || angle >= std::f64::consts::PI {
            return Err(PclError::invalid_parameters(
                "Invalid maximum surface angle",
                "angle",
                "value between 0 and PI",
                &angle.to_string(),
            ));
        }
        ffi::set_maximum_surface_angle_greedy(self.inner.pin_mut(), angle);
        Ok(())
    }

    /// Get the current maximum surface angle
    pub fn maximum_surface_angle(&mut self) -> f64 {
        ffi::get_maximum_surface_angle_greedy(&self.inner)
    }

    /// Set whether to use normal consistency check
    pub fn set_normal_consistency(&mut self, consistency: bool) {
        ffi::set_normal_consistency_greedy(self.inner.pin_mut(), consistency);
    }

    /// Get whether normal consistency check is used
    pub fn normal_consistency(&mut self) -> bool {
        ffi::get_normal_consistency_greedy(&self.inner)
    }

    /// Set whether to use consistent vertex ordering
    pub fn set_consistent_vertex_ordering(&mut self, ordering: bool) {
        ffi::set_consistent_vertex_ordering_greedy(self.inner.pin_mut(), ordering);
    }

    /// Get whether consistent vertex ordering is used
    pub fn consistent_vertex_ordering(&mut self) -> bool {
        ffi::get_consistent_vertex_ordering_greedy(&self.inner)
    }

    /// Set the input point cloud with normals
    pub fn set_input_cloud<T: PointCloudWithNormals>(&mut self, cloud: &T) -> PclResult<()> {
        if cloud.is_empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_greedy(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    /// Perform triangulation and return the resulting mesh
    pub fn reconstruct(&mut self, mesh: &mut PolygonMesh) -> PclResult<()> {
        let result = ffi::reconstruct_mesh_greedy(self.inner.pin_mut(), mesh.as_raw_mut());
        if result < 0 {
            return Err(PclError::ProcessingFailed {
                message: "Greedy Projection Triangulation failed".to_string(),
            });
        }
        Ok(())
    }
}

impl Default for GreedyProjectionTriangulation {
    fn default() -> Self {
        Self::new().expect("Failed to create default GreedyProjectionTriangulation")
    }
}

/// Builder for GreedyProjectionTriangulation configuration
pub struct GreedyProjectionTriangulationBuilder {
    mu: Option<f64>,
    search_radius: Option<f64>,
    minimum_angle: Option<f64>,
    maximum_angle: Option<f64>,
    maximum_nearest_neighbors: Option<i32>,
    maximum_surface_angle: Option<f64>,
    normal_consistency: Option<bool>,
    consistent_vertex_ordering: Option<bool>,
}

impl GreedyProjectionTriangulationBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            mu: None,
            search_radius: None,
            minimum_angle: None,
            maximum_angle: None,
            maximum_nearest_neighbors: None,
            maximum_surface_angle: None,
            normal_consistency: None,
            consistent_vertex_ordering: None,
        }
    }

    /// Set the mu parameter
    pub fn mu(mut self, mu: f64) -> Self {
        self.mu = Some(mu);
        self
    }

    /// Set the search radius
    pub fn search_radius(mut self, radius: f64) -> Self {
        self.search_radius = Some(radius);
        self
    }

    /// Set the minimum angle (in radians)
    pub fn minimum_angle(mut self, angle: f64) -> Self {
        self.minimum_angle = Some(angle);
        self
    }

    /// Set the maximum angle (in radians)
    pub fn maximum_angle(mut self, angle: f64) -> Self {
        self.maximum_angle = Some(angle);
        self
    }

    /// Set the maximum nearest neighbors
    pub fn maximum_nearest_neighbors(mut self, neighbors: i32) -> Self {
        self.maximum_nearest_neighbors = Some(neighbors);
        self
    }

    /// Set the maximum surface angle (in radians)
    pub fn maximum_surface_angle(mut self, angle: f64) -> Self {
        self.maximum_surface_angle = Some(angle);
        self
    }

    /// Set normal consistency
    pub fn normal_consistency(mut self, consistency: bool) -> Self {
        self.normal_consistency = Some(consistency);
        self
    }

    /// Set consistent vertex ordering
    pub fn consistent_vertex_ordering(mut self, ordering: bool) -> Self {
        self.consistent_vertex_ordering = Some(ordering);
        self
    }

    /// Build the GreedyProjectionTriangulation instance
    pub fn build(self) -> PclResult<GreedyProjectionTriangulation> {
        let mut gp3 = GreedyProjectionTriangulation::new()?;

        if let Some(mu) = self.mu {
            gp3.set_mu(mu)?;
        }
        if let Some(radius) = self.search_radius {
            gp3.set_search_radius(radius)?;
        }
        if let Some(angle) = self.minimum_angle {
            gp3.set_minimum_angle(angle)?;
        }
        if let Some(angle) = self.maximum_angle {
            gp3.set_maximum_angle(angle)?;
        }
        if let Some(neighbors) = self.maximum_nearest_neighbors {
            gp3.set_maximum_nearest_neighbors(neighbors)?;
        }
        if let Some(angle) = self.maximum_surface_angle {
            gp3.set_maximum_surface_angle(angle)?;
        }
        if let Some(consistency) = self.normal_consistency {
            gp3.set_normal_consistency(consistency);
        }
        if let Some(ordering) = self.consistent_vertex_ordering {
            gp3.set_consistent_vertex_ordering(ordering);
        }

        Ok(gp3)
    }
}

impl Default for GreedyProjectionTriangulationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_greedy_projection_creation() {
        let gp3 = GreedyProjectionTriangulation::new();
        assert!(gp3.is_ok());
    }

    #[test]
    fn test_greedy_projection_builder() {
        let gp3 = GreedyProjectionTriangulationBuilder::new()
            .mu(2.5)
            .search_radius(0.025)
            .minimum_angle(std::f64::consts::PI / 18.0) // 10 degrees
            .maximum_angle(2.0 * std::f64::consts::PI / 3.0) // 120 degrees
            .maximum_nearest_neighbors(100)
            .maximum_surface_angle(std::f64::consts::PI / 4.0) // 45 degrees
            .normal_consistency(false)
            .consistent_vertex_ordering(true)
            .build();

        assert!(gp3.is_ok());
        let mut gp3 = gp3.unwrap();
        assert_eq!(gp3.mu(), 2.5);
        assert_eq!(gp3.search_radius(), 0.025);
        assert!(!gp3.normal_consistency());
        assert!(gp3.consistent_vertex_ordering());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut gp3 = GreedyProjectionTriangulation::new().unwrap();

        assert!(gp3.set_mu(-1.0).is_err());
        assert!(gp3.set_mu(0.0).is_err());
        assert!(gp3.set_search_radius(-1.0).is_err());
        assert!(gp3.set_search_radius(0.0).is_err());
        assert!(gp3.set_minimum_angle(-1.0).is_err());
        assert!(gp3.set_minimum_angle(std::f64::consts::PI).is_err());
        assert!(gp3.set_maximum_angle(-1.0).is_err());
        assert!(gp3.set_maximum_angle(std::f64::consts::PI + 0.1).is_err());
        assert!(gp3.set_maximum_nearest_neighbors(-1).is_err());
        assert!(gp3.set_maximum_nearest_neighbors(0).is_err());
        assert!(gp3.set_maximum_surface_angle(-1.0).is_err());
        assert!(gp3.set_maximum_surface_angle(std::f64::consts::PI).is_err());
    }
}
