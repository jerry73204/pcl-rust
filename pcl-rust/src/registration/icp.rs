//! Iterative Closest Point (ICP) algorithm implementation
//!
//! ICP is a popular algorithm for aligning two point clouds by iteratively
//! minimizing the distance between corresponding points.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use crate::registration::{RegistrationXYZ, RegistrationXYZRGB, TransformationMatrix};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// ICP algorithm for PointXYZ clouds
pub struct IcpXYZ {
    inner: UniquePtr<ffi::IterativeClosestPoint_PointXYZ>,
}

impl IcpXYZ {
    /// Create a new ICP instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_icp_xyz();
        if inner.is_null() {
            Err(PclError::InvalidState {
                message: "Failed to create ICP instance".to_string(),
                expected_state: "valid ICP object".to_string(),
                actual_state: "null pointer".to_string(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, iterations: i32) -> PclResult<()> {
        if iterations <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid iteration count",
                "iterations",
                "positive integer",
                iterations.to_string(),
            ));
        }
        ffi::set_max_iterations_icp_xyz(self.inner.pin_mut(), iterations);
        Ok(())
    }

    /// Get the maximum number of iterations
    pub fn get_max_iterations(&mut self) -> i32 {
        ffi::get_max_iterations_icp_xyz(self.inner.pin_mut())
    }

    /// Set the transformation epsilon (convergence criterion)
    pub fn set_transformation_epsilon(&mut self, epsilon: f64) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid transformation epsilon",
                "epsilon",
                "non-negative value",
                epsilon.to_string(),
            ));
        }
        ffi::set_transformation_epsilon_icp_xyz(self.inner.pin_mut(), epsilon);
        Ok(())
    }

    /// Get the transformation epsilon
    pub fn get_transformation_epsilon(&mut self) -> f64 {
        ffi::get_transformation_epsilon_icp_xyz(self.inner.pin_mut())
    }

    /// Set the Euclidean fitness epsilon (convergence criterion)
    pub fn set_euclidean_fitness_epsilon(&mut self, epsilon: f64) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid Euclidean fitness epsilon",
                "epsilon",
                "non-negative value",
                epsilon.to_string(),
            ));
        }
        ffi::set_euclidean_fitness_epsilon_icp_xyz(self.inner.pin_mut(), epsilon);
        Ok(())
    }

    /// Get the Euclidean fitness epsilon
    pub fn get_euclidean_fitness_epsilon(&mut self) -> f64 {
        ffi::get_euclidean_fitness_epsilon_icp_xyz(self.inner.pin_mut())
    }

    /// Set the maximum correspondence distance
    pub fn set_max_correspondence_distance(&mut self, distance: f64) -> PclResult<()> {
        if distance <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid correspondence distance",
                "distance",
                "positive value",
                distance.to_string(),
            ));
        }
        ffi::set_max_correspondence_distance_icp_xyz(self.inner.pin_mut(), distance);
        Ok(())
    }

    /// Get the maximum correspondence distance
    pub fn get_max_correspondence_distance(&mut self) -> f64 {
        ffi::get_max_correspondence_distance_icp_xyz(self.inner.pin_mut())
    }
}

impl Default for IcpXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default ICP instance")
    }
}

impl RegistrationXYZ for IcpXYZ {
    fn set_input_source(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Source cloud is empty"));
        }
        ffi::set_input_source_icp_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn set_input_target(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Target cloud is empty"));
        }
        ffi::set_input_target_icp_xyz(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn align(&mut self) -> PclResult<PointCloudXYZ> {
        let result = ffi::align_icp_xyz(self.inner.pin_mut());
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "ICP alignment failed".to_string(),
                expected_state: "successful alignment".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }

    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZ> {
        let guess_vec = initial_guess.to_vec();
        let result = ffi::align_with_guess_icp_xyz(self.inner.pin_mut(), &guess_vec);
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "ICP alignment with guess failed".to_string(),
                expected_state: "successful alignment".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZ::from_unique_ptr(result))
        }
    }

    fn has_converged(&mut self) -> bool {
        ffi::has_converged_icp_xyz(self.inner.pin_mut())
    }

    fn get_fitness_score(&mut self) -> f64 {
        ffi::get_fitness_score_icp_xyz(self.inner.pin_mut())
    }

    fn get_final_transformation(&mut self) -> TransformationMatrix {
        let transform_vec = ffi::get_final_transformation_icp_xyz(self.inner.pin_mut());
        TransformationMatrix::from_vec(&transform_vec).unwrap_or_default()
    }
}

/// ICP algorithm for PointXYZRGB clouds
pub struct IcpXYZRGB {
    inner: UniquePtr<ffi::IterativeClosestPoint_PointXYZRGB>,
}

impl IcpXYZRGB {
    /// Create a new ICP instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_icp_xyzrgb();
        if inner.is_null() {
            Err(PclError::InvalidState {
                message: "Failed to create ICP instance".to_string(),
                expected_state: "valid ICP object".to_string(),
                actual_state: "null pointer".to_string(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, iterations: i32) -> PclResult<()> {
        if iterations <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid iteration count",
                "iterations",
                "positive integer",
                iterations.to_string(),
            ));
        }
        ffi::set_max_iterations_icp_xyzrgb(self.inner.pin_mut(), iterations);
        Ok(())
    }

    /// Get the maximum number of iterations
    pub fn get_max_iterations(&mut self) -> i32 {
        ffi::get_max_iterations_icp_xyzrgb(self.inner.pin_mut())
    }

    /// Set the transformation epsilon (convergence criterion)
    pub fn set_transformation_epsilon(&mut self, epsilon: f64) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid transformation epsilon",
                "epsilon",
                "non-negative value",
                epsilon.to_string(),
            ));
        }
        ffi::set_transformation_epsilon_icp_xyzrgb(self.inner.pin_mut(), epsilon);
        Ok(())
    }

    /// Get the transformation epsilon
    pub fn get_transformation_epsilon(&mut self) -> f64 {
        ffi::get_transformation_epsilon_icp_xyzrgb(self.inner.pin_mut())
    }

    /// Set the Euclidean fitness epsilon (convergence criterion)
    pub fn set_euclidean_fitness_epsilon(&mut self, epsilon: f64) -> PclResult<()> {
        if epsilon < 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid Euclidean fitness epsilon",
                "epsilon",
                "non-negative value",
                epsilon.to_string(),
            ));
        }
        ffi::set_euclidean_fitness_epsilon_icp_xyzrgb(self.inner.pin_mut(), epsilon);
        Ok(())
    }

    /// Get the Euclidean fitness epsilon
    pub fn get_euclidean_fitness_epsilon(&mut self) -> f64 {
        ffi::get_euclidean_fitness_epsilon_icp_xyzrgb(self.inner.pin_mut())
    }

    /// Set the maximum correspondence distance
    pub fn set_max_correspondence_distance(&mut self, distance: f64) -> PclResult<()> {
        if distance <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid correspondence distance",
                "distance",
                "positive value",
                distance.to_string(),
            ));
        }
        ffi::set_max_correspondence_distance_icp_xyzrgb(self.inner.pin_mut(), distance);
        Ok(())
    }

    /// Get the maximum correspondence distance
    pub fn get_max_correspondence_distance(&mut self) -> f64 {
        ffi::get_max_correspondence_distance_icp_xyzrgb(self.inner.pin_mut())
    }
}

impl Default for IcpXYZRGB {
    fn default() -> Self {
        Self::new().expect("Failed to create default ICP instance")
    }
}

impl RegistrationXYZRGB for IcpXYZRGB {
    fn set_input_source(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Source cloud is empty"));
        }
        ffi::set_input_source_icp_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn set_input_target(&mut self, cloud: &PointCloudXYZRGB) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Target cloud is empty"));
        }
        ffi::set_input_target_icp_xyzrgb(self.inner.pin_mut(), cloud.as_raw());
        Ok(())
    }

    fn align(&mut self) -> PclResult<PointCloudXYZRGB> {
        let result = ffi::align_icp_xyzrgb(self.inner.pin_mut());
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "ICP alignment failed".to_string(),
                expected_state: "successful alignment".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZRGB::from_unique_ptr(result))
        }
    }

    fn align_with_guess(
        &mut self,
        initial_guess: &TransformationMatrix,
    ) -> PclResult<PointCloudXYZRGB> {
        let guess_vec = initial_guess.to_vec();
        let result = ffi::align_with_guess_icp_xyzrgb(self.inner.pin_mut(), &guess_vec);
        if result.is_null() {
            Err(PclError::InvalidState {
                message: "ICP alignment with guess failed".to_string(),
                expected_state: "successful alignment".to_string(),
                actual_state: "null result".to_string(),
            })
        } else {
            Ok(PointCloudXYZRGB::from_unique_ptr(result))
        }
    }

    fn has_converged(&mut self) -> bool {
        ffi::has_converged_icp_xyzrgb(self.inner.pin_mut())
    }

    fn get_fitness_score(&mut self) -> f64 {
        ffi::get_fitness_score_icp_xyzrgb(self.inner.pin_mut())
    }

    fn get_final_transformation(&mut self) -> TransformationMatrix {
        let transform_vec = ffi::get_final_transformation_icp_xyzrgb(self.inner.pin_mut());
        TransformationMatrix::from_vec(&transform_vec).unwrap_or_default()
    }
}

/// Builder for ICP configuration (PointXYZ)
pub struct IcpXYZBuilder {
    max_iterations: Option<i32>,
    transformation_epsilon: Option<f64>,
    euclidean_fitness_epsilon: Option<f64>,
    max_correspondence_distance: Option<f64>,
}

impl IcpXYZBuilder {
    /// Create a new ICP builder
    pub fn new() -> Self {
        Self {
            max_iterations: None,
            transformation_epsilon: None,
            euclidean_fitness_epsilon: None,
            max_correspondence_distance: None,
        }
    }

    /// Set the maximum number of iterations
    pub fn max_iterations(mut self, iterations: i32) -> Self {
        self.max_iterations = Some(iterations);
        self
    }

    /// Set the transformation epsilon
    pub fn transformation_epsilon(mut self, epsilon: f64) -> Self {
        self.transformation_epsilon = Some(epsilon);
        self
    }

    /// Set the Euclidean fitness epsilon
    pub fn euclidean_fitness_epsilon(mut self, epsilon: f64) -> Self {
        self.euclidean_fitness_epsilon = Some(epsilon);
        self
    }

    /// Set the maximum correspondence distance
    pub fn max_correspondence_distance(mut self, distance: f64) -> Self {
        self.max_correspondence_distance = Some(distance);
        self
    }

    /// Build the ICP instance
    pub fn build(self) -> PclResult<IcpXYZ> {
        let mut icp = IcpXYZ::new()?;

        if let Some(iterations) = self.max_iterations {
            icp.set_max_iterations(iterations)?;
        }

        if let Some(epsilon) = self.transformation_epsilon {
            icp.set_transformation_epsilon(epsilon)?;
        }

        if let Some(epsilon) = self.euclidean_fitness_epsilon {
            icp.set_euclidean_fitness_epsilon(epsilon)?;
        }

        if let Some(distance) = self.max_correspondence_distance {
            icp.set_max_correspondence_distance(distance)?;
        }

        Ok(icp)
    }
}

impl Default for IcpXYZBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for ICP configuration (PointXYZRGB)
pub struct IcpXYZRGBBuilder {
    max_iterations: Option<i32>,
    transformation_epsilon: Option<f64>,
    euclidean_fitness_epsilon: Option<f64>,
    max_correspondence_distance: Option<f64>,
}

impl IcpXYZRGBBuilder {
    /// Create a new ICP builder
    pub fn new() -> Self {
        Self {
            max_iterations: None,
            transformation_epsilon: None,
            euclidean_fitness_epsilon: None,
            max_correspondence_distance: None,
        }
    }

    /// Set the maximum number of iterations
    pub fn max_iterations(mut self, iterations: i32) -> Self {
        self.max_iterations = Some(iterations);
        self
    }

    /// Set the transformation epsilon
    pub fn transformation_epsilon(mut self, epsilon: f64) -> Self {
        self.transformation_epsilon = Some(epsilon);
        self
    }

    /// Set the Euclidean fitness epsilon
    pub fn euclidean_fitness_epsilon(mut self, epsilon: f64) -> Self {
        self.euclidean_fitness_epsilon = Some(epsilon);
        self
    }

    /// Set the maximum correspondence distance
    pub fn max_correspondence_distance(mut self, distance: f64) -> Self {
        self.max_correspondence_distance = Some(distance);
        self
    }

    /// Build the ICP instance
    pub fn build(self) -> PclResult<IcpXYZRGB> {
        let mut icp = IcpXYZRGB::new()?;

        if let Some(iterations) = self.max_iterations {
            icp.set_max_iterations(iterations)?;
        }

        if let Some(epsilon) = self.transformation_epsilon {
            icp.set_transformation_epsilon(epsilon)?;
        }

        if let Some(epsilon) = self.euclidean_fitness_epsilon {
            icp.set_euclidean_fitness_epsilon(epsilon)?;
        }

        if let Some(distance) = self.max_correspondence_distance {
            icp.set_max_correspondence_distance(distance)?;
        }

        Ok(icp)
    }
}

impl Default for IcpXYZRGBBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_icp_creation() {
        let icp = IcpXYZ::new();
        assert!(icp.is_ok());
    }

    #[test]
    fn test_icp_builder() {
        let icp = IcpXYZBuilder::new()
            .max_iterations(50)
            .transformation_epsilon(1e-8)
            .euclidean_fitness_epsilon(1e-6)
            .max_correspondence_distance(0.05)
            .build();

        assert!(icp.is_ok());
        let mut icp = icp.unwrap();
        assert_eq!(icp.get_max_iterations(), 50);
        assert_eq!(icp.get_transformation_epsilon(), 1e-8);
        assert_eq!(icp.get_euclidean_fitness_epsilon(), 1e-6);
        assert_eq!(icp.get_max_correspondence_distance(), 0.05);
    }

    #[test]
    fn test_invalid_parameters() {
        let mut icp = IcpXYZ::new().unwrap();

        assert!(icp.set_max_iterations(-1).is_err());
        assert!(icp.set_transformation_epsilon(-1.0).is_err());
        assert!(icp.set_euclidean_fitness_epsilon(-1.0).is_err());
        assert!(icp.set_max_correspondence_distance(-1.0).is_err());
    }
}
