//! Sample Consensus (SAC) based segmentation
//!
//! This module provides RANSAC-based segmentation for finding geometric models
//! in point clouds such as planes, spheres, cylinders, etc.

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::segmentation::{method_types, model_types};
use cxx::UniquePtr;
use pcl_sys::ffi;

/// SAC model types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelType {
    /// Plane model
    Plane,
    /// Line model
    Line,
    /// 2D Circle model
    Circle2D,
    /// 3D Circle model
    Circle3D,
    /// Sphere model
    Sphere,
    /// Cylinder model
    Cylinder,
    /// Cone model
    Cone,
}

impl ModelType {
    /// Convert to PCL model type constant
    pub fn to_pcl_constant(self) -> i32 {
        match self {
            ModelType::Plane => model_types::SACMODEL_PLANE,
            ModelType::Line => model_types::SACMODEL_LINE,
            ModelType::Circle2D => model_types::SACMODEL_CIRCLE2D,
            ModelType::Circle3D => model_types::SACMODEL_CIRCLE3D,
            ModelType::Sphere => model_types::SACMODEL_SPHERE,
            ModelType::Cylinder => model_types::SACMODEL_CYLINDER,
            ModelType::Cone => model_types::SACMODEL_CONE,
        }
    }
}

/// SAC method types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MethodType {
    /// RANSAC (Random Sample Consensus)
    Ransac,
    /// LMedS (Least Median of Squares)
    LMedS,
    /// MSAC (M-estimator Sample Consensus)
    MSAC,
    /// RRANSAC (Randomized RANSAC)
    RRANSAC,
    /// RMSAC (Randomized MSAC)
    RMSAC,
    /// MLESAC (Maximum Likelihood Estimation Sample Consensus)
    MLESAC,
    /// PROSAC (Progressive Sample Consensus)
    PROSAC,
}

impl MethodType {
    /// Convert to PCL method type constant
    pub fn to_pcl_constant(self) -> i32 {
        match self {
            MethodType::Ransac => method_types::SAC_RANSAC,
            MethodType::LMedS => method_types::SAC_LMEDS,
            MethodType::MSAC => method_types::SAC_MSAC,
            MethodType::RRANSAC => method_types::SAC_RRANSAC,
            MethodType::RMSAC => method_types::SAC_RMSAC,
            MethodType::MLESAC => method_types::SAC_MLESAC,
            MethodType::PROSAC => method_types::SAC_PROSAC,
        }
    }
}

/// SAC segmentation result containing inliers and model coefficients
#[derive(Debug, Clone)]
pub struct SegmentationResult {
    /// Indices of inlier points
    pub inliers: Vec<i32>,
    /// Model coefficients (interpretation depends on model type)
    pub coefficients: Vec<f32>,
}

/// SAC segmentation for PointXYZ clouds
///
/// This algorithm uses sample consensus methods like RANSAC to find geometric models
/// in point clouds. It's particularly useful for finding planes, spheres, and other
/// parametric shapes in noisy data.
pub struct SacSegmentationXYZ {
    inner: UniquePtr<ffi::SACSegmentation_PointXYZ>,
}

impl SacSegmentationXYZ {
    /// Create a new SAC segmentation instance
    pub fn new() -> PclResult<Self> {
        let inner = ffi::new_sac_segmentation_xyz();
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "SACSegmentation".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Create a new instance with builder pattern
    pub fn builder() -> SacSegmentationBuilder {
        SacSegmentationBuilder::new()
    }

    /// Set the model type to segment
    pub fn set_model_type(&mut self, model_type: ModelType) -> PclResult<()> {
        ffi::set_model_type_sac_xyz(self.inner.pin_mut(), model_type.to_pcl_constant());
        Ok(())
    }

    /// Get the current model type
    pub fn model_type(&self) -> ModelType {
        let model_type = ffi::get_model_type_sac_xyz(&self.inner);
        match model_type {
            x if x == model_types::SACMODEL_PLANE => ModelType::Plane,
            x if x == model_types::SACMODEL_LINE => ModelType::Line,
            x if x == model_types::SACMODEL_CIRCLE2D => ModelType::Circle2D,
            x if x == model_types::SACMODEL_CIRCLE3D => ModelType::Circle3D,
            x if x == model_types::SACMODEL_SPHERE => ModelType::Sphere,
            x if x == model_types::SACMODEL_CYLINDER => ModelType::Cylinder,
            x if x == model_types::SACMODEL_CONE => ModelType::Cone,
            _ => ModelType::Plane, // Default fallback
        }
    }

    /// Set the method type for sample consensus
    pub fn set_method_type(&mut self, method_type: MethodType) -> PclResult<()> {
        ffi::set_method_type_sac_xyz(self.inner.pin_mut(), method_type.to_pcl_constant());
        Ok(())
    }

    /// Get the current method type
    pub fn method_type(&self) -> MethodType {
        let method_type = ffi::get_method_type_sac_xyz(&self.inner);
        match method_type {
            x if x == method_types::SAC_RANSAC => MethodType::Ransac,
            x if x == method_types::SAC_LMEDS => MethodType::LMedS,
            x if x == method_types::SAC_MSAC => MethodType::MSAC,
            x if x == method_types::SAC_RRANSAC => MethodType::RRANSAC,
            x if x == method_types::SAC_RMSAC => MethodType::RMSAC,
            x if x == method_types::SAC_MLESAC => MethodType::MLESAC,
            x if x == method_types::SAC_PROSAC => MethodType::PROSAC,
            _ => MethodType::Ransac, // Default fallback
        }
    }

    /// Set the distance threshold for inlier detection
    pub fn set_distance_threshold(&mut self, threshold: f64) -> PclResult<()> {
        if threshold <= 0.0 {
            return Err(PclError::invalid_parameters(
                "Invalid distance threshold",
                "distance_threshold",
                "positive value",
                threshold.to_string(),
            ));
        }
        ffi::set_distance_threshold_sac_xyz(self.inner.pin_mut(), threshold);
        Ok(())
    }

    /// Get the distance threshold
    pub fn distance_threshold(&self) -> f64 {
        ffi::get_distance_threshold_sac_xyz(&self.inner)
    }

    /// Set the maximum number of iterations
    pub fn set_max_iterations(&mut self, max_iterations: i32) -> PclResult<()> {
        if max_iterations <= 0 {
            return Err(PclError::invalid_parameters(
                "Invalid maximum iterations",
                "max_iterations",
                "positive value",
                max_iterations.to_string(),
            ));
        }
        ffi::set_max_iterations_sac_xyz(self.inner.pin_mut(), max_iterations);
        Ok(())
    }

    /// Get the maximum number of iterations
    pub fn max_iterations(&self) -> i32 {
        ffi::get_max_iterations_sac_xyz(&self.inner)
    }

    /// Set whether to optimize coefficients
    pub fn set_optimize_coefficients(&mut self, optimize: bool) -> PclResult<()> {
        ffi::set_optimize_coefficients_sac_xyz(self.inner.pin_mut(), optimize);
        Ok(())
    }

    /// Get whether coefficients are optimized
    pub fn optimize_coefficients(&self) -> bool {
        ffi::get_optimize_coefficients_sac_xyz(&self.inner)
    }

    /// Perform segmentation and return the result
    pub fn segment_model(&mut self) -> PclResult<SegmentationResult> {
        let mut inliers = Vec::new();
        let mut coefficients = Vec::new();

        let success = ffi::segment_sac_xyz(self.inner.pin_mut(), &mut inliers, &mut coefficients);

        if !success {
            return Err(PclError::ProcessingFailed {
                message: "SAC segmentation failed to find a model".to_string(),
            });
        }

        Ok(SegmentationResult {
            inliers,
            coefficients,
        })
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if cloud.empty() {
            return Err(PclError::invalid_point_cloud("Input cloud is empty"));
        }
        ffi::set_input_cloud_sac_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }
}

impl Default for SacSegmentationXYZ {
    fn default() -> Self {
        Self::new().expect("Failed to create default SacSegmentationXYZ")
    }
}

/// Builder for SacSegmentationXYZ
pub struct SacSegmentationBuilder {
    model_type: Option<ModelType>,
    method_type: Option<MethodType>,
    distance_threshold: Option<f64>,
    max_iterations: Option<i32>,
    optimize_coefficients: Option<bool>,
}

impl SacSegmentationBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            model_type: None,
            method_type: None,
            distance_threshold: None,
            max_iterations: None,
            optimize_coefficients: None,
        }
    }

    /// Set the model type
    pub fn model_type(mut self, model_type: ModelType) -> Self {
        self.model_type = Some(model_type);
        self
    }

    /// Set the method type
    pub fn method_type(mut self, method_type: MethodType) -> Self {
        self.method_type = Some(method_type);
        self
    }

    /// Set the distance threshold
    pub fn distance_threshold(mut self, threshold: f64) -> Self {
        self.distance_threshold = Some(threshold);
        self
    }

    /// Set the maximum iterations
    pub fn max_iterations(mut self, max_iterations: i32) -> Self {
        self.max_iterations = Some(max_iterations);
        self
    }

    /// Set whether to optimize coefficients
    pub fn optimize_coefficients(mut self, optimize: bool) -> Self {
        self.optimize_coefficients = Some(optimize);
        self
    }

    /// Build the SacSegmentation instance
    pub fn build(self) -> PclResult<SacSegmentationXYZ> {
        let mut segmentation = SacSegmentationXYZ::new()?;

        if let Some(model_type) = self.model_type {
            segmentation.set_model_type(model_type)?;
        }

        if let Some(method_type) = self.method_type {
            segmentation.set_method_type(method_type)?;
        }

        if let Some(threshold) = self.distance_threshold {
            segmentation.set_distance_threshold(threshold)?;
        }

        if let Some(max_iter) = self.max_iterations {
            segmentation.set_max_iterations(max_iter)?;
        }

        if let Some(optimize) = self.optimize_coefficients {
            segmentation.set_optimize_coefficients(optimize)?;
        }

        Ok(segmentation)
    }
}

impl Default for SacSegmentationBuilder {
    fn default() -> Self {
        Self::new()
    }
}
