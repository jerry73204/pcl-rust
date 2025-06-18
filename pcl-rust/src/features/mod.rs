//! Feature extraction algorithms for point clouds
//!
//! This module provides safe Rust interfaces for PCL's feature extraction algorithms,
//! including normal estimation and various feature descriptors like FPFH and PFH.
//!
//! ## Current Status
//!
//! **FFI Status**: Complete declarations - C++ implementations exist in features.cpp and
//! all FFI function declarations are present in functions.h and lib.rs
//!
//! **Rust API Status**: Complete structure with todo!() placeholders - All types and methods
//! are properly structured but have linkage issues at the cxx bridge level
//!
//! ## Available Features
//! - Normal estimation (single-threaded and OpenMP parallel)
//! - FPFH (Fast Point Feature Histogram) estimation
//! - PFH (Point Feature Histogram) estimation
//! - Feature cloud access and manipulation
//! - Builder patterns for easy configuration
//! - Quick functions for common workflows
//!
//! ## Usage Examples
//!
//! ```rust,no_run
//! use pcl::features::{NormalEstimation, FpfhEstimation};
//! use pcl::common::PointCloudXYZ;
//!
//! // Note: These examples show the intended API once FFI linkage is resolved
//! fn estimate_features(cloud: &PointCloudXYZ) -> pcl::error::PclResult<()> {
//!     // Estimate normals
//!     let mut normal_est = NormalEstimation::new()?;
//!     normal_est.set_input_cloud(cloud)?;
//!     normal_est.set_radius_search(0.03);
//!     let normals = normal_est.compute()?;
//!
//!     // Estimate FPFH features
//!     let mut fpfh_est = FpfhEstimation::new()?;
//!     fpfh_est.set_input_cloud(cloud)?;
//!     fpfh_est.set_input_normals(&normals)?;
//!     fpfh_est.set_radius_search(0.05);
//!     let features = fpfh_est.compute()?;
//!
//!     Ok(())
//! }
//! ```

use crate::common::PointCloudXYZ;
use crate::error::{PclError, PclResult};
use crate::search::KdTreeXYZ;

/// Normal vector with curvature information
#[derive(Debug, Clone)]
pub struct Normal {
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
    pub curvature: f32,
}

impl Normal {
    /// Create a new Normal from components
    pub fn new(normal_x: f32, normal_y: f32, normal_z: f32, curvature: f32) -> Self {
        Self {
            normal_x,
            normal_y,
            normal_z,
            curvature,
        }
    }

    /// Create a new Normal with zero values
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }

    /// Get the normal vector as a 3D array
    pub fn vector(&self) -> [f32; 3] {
        [self.normal_x, self.normal_y, self.normal_z]
    }

    /// Get the magnitude of the normal vector
    pub fn magnitude(&self) -> f32 {
        (self.normal_x * self.normal_x
            + self.normal_y * self.normal_y
            + self.normal_z * self.normal_z)
            .sqrt()
    }

    /// Normalize the normal vector
    pub fn normalize(&mut self) {
        let mag = self.magnitude();
        if mag > 0.0 {
            self.normal_x /= mag;
            self.normal_y /= mag;
            self.normal_z /= mag;
        }
    }

    /// Get a normalized copy of this normal
    pub fn normalized(&self) -> Self {
        let mut normal = *self;
        normal.normalize();
        normal
    }

    /// Check if this normal is valid (non-zero and finite)
    pub fn is_valid(&self) -> bool {
        self.normal_x.is_finite()
            && self.normal_y.is_finite()
            && self.normal_z.is_finite()
            && self.curvature.is_finite()
            && self.magnitude() > f32::EPSILON
    }

    /// Compute the dot product with another normal
    pub fn dot(&self, other: &Normal) -> f32 {
        self.normal_x * other.normal_x
            + self.normal_y * other.normal_y
            + self.normal_z * other.normal_z
    }

    /// Compute the angle with another normal in radians
    pub fn angle_with(&self, other: &Normal) -> f32 {
        let dot = self.dot(other);
        let mag_product = self.magnitude() * other.magnitude();
        if mag_product > f32::EPSILON {
            (dot / mag_product).clamp(-1.0, 1.0).acos()
        } else {
            0.0
        }
    }
}

impl Default for Normal {
    fn default() -> Self {
        Self::zero()
    }
}

impl Copy for Normal {}

impl From<[f32; 4]> for Normal {
    fn from(array: [f32; 4]) -> Self {
        Self::new(array[0], array[1], array[2], array[3])
    }
}

impl From<Normal> for [f32; 4] {
    fn from(normal: Normal) -> Self {
        [
            normal.normal_x,
            normal.normal_y,
            normal.normal_z,
            normal.curvature,
        ]
    }
}

/// Point cloud of normal vectors  
pub struct NormalCloud {
    inner: cxx::UniquePtr<pcl_sys::ffi::PointCloud_Normal>,
}

impl Default for NormalCloud {
    fn default() -> Self {
        Self::new()
    }
}

impl NormalCloud {
    /// Create a new empty normal cloud
    /// Note: This creates a null pointer since we can't create empty clouds directly
    /// Use NormalEstimation::compute() to get actual normal clouds
    pub fn new() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }

    /// Get the size of the normal cloud
    pub fn size(&self) -> usize {
        if self.inner.is_null() {
            0
        } else {
            pcl_sys::ffi::size_normal(self.inner.as_ref().unwrap())
        }
    }

    /// Check if the normal cloud is empty
    pub fn is_empty(&self) -> bool {
        if self.inner.is_null() {
            true
        } else {
            pcl_sys::ffi::empty_normal(self.inner.as_ref().unwrap())
        }
    }

    /// Get a normal at the specified index
    pub fn get_normal(&self, index: usize) -> PclResult<Normal> {
        if self.inner.is_null() {
            return Err(PclError::invalid_point_cloud("Normal cloud is null"));
        }
        if index >= self.size() {
            return Err(PclError::InvalidParameter {
                param: "index".into(),
                message: format!(
                    "Index {} out of bounds for cloud size {}",
                    index,
                    self.size()
                ),
            });
        }
        let normal_data = pcl_sys::ffi::get_normal_at(self.inner.as_ref().unwrap(), index);
        if normal_data.len() >= 4 {
            Ok(Normal::new(
                normal_data[0],
                normal_data[1],
                normal_data[2],
                normal_data[3],
            ))
        } else {
            Err(PclError::ComputationFailed(
                "Invalid normal data returned".into(),
            ))
        }
    }
}

/// FPFH (Fast Point Feature Histogram) signature
#[derive(Debug, Clone)]
pub struct FpfhSignature {
    pub histogram: [f32; 33],
}

impl FpfhSignature {
    /// Create a new FPFH signature
    pub fn new() -> Self {
        Self {
            histogram: [0.0; 33],
        }
    }

    /// Get the histogram as a slice
    pub fn histogram(&self) -> &[f32] {
        &self.histogram
    }
}

impl Default for FpfhSignature {
    fn default() -> Self {
        Self::new()
    }
}

/// Point cloud of FPFH signatures
pub struct FpfhCloud {
    inner: cxx::UniquePtr<pcl_sys::ffi::PointCloud_FPFHSignature33>,
}

impl Default for FpfhCloud {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for PfhCloud {
    fn default() -> Self {
        Self::new()
    }
}

impl FpfhCloud {
    /// Create a new empty FPFH cloud
    /// Note: This creates a null pointer since we can't create empty clouds directly
    /// Use FpfhEstimation::compute() to get actual FPFH clouds
    pub fn new() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }

    /// Get the size of the FPFH cloud
    pub fn size(&self) -> usize {
        if self.inner.is_null() {
            0
        } else {
            pcl_sys::ffi::size_fpfh(self.inner.as_ref().unwrap())
        }
    }

    /// Check if the FPFH cloud is empty
    pub fn is_empty(&self) -> bool {
        if self.inner.is_null() {
            true
        } else {
            pcl_sys::ffi::empty_fpfh(self.inner.as_ref().unwrap())
        }
    }

    /// Get an FPFH signature at the specified index
    pub fn get_signature(&self, index: usize) -> PclResult<FpfhSignature> {
        if self.inner.is_null() {
            return Err(PclError::invalid_point_cloud("FPFH cloud is null"));
        }
        if index >= self.size() {
            return Err(PclError::InvalidParameter {
                param: "index".into(),
                message: format!(
                    "Index {} out of bounds for cloud size {}",
                    index,
                    self.size()
                ),
            });
        }
        let histogram_data =
            pcl_sys::ffi::get_fpfh_signature_at(self.inner.as_ref().unwrap(), index);
        if histogram_data.len() >= 33 {
            let mut histogram = [0.0f32; 33];
            histogram.copy_from_slice(&histogram_data[0..33]);
            Ok(FpfhSignature { histogram })
        } else {
            Err(PclError::ComputationFailed(
                "Invalid FPFH signature data returned".into(),
            ))
        }
    }

    /// Get the histogram data for all signatures
    pub fn get_histograms(&self) -> PclResult<Vec<[f32; 33]>> {
        if self.inner.is_null() {
            return Err(PclError::invalid_point_cloud("FPFH cloud is null"));
        }
        let size = self.size();
        let mut result = Vec::with_capacity(size);
        for i in 0..size {
            let signature = self.get_signature(i)?;
            result.push(signature.histogram);
        }
        Ok(result)
    }
}

/// PFH (Point Feature Histogram) signature
#[derive(Debug, Clone)]
pub struct PfhSignature {
    pub histogram: [f32; 125],
}

impl PfhSignature {
    /// Create a new PFH signature
    pub fn new() -> Self {
        Self {
            histogram: [0.0; 125],
        }
    }

    /// Get the histogram as a slice
    pub fn histogram(&self) -> &[f32] {
        &self.histogram
    }
}

impl Default for PfhSignature {
    fn default() -> Self {
        Self::new()
    }
}

/// Point cloud of PFH signatures
pub struct PfhCloud {
    inner: cxx::UniquePtr<pcl_sys::ffi::PointCloud_PFHSignature125>,
}

impl PfhCloud {
    /// Create a new empty PFH cloud
    /// Note: This creates a null pointer since we can't create empty clouds directly
    /// Use PfhEstimation::compute() to get actual PFH clouds
    pub fn new() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }

    /// Get the size of the PFH cloud
    pub fn size(&self) -> usize {
        if self.inner.is_null() {
            0
        } else {
            pcl_sys::ffi::size_pfh(self.inner.as_ref().unwrap())
        }
    }

    /// Check if the PFH cloud is empty
    pub fn is_empty(&self) -> bool {
        if self.inner.is_null() {
            true
        } else {
            pcl_sys::ffi::empty_pfh(self.inner.as_ref().unwrap())
        }
    }

    /// Get a PFH signature at the specified index
    pub fn get_signature(&self, index: usize) -> PclResult<PfhSignature> {
        if self.inner.is_null() {
            return Err(PclError::invalid_point_cloud("PFH cloud is null"));
        }
        if index >= self.size() {
            return Err(PclError::InvalidParameter {
                param: "index".into(),
                message: format!(
                    "Index {} out of bounds for cloud size {}",
                    index,
                    self.size()
                ),
            });
        }
        let histogram_data =
            pcl_sys::ffi::get_pfh_signature_at(self.inner.as_ref().unwrap(), index);
        if histogram_data.len() >= 125 {
            let mut histogram = [0.0f32; 125];
            histogram.copy_from_slice(&histogram_data[0..125]);
            Ok(PfhSignature { histogram })
        } else {
            Err(PclError::ComputationFailed(
                "Invalid PFH signature data returned".into(),
            ))
        }
    }

    /// Get the histogram data for all signatures
    pub fn get_histograms(&self) -> PclResult<Vec<[f32; 125]>> {
        if self.inner.is_null() {
            return Err(PclError::invalid_point_cloud("PFH cloud is null"));
        }
        let size = self.size();
        let mut result = Vec::with_capacity(size);
        for i in 0..size {
            let signature = self.get_signature(i)?;
            result.push(signature.histogram);
        }
        Ok(result)
    }
}

/// Normal estimation algorithm
pub struct NormalEstimation {
    inner: cxx::UniquePtr<pcl_sys::ffi::NormalEstimation_PointXYZ_Normal>,
}

impl NormalEstimation {
    /// Create a new normal estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = pcl_sys::ffi::new_normal_estimation_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "NormalEstimation".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "NormalEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_input_cloud_normal_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the search method
    pub fn set_search_method(&mut self, tree: &KdTreeXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "NormalEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_search_method_normal_xyz(self.inner.pin_mut(), tree.inner());
        Ok(())
    }

    /// Set the radius for neighborhood search
    pub fn set_radius_search(&mut self, radius: f64) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_radius_search_normal_xyz(self.inner.pin_mut(), radius);
        }
    }

    /// Get the radius for neighborhood search
    pub fn get_radius_search(&mut self) -> f64 {
        if self.inner.is_null() {
            0.0
        } else {
            pcl_sys::ffi::get_radius_search_normal_xyz(self.inner.pin_mut())
        }
    }

    /// Set the number of nearest neighbors for search
    pub fn set_k_search(&mut self, k: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_k_search_normal_xyz(self.inner.pin_mut(), k);
        }
    }

    /// Get the number of nearest neighbors for search
    pub fn get_k_search(&mut self) -> i32 {
        if self.inner.is_null() {
            0
        } else {
            pcl_sys::ffi::get_k_search_normal_xyz(self.inner.pin_mut())
        }
    }

    /// Set the viewpoint for normal orientation
    pub fn set_view_point(&mut self, vpx: f32, vpy: f32, vpz: f32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_view_point_normal_xyz(self.inner.pin_mut(), vpx, vpy, vpz);
        }
    }

    /// Get the viewpoint for normal orientation
    pub fn get_view_point(&mut self) -> [f32; 3] {
        if self.inner.is_null() {
            [0.0, 0.0, 0.0]
        } else {
            let view_point = pcl_sys::ffi::get_view_point_normal_xyz(self.inner.pin_mut());
            if view_point.len() >= 3 {
                [view_point[0], view_point[1], view_point[2]]
            } else {
                [0.0, 0.0, 0.0]
            }
        }
    }

    /// Set whether to use sensor origin as viewpoint
    pub fn set_use_sensor_origin(&mut self, use_sensor_origin: bool) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_use_sensor_origin_normal_xyz(self.inner.pin_mut(), use_sensor_origin);
        }
    }

    /// Compute normals for the input cloud
    pub fn compute(&mut self) -> PclResult<NormalCloud> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "NormalEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        let normals = pcl_sys::ffi::compute_normals_xyz(self.inner.pin_mut());
        if normals.is_null() {
            Err(PclError::ComputationFailed(
                "Failed to compute normals".into(),
            ))
        } else {
            Ok(NormalCloud { inner: normals })
        }
    }
}

// Note: Default is not implemented for NormalEstimation because the constructor
// can fail with FFI errors. Use NormalEstimation::new() instead.

/// OpenMP-accelerated normal estimation algorithm
pub struct NormalEstimationOmp {
    inner: cxx::UniquePtr<pcl_sys::ffi::NormalEstimationOMP_PointXYZ_Normal>,
}

impl NormalEstimationOmp {
    /// Create a new OpenMP normal estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = pcl_sys::ffi::new_normal_estimation_omp_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "NormalEstimationOMP".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "NormalEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_input_cloud_normal_omp_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the number of threads for parallel computation  
    pub fn set_number_of_threads(&mut self, threads: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_number_of_threads_normal_omp_xyz(self.inner.pin_mut(), threads);
        }
    }

    /// Get the number of threads for parallel computation
    pub fn get_number_of_threads(&mut self) -> i32 {
        if self.inner.is_null() {
            0
        } else {
            pcl_sys::ffi::get_number_of_threads_normal_omp_xyz(self.inner.pin_mut())
        }
    }

    /// Set the radius for neighborhood search
    pub fn set_radius_search(&mut self, radius: f64) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_radius_search_normal_omp_xyz(self.inner.pin_mut(), radius);
        }
    }

    /// Set the number of nearest neighbors for search
    pub fn set_k_search(&mut self, k: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_k_search_normal_omp_xyz(self.inner.pin_mut(), k);
        }
    }

    /// Compute normals for the input cloud
    pub fn compute(&mut self) -> PclResult<NormalCloud> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "NormalEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        let normals = pcl_sys::ffi::compute_normals_omp_xyz(self.inner.pin_mut());
        if normals.is_null() {
            Err(PclError::ComputationFailed(
                "Failed to compute normals with OpenMP".into(),
            ))
        } else {
            Ok(NormalCloud { inner: normals })
        }
    }
}

impl Default for NormalEstimationOmp {
    fn default() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }
}

/// FPFH (Fast Point Feature Histogram) feature estimation
pub struct FpfhEstimation {
    inner: cxx::UniquePtr<pcl_sys::ffi::FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
}

impl FpfhEstimation {
    /// Create a new FPFH estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = pcl_sys::ffi::new_fpfh_estimation_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "FPFHEstimation".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the radius for neighborhood search
    pub fn set_radius_search(&mut self, radius: f64) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_radius_search_fpfh_xyz(self.inner.pin_mut(), radius);
        }
    }

    /// Set the number of nearest neighbors for search
    pub fn set_k_search(&mut self, k: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_k_search_fpfh_xyz(self.inner.pin_mut(), k);
        }
    }

    /// Set the input normal cloud
    pub fn set_input_normals(&mut self, normals: &NormalCloud) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        if normals.inner.is_null() {
            return Err(PclError::InvalidParameter {
                param: "normals".into(),
                message: "Normal cloud is null".into(),
            });
        }
        pcl_sys::ffi::set_input_normals_fpfh_xyz(
            self.inner.pin_mut(),
            normals.inner.as_ref().unwrap(),
        );
        Ok(())
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_input_cloud_fpfh_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the search method
    pub fn set_search_method(&mut self, tree: &KdTreeXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_search_method_fpfh_xyz(self.inner.pin_mut(), tree.inner());
        Ok(())
    }

    /// Compute FPFH features for the input cloud
    pub fn compute(&mut self) -> PclResult<FpfhCloud> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        let features = pcl_sys::ffi::compute_fpfh_xyz(self.inner.pin_mut());
        if features.is_null() {
            Err(PclError::ComputationFailed(
                "Failed to compute FPFH features".into(),
            ))
        } else {
            Ok(FpfhCloud { inner: features })
        }
    }
}

impl Default for FpfhEstimation {
    fn default() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }
}

/// OpenMP-accelerated FPFH feature estimation
pub struct FpfhEstimationOmp {
    inner: cxx::UniquePtr<pcl_sys::ffi::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
}

impl FpfhEstimationOmp {
    /// Create a new OpenMP FPFH estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = pcl_sys::ffi::new_fpfh_estimation_omp_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "FPFHEstimationOMP".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the number of threads for parallel computation
    pub fn set_number_of_threads(&mut self, threads: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_number_of_threads_fpfh_omp_xyz(self.inner.pin_mut(), threads);
        }
    }

    /// Set the radius for neighborhood search
    pub fn set_radius_search(&mut self, radius: f64) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_radius_search_fpfh_omp_xyz(self.inner.pin_mut(), radius);
        }
    }

    /// Set the input normal cloud
    pub fn set_input_normals(&mut self, normals: &NormalCloud) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        if normals.inner.is_null() {
            return Err(PclError::InvalidParameter {
                param: "normals".into(),
                message: "Normal cloud is null".into(),
            });
        }
        pcl_sys::ffi::set_input_normals_fpfh_omp_xyz(
            self.inner.pin_mut(),
            normals.inner.as_ref().unwrap(),
        );
        Ok(())
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_input_cloud_fpfh_omp_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the search method
    pub fn set_search_method(&mut self, tree: &KdTreeXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_search_method_fpfh_omp_xyz(self.inner.pin_mut(), tree.inner());
        Ok(())
    }

    /// Compute FPFH features for the input cloud
    pub fn compute(&mut self) -> PclResult<FpfhCloud> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "FPFHEstimationOMP not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        let features = pcl_sys::ffi::compute_fpfh_omp_xyz(self.inner.pin_mut());
        if features.is_null() {
            Err(PclError::ComputationFailed(
                "Failed to compute FPFH features with OpenMP".into(),
            ))
        } else {
            Ok(FpfhCloud { inner: features })
        }
    }
}

impl Default for FpfhEstimationOmp {
    fn default() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }
}

/// PFH (Point Feature Histogram) feature estimation
pub struct PfhEstimation {
    inner: cxx::UniquePtr<pcl_sys::ffi::PFHEstimation_PointXYZ_Normal_PFHSignature125>,
}

impl PfhEstimation {
    /// Create a new PFH estimation instance
    pub fn new() -> PclResult<Self> {
        let inner = pcl_sys::ffi::new_pfh_estimation_xyz();
        if inner.is_null() {
            Err(PclError::CreationFailed {
                typename: "PFHEstimation".into(),
            })
        } else {
            Ok(Self { inner })
        }
    }

    /// Set the radius for neighborhood search
    pub fn set_radius_search(&mut self, radius: f64) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_radius_search_pfh_xyz(self.inner.pin_mut(), radius);
        }
    }

    /// Set the number of nearest neighbors for search
    pub fn set_k_search(&mut self, k: i32) {
        if !self.inner.is_null() {
            pcl_sys::ffi::set_k_search_pfh_xyz(self.inner.pin_mut(), k);
        }
    }

    /// Set the input normal cloud
    pub fn set_input_normals(&mut self, normals: &NormalCloud) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "PFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        if normals.inner.is_null() {
            return Err(PclError::InvalidParameter {
                param: "normals".into(),
                message: "Normal cloud is null".into(),
            });
        }
        pcl_sys::ffi::set_input_normals_pfh_xyz(
            self.inner.pin_mut(),
            normals.inner.as_ref().unwrap(),
        );
        Ok(())
    }

    /// Set the input point cloud
    pub fn set_input_cloud(&mut self, cloud: &PointCloudXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "PFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_input_cloud_pfh_xyz(self.inner.pin_mut(), cloud.inner());
        Ok(())
    }

    /// Set the search method
    pub fn set_search_method(&mut self, tree: &KdTreeXYZ) -> PclResult<()> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "PFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        pcl_sys::ffi::set_search_method_pfh_xyz(self.inner.pin_mut(), tree.inner());
        Ok(())
    }

    /// Compute PFH features for the input cloud
    pub fn compute(&mut self) -> PclResult<PfhCloud> {
        if self.inner.is_null() {
            return Err(PclError::InvalidState {
                message: "PFHEstimation not initialized".into(),
                expected_state: "initialized".into(),
                actual_state: "null".into(),
            });
        }
        let features = pcl_sys::ffi::compute_pfh_xyz(self.inner.pin_mut());
        if features.is_null() {
            Err(PclError::ComputationFailed(
                "Failed to compute PFH features".into(),
            ))
        } else {
            Ok(PfhCloud { inner: features })
        }
    }
}

impl Default for PfhEstimation {
    fn default() -> Self {
        Self {
            inner: cxx::UniquePtr::null(),
        }
    }
}

/// Helper functions for feature extraction
pub mod helpers {
    use super::*;

    /// Extract histogram data from FPFH signature
    pub fn extract_fpfh_histogram(signature: &FpfhSignature) -> [f32; 33] {
        signature.histogram
    }

    /// Extract histogram data from PFH signature
    pub fn extract_pfh_histogram(signature: &PfhSignature) -> [f32; 125] {
        signature.histogram
    }

    /// Extract normal vector data
    pub fn extract_normal_vector(normal: &Normal) -> Normal {
        *normal
    }
}

/// Quick functions for common feature extraction tasks
pub mod quick {
    use super::*;

    /// Estimate normals for a point cloud with default parameters
    pub fn estimate_normals(cloud: &PointCloudXYZ, radius: f64) -> PclResult<NormalCloud> {
        let mut normal_est = NormalEstimation::new()?;
        normal_est.set_input_cloud(cloud)?;
        normal_est.set_radius_search(radius);
        normal_est.compute()
    }

    /// Estimate normals using OpenMP with default parameters
    pub fn estimate_normals_omp(
        cloud: &PointCloudXYZ,
        radius: f64,
        threads: i32,
    ) -> PclResult<NormalCloud> {
        let mut normal_est = NormalEstimationOmp::new()?;
        normal_est.set_input_cloud(cloud)?;
        normal_est.set_radius_search(radius);
        normal_est.set_number_of_threads(threads);
        normal_est.compute()
    }

    /// Compute FPFH features with default parameters
    pub fn compute_fpfh(
        cloud: &PointCloudXYZ,
        normals: &NormalCloud,
        radius: f64,
    ) -> PclResult<FpfhCloud> {
        let mut fpfh_est = FpfhEstimation::new()?;
        fpfh_est.set_input_cloud(cloud)?;
        fpfh_est.set_input_normals(normals)?;
        fpfh_est.set_radius_search(radius);
        fpfh_est.compute()
    }

    /// Compute PFH features with default parameters
    pub fn compute_pfh(
        cloud: &PointCloudXYZ,
        normals: &NormalCloud,
        radius: f64,
    ) -> PclResult<PfhCloud> {
        let mut pfh_est = PfhEstimation::new()?;
        pfh_est.set_input_cloud(cloud)?;
        pfh_est.set_input_normals(normals)?;
        pfh_est.set_radius_search(radius);
        pfh_est.compute()
    }

    /// Estimate normals with k-nearest neighbors search
    pub fn estimate_normals_k(cloud: &PointCloudXYZ, k: i32) -> PclResult<NormalCloud> {
        let mut normal_est = NormalEstimation::new()?;
        normal_est.set_input_cloud(cloud)?;
        normal_est.set_k_search(k);
        normal_est.compute()
    }

    /// Complete feature pipeline: normals + FPFH with default parameters
    pub fn compute_fpfh_pipeline(
        cloud: &PointCloudXYZ,
        normal_radius: f64,
        fpfh_radius: f64,
    ) -> PclResult<(NormalCloud, FpfhCloud)> {
        let normals = estimate_normals(cloud, normal_radius)?;
        let features = compute_fpfh(cloud, &normals, fpfh_radius)?;
        Ok((normals, features))
    }

    /// Complete feature pipeline: normals + PFH with default parameters
    pub fn compute_pfh_pipeline(
        cloud: &PointCloudXYZ,
        normal_radius: f64,
        pfh_radius: f64,
    ) -> PclResult<(NormalCloud, PfhCloud)> {
        let normals = estimate_normals(cloud, normal_radius)?;
        let features = compute_pfh(cloud, &normals, pfh_radius)?;
        Ok((normals, features))
    }
}

/// Builder patterns for feature estimation
pub mod builders {
    use super::*;

    /// Builder for normal estimation with configurable parameters
    pub struct NormalEstimationBuilder {
        radius: Option<f64>,
        k: Option<i32>,
        view_point: Option<[f32; 3]>,
        use_sensor_origin: bool,
        use_omp: bool,
        num_threads: Option<i32>,
    }

    impl NormalEstimationBuilder {
        /// Create a new normal estimation builder
        pub fn new() -> Self {
            Self {
                radius: None,
                k: None,
                view_point: None,
                use_sensor_origin: false,
                use_omp: false,
                num_threads: None,
            }
        }

        /// Set the radius for neighborhood search
        pub fn radius(mut self, radius: f64) -> Self {
            self.radius = Some(radius);
            self
        }

        /// Set the number of nearest neighbors for search
        pub fn k_search(mut self, k: i32) -> Self {
            self.k = Some(k);
            self
        }

        /// Set the viewpoint for normal orientation
        pub fn view_point(mut self, x: f32, y: f32, z: f32) -> Self {
            self.view_point = Some([x, y, z]);
            self
        }

        /// Use sensor origin as viewpoint
        pub fn use_sensor_origin(mut self) -> Self {
            self.use_sensor_origin = true;
            self
        }

        /// Enable OpenMP acceleration
        pub fn with_omp(mut self, threads: Option<i32>) -> Self {
            self.use_omp = true;
            self.num_threads = threads;
            self
        }

        /// Build and compute normals for the given point cloud
        pub fn compute(&self, cloud: &PointCloudXYZ) -> PclResult<NormalCloud> {
            if self.use_omp {
                let mut est = NormalEstimationOmp::new()?;
                est.set_input_cloud(cloud)?;

                if let Some(radius) = self.radius {
                    est.set_radius_search(radius);
                }
                if let Some(k) = self.k {
                    est.set_k_search(k);
                }
                if let Some(threads) = self.num_threads {
                    est.set_number_of_threads(threads);
                }

                est.compute()
            } else {
                let mut est = NormalEstimation::new()?;
                est.set_input_cloud(cloud)?;

                if let Some(radius) = self.radius {
                    est.set_radius_search(radius);
                }
                if let Some(k) = self.k {
                    est.set_k_search(k);
                }
                if let Some(vp) = self.view_point {
                    est.set_view_point(vp[0], vp[1], vp[2]);
                }
                if self.use_sensor_origin {
                    est.set_use_sensor_origin(true);
                }

                est.compute()
            }
        }
    }

    impl Default for NormalEstimationBuilder {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Builder for FPFH estimation with configurable parameters
    pub struct FpfhEstimationBuilder {
        radius: Option<f64>,
        k: Option<i32>,
        use_omp: bool,
        num_threads: Option<i32>,
    }

    impl FpfhEstimationBuilder {
        /// Create a new FPFH estimation builder
        pub fn new() -> Self {
            Self {
                radius: None,
                k: None,
                use_omp: false,
                num_threads: None,
            }
        }

        /// Set the radius for neighborhood search
        pub fn radius(mut self, radius: f64) -> Self {
            self.radius = Some(radius);
            self
        }

        /// Set the number of nearest neighbors for search
        pub fn k_search(mut self, k: i32) -> Self {
            self.k = Some(k);
            self
        }

        /// Enable OpenMP acceleration
        pub fn with_omp(mut self, threads: Option<i32>) -> Self {
            self.use_omp = true;
            self.num_threads = threads;
            self
        }

        /// Build and compute FPFH features for the given point cloud and normals
        pub fn compute(
            &self,
            cloud: &PointCloudXYZ,
            normals: &NormalCloud,
        ) -> PclResult<FpfhCloud> {
            if self.use_omp {
                let mut est = FpfhEstimationOmp::new()?;
                est.set_input_cloud(cloud)?;
                est.set_input_normals(normals)?;

                if let Some(radius) = self.radius {
                    est.set_radius_search(radius);
                }
                if let Some(threads) = self.num_threads {
                    est.set_number_of_threads(threads);
                }

                est.compute()
            } else {
                let mut est = FpfhEstimation::new()?;
                est.set_input_cloud(cloud)?;
                est.set_input_normals(normals)?;

                if let Some(radius) = self.radius {
                    est.set_radius_search(radius);
                }
                if let Some(k) = self.k {
                    est.set_k_search(k);
                }

                est.compute()
            }
        }
    }

    impl Default for FpfhEstimationBuilder {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// Utility functions for feature analysis and processing
pub mod utils {
    use super::*;

    /// Feature statistics for analyzing descriptor quality
    #[derive(Debug, Clone)]
    pub struct FeatureStats {
        pub mean: f32,
        pub std_dev: f32,
        pub min: f32,
        pub max: f32,
        pub histogram_entropy: f32,
    }

    impl FeatureStats {
        /// Compute statistics for a single histogram
        pub fn from_histogram(histogram: &[f32]) -> Self {
            let len = histogram.len() as f32;
            let sum: f32 = histogram.iter().sum();
            let mean = sum / len;

            let variance: f32 = histogram.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / len;
            let std_dev = variance.sqrt();

            let min = histogram.iter().fold(f32::INFINITY, |a, &b| a.min(b));
            let max = histogram.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));

            // Compute entropy
            let total: f32 = histogram.iter().sum();
            let entropy = if total > 0.0 {
                -histogram
                    .iter()
                    .filter(|&&x| x > 0.0)
                    .map(|&x| {
                        let p = x / total;
                        p * p.ln()
                    })
                    .sum::<f32>()
            } else {
                0.0
            };

            Self {
                mean,
                std_dev,
                min,
                max,
                histogram_entropy: entropy,
            }
        }
    }

    /// Compare two FPFH signatures using various distance metrics
    pub fn compare_fpfh_signatures(sig1: &FpfhSignature, sig2: &FpfhSignature) -> f32 {
        euclidean_distance(&sig1.histogram, &sig2.histogram)
    }

    /// Compare two PFH signatures using various distance metrics
    pub fn compare_pfh_signatures(sig1: &PfhSignature, sig2: &PfhSignature) -> f32 {
        euclidean_distance(&sig1.histogram, &sig2.histogram)
    }

    /// Compute Euclidean distance between two histograms
    pub fn euclidean_distance(hist1: &[f32], hist2: &[f32]) -> f32 {
        if hist1.len() != hist2.len() {
            return f32::INFINITY;
        }

        hist1
            .iter()
            .zip(hist2.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f32>()
            .sqrt()
    }

    /// Compute chi-squared distance between two histograms
    pub fn chi_squared_distance(hist1: &[f32], hist2: &[f32]) -> f32 {
        if hist1.len() != hist2.len() {
            return f32::INFINITY;
        }

        hist1
            .iter()
            .zip(hist2.iter())
            .map(|(a, b)| {
                let sum = a + b;
                if sum > f32::EPSILON {
                    (a - b).powi(2) / sum
                } else {
                    0.0
                }
            })
            .sum::<f32>()
            * 0.5
    }

    /// Compute histogram intersection (similarity measure)
    pub fn histogram_intersection(hist1: &[f32], hist2: &[f32]) -> f32 {
        if hist1.len() != hist2.len() {
            return 0.0;
        }

        hist1.iter().zip(hist2.iter()).map(|(a, b)| a.min(*b)).sum()
    }

    /// Normalize a histogram to sum to 1.0
    pub fn normalize_histogram(histogram: &mut [f32]) {
        let sum: f32 = histogram.iter().sum();
        if sum > f32::EPSILON {
            histogram.iter_mut().for_each(|x| *x /= sum);
        }
    }

    /// Check if a normal vector is valid (finite and non-zero)
    pub fn is_valid_normal(normal: &Normal) -> bool {
        normal.is_valid()
    }

    /// Find the most similar signature in a collection
    pub fn find_best_match_fpfh(
        query: &FpfhSignature,
        candidates: &[FpfhSignature],
    ) -> Option<(usize, f32)> {
        candidates
            .iter()
            .enumerate()
            .map(|(i, candidate)| (i, compare_fpfh_signatures(query, candidate)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// Find the most similar signature in a collection
    pub fn find_best_match_pfh(
        query: &PfhSignature,
        candidates: &[PfhSignature],
    ) -> Option<(usize, f32)> {
        candidates
            .iter()
            .enumerate()
            .map(|(i, candidate)| (i, compare_pfh_signatures(query, candidate)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }
}

/// Constants for feature extraction
pub mod constants {
    /// Default radius for normal estimation (in meters)
    pub const DEFAULT_NORMAL_RADIUS: f64 = 0.03;

    /// Default number of neighbors for normal estimation
    pub const DEFAULT_NORMAL_K: i32 = 30;

    /// Default radius for FPFH estimation (in meters)
    pub const DEFAULT_FPFH_RADIUS: f64 = 0.05;

    /// Default radius for PFH estimation (in meters)
    pub const DEFAULT_PFH_RADIUS: f64 = 0.05;

    /// Default number of threads for OpenMP operations
    pub const DEFAULT_OMP_THREADS: i32 = 4;

    /// Size of FPFH histogram
    pub const FPFH_HISTOGRAM_SIZE: usize = 33;

    /// Size of PFH histogram
    pub const PFH_HISTOGRAM_SIZE: usize = 125;

    /// Minimum curvature threshold for valid normals
    pub const MIN_CURVATURE_THRESHOLD: f32 = 0.001;

    /// Maximum curvature threshold for valid normals
    pub const MAX_CURVATURE_THRESHOLD: f32 = 1.0;
}
