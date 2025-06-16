//! Keypoint detection algorithms for point clouds
//!
//! This module provides keypoint detection algorithms that can identify
//! distinctive points in point clouds, useful for features matching,
//! registration, and object recognition.

pub mod harris;
pub mod iss;
pub mod sift;

// Re-export main types for convenience
pub use harris::Harris3D;
pub use iss::Iss3D;
pub use sift::SiftKeypoint;

/// Types representing different scales of keypoints
#[derive(Debug, Clone, Copy)]
pub struct PointWithScale {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub scale: f32,
}

impl From<Vec<f32>> for PointWithScale {
    fn from(coords: Vec<f32>) -> Self {
        assert_eq!(coords.len(), 4, "PointWithScale requires 4 coordinates");
        Self {
            x: coords[0],
            y: coords[1],
            z: coords[2],
            scale: coords[3],
        }
    }
}

impl From<&[f32]> for PointWithScale {
    fn from(coords: &[f32]) -> Self {
        assert_eq!(coords.len(), 4, "PointWithScale requires 4 coordinates");
        Self {
            x: coords[0],
            y: coords[1],
            z: coords[2],
            scale: coords[3],
        }
    }
}

/// Common trait for all keypoint detectors
pub trait KeypointDetector<InputCloud, OutputCloud> {
    /// Set the input point cloud for keypoint detection
    fn set_input_cloud(&mut self, cloud: &InputCloud) -> crate::error::PclResult<()>;

    /// Compute keypoints from the input cloud
    fn compute(&mut self) -> crate::error::PclResult<OutputCloud>;
}

/// Builder pattern trait for configurable keypoint detectors
pub trait KeypointBuilder<T> {
    /// Build the configured keypoint detector
    fn build(self) -> crate::error::PclResult<T>;
}
