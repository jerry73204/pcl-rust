//! Error handling for PCL operations
//!
//! This module defines error types and result types used throughout the PCL crate.

use thiserror::Error;

/// Result type for PCL operations
pub type PclResult<T> = Result<T, PclError>;

/// Errors that can occur in PCL operations
#[derive(Error, Debug)]
pub enum PclError {
    #[error("Invalid point cloud: {0}")]
    InvalidPointCloud(String),

    #[error("Search operation failed: {0}")]
    SearchFailed(String),

    #[error("Octree operation failed: {0}")]
    OctreeFailed(String),

    #[error("Invalid parameters: {0}")]
    InvalidParameters(String),

    #[error("Memory allocation failed")]
    MemoryAllocation,

    #[error("PCL internal error: {0}")]
    InternalError(String),

    #[error("Feature not implemented: {0}")]
    NotImplemented(String),
}

impl From<cxx::Exception> for PclError {
    fn from(e: cxx::Exception) -> Self {
        PclError::InternalError(e.what().to_string())
    }
}
