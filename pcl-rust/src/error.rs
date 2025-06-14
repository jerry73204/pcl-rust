//! Error handling for PCL operations
//!
//! This module defines error types and result types used throughout the PCL crate.

use std::fmt;
use thiserror::Error;

/// Result type for PCL operations
pub type PclResult<T> = Result<T, PclError>;

/// Errors that can occur in PCL operations
#[derive(Error, Debug)]
pub enum PclError {
    /// Error related to invalid point cloud data or operations
    #[error("Invalid point cloud: {message}")]
    InvalidPointCloud {
        message: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Error during search operations (KdTree, Octree, etc.)
    #[error("Search operation failed: {message}")]
    SearchFailed {
        message: String,
        operation: SearchOperation,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Error during octree-specific operations
    #[error("Octree operation failed: {message}")]
    OctreeFailed {
        message: String,
        operation: OctreeOperation,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid parameters provided to a function
    #[error("Invalid parameters: {message}")]
    InvalidParameters {
        message: String,
        parameter: String,
        expected: String,
        actual: String,
    },

    /// Memory allocation failure
    #[error("Memory allocation failed: {size} bytes requested")]
    MemoryAllocation { size: usize },

    /// Internal PCL library error
    #[error("PCL internal error: {message}")]
    InternalError {
        message: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Feature not yet implemented
    #[error("Feature not implemented: {feature}")]
    NotImplemented {
        feature: String,
        workaround: Option<String>,
    },

    /// I/O related errors
    #[error("I/O error: {message}")]
    IoError {
        message: String,
        path: Option<std::path::PathBuf>,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Configuration errors
    #[error("Configuration error: {message}")]
    ConfigurationError {
        message: String,
        field: String,
        suggestion: Option<String>,
    },

    /// State-related errors (e.g., operation on uninitialized object)
    #[error("Invalid state: {message}")]
    InvalidState {
        message: String,
        expected_state: String,
        actual_state: String,
    },
}

/// Types of search operations that can fail
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchOperation {
    NearestKSearch,
    RadiusSearch,
    VoxelSearch,
    SetInputCloud,
    Configuration,
}

impl fmt::Display for SearchOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SearchOperation::NearestKSearch => write!(f, "nearest-k search"),
            SearchOperation::RadiusSearch => write!(f, "radius search"),
            SearchOperation::VoxelSearch => write!(f, "voxel search"),
            SearchOperation::SetInputCloud => write!(f, "set input cloud"),
            SearchOperation::Configuration => write!(f, "configuration"),
        }
    }
}

/// Types of octree operations that can fail
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OctreeOperation {
    Construction,
    AddPoints,
    DeleteTree,
    GetCentroids,
    SetResolution,
}

impl fmt::Display for OctreeOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OctreeOperation::Construction => write!(f, "construction"),
            OctreeOperation::AddPoints => write!(f, "add points"),
            OctreeOperation::DeleteTree => write!(f, "delete tree"),
            OctreeOperation::GetCentroids => write!(f, "get centroids"),
            OctreeOperation::SetResolution => write!(f, "set resolution"),
        }
    }
}

impl PclError {
    /// Create a new InvalidPointCloud error with a message
    pub fn invalid_point_cloud(message: impl Into<String>) -> Self {
        Self::InvalidPointCloud {
            message: message.into(),
            source: None,
        }
    }

    /// Create a new SearchFailed error
    pub fn search_failed(message: impl Into<String>, operation: SearchOperation) -> Self {
        Self::SearchFailed {
            message: message.into(),
            operation,
            source: None,
        }
    }

    /// Create a new OctreeFailed error
    pub fn octree_failed(message: impl Into<String>, operation: OctreeOperation) -> Self {
        Self::OctreeFailed {
            message: message.into(),
            operation,
            source: None,
        }
    }

    /// Create a new InvalidParameters error
    pub fn invalid_parameters(
        message: impl Into<String>,
        parameter: impl Into<String>,
        expected: impl Into<String>,
        actual: impl Into<String>,
    ) -> Self {
        Self::InvalidParameters {
            message: message.into(),
            parameter: parameter.into(),
            expected: expected.into(),
            actual: actual.into(),
        }
    }

    /// Create a new NotImplemented error with optional workaround
    pub fn not_implemented(feature: impl Into<String>, workaround: Option<String>) -> Self {
        Self::NotImplemented {
            feature: feature.into(),
            workaround,
        }
    }

    /// Create a new InvalidState error
    pub fn invalid_state(
        message: impl Into<String>,
        expected: impl Into<String>,
        actual: impl Into<String>,
    ) -> Self {
        Self::InvalidState {
            message: message.into(),
            expected_state: expected.into(),
            actual_state: actual.into(),
        }
    }

    /// Add a source error to this error (for error chaining)
    pub fn with_source(mut self, source: impl std::error::Error + Send + Sync + 'static) -> Self {
        match &mut self {
            PclError::InvalidPointCloud { source: src, .. }
            | PclError::SearchFailed { source: src, .. }
            | PclError::OctreeFailed { source: src, .. }
            | PclError::InternalError { source: src, .. }
            | PclError::IoError { source: src, .. } => {
                *src = Some(Box::new(source));
            }
            _ => {}
        }
        self
    }

    /// Get a suggestion for how to fix this error, if available
    pub fn suggestion(&self) -> Option<String> {
        match self {
            PclError::InvalidParameters {
                expected,
                parameter,
                ..
            } => Some(format!("Parameter '{}' should be {}", parameter, expected)),
            PclError::NotImplemented { workaround, .. } => workaround.clone(),
            PclError::ConfigurationError { suggestion, .. } => suggestion.clone(),
            PclError::InvalidState { expected_state, .. } => Some(format!(
                "Ensure the object is in '{}' state before this operation",
                expected_state
            )),
            PclError::SearchFailed {
                operation: SearchOperation::SetInputCloud,
                ..
            } => Some("Ensure the point cloud is not empty and contains valid points".to_string()),
            PclError::MemoryAllocation { size } => Some(format!(
                "Try reducing the allocation size or freeing memory. Requested: {} bytes",
                size
            )),
            _ => None,
        }
    }
}

impl From<cxx::Exception> for PclError {
    fn from(e: cxx::Exception) -> Self {
        PclError::InternalError {
            message: e.what().to_string(),
            source: None,
        }
    }
}

impl From<std::io::Error> for PclError {
    fn from(e: std::io::Error) -> Self {
        PclError::IoError {
            message: e.to_string(),
            path: None,
            source: Some(Box::new(e)),
        }
    }
}

/// Extension trait for Result types to add context
pub trait ResultExt<T> {
    /// Add context to an error
    fn context(self, msg: &str) -> Result<T, PclError>;

    /// Add context with a lazily evaluated message
    fn with_context<F>(self, f: F) -> Result<T, PclError>
    where
        F: FnOnce() -> String;
}

impl<T, E> ResultExt<T> for Result<T, E>
where
    E: Into<PclError>,
{
    fn context(self, msg: &str) -> Result<T, PclError> {
        self.map_err(|e| {
            let mut err = e.into();
            match &mut err {
                PclError::InvalidPointCloud { message, .. }
                | PclError::SearchFailed { message, .. }
                | PclError::OctreeFailed { message, .. }
                | PclError::InternalError { message, .. } => {
                    *message = format!("{}: {}", msg, message);
                }
                _ => {}
            }
            err
        })
    }

    fn with_context<F>(self, f: F) -> Result<T, PclError>
    where
        F: FnOnce() -> String,
    {
        self.map_err(|e| {
            let mut err = e.into();
            let ctx = f();
            match &mut err {
                PclError::InvalidPointCloud { message, .. }
                | PclError::SearchFailed { message, .. }
                | PclError::OctreeFailed { message, .. }
                | PclError::InternalError { message, .. } => {
                    *message = format!("{}: {}", ctx, message);
                }
                _ => {}
            }
            err
        })
    }
}
