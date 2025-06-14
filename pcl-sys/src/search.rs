//! Search algorithms and spatial data structures
//!
//! This module provides FFI bindings for PCL's search interfaces
//! including generic search, nearest neighbor, and radius search.

// Re-export types from the main FFI module
pub use crate::ffi::{KdTree_PointXYZ as KdTreeXYZ, KdTree_PointXYZRGB as KdTreeXYZRGB};
