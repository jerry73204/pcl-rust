//! Octree spatial data structures
//!
//! This module provides FFI bindings for PCL's octree implementations
//! for spatial organization and fast spatial queries.

// Re-export types from the main FFI module
pub use crate::ffi::{
    OctreePointCloudSearch_PointXYZ as OctreeSearchXYZ,
    OctreePointCloudVoxelCentroid_PointXYZ as OctreeVoxelCentroidXYZ,
};
