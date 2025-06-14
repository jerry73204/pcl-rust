//! FFI bindings for PCL I/O operations
//!
//! This module provides low-level FFI bindings for PCL's file I/O functionality,
//! including PCD and PLY file format support.

// Re-export I/O functions from the main FFI module
pub use crate::ffi::{
    // PCD I/O functions for PointXYZ
    load_pcd_file_xyz,
    // PCD I/O functions for PointXYZI
    load_pcd_file_xyzi,
    // PCD I/O functions for PointXYZRGB
    load_pcd_file_xyzrgb,
    // PLY I/O functions for PointXYZ
    load_ply_file_xyz,
    // PLY I/O functions for PointXYZI
    load_ply_file_xyzi,
    // PLY I/O functions for PointXYZRGB
    load_ply_file_xyzrgb,
    save_pcd_file_ascii_xyz,
    save_pcd_file_ascii_xyzi,
    save_pcd_file_ascii_xyzrgb,
    save_pcd_file_binary_compressed_xyz,
    save_pcd_file_binary_compressed_xyzi,
    save_pcd_file_binary_compressed_xyzrgb,
    save_pcd_file_binary_xyz,
    save_pcd_file_binary_xyzi,
    save_pcd_file_binary_xyzrgb,
    save_pcd_file_xyz,
    save_pcd_file_xyzi,
    save_pcd_file_xyzrgb,
    save_ply_file_ascii_xyz,
    save_ply_file_ascii_xyzi,
    save_ply_file_ascii_xyzrgb,
    save_ply_file_binary_xyz,
    save_ply_file_binary_xyzi,
    save_ply_file_binary_xyzrgb,
    save_ply_file_xyz,
    save_ply_file_xyzi,
    save_ply_file_xyzrgb,
};

// Re-export point cloud types for convenience
pub use crate::ffi::{
    PointCloud_PointXYZ as PointCloudXYZ, PointCloud_PointXYZI as PointCloudXYZI,
    PointCloud_PointXYZRGB as PointCloudXYZRGB,
};
