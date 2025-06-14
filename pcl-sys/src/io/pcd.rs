//! PCD (Point Cloud Data) file format FFI bindings
//!
//! This module re-exports PCD I/O functions from the main FFI bridge.

// Re-export PCD functions from parent module
pub use super::{
    // PCD I/O functions for PointXYZ
    load_pcd_file_xyz, save_pcd_file_xyz, save_pcd_file_ascii_xyz, save_pcd_file_binary_xyz,
    save_pcd_file_binary_compressed_xyz,
    // PCD I/O functions for PointXYZI
    load_pcd_file_xyzi, save_pcd_file_xyzi, save_pcd_file_ascii_xyzi, save_pcd_file_binary_xyzi,
    save_pcd_file_binary_compressed_xyzi,
    // PCD I/O functions for PointXYZRGB
    load_pcd_file_xyzrgb, save_pcd_file_xyzrgb, save_pcd_file_ascii_xyzrgb,
    save_pcd_file_binary_xyzrgb, save_pcd_file_binary_compressed_xyzrgb,
};