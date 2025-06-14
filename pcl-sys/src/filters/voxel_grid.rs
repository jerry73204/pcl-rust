//! VoxelGrid filter FFI bindings
//!
//! VoxelGrid filter performs downsampling by creating a 3D voxel grid
//! and representing each voxel with a single point (typically the centroid).

// Re-export VoxelGrid types and functions from parent module
pub use super::{
    VoxelGrid_PointXYZ as VoxelGridXYZ,
    VoxelGrid_PointXYZRGB as VoxelGridXYZRGB,
    
    new_voxel_grid_xyz, set_input_cloud_voxel_xyz, set_leaf_size_xyz, filter_voxel_xyz,
    new_voxel_grid_xyzrgb, set_input_cloud_voxel_xyzrgb, set_leaf_size_xyzrgb, filter_voxel_xyzrgb,
};