//! FFI bindings for PCL filters
//!
//! This module provides low-level FFI bindings for PCL's filtering algorithms
//! including downsampling, outlier removal, and pass-through filtering.

// Re-export filter types and functions from the main FFI module
pub use crate::ffi::{
    // Filter types
    PassThrough_PointXYZ,
    PassThrough_PointXYZRGB,
    RadiusOutlierRemoval_PointXYZ,
    RadiusOutlierRemoval_PointXYZRGB,

    StatisticalOutlierRemoval_PointXYZ,
    StatisticalOutlierRemoval_PointXYZRGB,
    VoxelGrid_PointXYZ,
    VoxelGrid_PointXYZRGB,
    filter_pass_xyz,

    filter_pass_xyzrgb,

    filter_radius_xyz,

    filter_radius_xyzrgb,
    filter_statistical_xyz,

    filter_statistical_xyzrgb,

    filter_voxel_xyz,
    filter_voxel_xyzrgb,

    get_filter_field_name_xyz,
    get_filter_field_name_xyzrgb,
    get_filter_limits_negative_xyz,
    get_filter_limits_negative_xyzrgb,
    get_keep_organized_xyz,
    get_keep_organized_xyzrgb,
    // PassThrough filter functions
    new_pass_through_xyz,
    new_pass_through_xyzrgb,
    // RadiusOutlierRemoval filter functions
    new_radius_outlier_removal_xyz,
    new_radius_outlier_removal_xyzrgb,
    // StatisticalOutlierRemoval filter functions
    new_statistical_outlier_removal_xyz,
    new_statistical_outlier_removal_xyzrgb,
    // VoxelGrid filter functions
    new_voxel_grid_xyz,
    new_voxel_grid_xyzrgb,
    set_filter_field_name_xyz,
    set_filter_field_name_xyzrgb,
    set_filter_limits_negative_xyz,
    set_filter_limits_negative_xyzrgb,
    set_filter_limits_xyz,
    set_filter_limits_xyzrgb,
    set_input_cloud_pass_xyz,
    set_input_cloud_pass_xyzrgb,
    set_input_cloud_radius_xyz,
    set_input_cloud_radius_xyzrgb,
    set_input_cloud_statistical_xyz,
    set_input_cloud_statistical_xyzrgb,
    set_input_cloud_voxel_xyz,
    set_input_cloud_voxel_xyzrgb,
    set_keep_organized_xyz,
    set_keep_organized_xyzrgb,
    set_leaf_size_xyz,
    set_leaf_size_xyzrgb,
    set_mean_k_statistical_xyz,
    set_mean_k_statistical_xyzrgb,
    set_min_neighbors_in_radius_xyz,
    set_min_neighbors_in_radius_xyzrgb,
    set_negative_radius_xyz,
    set_negative_radius_xyzrgb,
    set_negative_statistical_xyz,
    set_negative_statistical_xyzrgb,
    set_radius_search_xyz,
    set_radius_search_xyzrgb,
    set_std_dev_mul_thresh_statistical_xyz,
    set_std_dev_mul_thresh_statistical_xyzrgb,
};

// Re-export point cloud types for convenience
pub use crate::ffi::{
    PointCloud_PointXYZ as PointCloudXYZ, PointCloud_PointXYZRGB as PointCloudXYZRGB,
};
