//! PassThrough filter FFI bindings
//!
//! PassThrough filter removes points that are outside a specified range
//! along a given dimension (x, y, z).

// Re-export PassThrough types and functions from parent module
pub use super::{
    PassThrough_PointXYZ as PassThroughXYZ,
    PassThrough_PointXYZRGB as PassThroughXYZRGB,
    
    new_pass_through_xyz, set_input_cloud_pass_xyz, set_filter_field_name_xyz,
    get_filter_field_name_xyz, set_filter_limits_xyz, set_filter_limits_negative_xyz,
    get_filter_limits_negative_xyz, set_keep_organized_xyz, get_keep_organized_xyz,
    filter_pass_xyz,
    
    new_pass_through_xyzrgb, set_input_cloud_pass_xyzrgb, set_filter_field_name_xyzrgb,
    get_filter_field_name_xyzrgb, set_filter_limits_xyzrgb, set_filter_limits_negative_xyzrgb,
    get_filter_limits_negative_xyzrgb, set_keep_organized_xyzrgb, get_keep_organized_xyzrgb,
    filter_pass_xyzrgb,
};