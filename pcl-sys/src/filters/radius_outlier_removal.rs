//! RadiusOutlierRemoval filter FFI bindings
//!
//! RadiusOutlierRemoval filter removes points that have fewer than a specified
//! number of neighbors within a given radius.

// Re-export RadiusOutlierRemoval types and functions from parent module
pub use super::{
    RadiusOutlierRemoval_PointXYZ as RadiusOutlierRemovalXYZ,
    RadiusOutlierRemoval_PointXYZRGB as RadiusOutlierRemovalXYZRGB,
    
    new_radius_outlier_removal_xyz, set_input_cloud_radius_xyz, set_radius_search_xyz,
    set_min_neighbors_in_radius_xyz, set_negative_radius_xyz, filter_radius_xyz,
    
    new_radius_outlier_removal_xyzrgb, set_input_cloud_radius_xyzrgb, set_radius_search_xyzrgb,
    set_min_neighbors_in_radius_xyzrgb, set_negative_radius_xyzrgb, filter_radius_xyzrgb,
};