//! StatisticalOutlierRemoval filter FFI bindings
//!
//! StatisticalOutlierRemoval filter removes points that are statistical outliers
//! based on their distance to neighboring points.

// Re-export StatisticalOutlierRemoval types and functions from parent module
pub use super::{
    StatisticalOutlierRemoval_PointXYZ as StatisticalOutlierRemovalXYZ,
    StatisticalOutlierRemoval_PointXYZRGB as StatisticalOutlierRemovalXYZRGB,
    
    new_statistical_outlier_removal_xyz, set_input_cloud_statistical_xyz, set_mean_k_statistical_xyz,
    set_std_dev_mul_thresh_statistical_xyz, set_negative_statistical_xyz, filter_statistical_xyz,
    
    new_statistical_outlier_removal_xyzrgb, set_input_cloud_statistical_xyzrgb, set_mean_k_statistical_xyzrgb,
    set_std_dev_mul_thresh_statistical_xyzrgb, set_negative_statistical_xyzrgb, filter_statistical_xyzrgb,
};