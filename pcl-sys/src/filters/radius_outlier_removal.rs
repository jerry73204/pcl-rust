//! RadiusOutlierRemoval filter FFI bindings
//!
//! RadiusOutlierRemoval filter removes points that have fewer than a specified
//! number of neighbors within a given radius.

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");
        include!("cxx/types.h");

        // RadiusOutlierRemoval filter types
        type RadiusOutlierRemoval_PointXYZ = crate::ffi::RadiusOutlierRemoval_PointXYZ;
        type RadiusOutlierRemoval_PointXYZRGB = crate::ffi::RadiusOutlierRemoval_PointXYZRGB;
        
        // Point cloud types
        type PointCloud_PointXYZ = crate::ffi::PointCloud_PointXYZ;
        type PointCloud_PointXYZRGB = crate::ffi::PointCloud_PointXYZRGB;

        // RadiusOutlierRemoval creation and configuration - PointXYZ
        fn new_radius_outlier_removal_xyz() -> UniquePtr<RadiusOutlierRemoval_PointXYZ>;
        fn set_input_cloud_ror_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_radius_search_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, radius: f64);
        fn get_radius_search_xyz(filter: &RadiusOutlierRemoval_PointXYZ) -> f64;
        fn set_min_neighbors_in_radius_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, min_pts: i32);
        fn get_min_neighbors_in_radius_xyz(filter: &RadiusOutlierRemoval_PointXYZ) -> i32;
        fn set_negative_ror_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, negative: bool);
        fn get_negative_ror_xyz(filter: &RadiusOutlierRemoval_PointXYZ) -> bool;
        fn filter_ror_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>) -> UniquePtr<PointCloud_PointXYZ>;

        // RadiusOutlierRemoval creation and configuration - PointXYZRGB
        fn new_radius_outlier_removal_xyzrgb() -> UniquePtr<RadiusOutlierRemoval_PointXYZRGB>;
        fn set_input_cloud_ror_xyzrgb(filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>, cloud: &PointCloud_PointXYZRGB);
        fn set_radius_search_xyzrgb(filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>, radius: f64);
        fn get_radius_search_xyzrgb(filter: &RadiusOutlierRemoval_PointXYZRGB) -> f64;
        fn set_min_neighbors_in_radius_xyzrgb(filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>, min_pts: i32);
        fn get_min_neighbors_in_radius_xyzrgb(filter: &RadiusOutlierRemoval_PointXYZRGB) -> i32;
        fn set_negative_ror_xyzrgb(filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>, negative: bool);
        fn get_negative_ror_xyzrgb(filter: &RadiusOutlierRemoval_PointXYZRGB) -> bool;
        fn filter_ror_xyzrgb(filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>) -> UniquePtr<PointCloud_PointXYZRGB>;
    }
}