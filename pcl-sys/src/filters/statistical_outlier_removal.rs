//! StatisticalOutlierRemoval filter FFI bindings
//!
//! StatisticalOutlierRemoval filter removes points that are statistical outliers
//! based on their distance to neighboring points.

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");
        include!("cxx/types.h");

        // StatisticalOutlierRemoval filter types
        type StatisticalOutlierRemoval_PointXYZ = crate::ffi::StatisticalOutlierRemoval_PointXYZ;
        type StatisticalOutlierRemoval_PointXYZRGB = crate::ffi::StatisticalOutlierRemoval_PointXYZRGB;
        
        // Point cloud types
        type PointCloud_PointXYZ = crate::ffi::PointCloud_PointXYZ;
        type PointCloud_PointXYZRGB = crate::ffi::PointCloud_PointXYZRGB;

        // StatisticalOutlierRemoval creation and configuration - PointXYZ
        fn new_statistical_outlier_removal_xyz() -> UniquePtr<StatisticalOutlierRemoval_PointXYZ>;
        fn set_input_cloud_sor_xyz(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_mean_k_xyz(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>, k: i32);
        fn get_mean_k_xyz(filter: &StatisticalOutlierRemoval_PointXYZ) -> i32;
        fn set_std_dev_mul_thresh_xyz(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>, thresh: f64);
        fn get_std_dev_mul_thresh_xyz(filter: &StatisticalOutlierRemoval_PointXYZ) -> f64;
        fn set_negative_sor_xyz(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>, negative: bool);
        fn get_negative_sor_xyz(filter: &StatisticalOutlierRemoval_PointXYZ) -> bool;
        fn filter_sor_xyz(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>) -> UniquePtr<PointCloud_PointXYZ>;

        // StatisticalOutlierRemoval creation and configuration - PointXYZRGB
        fn new_statistical_outlier_removal_xyzrgb() -> UniquePtr<StatisticalOutlierRemoval_PointXYZRGB>;
        fn set_input_cloud_sor_xyzrgb(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>, cloud: &PointCloud_PointXYZRGB);
        fn set_mean_k_xyzrgb(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>, k: i32);
        fn get_mean_k_xyzrgb(filter: &StatisticalOutlierRemoval_PointXYZRGB) -> i32;
        fn set_std_dev_mul_thresh_xyzrgb(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>, thresh: f64);
        fn get_std_dev_mul_thresh_xyzrgb(filter: &StatisticalOutlierRemoval_PointXYZRGB) -> f64;
        fn set_negative_sor_xyzrgb(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>, negative: bool);
        fn get_negative_sor_xyzrgb(filter: &StatisticalOutlierRemoval_PointXYZRGB) -> bool;
        fn filter_sor_xyzrgb(filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>) -> UniquePtr<PointCloud_PointXYZRGB>;
    }
}