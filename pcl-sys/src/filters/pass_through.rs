//! PassThrough filter FFI bindings
//!
//! PassThrough filter removes points that are outside a specified range
//! along a given dimension (x, y, z).

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");
        include!("cxx/types.h");

        // PassThrough filter types
        type PassThrough_PointXYZ = crate::ffi::PassThrough_PointXYZ;
        type PassThrough_PointXYZRGB = crate::ffi::PassThrough_PointXYZRGB;
        
        // Point cloud types
        type PointCloud_PointXYZ = crate::ffi::PointCloud_PointXYZ;
        type PointCloud_PointXYZRGB = crate::ffi::PointCloud_PointXYZRGB;

        // PassThrough creation and configuration - PointXYZ
        fn new_pass_through_xyz() -> UniquePtr<PassThrough_PointXYZ>;
        fn set_input_cloud_pass_xyz(filter: Pin<&mut PassThrough_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_filter_field_name_xyz(filter: Pin<&mut PassThrough_PointXYZ>, field_name: &str);
        fn get_filter_field_name_xyz(filter: &PassThrough_PointXYZ) -> String;
        fn set_filter_limits_xyz(filter: Pin<&mut PassThrough_PointXYZ>, min: f32, max: f32);
        fn set_filter_limits_negative_xyz(filter: Pin<&mut PassThrough_PointXYZ>, negative: bool);
        fn get_filter_limits_negative_xyz(filter: &PassThrough_PointXYZ) -> bool;
        fn set_keep_organized_xyz(filter: Pin<&mut PassThrough_PointXYZ>, keep_organized: bool);
        fn get_keep_organized_xyz(filter: &PassThrough_PointXYZ) -> bool;
        fn filter_pass_xyz(filter: Pin<&mut PassThrough_PointXYZ>) -> UniquePtr<PointCloud_PointXYZ>;

        // PassThrough creation and configuration - PointXYZRGB
        fn new_pass_through_xyzrgb() -> UniquePtr<PassThrough_PointXYZRGB>;
        fn set_input_cloud_pass_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, cloud: &PointCloud_PointXYZRGB);
        fn set_filter_field_name_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, field_name: &str);
        fn get_filter_field_name_xyzrgb(filter: &PassThrough_PointXYZRGB) -> String;
        fn set_filter_limits_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, min: f32, max: f32);
        fn set_filter_limits_negative_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, negative: bool);
        fn get_filter_limits_negative_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        fn set_keep_organized_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, keep_organized: bool);
        fn get_keep_organized_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        fn filter_pass_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>) -> UniquePtr<PointCloud_PointXYZRGB>;
    }
}