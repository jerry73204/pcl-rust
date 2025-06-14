//! PLY (Polygon File Format) file format FFI bindings

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");

        // Type declarations - must be repeated for each bridge
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZI;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;

        // PLY I/O functions for PointXYZ
        fn load_ply_file_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        fn save_ply_file_xyz(file_name: &str, cloud: &PointCloud_PointXYZ, binary: bool) -> i32;
        fn save_ply_file_ascii_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        fn save_ply_file_binary_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;

        // PLY I/O functions for PointXYZI
        fn load_ply_file_xyzi(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZI>) -> i32;
        fn save_ply_file_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI, binary: bool) -> i32;
        fn save_ply_file_ascii_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        fn save_ply_file_binary_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;

        // PLY I/O functions for PointXYZRGB
        fn load_ply_file_xyzrgb(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZRGB>) -> i32;
        fn save_ply_file_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
            binary: bool,
        ) -> i32;
        fn save_ply_file_ascii_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        fn save_ply_file_binary_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
    }
}
