//! VoxelGrid filter FFI bindings
//!
//! VoxelGrid filter performs downsampling by creating a 3D voxel grid
//! and representing each voxel with a single point (typically the centroid).

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");
        include!("cxx/types.h");

        // VoxelGrid filter types
        type VoxelGrid_PointXYZ = crate::ffi::VoxelGrid_PointXYZ;
        type VoxelGrid_PointXYZRGB = crate::ffi::VoxelGrid_PointXYZRGB;
        
        // Point cloud types
        type PointCloud_PointXYZ = crate::ffi::PointCloud_PointXYZ;
        type PointCloud_PointXYZRGB = crate::ffi::PointCloud_PointXYZRGB;

        // VoxelGrid creation and configuration - PointXYZ
        fn new_voxel_grid_xyz() -> UniquePtr<VoxelGrid_PointXYZ>;
        fn set_input_cloud_voxel_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_leaf_size_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, lx: f32, ly: f32, lz: f32);
        fn get_leaf_size_xyz(filter: &VoxelGrid_PointXYZ) -> [f32; 3];
        fn set_minimum_points_number_per_voxel_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, min_points: u32);
        fn get_minimum_points_number_per_voxel_xyz(filter: &VoxelGrid_PointXYZ) -> u32;
        fn set_downsample_all_data_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, downsample: bool);
        fn get_downsample_all_data_xyz(filter: &VoxelGrid_PointXYZ) -> bool;
        fn filter_voxel_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>) -> UniquePtr<PointCloud_PointXYZ>;

        // VoxelGrid creation and configuration - PointXYZRGB
        fn new_voxel_grid_xyzrgb() -> UniquePtr<VoxelGrid_PointXYZRGB>;
        fn set_input_cloud_voxel_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, cloud: &PointCloud_PointXYZRGB);
        fn set_leaf_size_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, lx: f32, ly: f32, lz: f32);
        fn get_leaf_size_xyzrgb(filter: &VoxelGrid_PointXYZRGB) -> [f32; 3];
        fn set_minimum_points_number_per_voxel_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, min_points: u32);
        fn get_minimum_points_number_per_voxel_xyzrgb(filter: &VoxelGrid_PointXYZRGB) -> u32;
        fn set_downsample_all_data_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, downsample: bool);
        fn get_downsample_all_data_xyzrgb(filter: &VoxelGrid_PointXYZRGB) -> bool;
        fn filter_voxel_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>) -> UniquePtr<PointCloud_PointXYZRGB>;
    }
}