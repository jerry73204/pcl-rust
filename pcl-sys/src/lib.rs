//! Low-level FFI bindings for Point Cloud Library (PCL)
//!
//! This crate provides unsafe FFI bindings to PCL using the cxx crate.
//! For safe, idiomatic Rust APIs, use the `pcl` crate instead.

pub mod common;
pub mod octree;
pub mod search;

// Re-export cxx types that may be useful
pub use cxx::{SharedPtr, UniquePtr};

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");

        // Point types
        #[namespace = "pcl"]
        type PointXYZ;
        #[namespace = "pcl"]
        type PointXYZI;
        #[namespace = "pcl"]
        type PointXYZRGB;

        // PointCloud template instantiations
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;

        // Search types
        #[namespace = "pcl::search"]
        type KdTree_PointXYZ;
        #[namespace = "pcl::search"]
        type KdTree_PointXYZRGB;

        // Octree types
        #[namespace = "pcl::octree"]
        type OctreePointCloudSearch_PointXYZ;
        #[namespace = "pcl::octree"]
        type OctreePointCloudVoxelCentroid_PointXYZ;

        // Point cloud functions
        fn new_point_cloud_xyz() -> UniquePtr<PointCloud_PointXYZ>;
        fn new_point_cloud_xyzrgb() -> UniquePtr<PointCloud_PointXYZRGB>;
        fn size(cloud: &PointCloud_PointXYZ) -> usize;
        fn size_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> usize;
        fn clear(cloud: Pin<&mut PointCloud_PointXYZ>);
        fn clear_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>);
        fn empty(cloud: &PointCloud_PointXYZ) -> bool;
        fn empty_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;
        fn reserve_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);
        fn reserve_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);
        fn resize_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);
        fn resize_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);
        fn width(cloud: &PointCloud_PointXYZ) -> u32;
        fn height(cloud: &PointCloud_PointXYZ) -> u32;
        fn width_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;
        fn height_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;
        fn is_dense(cloud: &PointCloud_PointXYZ) -> bool;
        fn is_dense_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;

        // Point field access functions
        fn get_x(point: &PointXYZ) -> f32;
        fn get_y(point: &PointXYZ) -> f32;
        fn get_z(point: &PointXYZ) -> f32;
        fn get_x_xyzi(point: &PointXYZI) -> f32;
        fn get_y_xyzi(point: &PointXYZI) -> f32;
        fn get_z_xyzi(point: &PointXYZI) -> f32;
        fn get_intensity(point: &PointXYZI) -> f32;
        fn get_x_xyzrgb(point: &PointXYZRGB) -> f32;
        fn get_y_xyzrgb(point: &PointXYZRGB) -> f32;
        fn get_z_xyzrgb(point: &PointXYZRGB) -> f32;
        fn get_r(point: &PointXYZRGB) -> u8;
        fn get_g(point: &PointXYZRGB) -> u8;
        fn get_b(point: &PointXYZRGB) -> u8;

        // Search functions
        fn new_kdtree_xyz() -> UniquePtr<KdTree_PointXYZ>;
        fn new_kdtree_xyzrgb() -> UniquePtr<KdTree_PointXYZRGB>;
        fn nearest_k_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, k: i32) -> Vec<i32>;
        fn nearest_k_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            k: i32,
        ) -> Vec<i32>;
        fn radius_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, radius: f64)
        -> Vec<i32>;
        fn radius_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            radius: f64,
        ) -> Vec<i32>;
        fn set_input_cloud_xyz(searcher: Pin<&mut KdTree_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_input_cloud_xyzrgb(
            searcher: Pin<&mut KdTree_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn get_epsilon_xyz(searcher: &KdTree_PointXYZ) -> f32;
        fn set_epsilon_xyz(searcher: Pin<&mut KdTree_PointXYZ>, epsilon: f32);
        fn get_epsilon_xyzrgb(searcher: &KdTree_PointXYZRGB) -> f32;
        fn set_epsilon_xyzrgb(searcher: Pin<&mut KdTree_PointXYZRGB>, epsilon: f32);

        // Octree functions
        fn new_octree_search_xyz(resolution: f64) -> UniquePtr<OctreePointCloudSearch_PointXYZ>;
        fn new_octree_voxel_centroid_xyz(
            resolution: f64,
        ) -> UniquePtr<OctreePointCloudVoxelCentroid_PointXYZ>;
        fn set_input_cloud_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn add_points_from_input_cloud_xyz(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>);
        fn nearest_k_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
            k: i32,
        ) -> Vec<i32>;
        fn radius_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
            radius: f64,
        ) -> Vec<i32>;
        fn voxel_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
        ) -> Vec<i32>;
        fn get_resolution(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> f64;
        fn get_tree_depth(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> u32;
        fn get_leaf_count(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> usize;
        fn get_branch_count(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> usize;
    }
}

// Type aliases for convenience
pub type PointXYZ = ffi::PointXYZ;
pub type PointXYZI = ffi::PointXYZI;
pub type PointXYZRGB = ffi::PointXYZRGB;
pub type PointCloudXYZ = ffi::PointCloud_PointXYZ;
pub type PointCloudXYZRGB = ffi::PointCloud_PointXYZRGB;
pub type KdTreeXYZ = ffi::KdTree_PointXYZ;
pub type KdTreeXYZRGB = ffi::KdTree_PointXYZRGB;
pub type OctreeSearchXYZ = ffi::OctreePointCloudSearch_PointXYZ;
pub type OctreeVoxelCentroidXYZ = ffi::OctreePointCloudVoxelCentroid_PointXYZ;
