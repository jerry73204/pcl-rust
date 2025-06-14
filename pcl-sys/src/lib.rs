//! Low-level FFI bindings for Point Cloud Library (PCL)
//!
//! This crate provides unsafe FFI bindings to PCL using the cxx crate.
//! For safe, idiomatic Rust APIs, use the `pcl` crate instead.

// All modules are working with the current FFI bridge
pub mod common;
pub mod features;
pub mod filters;
pub mod io;
pub mod keypoints;
pub mod octree;
pub mod registration;
pub mod sample_consensus;
pub mod search;
pub mod segmentation;

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
        type PointCloud_PointXYZI;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;

        // Search types
        #[namespace = "pcl::search"]
        type KdTree_PointXYZ;
        #[namespace = "pcl::search"]
        type KdTree_PointXYZI;
        #[namespace = "pcl::search"]
        type KdTree_PointXYZRGB;

        // Octree types
        #[namespace = "pcl::octree"]
        type OctreePointCloudSearch_PointXYZ;
        #[namespace = "pcl::octree"]
        type OctreePointCloudVoxelCentroid_PointXYZ;

        // Point cloud functions - BASIC FUNCTIONS ONLY
        fn new_point_cloud_xyz() -> UniquePtr<PointCloud_PointXYZ>;
        fn new_point_cloud_xyzi() -> UniquePtr<PointCloud_PointXYZI>;
        fn new_point_cloud_xyzrgb() -> UniquePtr<PointCloud_PointXYZRGB>;
        fn size(cloud: &PointCloud_PointXYZ) -> usize;
        fn size_xyzi(cloud: &PointCloud_PointXYZI) -> usize;
        fn size_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> usize;
        fn clear(cloud: Pin<&mut PointCloud_PointXYZ>);
        fn clear_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>);
        fn clear_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>);
        fn empty(cloud: &PointCloud_PointXYZ) -> bool;
        fn empty_xyzi(cloud: &PointCloud_PointXYZI) -> bool;
        fn empty_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;

        // Additional basic point cloud functions
        fn reserve_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);
        fn reserve_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>, n: usize);
        fn reserve_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);
        fn resize_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);
        fn resize_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>, n: usize);
        fn resize_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);
        fn width(cloud: &PointCloud_PointXYZ) -> u32;
        fn height(cloud: &PointCloud_PointXYZ) -> u32;
        fn width_xyzi(cloud: &PointCloud_PointXYZI) -> u32;
        fn height_xyzi(cloud: &PointCloud_PointXYZI) -> u32;
        fn width_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;
        fn height_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;
        fn is_dense(cloud: &PointCloud_PointXYZ) -> bool;
        fn is_dense_xyzi(cloud: &PointCloud_PointXYZI) -> bool;
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
        fn new_kdtree_xyzi() -> UniquePtr<KdTree_PointXYZI>;
        fn new_kdtree_xyzrgb() -> UniquePtr<KdTree_PointXYZRGB>;
        fn nearest_k_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, k: i32) -> Vec<i32>;
        fn nearest_k_search_xyzi(
            searcher: &KdTree_PointXYZI,
            point: &PointXYZI,
            k: i32,
        ) -> Vec<i32>;
        fn nearest_k_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            k: i32,
        ) -> Vec<i32>;
        fn radius_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, radius: f64)
        -> Vec<i32>;
        fn radius_search_xyzi(
            searcher: &KdTree_PointXYZI,
            point: &PointXYZI,
            radius: f64,
        ) -> Vec<i32>;
        fn radius_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            radius: f64,
        ) -> Vec<i32>;
        fn set_input_cloud_xyz(searcher: Pin<&mut KdTree_PointXYZ>, cloud: &PointCloud_PointXYZ);
        fn set_input_cloud_xyzi(searcher: Pin<&mut KdTree_PointXYZI>, cloud: &PointCloud_PointXYZI);
        fn set_input_cloud_xyzrgb(
            searcher: Pin<&mut KdTree_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn get_epsilon_xyz(searcher: &KdTree_PointXYZ) -> f32;
        fn set_epsilon_xyz(searcher: Pin<&mut KdTree_PointXYZ>, epsilon: f32);
        fn get_epsilon_xyzi(searcher: &KdTree_PointXYZI) -> f32;
        fn set_epsilon_xyzi(searcher: Pin<&mut KdTree_PointXYZI>, epsilon: f32);
        fn get_epsilon_xyzrgb(searcher: &KdTree_PointXYZRGB) -> f32;
        fn set_epsilon_xyzrgb(searcher: Pin<&mut KdTree_PointXYZRGB>, epsilon: f32);

        // Octree functions - BASIC FUNCTIONS ONLY
        fn new_octree_search_xyz(resolution: f64) -> UniquePtr<OctreePointCloudSearch_PointXYZ>;
        fn new_octree_voxel_centroid_xyz(
            resolution: f64,
        ) -> UniquePtr<OctreePointCloudVoxelCentroid_PointXYZ>;
        fn set_input_cloud_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn add_points_from_input_cloud_xyz(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>);

        // Point manipulation functions
        fn get_point_coords(cloud: &PointCloud_PointXYZ, index: usize) -> Vec<f32>;
        fn get_point_coords_xyzi(cloud: &PointCloud_PointXYZI, index: usize) -> Vec<f32>;
        fn get_point_coords_xyzrgb(cloud: &PointCloud_PointXYZRGB, index: usize) -> Vec<f32>;
        fn set_point_coords(cloud: Pin<&mut PointCloud_PointXYZ>, index: usize, coords: &[f32]);
        fn set_point_coords_xyzi(
            cloud: Pin<&mut PointCloud_PointXYZI>,
            index: usize,
            coords: &[f32],
        );
        fn set_point_coords_xyzrgb(
            cloud: Pin<&mut PointCloud_PointXYZRGB>,
            index: usize,
            coords: &[f32],
        );
        fn push_back_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, coords: &[f32]);
        fn push_back_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>, coords: &[f32]);
        fn push_back_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, coords: &[f32]);

        // Additional octree functions
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

        // OctreeVoxelCentroid functions
        fn set_input_cloud_voxel_centroid_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn add_points_from_input_cloud_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        );
        fn get_voxel_centroids_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        fn get_resolution_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> f64;
        fn get_tree_depth_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> u32;
        fn delete_tree_voxel_xyz(octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>);
    }
}
