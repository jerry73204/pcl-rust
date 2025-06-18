//! Low-level FFI bindings for Point Cloud Library (PCL)
//!
//! This crate provides unsafe FFI bindings to PCL using the cxx crate.
//! For safe, idiomatic Rust APIs, use the `pcl` crate instead.

// Module declarations with feature gates

pub mod common;
#[cfg(feature = "features")]
pub mod features;
#[cfg(feature = "filters")]
pub mod filters;
#[cfg(feature = "io")]
pub mod io;
#[cfg(feature = "keypoints")]
pub mod keypoints;
#[cfg(feature = "octree")]
pub mod octree;
#[cfg(feature = "registration")]
pub mod registration;
#[cfg(feature = "sample_consensus")]
pub mod sample_consensus;
#[cfg(feature = "search")]
pub mod search;
#[cfg(feature = "segmentation")]
pub mod segmentation;
#[cfg(feature = "surface")]
pub mod surface;
#[cfg(feature = "visualization")]
pub mod visualization;

// Re-export cxx types that may be useful
pub use cxx::{SharedPtr, UniquePtr};

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");
        include!("cxx/visualization_stubs.h");

        // Point types - always available as they're fundamental
        #[namespace = "pcl"]
        type PointXYZ;
        #[namespace = "pcl"]
        type PointXYZI;
        #[namespace = "pcl"]
        type PointXYZRGB;
        #[namespace = "pcl"]
        type PointNormal;
        #[namespace = "pcl"]
        type PointXYZRGBA;

        // PointCloud template instantiations - always available
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZI;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGBA;

        // Point cloud functions - part of common feature

        fn new_point_cloud_xyz() -> UniquePtr<PointCloud_PointXYZ>;

        fn new_point_cloud_xyzi() -> UniquePtr<PointCloud_PointXYZI>;

        fn new_point_cloud_xyzrgb() -> UniquePtr<PointCloud_PointXYZRGB>;

        fn new_point_cloud_xyzrgba() -> UniquePtr<PointCloud_PointXYZRGBA>;

        fn new_point_cloud_point_normal() -> UniquePtr<PointCloud_PointNormal>;

        fn size(cloud: &PointCloud_PointXYZ) -> usize;

        fn size_xyzi(cloud: &PointCloud_PointXYZI) -> usize;

        fn size_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> usize;

        fn size_point_normal(cloud: &PointCloud_PointNormal) -> usize;

        fn clear(cloud: Pin<&mut PointCloud_PointXYZ>);

        fn clear_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>);

        fn clear_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>);

        fn clear_point_normal(cloud: Pin<&mut PointCloud_PointNormal>);

        fn empty(cloud: &PointCloud_PointXYZ) -> bool;

        fn empty_xyzi(cloud: &PointCloud_PointXYZI) -> bool;

        fn empty_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;

        fn empty_point_normal(cloud: &PointCloud_PointNormal) -> bool;

        // Additional basic point cloud functions - part of common

        fn reserve_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);

        fn reserve_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>, n: usize);

        fn reserve_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);

        fn reserve_point_normal(cloud: Pin<&mut PointCloud_PointNormal>, n: usize);

        fn resize_xyz(cloud: Pin<&mut PointCloud_PointXYZ>, n: usize);

        fn resize_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>, n: usize);

        fn resize_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>, n: usize);

        fn resize_point_normal(cloud: Pin<&mut PointCloud_PointNormal>, n: usize);

        fn width(cloud: &PointCloud_PointXYZ) -> u32;

        fn height(cloud: &PointCloud_PointXYZ) -> u32;

        fn width_xyzi(cloud: &PointCloud_PointXYZI) -> u32;

        fn height_xyzi(cloud: &PointCloud_PointXYZI) -> u32;

        fn width_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;

        fn height_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> u32;

        fn width_point_normal(cloud: &PointCloud_PointNormal) -> u32;

        fn height_point_normal(cloud: &PointCloud_PointNormal) -> u32;

        fn is_dense(cloud: &PointCloud_PointXYZ) -> bool;

        fn is_dense_xyzi(cloud: &PointCloud_PointXYZI) -> bool;

        fn is_dense_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;

        fn is_dense_point_normal(cloud: &PointCloud_PointNormal) -> bool;

        // Point field access functions - part of common

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

        fn get_x_point_normal(point: &PointNormal) -> f32;

        fn get_y_point_normal(point: &PointNormal) -> f32;

        fn get_z_point_normal(point: &PointNormal) -> f32;

        fn get_normal_x_point_normal(point: &PointNormal) -> f32;

        fn get_normal_y_point_normal(point: &PointNormal) -> f32;

        fn get_normal_z_point_normal(point: &PointNormal) -> f32;

        // Point manipulation functions - part of common

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

        fn push_back_point_normal(cloud: Pin<&mut PointCloud_PointNormal>, coords: &[f32]);

        // Search types - always available but functions conditionally compiled
        #[namespace = "pcl::search"]
        type KdTree_PointXYZ;
        #[namespace = "pcl::search"]
        type KdTree_PointXYZI;
        #[namespace = "pcl::search"]
        type KdTree_PointXYZRGB;

        #[cfg(feature = "search")]
        fn new_kdtree_xyz() -> UniquePtr<KdTree_PointXYZ>;
        #[cfg(feature = "search")]
        fn new_kdtree_xyzi() -> UniquePtr<KdTree_PointXYZI>;
        #[cfg(feature = "search")]
        fn new_kdtree_xyzrgb() -> UniquePtr<KdTree_PointXYZRGB>;
        #[cfg(feature = "search")]
        fn nearest_k_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, k: i32) -> Vec<i32>;
        #[cfg(feature = "search")]
        fn nearest_k_search_xyzi(
            searcher: &KdTree_PointXYZI,
            point: &PointXYZI,
            k: i32,
        ) -> Vec<i32>;
        #[cfg(feature = "search")]
        fn nearest_k_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            k: i32,
        ) -> Vec<i32>;
        #[cfg(feature = "search")]
        fn radius_search_xyz(searcher: &KdTree_PointXYZ, point: &PointXYZ, radius: f64)
        -> Vec<i32>;
        #[cfg(feature = "search")]
        fn radius_search_xyzi(
            searcher: &KdTree_PointXYZI,
            point: &PointXYZI,
            radius: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "search")]
        fn radius_search_xyzrgb(
            searcher: &KdTree_PointXYZRGB,
            point: &PointXYZRGB,
            radius: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "search")]
        fn set_input_cloud_xyz(searcher: Pin<&mut KdTree_PointXYZ>, cloud: &PointCloud_PointXYZ);
        #[cfg(feature = "search")]
        fn set_input_cloud_xyzi(searcher: Pin<&mut KdTree_PointXYZI>, cloud: &PointCloud_PointXYZI);
        #[cfg(feature = "search")]
        fn set_input_cloud_xyzrgb(
            searcher: Pin<&mut KdTree_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "search")]
        fn get_epsilon_xyz(searcher: &KdTree_PointXYZ) -> f32;
        #[cfg(feature = "search")]
        fn set_epsilon_xyz(searcher: Pin<&mut KdTree_PointXYZ>, epsilon: f32);
        #[cfg(feature = "search")]
        fn get_epsilon_xyzi(searcher: &KdTree_PointXYZI) -> f32;
        #[cfg(feature = "search")]
        fn set_epsilon_xyzi(searcher: Pin<&mut KdTree_PointXYZI>, epsilon: f32);
        #[cfg(feature = "search")]
        fn get_epsilon_xyzrgb(searcher: &KdTree_PointXYZRGB) -> f32;
        #[cfg(feature = "search")]
        fn set_epsilon_xyzrgb(searcher: Pin<&mut KdTree_PointXYZRGB>, epsilon: f32);

        // Octree types - always available but functions conditionally compiled
        #[namespace = "pcl::octree"]
        type OctreePointCloudSearch_PointXYZ;
        #[namespace = "pcl::octree"]
        type OctreePointCloudVoxelCentroid_PointXYZ;

        #[cfg(feature = "octree")]
        fn new_octree_search_xyz(resolution: f64) -> UniquePtr<OctreePointCloudSearch_PointXYZ>;
        #[cfg(feature = "octree")]
        fn new_octree_voxel_centroid_xyz(
            resolution: f64,
        ) -> UniquePtr<OctreePointCloudVoxelCentroid_PointXYZ>;
        #[cfg(feature = "octree")]
        fn set_input_cloud_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "octree")]
        fn add_points_from_input_cloud_xyz(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>);
        #[cfg(feature = "octree")]
        fn nearest_k_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
            k: i32,
        ) -> Vec<i32>;
        #[cfg(feature = "octree")]
        fn radius_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
            radius: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "octree")]
        fn voxel_search_octree_xyz(
            octree: Pin<&mut OctreePointCloudSearch_PointXYZ>,
            point: &PointXYZ,
        ) -> Vec<i32>;
        #[cfg(feature = "octree")]
        fn get_resolution(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> f64;
        #[cfg(feature = "octree")]
        fn get_tree_depth(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> u32;
        #[cfg(feature = "octree")]
        fn get_leaf_count(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> usize;
        #[cfg(feature = "octree")]
        fn get_branch_count(octree: Pin<&mut OctreePointCloudSearch_PointXYZ>) -> usize;

        // OctreeVoxelCentroid functions
        #[cfg(feature = "octree")]
        fn set_input_cloud_voxel_centroid_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "octree")]
        fn add_points_from_input_cloud_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        );
        #[cfg(feature = "octree")]
        fn get_voxel_centroids_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        #[cfg(feature = "octree")]
        fn get_resolution_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "octree")]
        fn get_tree_depth_voxel_xyz(
            octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>,
        ) -> u32;
        #[cfg(feature = "octree")]
        fn delete_tree_voxel_xyz(octree: Pin<&mut OctreePointCloudVoxelCentroid_PointXYZ>);

        // I/O functions
        // PCD I/O functions for PointXYZ
        #[cfg(feature = "io")]
        fn load_pcd_file_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_xyz(file_name: &str, cloud: &PointCloud_PointXYZ, binary: bool) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_ascii_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_compressed_xyz(file_name: &str, cloud: &PointCloud_PointXYZ)
        -> i32;

        // PCD I/O functions for PointXYZI
        #[cfg(feature = "io")]
        fn load_pcd_file_xyzi(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZI>) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI, binary: bool) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_ascii_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_compressed_xyzi(
            file_name: &str,
            cloud: &PointCloud_PointXYZI,
        ) -> i32;

        // PCD I/O functions for PointXYZRGB
        #[cfg(feature = "io")]
        fn load_pcd_file_xyzrgb(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZRGB>) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
            binary: bool,
        ) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_ascii_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        #[cfg(feature = "io")]
        fn save_pcd_file_binary_compressed_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
        ) -> i32;

        // PLY I/O functions for PointXYZ
        #[cfg(feature = "io")]
        fn load_ply_file_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_xyz(file_name: &str, cloud: &PointCloud_PointXYZ, binary: bool) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_ascii_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_binary_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;

        // PLY I/O functions for PointXYZI
        #[cfg(feature = "io")]
        fn load_ply_file_xyzi(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZI>) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI, binary: bool) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_ascii_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_binary_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;

        // PLY I/O functions for PointXYZRGB
        #[cfg(feature = "io")]
        fn load_ply_file_xyzrgb(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZRGB>) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
            binary: bool,
        ) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_ascii_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        #[cfg(feature = "io")]
        fn save_ply_file_binary_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;

        // Format auto-detection functions
        #[cfg(feature = "io")]
        fn detect_format_from_extension(file_name: &str) -> i32;
        #[cfg(feature = "io")]
        fn detect_format_from_content(file_name: &str) -> i32;
        #[cfg(feature = "io")]
        fn detect_file_format(file_name: &str) -> i32;

        // Auto-loading functions that detect format automatically
        #[cfg(feature = "io")]
        fn load_point_cloud_auto_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        #[cfg(feature = "io")]
        fn load_point_cloud_auto_xyzi(
            file_name: &str,
            cloud: Pin<&mut PointCloud_PointXYZI>,
        ) -> i32;
        #[cfg(feature = "io")]
        fn load_point_cloud_auto_xyzrgb(
            file_name: &str,
            cloud: Pin<&mut PointCloud_PointXYZRGB>,
        ) -> i32;

        // Filter types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type PassThrough_PointXYZ;
        #[namespace = "pcl"]
        type PassThrough_PointXYZRGB;
        #[namespace = "pcl"]
        type VoxelGrid_PointXYZ;
        #[namespace = "pcl"]
        type VoxelGrid_PointXYZRGB;
        #[namespace = "pcl"]
        type StatisticalOutlierRemoval_PointXYZ;
        #[namespace = "pcl"]
        type StatisticalOutlierRemoval_PointXYZRGB;
        #[namespace = "pcl"]
        type RadiusOutlierRemoval_PointXYZ;
        #[namespace = "pcl"]
        type RadiusOutlierRemoval_PointXYZRGB;

        // Filter functions - PassThrough PointXYZ
        #[cfg(feature = "filters")]
        fn new_pass_through_xyz() -> UniquePtr<PassThrough_PointXYZ>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_pass_xyz(
            filter: Pin<&mut PassThrough_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "filters")]
        fn set_filter_field_name_xyz(filter: Pin<&mut PassThrough_PointXYZ>, field_name: &str);
        #[cfg(feature = "filters")]
        fn get_filter_field_name_xyz(filter: &PassThrough_PointXYZ) -> String;
        #[cfg(feature = "filters")]
        fn set_filter_limits_xyz(filter: Pin<&mut PassThrough_PointXYZ>, min: f32, max: f32);
        #[cfg(feature = "filters")]
        fn set_filter_limits_negative_xyz(filter: Pin<&mut PassThrough_PointXYZ>, negative: bool);
        #[cfg(feature = "filters")]
        fn get_filter_limits_negative_xyz(filter: &PassThrough_PointXYZ) -> bool;
        #[cfg(feature = "filters")]
        fn set_keep_organized_xyz(filter: Pin<&mut PassThrough_PointXYZ>, keep_organized: bool);
        #[cfg(feature = "filters")]
        fn get_keep_organized_xyz(filter: &PassThrough_PointXYZ) -> bool;
        #[cfg(feature = "filters")]
        fn filter_pass_xyz(
            filter: Pin<&mut PassThrough_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - PassThrough PointXYZRGB
        #[cfg(feature = "filters")]
        fn new_pass_through_xyzrgb() -> UniquePtr<PassThrough_PointXYZRGB>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_pass_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "filters")]
        fn set_filter_field_name_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            field_name: &str,
        );
        #[cfg(feature = "filters")]
        fn get_filter_field_name_xyzrgb(filter: &PassThrough_PointXYZRGB) -> String;
        #[cfg(feature = "filters")]
        fn set_filter_limits_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, min: f32, max: f32);
        #[cfg(feature = "filters")]
        fn set_filter_limits_negative_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            negative: bool,
        );
        #[cfg(feature = "filters")]
        fn get_filter_limits_negative_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        #[cfg(feature = "filters")]
        fn set_keep_organized_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            keep_organized: bool,
        );
        #[cfg(feature = "filters")]
        fn get_keep_organized_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        #[cfg(feature = "filters")]
        fn filter_pass_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - VoxelGrid PointXYZ
        #[cfg(feature = "filters")]
        fn new_voxel_grid_xyz() -> UniquePtr<VoxelGrid_PointXYZ>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_voxel_xyz(
            filter: Pin<&mut VoxelGrid_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "filters")]
        fn set_leaf_size_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, lx: f32, ly: f32, lz: f32);
        #[cfg(feature = "filters")]
        fn filter_voxel_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>)
        -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - VoxelGrid PointXYZRGB
        #[cfg(feature = "filters")]
        fn new_voxel_grid_xyzrgb() -> UniquePtr<VoxelGrid_PointXYZRGB>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_voxel_xyzrgb(
            filter: Pin<&mut VoxelGrid_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "filters")]
        fn set_leaf_size_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, lx: f32, ly: f32, lz: f32);
        #[cfg(feature = "filters")]
        fn filter_voxel_xyzrgb(
            filter: Pin<&mut VoxelGrid_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - StatisticalOutlierRemoval PointXYZ
        #[cfg(feature = "filters")]
        fn new_statistical_outlier_removal_xyz() -> UniquePtr<StatisticalOutlierRemoval_PointXYZ>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "filters")]
        fn set_mean_k_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            mean_k: i32,
        );
        #[cfg(feature = "filters")]
        fn set_std_dev_mul_thresh_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            stddev_mult: f64,
        );
        #[cfg(feature = "filters")]
        fn set_negative_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            negative: bool,
        );
        #[cfg(feature = "filters")]
        fn filter_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - StatisticalOutlierRemoval PointXYZRGB
        #[cfg(feature = "filters")]
        fn new_statistical_outlier_removal_xyzrgb()
        -> UniquePtr<StatisticalOutlierRemoval_PointXYZRGB>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "filters")]
        fn set_mean_k_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            mean_k: i32,
        );
        #[cfg(feature = "filters")]
        fn set_std_dev_mul_thresh_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            stddev_mult: f64,
        );
        #[cfg(feature = "filters")]
        fn set_negative_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            negative: bool,
        );
        #[cfg(feature = "filters")]
        fn filter_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - RadiusOutlierRemoval PointXYZ
        #[cfg(feature = "filters")]
        fn new_radius_outlier_removal_xyz() -> UniquePtr<RadiusOutlierRemoval_PointXYZ>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "filters")]
        fn set_radius_search_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, radius: f64);
        #[cfg(feature = "filters")]
        fn set_min_neighbors_in_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
            min_neighbors: i32,
        );
        #[cfg(feature = "filters")]
        fn set_negative_radius_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, negative: bool);
        #[cfg(feature = "filters")]
        fn filter_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - RadiusOutlierRemoval PointXYZRGB
        #[cfg(feature = "filters")]
        fn new_radius_outlier_removal_xyzrgb() -> UniquePtr<RadiusOutlierRemoval_PointXYZRGB>;
        #[cfg(feature = "filters")]
        fn set_input_cloud_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "filters")]
        fn set_radius_search_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            radius: f64,
        );
        #[cfg(feature = "filters")]
        fn set_min_neighbors_in_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            min_neighbors: i32,
        );
        #[cfg(feature = "filters")]
        fn set_negative_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            negative: bool,
        );
        #[cfg(feature = "filters")]
        fn filter_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Surface reconstruction types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type MarchingCubesHoppe_PointXYZ;
        #[namespace = "pcl"]
        type MarchingCubesRBF_PointXYZ;
        #[namespace = "pcl"]
        type OrganizedFastMesh_PointXYZ;
        #[namespace = "pcl"]
        type PolygonMesh;
        #[namespace = "pcl"]
        type PointCloud_PointNormal;

        // Feature types - conditionally compiled but types always available
        #[namespace = "pcl"]
        type Normal;
        #[namespace = "pcl"]
        type FPFHSignature33;
        #[namespace = "pcl"]
        type PFHSignature125;
        #[namespace = "pcl"]
        type PointCloud_Normal;
        #[namespace = "pcl"]
        type PointCloud_FPFHSignature33;
        #[namespace = "pcl"]
        type PointCloud_PFHSignature125;
        #[namespace = "pcl"]
        type NormalEstimation_PointXYZ_Normal;
        #[namespace = "pcl"]
        type NormalEstimationOMP_PointXYZ_Normal;
        #[namespace = "pcl"]
        type FPFHEstimation_PointXYZ_Normal_FPFHSignature33;
        #[namespace = "pcl"]
        type FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33;
        #[namespace = "pcl"]
        type PFHEstimation_PointXYZ_Normal_PFHSignature125;

        #[namespace = "pcl"]
        type Poisson_PointNormal;
        #[namespace = "pcl"]
        type GreedyProjectionTriangulation_PointNormal;
        #[namespace = "pcl"]
        type MovingLeastSquares_PointXYZ_PointNormal;

        // Marching Cubes Hoppe reconstruction functions
        #[cfg(feature = "surface")]
        fn new_marching_cubes_hoppe_xyz() -> UniquePtr<MarchingCubesHoppe_PointXYZ>;
        #[cfg(feature = "surface")]
        fn set_iso_level_hoppe_xyz(mc: Pin<&mut MarchingCubesHoppe_PointXYZ>, iso_level: f32);
        #[cfg(feature = "surface")]
        fn get_iso_level_hoppe_xyz(mc: Pin<&mut MarchingCubesHoppe_PointXYZ>) -> f32;
        #[cfg(feature = "surface")]
        fn set_grid_resolution_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            res_x: i32,
            res_y: i32,
            res_z: i32,
        );
        #[cfg(feature = "surface")]
        fn set_percentage_extend_grid_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            percentage: f32,
        );
        #[cfg(feature = "surface")]
        fn set_input_cloud_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "surface")]
        fn perform_reconstruction_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Marching Cubes RBF reconstruction functions
        #[cfg(feature = "surface")]
        fn new_marching_cubes_rbf_xyz() -> UniquePtr<MarchingCubesRBF_PointXYZ>;
        #[cfg(feature = "surface")]
        fn set_iso_level_rbf_xyz(mc: Pin<&mut MarchingCubesRBF_PointXYZ>, iso_level: f32);
        #[cfg(feature = "surface")]
        fn get_iso_level_rbf_xyz(mc: Pin<&mut MarchingCubesRBF_PointXYZ>) -> f32;
        #[cfg(feature = "surface")]
        fn set_grid_resolution_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            res_x: i32,
            res_y: i32,
            res_z: i32,
        );
        #[cfg(feature = "surface")]
        fn set_percentage_extend_grid_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            percentage: f32,
        );
        #[cfg(feature = "surface")]
        fn set_off_surface_displacement_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            displacement: f32,
        );
        #[cfg(feature = "surface")]
        fn set_input_cloud_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "surface")]
        fn perform_reconstruction_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Organized Fast Mesh functions
        #[cfg(feature = "surface")]
        fn new_organized_fast_mesh_xyz() -> UniquePtr<OrganizedFastMesh_PointXYZ>;
        #[cfg(feature = "surface")]
        fn set_triangle_pixel_size_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            triangle_size: i32,
        );
        #[cfg(feature = "surface")]
        fn get_triangle_pixel_size_xyz(ofm: &OrganizedFastMesh_PointXYZ) -> i32;
        #[cfg(feature = "surface")]
        fn set_triangulation_type_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            triangle_type: i32,
        );
        #[cfg(feature = "surface")]
        fn get_triangulation_type_xyz(ofm: &OrganizedFastMesh_PointXYZ) -> i32;
        #[cfg(feature = "surface")]
        fn set_input_cloud_ofm_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "surface")]
        fn perform_reconstruction_ofm_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Polygon mesh utility functions
        #[cfg(feature = "surface")]
        fn new_polygon_mesh() -> UniquePtr<PolygonMesh>;
        #[cfg(feature = "surface")]
        fn get_polygon_count(mesh: &PolygonMesh) -> usize;
        #[cfg(feature = "surface")]
        fn get_vertex_count(mesh: &PolygonMesh) -> usize;
        #[cfg(feature = "surface")]
        fn is_valid_mesh(mesh: &PolygonMesh) -> bool;
        #[cfg(feature = "surface")]
        fn save_polygon_mesh_ply(mesh: &PolygonMesh, filename: &str) -> i32;
        #[cfg(feature = "surface")]
        fn save_polygon_mesh_obj(mesh: &PolygonMesh, filename: &str) -> i32;
        #[cfg(feature = "surface")]
        fn save_polygon_mesh_vtk(mesh: &PolygonMesh, filename: &str) -> i32;

        // Poisson Surface Reconstruction
        #[cfg(feature = "surface")]
        fn new_poisson() -> UniquePtr<Poisson_PointNormal>;
        #[cfg(feature = "surface")]
        fn set_depth_poisson(poisson: Pin<&mut Poisson_PointNormal>, depth: i32);
        #[cfg(feature = "surface")]
        fn get_depth_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> i32;
        #[cfg(feature = "surface")]
        fn set_min_depth_poisson(poisson: Pin<&mut Poisson_PointNormal>, min_depth: i32);
        #[cfg(feature = "surface")]
        fn get_min_depth_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> i32;
        #[cfg(feature = "surface")]
        fn set_point_weight_poisson(poisson: Pin<&mut Poisson_PointNormal>, weight: f32);
        #[cfg(feature = "surface")]
        fn get_point_weight_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> f32;
        #[cfg(feature = "surface")]
        fn set_scale_poisson(poisson: Pin<&mut Poisson_PointNormal>, scale: f32);
        #[cfg(feature = "surface")]
        fn get_scale_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> f32;
        #[cfg(feature = "surface")]
        fn set_solver_divide_poisson(poisson: Pin<&mut Poisson_PointNormal>, solver_divide: i32);
        #[cfg(feature = "surface")]
        fn get_solver_divide_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> i32;
        #[cfg(feature = "surface")]
        fn set_iso_divide_poisson(poisson: Pin<&mut Poisson_PointNormal>, iso_divide: i32);
        #[cfg(feature = "surface")]
        fn get_iso_divide_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> i32;
        #[cfg(feature = "surface")]
        fn set_samples_per_node_poisson(
            poisson: Pin<&mut Poisson_PointNormal>,
            samples_per_node: f32,
        );
        #[cfg(feature = "surface")]
        fn get_samples_per_node_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> f32;
        #[cfg(feature = "surface")]
        fn set_confidence_poisson(poisson: Pin<&mut Poisson_PointNormal>, confidence: bool);
        #[cfg(feature = "surface")]
        fn get_confidence_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> bool;
        #[cfg(feature = "surface")]
        fn set_output_polygons_poisson(
            poisson: Pin<&mut Poisson_PointNormal>,
            output_polygons: bool,
        );
        #[cfg(feature = "surface")]
        fn get_output_polygons_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> bool;
        #[cfg(feature = "surface")]
        fn set_degree_poisson(poisson: Pin<&mut Poisson_PointNormal>, degree: i32);
        #[cfg(feature = "surface")]
        fn get_degree_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> i32;
        #[cfg(feature = "surface")]
        fn set_manifold_poisson(poisson: Pin<&mut Poisson_PointNormal>, manifold: bool);
        #[cfg(feature = "surface")]
        fn get_manifold_poisson(poisson: Pin<&mut Poisson_PointNormal>) -> bool;
        #[cfg(feature = "surface")]
        fn set_input_cloud_poisson(
            poisson: Pin<&mut Poisson_PointNormal>,
            cloud: &PointCloud_PointNormal,
        );
        #[cfg(feature = "surface")]
        fn reconstruct_mesh_poisson(
            poisson: Pin<&mut Poisson_PointNormal>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Greedy Projection Triangulation
        #[cfg(feature = "surface")]
        fn new_greedy_projection_triangulation()
        -> UniquePtr<GreedyProjectionTriangulation_PointNormal>;
        #[cfg(feature = "surface")]
        fn set_mu_greedy(gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>, mu: f64);
        #[cfg(feature = "surface")]
        fn get_mu_greedy(gpt: &GreedyProjectionTriangulation_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_search_radius_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            radius: f64,
        );
        #[cfg(feature = "surface")]
        fn get_search_radius_greedy(gpt: &GreedyProjectionTriangulation_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_minimum_angle_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            angle: f64,
        );
        #[cfg(feature = "surface")]
        fn get_minimum_angle_greedy(gpt: &GreedyProjectionTriangulation_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_maximum_angle_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            angle: f64,
        );
        #[cfg(feature = "surface")]
        fn get_maximum_angle_greedy(gpt: &GreedyProjectionTriangulation_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_maximum_nearest_neighbors_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            max_nn: i32,
        );
        #[cfg(feature = "surface")]
        fn get_maximum_nearest_neighbors_greedy(
            gpt: &GreedyProjectionTriangulation_PointNormal,
        ) -> i32;
        #[cfg(feature = "surface")]
        fn set_maximum_surface_angle_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            angle: f64,
        );
        #[cfg(feature = "surface")]
        fn get_maximum_surface_angle_greedy(gpt: &GreedyProjectionTriangulation_PointNormal)
        -> f64;
        #[cfg(feature = "surface")]
        fn set_normal_consistency_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            consistent: bool,
        );
        #[cfg(feature = "surface")]
        fn get_normal_consistency_greedy(gpt: &GreedyProjectionTriangulation_PointNormal) -> bool;
        #[cfg(feature = "surface")]
        fn set_consistent_vertex_ordering_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            consistent: bool,
        );
        #[cfg(feature = "surface")]
        fn get_consistent_vertex_ordering_greedy(
            gpt: &GreedyProjectionTriangulation_PointNormal,
        ) -> bool;
        #[cfg(feature = "surface")]
        fn set_input_cloud_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            cloud: &PointCloud_PointNormal,
        );
        #[cfg(feature = "surface")]
        fn reconstruct_mesh_greedy(
            gpt: Pin<&mut GreedyProjectionTriangulation_PointNormal>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Moving Least Squares
        #[cfg(feature = "surface")]
        fn new_moving_least_squares() -> UniquePtr<MovingLeastSquares_PointXYZ_PointNormal>;
        #[cfg(feature = "surface")]
        fn set_search_radius_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            radius: f64,
        );
        #[cfg(feature = "surface")]
        fn get_search_radius_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_polynomial_order_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            order: i32,
        );
        #[cfg(feature = "surface")]
        fn get_polynomial_order_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> i32;
        #[cfg(feature = "surface")]
        fn set_sqr_gauss_param_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            sqr_gauss_param: f64,
        );
        #[cfg(feature = "surface")]
        fn get_sqr_gauss_param_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_compute_normals_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            compute_normals: bool,
        );
        #[cfg(feature = "surface")]
        fn set_upsample_method_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            method: i32,
        );
        #[cfg(feature = "surface")]
        fn set_upsampling_radius_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            radius: f64,
        );
        #[cfg(feature = "surface")]
        fn get_upsampling_radius_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_upsampling_step_size_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            step_size: f64,
        );
        #[cfg(feature = "surface")]
        fn get_upsampling_step_size_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> f64;
        #[cfg(feature = "surface")]
        fn set_desired_num_points_in_radius_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            num_points: i32,
        );
        #[cfg(feature = "surface")]
        fn set_dilation_voxel_size_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            voxel_size: f32,
        );
        #[cfg(feature = "surface")]
        fn get_dilation_voxel_size_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> f32;
        #[cfg(feature = "surface")]
        fn set_dilation_iterations_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            iterations: i32,
        );
        #[cfg(feature = "surface")]
        fn get_dilation_iterations_mls(mls: &MovingLeastSquares_PointXYZ_PointNormal) -> i32;
        #[cfg(feature = "surface")]
        fn set_input_cloud_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "surface")]
        fn process_mls(
            mls: Pin<&mut MovingLeastSquares_PointXYZ_PointNormal>,
        ) -> UniquePtr<PointCloud_PointNormal>;

        // Visualization types - always available but functions conditionally compiled
        #[namespace = "pcl::visualization"]
        type PCLVisualizer;
        #[namespace = "pcl::visualization"]
        type CloudViewer;

        // PCLVisualizer functions
        #[cfg(feature = "visualization")]
        fn new_pcl_visualizer(window_name: &str) -> UniquePtr<PCLVisualizer>;
        #[cfg(feature = "visualization")]
        fn add_point_cloud_xyz(
            viewer: Pin<&mut PCLVisualizer>,
            cloud: &PointCloud_PointXYZ,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn add_point_cloud_xyzrgb(
            viewer: Pin<&mut PCLVisualizer>,
            cloud: &PointCloud_PointXYZRGB,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn update_point_cloud_xyz(
            viewer: Pin<&mut PCLVisualizer>,
            cloud: &PointCloud_PointXYZ,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn update_point_cloud_xyzrgb(
            viewer: Pin<&mut PCLVisualizer>,
            cloud: &PointCloud_PointXYZRGB,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn remove_point_cloud(viewer: Pin<&mut PCLVisualizer>, id: &str) -> i32;
        #[cfg(feature = "visualization")]
        fn set_point_cloud_render_properties_xyz(
            viewer: Pin<&mut PCLVisualizer>,
            property: i32,
            value: f64,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn set_background_color(viewer: Pin<&mut PCLVisualizer>, r: f64, g: f64, b: f64) -> i32;
        #[cfg(feature = "visualization")]
        fn add_coordinate_system(viewer: Pin<&mut PCLVisualizer>, scale: f64, id: &str) -> i32;
        #[cfg(feature = "visualization")]
        fn spin_once(viewer: Pin<&mut PCLVisualizer>, time: i32) -> i32;
        #[cfg(feature = "visualization")]
        fn spin(viewer: Pin<&mut PCLVisualizer>);
        #[cfg(feature = "visualization")]
        fn was_stopped(viewer: &PCLVisualizer) -> bool;
        #[cfg(feature = "visualization")]
        fn close(viewer: Pin<&mut PCLVisualizer>);
        #[cfg(feature = "visualization")]
        fn reset_stopped_flag(viewer: Pin<&mut PCLVisualizer>);

        // Camera control functions
        #[cfg(feature = "visualization")]
        #[allow(clippy::too_many_arguments)]
        fn set_camera_position(
            viewer: Pin<&mut PCLVisualizer>,
            pos_x: f64,
            pos_y: f64,
            pos_z: f64,
            view_x: f64,
            view_y: f64,
            view_z: f64,
            up_x: f64,
            up_y: f64,
            up_z: f64,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn reset_camera(viewer: Pin<&mut PCLVisualizer>) -> i32;

        // CloudViewer functions (simpler interface)
        #[cfg(feature = "visualization")]
        fn new_cloud_viewer(window_name: &str) -> UniquePtr<CloudViewer>;
        #[cfg(feature = "visualization")]
        fn show_cloud_xyz(
            viewer: Pin<&mut CloudViewer>,
            cloud: &PointCloud_PointXYZ,
            cloud_name: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn show_cloud_xyzrgb(
            viewer: Pin<&mut CloudViewer>,
            cloud: &PointCloud_PointXYZRGB,
            cloud_name: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn cloud_viewer_was_stopped(viewer: &CloudViewer) -> bool;
        #[cfg(feature = "visualization")]
        fn wait_for_cloud_viewer(viewer: Pin<&mut CloudViewer>, time_ms: i32);

        // Additional utility functions
        #[cfg(feature = "visualization")]
        fn set_point_cloud_color_xyz(
            viewer: Pin<&mut PCLVisualizer>,
            r: f64,
            g: f64,
            b: f64,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        #[allow(clippy::too_many_arguments)]
        fn add_text(
            viewer: Pin<&mut PCLVisualizer>,
            text: &str,
            xpos: i32,
            ypos: i32,
            r: f64,
            g: f64,
            b: f64,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn add_sphere_xyz(
            viewer: Pin<&mut PCLVisualizer>,
            center: &PointXYZ,
            radius: f64,
            r: f64,
            g: f64,
            b: f64,
            id: &str,
        ) -> i32;
        #[cfg(feature = "visualization")]
        fn remove_shape(viewer: Pin<&mut PCLVisualizer>, id: &str) -> i32;
        #[cfg(feature = "visualization")]
        fn register_keyboard_callback(viewer: Pin<&mut PCLVisualizer>) -> i32;

        // Sample consensus types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type RandomSampleConsensus_PointXYZ;
        #[namespace = "pcl"]
        type RandomSampleConsensus_PointXYZRGB;
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZRGB;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZRGB;

        // RANSAC functions - PointXYZ
        #[cfg(feature = "sample_consensus")]
        fn new_ransac_plane_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;
        #[cfg(feature = "sample_consensus")]
        fn new_ransac_sphere_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;
        #[cfg(feature = "sample_consensus")]
        fn set_distance_threshold_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            threshold: f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_distance_threshold_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> f64;
        #[cfg(feature = "sample_consensus")]
        fn set_max_iterations_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            max_iterations: i32,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_max_iterations_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn set_probability_xyz(ransac: Pin<&mut RandomSampleConsensus_PointXYZ>, probability: f64);
        #[cfg(feature = "sample_consensus")]
        fn get_probability_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> f64;
        #[cfg(feature = "sample_consensus")]
        fn compute_model_xyz(ransac: Pin<&mut RandomSampleConsensus_PointXYZ>) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn refine_model_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            sigma: f64,
            max_iterations: u32,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_inliers_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn get_model_coefficients_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<f32>;
        #[cfg(feature = "sample_consensus")]
        fn get_inliers_count_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> usize;

        // RANSAC functions - PointXYZRGB
        #[cfg(feature = "sample_consensus")]
        fn new_ransac_plane_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;
        #[cfg(feature = "sample_consensus")]
        fn new_ransac_sphere_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;
        #[cfg(feature = "sample_consensus")]
        fn set_distance_threshold_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            threshold: f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_distance_threshold_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> f64;
        #[cfg(feature = "sample_consensus")]
        fn set_max_iterations_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            max_iterations: i32,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_max_iterations_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn set_probability_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            probability: f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_probability_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> f64;
        #[cfg(feature = "sample_consensus")]
        fn compute_model_xyzrgb(ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn refine_model_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            sigma: f64,
            max_iterations: u32,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_inliers_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn get_model_coefficients_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> Vec<f32>;
        #[cfg(feature = "sample_consensus")]
        fn get_inliers_count_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> usize;

        // Model functions - PointXYZ plane
        #[cfg(feature = "sample_consensus")]
        fn new_sac_model_plane_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<SampleConsensusModelPlane_PointXYZ>;
        #[cfg(feature = "sample_consensus")]
        fn compute_model_coefficients_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            sample_indices: &Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_distances_to_model_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            model_coefficients: &Vec<f32>,
            distances: &mut Vec<f64>,
        );
        #[cfg(feature = "sample_consensus")]
        fn select_within_distance_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn count_within_distance_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn optimize_model_coefficients_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            optimized_coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn project_points_plane_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            copy_data_fields: bool,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Model functions - PointXYZ sphere
        #[cfg(feature = "sample_consensus")]
        fn new_sac_model_sphere_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<SampleConsensusModelSphere_PointXYZ>;
        #[cfg(feature = "sample_consensus")]
        fn set_radius_limits_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            min_radius: f64,
            max_radius: f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_radius_limits_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            min_radius: &mut f64,
            max_radius: &mut f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn compute_model_coefficients_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            sample_indices: &Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_distances_to_model_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            model_coefficients: &Vec<f32>,
            distances: &mut Vec<f64>,
        );
        #[cfg(feature = "sample_consensus")]
        fn select_within_distance_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn count_within_distance_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn optimize_model_coefficients_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            optimized_coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn project_points_sphere_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            copy_data_fields: bool,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Model functions - PointXYZRGB plane
        #[cfg(feature = "sample_consensus")]
        fn new_sac_model_plane_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<SampleConsensusModelPlane_PointXYZRGB>;
        #[cfg(feature = "sample_consensus")]
        fn compute_model_coefficients_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            sample_indices: &Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_distances_to_model_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            distances: &mut Vec<f64>,
        );
        #[cfg(feature = "sample_consensus")]
        fn select_within_distance_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn count_within_distance_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn optimize_model_coefficients_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            optimized_coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn project_points_plane_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            copy_data_fields: bool,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Model functions - PointXYZRGB sphere
        #[cfg(feature = "sample_consensus")]
        fn new_sac_model_sphere_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<SampleConsensusModelSphere_PointXYZRGB>;
        #[cfg(feature = "sample_consensus")]
        fn set_radius_limits_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            min_radius: f64,
            max_radius: f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn get_radius_limits_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            min_radius: &mut f64,
            max_radius: &mut f64,
        );
        #[cfg(feature = "sample_consensus")]
        fn compute_model_coefficients_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            sample_indices: &Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn get_distances_to_model_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            distances: &mut Vec<f64>,
        );
        #[cfg(feature = "sample_consensus")]
        fn select_within_distance_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> Vec<i32>;
        #[cfg(feature = "sample_consensus")]
        fn count_within_distance_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            model_coefficients: &Vec<f32>,
            threshold: f64,
        ) -> i32;
        #[cfg(feature = "sample_consensus")]
        fn optimize_model_coefficients_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            optimized_coefficients: &mut Vec<f32>,
        ) -> bool;
        #[cfg(feature = "sample_consensus")]
        fn project_points_sphere_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            inliers: &Vec<i32>,
            model_coefficients: &Vec<f32>,
            copy_data_fields: bool,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Feature estimation functions - Normal estimation PointXYZ
        #[cfg(feature = "features")]
        fn new_normal_estimation_xyz() -> UniquePtr<NormalEstimation_PointXYZ_Normal>;
        #[cfg(feature = "features")]
        fn set_input_cloud_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_search_method_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_radius_search_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            radius: f64,
        );
        #[cfg(feature = "features")]
        fn get_radius_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> f64;
        #[cfg(feature = "features")]
        fn set_k_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>, k: i32);
        #[cfg(feature = "features")]
        fn get_k_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> i32;
        #[cfg(feature = "features")]
        fn set_view_point_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            vpx: f32,
            vpy: f32,
            vpz: f32,
        );
        #[cfg(feature = "features")]
        fn get_view_point_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> Vec<f32>;
        #[cfg(feature = "features")]
        fn set_use_sensor_origin_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            use_sensor_origin: bool,
        );
        #[cfg(feature = "features")]
        fn compute_normals_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
        ) -> UniquePtr<PointCloud_Normal>;

        // Normal estimation OMP functions
        #[cfg(feature = "features")]
        fn new_normal_estimation_omp_xyz() -> UniquePtr<NormalEstimationOMP_PointXYZ_Normal>;
        #[cfg(feature = "features")]
        fn set_input_cloud_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_search_method_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_radius_search_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            radius: f64,
        );
        #[cfg(feature = "features")]
        fn set_k_search_normal_omp_xyz(ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>, k: i32);
        #[cfg(feature = "features")]
        fn set_number_of_threads_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            threads: i32,
        );
        #[cfg(feature = "features")]
        fn get_number_of_threads_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
        ) -> i32;
        #[cfg(feature = "features")]
        fn compute_normals_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
        ) -> UniquePtr<PointCloud_Normal>;

        // FPFH estimation functions
        #[cfg(feature = "features")]
        fn new_fpfh_estimation_xyz() -> UniquePtr<FPFHEstimation_PointXYZ_Normal_FPFHSignature33>;
        #[cfg(feature = "features")]
        fn set_input_cloud_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_input_normals_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            normals: &PointCloud_Normal,
        );
        #[cfg(feature = "features")]
        fn set_search_method_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_radius_search_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            radius: f64,
        );
        #[cfg(feature = "features")]
        fn set_k_search_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            k: i32,
        );
        #[cfg(feature = "features")]
        fn compute_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
        ) -> UniquePtr<PointCloud_FPFHSignature33>;

        // FPFH OMP estimation functions
        #[cfg(feature = "features")]
        fn new_fpfh_estimation_omp_xyz()
        -> UniquePtr<FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>;
        #[cfg(feature = "features")]
        fn set_input_cloud_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_input_normals_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            normals: &PointCloud_Normal,
        );
        #[cfg(feature = "features")]
        fn set_search_method_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_radius_search_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            radius: f64,
        );
        #[cfg(feature = "features")]
        fn set_number_of_threads_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            threads: i32,
        );
        #[cfg(feature = "features")]
        fn compute_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
        ) -> UniquePtr<PointCloud_FPFHSignature33>;
        // PFH estimation functions
        #[cfg(feature = "features")]
        fn new_pfh_estimation_xyz() -> UniquePtr<PFHEstimation_PointXYZ_Normal_PFHSignature125>;
        #[cfg(feature = "features")]
        fn set_input_cloud_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_input_normals_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            normals: &PointCloud_Normal,
        );
        #[cfg(feature = "features")]
        fn set_search_method_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "features")]
        fn set_radius_search_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            radius: f64,
        );
        #[cfg(feature = "features")]
        fn set_k_search_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            k: i32,
        );
        #[cfg(feature = "features")]
        fn compute_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
        ) -> UniquePtr<PointCloud_PFHSignature125>;
        // Helper functions for feature extraction
        #[cfg(feature = "features")]
        fn get_fpfh_histogram(signature: &FPFHSignature33) -> Vec<f32>;
        #[cfg(feature = "features")]
        fn get_pfh_histogram(signature: &PFHSignature125) -> Vec<f32>;
        #[cfg(feature = "features")]
        fn get_normal_vector(normal: &Normal) -> Vec<f32>;

        // Feature cloud functions
        #[cfg(feature = "features")]
        fn size_normal(cloud: &PointCloud_Normal) -> usize;
        #[cfg(feature = "features")]
        fn empty_normal(cloud: &PointCloud_Normal) -> bool;
        #[cfg(feature = "features")]
        fn size_fpfh(cloud: &PointCloud_FPFHSignature33) -> usize;
        #[cfg(feature = "features")]
        fn empty_fpfh(cloud: &PointCloud_FPFHSignature33) -> bool;
        #[cfg(feature = "features")]
        fn size_pfh(cloud: &PointCloud_PFHSignature125) -> usize;
        #[cfg(feature = "features")]
        fn empty_pfh(cloud: &PointCloud_PFHSignature125) -> bool;

        // Individual feature access functions
        #[cfg(feature = "features")]
        fn get_normal_at(cloud: &PointCloud_Normal, index: usize) -> Vec<f32>;
        #[cfg(feature = "features")]
        fn get_fpfh_signature_at(cloud: &PointCloud_FPFHSignature33, index: usize) -> Vec<f32>;
        #[cfg(feature = "features")]
        fn get_pfh_signature_at(cloud: &PointCloud_PFHSignature125, index: usize) -> Vec<f32>;

        // Note: Bulk histogram access functions not supported due to cxx Vec<Vec<T>> limitation
        // Individual access via get_fpfh_signature_at and get_pfh_signature_at should be used instead

        // Registration types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type IterativeClosestPoint_PointXYZ;
        #[namespace = "pcl"]
        type IterativeClosestPoint_PointXYZRGB;
        #[namespace = "pcl"]
        type NormalDistributionsTransform_PointXYZ;
        #[namespace = "pcl"]
        type NormalDistributionsTransform_PointXYZRGB;

        // Registration functions - ICP for PointXYZ
        #[cfg(feature = "registration")]
        fn new_icp_xyz() -> UniquePtr<IterativeClosestPoint_PointXYZ>;
        #[cfg(feature = "registration")]
        fn set_input_source_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_max_iterations_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>, max_iter: i32);
        #[cfg(feature = "registration")]
        fn get_max_iterations_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> i32;
        #[cfg(feature = "registration")]
        fn set_transformation_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_transformation_epsilon_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>)
        -> f64;
        #[cfg(feature = "registration")]
        fn set_euclidean_fitness_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_euclidean_fitness_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_max_correspondence_distance_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            distance: f64,
        );
        #[cfg(feature = "registration")]
        fn get_max_correspondence_distance_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn align_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        #[cfg(feature = "registration")]
        fn align_with_guess_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        #[cfg(feature = "registration")]
        fn has_converged_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> bool;
        #[cfg(feature = "registration")]
        fn get_fitness_score_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> f64;
        #[cfg(feature = "registration")]
        fn get_final_transformation_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> Vec<f32>;

        // Registration functions - ICP for PointXYZRGB
        #[cfg(feature = "registration")]
        fn new_icp_xyzrgb() -> UniquePtr<IterativeClosestPoint_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn set_input_source_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "registration")]
        fn set_max_iterations_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            max_iter: i32,
        );
        #[cfg(feature = "registration")]
        fn get_max_iterations_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> i32;
        #[cfg(feature = "registration")]
        fn set_transformation_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_transformation_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_euclidean_fitness_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_euclidean_fitness_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_max_correspondence_distance_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            distance: f64,
        );
        #[cfg(feature = "registration")]
        fn get_max_correspondence_distance_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn align_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn align_with_guess_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn has_converged_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> bool;
        #[cfg(feature = "registration")]
        fn get_fitness_score_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> f64;
        #[cfg(feature = "registration")]
        fn get_final_transformation_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> Vec<f32>;

        // NDT registration functions - PointXYZ
        #[cfg(feature = "registration")]
        fn new_ndt_xyz() -> UniquePtr<NormalDistributionsTransform_PointXYZ>;
        #[cfg(feature = "registration")]
        fn set_input_source_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_resolution_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            resolution: f64,
        );
        #[cfg(feature = "registration")]
        fn get_resolution_ndt_xyz(ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>) -> f64;
        #[cfg(feature = "registration")]
        fn set_max_iterations_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            iterations: i32,
        );
        #[cfg(feature = "registration")]
        fn get_max_iterations_ndt_xyz(ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>) -> i32;
        #[cfg(feature = "registration")]
        fn set_transformation_epsilon_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_transformation_epsilon_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_step_size_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            step_size: f64,
        );
        #[cfg(feature = "registration")]
        fn get_step_size_ndt_xyz(ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>) -> f64;
        #[cfg(feature = "registration")]
        fn align_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        #[cfg(feature = "registration")]
        fn align_with_guess_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        #[cfg(feature = "registration")]
        fn has_converged_ndt_xyz(ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>) -> bool;
        #[cfg(feature = "registration")]
        fn get_fitness_score_ndt_xyz(ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>) -> f64;
        #[cfg(feature = "registration")]
        fn get_final_transformation_ndt_xyz(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZ>,
        ) -> Vec<f32>;

        // NDT registration functions - PointXYZRGB
        #[cfg(feature = "registration")]
        fn new_ndt_xyzrgb() -> UniquePtr<NormalDistributionsTransform_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn set_input_source_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "registration")]
        fn set_resolution_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            resolution: f64,
        );
        #[cfg(feature = "registration")]
        fn get_resolution_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_max_iterations_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            iterations: i32,
        );
        #[cfg(feature = "registration")]
        fn get_max_iterations_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> i32;
        #[cfg(feature = "registration")]
        fn set_transformation_epsilon_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            epsilon: f64,
        );
        #[cfg(feature = "registration")]
        fn get_transformation_epsilon_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn set_step_size_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            step_size: f64,
        );
        #[cfg(feature = "registration")]
        fn get_step_size_ndt_xyzrgb(ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>)
        -> f64;
        #[cfg(feature = "registration")]
        fn align_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn align_with_guess_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        #[cfg(feature = "registration")]
        fn has_converged_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> bool;
        #[cfg(feature = "registration")]
        fn get_fitness_score_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn get_final_transformation_ndt_xyzrgb(
            ndt: Pin<&mut NormalDistributionsTransform_PointXYZRGB>,
        ) -> Vec<f32>;

        // Correspondence estimation types
        #[namespace = "pcl::registration"]
        type CorrespondenceEstimation_PointXYZ;
        #[namespace = "pcl::registration"]
        type CorrespondenceRejectorSampleConsensus_PointXYZ;
        #[namespace = "pcl::registration"]
        type TransformationEstimationSVD_PointXYZ;

        // Correspondence estimation functions
        #[cfg(feature = "registration")]
        fn new_correspondence_estimation_xyz() -> UniquePtr<CorrespondenceEstimation_PointXYZ>;
        #[cfg(feature = "registration")]
        fn set_input_source_correspondence_xyz(
            est: Pin<&mut CorrespondenceEstimation_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_correspondence_xyz(
            est: Pin<&mut CorrespondenceEstimation_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn determine_correspondences_xyz(
            est: Pin<&mut CorrespondenceEstimation_PointXYZ>,
            correspondences: &mut Vec<i32>, // Flattened: [source_idx, target_idx, source_idx, target_idx, ...]
            distances: &mut Vec<f32>,       // Corresponding distances
        );
        #[cfg(feature = "registration")]
        fn determine_reciprocal_correspondences_xyz(
            est: Pin<&mut CorrespondenceEstimation_PointXYZ>,
            correspondences: &mut Vec<i32>, // Flattened: [source_idx, target_idx, source_idx, target_idx, ...]
            distances: &mut Vec<f32>,       // Corresponding distances
        );

        // Correspondence rejection functions
        #[cfg(feature = "registration")]
        fn new_correspondence_rejector_sac_xyz()
        -> UniquePtr<CorrespondenceRejectorSampleConsensus_PointXYZ>;
        #[cfg(feature = "registration")]
        fn set_input_source_rejector_xyz(
            rej: Pin<&mut CorrespondenceRejectorSampleConsensus_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_input_target_rejector_xyz(
            rej: Pin<&mut CorrespondenceRejectorSampleConsensus_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "registration")]
        fn set_inlier_threshold_rejector_xyz(
            rej: Pin<&mut CorrespondenceRejectorSampleConsensus_PointXYZ>,
            threshold: f64,
        );
        #[cfg(feature = "registration")]
        fn get_inlier_threshold_rejector_xyz(
            rej: Pin<&mut CorrespondenceRejectorSampleConsensus_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "registration")]
        fn get_correspondences_rejector_xyz(
            rej: Pin<&mut CorrespondenceRejectorSampleConsensus_PointXYZ>,
            correspondences: &Vec<i32>, // Input flattened correspondences
            distances: &Vec<f32>,       // Input distances
            remaining_correspondences: &mut Vec<i32>, // Output flattened correspondences
            remaining_distances: &mut Vec<f32>, // Output distances
        );

        // Transformation estimation functions
        #[cfg(feature = "registration")]
        fn new_transformation_estimation_svd_xyz() -> UniquePtr<TransformationEstimationSVD_PointXYZ>;
        #[cfg(feature = "registration")]
        fn estimate_rigid_transformation_xyz(
            est: Pin<&mut TransformationEstimationSVD_PointXYZ>,
            source: &PointCloud_PointXYZ,
            target: &PointCloud_PointXYZ,
            correspondences: &Vec<i32>, // Flattened correspondences
            distances: &Vec<f32>,       // Corresponding distances
            transformation: &mut Vec<f32>,
        );

        // Keypoints types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type HarrisKeypoint3D_PointXYZ_PointXYZI;
        #[namespace = "pcl"]
        type ISSKeypoint3D_PointXYZ_PointXYZ;
        #[namespace = "pcl"]
        type SIFTKeypoint_PointXYZI_PointWithScale;
        #[namespace = "pcl"]
        type PointWithScale;
        #[namespace = "pcl"]
        type PointCloud_PointWithScale;

        // Harris 3D keypoint detector functions
        #[cfg(feature = "keypoints")]
        fn new_harris_3d_xyz() -> UniquePtr<HarrisKeypoint3D_PointXYZ_PointXYZI>;
        #[cfg(feature = "keypoints")]
        fn set_input_cloud_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "keypoints")]
        fn set_search_method_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "keypoints")]
        fn set_radius_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            radius: f64,
        );
        #[cfg(feature = "keypoints")]
        fn set_threshold_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            threshold: f32,
        );
        #[cfg(feature = "keypoints")]
        fn set_non_max_suppression_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            suppress: bool,
        );
        #[cfg(feature = "keypoints")]
        fn set_refine_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            refine: bool,
        );
        #[cfg(feature = "keypoints")]
        fn compute_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
        ) -> UniquePtr<PointCloud_PointXYZI>;

        // ISS 3D keypoint detector functions
        #[cfg(feature = "keypoints")]
        fn new_iss_3d_xyz() -> UniquePtr<ISSKeypoint3D_PointXYZ_PointXYZ>;
        #[cfg(feature = "keypoints")]
        fn set_input_cloud_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "keypoints")]
        fn set_search_method_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            tree: &KdTree_PointXYZ,
        );
        #[cfg(feature = "keypoints")]
        fn set_salient_radius_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            radius: f64,
        );
        #[cfg(feature = "keypoints")]
        fn set_non_max_radius_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            radius: f64,
        );
        #[cfg(feature = "keypoints")]
        fn set_threshold21_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            threshold: f64,
        );
        #[cfg(feature = "keypoints")]
        fn set_threshold32_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            threshold: f64,
        );
        #[cfg(feature = "keypoints")]
        fn set_min_neighbors_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            min_neighbors: i32,
        );
        #[cfg(feature = "keypoints")]
        fn compute_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // SIFT keypoint detector functions
        #[cfg(feature = "keypoints")]
        fn new_sift_keypoint_xyzi() -> UniquePtr<SIFTKeypoint_PointXYZI_PointWithScale>;
        #[cfg(feature = "keypoints")]
        fn set_input_cloud_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            cloud: &PointCloud_PointXYZI,
        );
        #[cfg(feature = "keypoints")]
        fn set_search_method_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            tree: &KdTree_PointXYZI,
        );
        #[cfg(feature = "keypoints")]
        fn set_scales_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            min_scale: f32,
            nr_octaves: f32,
            nr_scales_per_octave: i32,
        );
        #[cfg(feature = "keypoints")]
        fn set_minimum_contrast_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            min_contrast: f32,
        );
        #[cfg(feature = "keypoints")]
        fn compute_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
        ) -> UniquePtr<PointCloud_PointWithScale>;

        // Helper functions for keypoint data access
        #[cfg(feature = "keypoints")]
        fn get_point_with_scale_coords(point: &PointWithScale) -> Vec<f32>;
        #[cfg(feature = "keypoints")]
        fn get_point_xyzi_coords(point: &PointXYZI) -> Vec<f32>;

        // Segmentation types - always available but functions conditionally compiled
        #[namespace = "pcl"]
        type RegionGrowing_PointXYZ_Normal;
        #[namespace = "pcl"]
        type RegionGrowingRGB_PointXYZRGB;
        #[namespace = "pcl"]
        type EuclideanClusterExtraction_PointXYZ;
        #[namespace = "pcl"]
        type SACSegmentation_PointXYZ;
        #[namespace = "pcl"]
        type MinCutSegmentation_PointXYZ;
        #[namespace = "pcl"]
        type ExtractPolygonalPrismData_PointXYZ;
        #[namespace = "pcl"]
        type ProgressiveMorphologicalFilter_PointXYZ;
        #[namespace = "pcl"]
        type ConditionalEuclideanClustering_PointXYZ;

        // Region Growing segmentation functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_region_growing_xyz() -> UniquePtr<RegionGrowing_PointXYZ_Normal>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_input_normals_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            normals: &PointCloud_Normal,
        );
        #[cfg(feature = "segmentation")]
        fn set_min_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            min_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_min_cluster_size_region_growing_xyz(rg: &RegionGrowing_PointXYZ_Normal) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_max_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            max_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_max_cluster_size_region_growing_xyz(rg: &RegionGrowing_PointXYZ_Normal) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_smoothness_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            threshold: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_smoothness_threshold_region_growing_xyz(rg: &RegionGrowing_PointXYZ_Normal) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_curvature_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            threshold: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_curvature_threshold_region_growing_xyz(rg: &RegionGrowing_PointXYZ_Normal) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_number_of_neighbours_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            k: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_number_of_neighbours_region_growing_xyz(rg: &RegionGrowing_PointXYZ_Normal) -> i32;
        #[cfg(feature = "segmentation")]
        fn extract_region_growing_xyz(rg: Pin<&mut RegionGrowing_PointXYZ_Normal>) -> Vec<i32>;

        // Region Growing RGB segmentation functions - PointXYZRGB
        #[cfg(feature = "segmentation")]
        fn new_region_growing_rgb_xyzrgb() -> UniquePtr<RegionGrowingRGB_PointXYZRGB>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        #[cfg(feature = "segmentation")]
        fn set_distance_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_distance_threshold_region_growing_rgb_xyzrgb(
            rg: &RegionGrowingRGB_PointXYZRGB,
        ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_point_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_point_color_threshold_region_growing_rgb_xyzrgb(
            rg: &RegionGrowingRGB_PointXYZRGB,
        ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_region_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_region_color_threshold_region_growing_rgb_xyzrgb(
            rg: &RegionGrowingRGB_PointXYZRGB,
        ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_min_cluster_size_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            min_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_min_cluster_size_region_growing_rgb_xyzrgb(rg: &RegionGrowingRGB_PointXYZRGB)
        -> i32;
        #[cfg(feature = "segmentation")]
        fn extract_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> Vec<i32>;

        // Euclidean Cluster Extraction functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_euclidean_cluster_extraction_xyz() -> UniquePtr<EuclideanClusterExtraction_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_euclidean_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_cluster_tolerance_euclidean_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            tolerance: f64,
        );
        #[cfg(feature = "segmentation")]
        fn get_cluster_tolerance_euclidean_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> f64;
        #[cfg(feature = "segmentation")]
        fn set_min_cluster_size_euclidean_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            min_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_min_cluster_size_euclidean_xyz(ec: &EuclideanClusterExtraction_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_max_cluster_size_euclidean_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            max_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_max_cluster_size_euclidean_xyz(ec: &EuclideanClusterExtraction_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn extract_euclidean_clusters_xyz(
            ec: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> Vec<i32>;

        // SAC Segmentation functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_sac_segmentation_xyz() -> UniquePtr<SACSegmentation_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_model_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, model_type: i32);
        #[cfg(feature = "segmentation")]
        fn get_model_type_sac_xyz(sac: &SACSegmentation_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_method_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, method_type: i32);
        #[cfg(feature = "segmentation")]
        fn get_method_type_sac_xyz(sac: &SACSegmentation_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_distance_threshold_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, threshold: f64);
        #[cfg(feature = "segmentation")]
        fn get_distance_threshold_sac_xyz(sac: &SACSegmentation_PointXYZ) -> f64;
        #[cfg(feature = "segmentation")]
        fn set_max_iterations_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, max_iter: i32);
        #[cfg(feature = "segmentation")]
        fn get_max_iterations_sac_xyz(sac: &SACSegmentation_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_optimize_coefficients_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            optimize: bool,
        );
        #[cfg(feature = "segmentation")]
        fn get_optimize_coefficients_sac_xyz(sac: &SACSegmentation_PointXYZ) -> bool;
        #[cfg(feature = "segmentation")]
        fn segment_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            inliers: &mut Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;

        // Min-Cut Segmentation functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_min_cut_segmentation_xyz() -> UniquePtr<MinCutSegmentation_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_min_cut_xyz(
            mc: Pin<&mut MinCutSegmentation_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_foreground_points_min_cut_xyz(
            mc: Pin<&mut MinCutSegmentation_PointXYZ>,
            foreground_points: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_sigma_min_cut_xyz(mc: Pin<&mut MinCutSegmentation_PointXYZ>, sigma: f64);
        #[cfg(feature = "segmentation")]
        fn get_sigma_min_cut_xyz(mc: &MinCutSegmentation_PointXYZ) -> f64;
        #[cfg(feature = "segmentation")]
        fn set_radius_min_cut_xyz(mc: Pin<&mut MinCutSegmentation_PointXYZ>, radius: f64);
        #[cfg(feature = "segmentation")]
        fn get_radius_min_cut_xyz(mc: &MinCutSegmentation_PointXYZ) -> f64;
        #[cfg(feature = "segmentation")]
        fn set_number_of_neighbours_min_cut_xyz(mc: Pin<&mut MinCutSegmentation_PointXYZ>, k: i32);
        #[cfg(feature = "segmentation")]
        fn get_number_of_neighbours_min_cut_xyz(mc: &MinCutSegmentation_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_source_weight_min_cut_xyz(mc: Pin<&mut MinCutSegmentation_PointXYZ>, weight: f64);
        #[cfg(feature = "segmentation")]
        fn get_source_weight_min_cut_xyz(mc: &MinCutSegmentation_PointXYZ) -> f64;
        #[cfg(feature = "segmentation")]
        fn extract_min_cut_xyz(mc: Pin<&mut MinCutSegmentation_PointXYZ>) -> Vec<i32>;

        // Extract Polygonal Prism Data functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_extract_polygonal_prism_xyz() -> UniquePtr<ExtractPolygonalPrismData_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_polygonal_prism_xyz(
            prism: Pin<&mut ExtractPolygonalPrismData_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_input_planar_hull_polygonal_prism_xyz(
            prism: Pin<&mut ExtractPolygonalPrismData_PointXYZ>,
            hull: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_height_limits_polygonal_prism_xyz(
            prism: Pin<&mut ExtractPolygonalPrismData_PointXYZ>,
            min_height: f64,
            max_height: f64,
        );
        #[cfg(feature = "segmentation")]
        fn get_height_limits_polygonal_prism_xyz(
            prism: &ExtractPolygonalPrismData_PointXYZ,
            min_height: &mut f64,
            max_height: &mut f64,
        );
        #[cfg(feature = "segmentation")]
        fn segment_polygonal_prism_xyz(
            prism: Pin<&mut ExtractPolygonalPrismData_PointXYZ>,
        ) -> Vec<i32>;

        // Progressive Morphological Filter functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_progressive_morphological_filter_xyz()
        -> UniquePtr<ProgressiveMorphologicalFilter_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_pmf_xyz(
            pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_max_window_size_pmf_xyz(
            pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>,
            size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_max_window_size_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_slope_pmf_xyz(pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>, slope: f32);
        #[cfg(feature = "segmentation")]
        fn get_slope_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_max_distance_pmf_xyz(
            pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>,
            distance: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_max_distance_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_initial_distance_pmf_xyz(
            pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>,
            distance: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_initial_distance_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_cell_size_pmf_xyz(pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>, size: f32);
        #[cfg(feature = "segmentation")]
        fn get_cell_size_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_base_pmf_xyz(pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>, base: f32);
        #[cfg(feature = "segmentation")]
        fn get_base_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_exponential_pmf_xyz(
            pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>,
            exponential: bool,
        );
        #[cfg(feature = "segmentation")]
        fn get_exponential_pmf_xyz(pmf: &ProgressiveMorphologicalFilter_PointXYZ) -> bool;
        #[cfg(feature = "segmentation")]
        fn extract_pmf_xyz(pmf: Pin<&mut ProgressiveMorphologicalFilter_PointXYZ>) -> Vec<i32>;

        // Conditional Euclidean Clustering functions - PointXYZ
        #[cfg(feature = "segmentation")]
        fn new_conditional_euclidean_clustering_xyz()
        -> UniquePtr<ConditionalEuclideanClustering_PointXYZ>;
        #[cfg(feature = "segmentation")]
        fn set_input_cloud_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        #[cfg(feature = "segmentation")]
        fn set_cluster_tolerance_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
            tolerance: f32,
        );
        #[cfg(feature = "segmentation")]
        fn get_cluster_tolerance_conditional_euclidean_xyz(
            cec: &ConditionalEuclideanClustering_PointXYZ,
        ) -> f32;
        #[cfg(feature = "segmentation")]
        fn set_min_cluster_size_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
            min_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_min_cluster_size_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
        ) -> i32;
        #[cfg(feature = "segmentation")]
        fn set_max_cluster_size_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
            max_size: i32,
        );
        #[cfg(feature = "segmentation")]
        fn get_max_cluster_size_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
        ) -> i32;
        #[cfg(feature = "segmentation")]
        fn segment_conditional_euclidean_xyz(
            cec: Pin<&mut ConditionalEuclideanClustering_PointXYZ>,
        ) -> Vec<i32>;

    }
}

// The PCL library is thread-safe for immutable operations
// These types are opaque C++ objects that PCL guarantees are safe to use across threads
unsafe impl Send for ffi::PointCloud_PointXYZ {}
unsafe impl Sync for ffi::PointCloud_PointXYZ {}
unsafe impl Send for ffi::PointCloud_PointXYZI {}
unsafe impl Sync for ffi::PointCloud_PointXYZI {}
unsafe impl Send for ffi::PointCloud_PointXYZRGB {}
unsafe impl Sync for ffi::PointCloud_PointXYZRGB {}

unsafe impl Send for ffi::PointXYZ {}
unsafe impl Sync for ffi::PointXYZ {}
unsafe impl Send for ffi::PointXYZI {}
unsafe impl Sync for ffi::PointXYZI {}
unsafe impl Send for ffi::PointXYZRGB {}
unsafe impl Sync for ffi::PointXYZRGB {}
unsafe impl Send for ffi::PointNormal {}
unsafe impl Sync for ffi::PointNormal {}
unsafe impl Send for ffi::PointCloud_PointNormal {}
unsafe impl Sync for ffi::PointCloud_PointNormal {}
