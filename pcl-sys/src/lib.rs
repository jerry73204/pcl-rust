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

        fn size(cloud: &PointCloud_PointXYZ) -> usize;

        fn size_xyzi(cloud: &PointCloud_PointXYZI) -> usize;

        fn size_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> usize;

        fn clear(cloud: Pin<&mut PointCloud_PointXYZ>);

        fn clear_xyzi(cloud: Pin<&mut PointCloud_PointXYZI>);

        fn clear_xyzrgb(cloud: Pin<&mut PointCloud_PointXYZRGB>);

        fn empty(cloud: &PointCloud_PointXYZ) -> bool;

        fn empty_xyzi(cloud: &PointCloud_PointXYZI) -> bool;

        fn empty_xyzrgb(cloud: &PointCloud_PointXYZRGB) -> bool;

        // Additional basic point cloud functions - part of common

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
    }
}
