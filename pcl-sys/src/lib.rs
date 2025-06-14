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
pub mod surface;

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
        #[namespace = "pcl"]
        type PointNormal;
        #[namespace = "pcl"]
        type PointXYZRGBA;

        // PointCloud template instantiations
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZI;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGBA;

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

        // I/O functions
        // PCD I/O functions for PointXYZ
        fn load_pcd_file_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        fn save_pcd_file_xyz(file_name: &str, cloud: &PointCloud_PointXYZ, binary: bool) -> i32;
        fn save_pcd_file_ascii_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        fn save_pcd_file_binary_xyz(file_name: &str, cloud: &PointCloud_PointXYZ) -> i32;
        fn save_pcd_file_binary_compressed_xyz(file_name: &str, cloud: &PointCloud_PointXYZ)
        -> i32;

        // PCD I/O functions for PointXYZI
        fn load_pcd_file_xyzi(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZI>) -> i32;
        fn save_pcd_file_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI, binary: bool) -> i32;
        fn save_pcd_file_ascii_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        fn save_pcd_file_binary_xyzi(file_name: &str, cloud: &PointCloud_PointXYZI) -> i32;
        fn save_pcd_file_binary_compressed_xyzi(
            file_name: &str,
            cloud: &PointCloud_PointXYZI,
        ) -> i32;

        // PCD I/O functions for PointXYZRGB
        fn load_pcd_file_xyzrgb(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZRGB>) -> i32;
        fn save_pcd_file_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
            binary: bool,
        ) -> i32;
        fn save_pcd_file_ascii_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        fn save_pcd_file_binary_xyzrgb(file_name: &str, cloud: &PointCloud_PointXYZRGB) -> i32;
        fn save_pcd_file_binary_compressed_xyzrgb(
            file_name: &str,
            cloud: &PointCloud_PointXYZRGB,
        ) -> i32;

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

        // Format auto-detection functions
        fn detect_format_from_extension(file_name: &str) -> i32;
        fn detect_format_from_content(file_name: &str) -> i32;
        fn detect_file_format(file_name: &str) -> i32;

        // Auto-loading functions that detect format automatically
        fn load_point_cloud_auto_xyz(file_name: &str, cloud: Pin<&mut PointCloud_PointXYZ>) -> i32;
        fn load_point_cloud_auto_xyzi(
            file_name: &str,
            cloud: Pin<&mut PointCloud_PointXYZI>,
        ) -> i32;
        fn load_point_cloud_auto_xyzrgb(
            file_name: &str,
            cloud: Pin<&mut PointCloud_PointXYZRGB>,
        ) -> i32;

        // Filter types
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
        fn new_pass_through_xyz() -> UniquePtr<PassThrough_PointXYZ>;
        fn set_input_cloud_pass_xyz(
            filter: Pin<&mut PassThrough_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_filter_field_name_xyz(filter: Pin<&mut PassThrough_PointXYZ>, field_name: &str);
        fn get_filter_field_name_xyz(filter: &PassThrough_PointXYZ) -> String;
        fn set_filter_limits_xyz(filter: Pin<&mut PassThrough_PointXYZ>, min: f32, max: f32);
        fn set_filter_limits_negative_xyz(filter: Pin<&mut PassThrough_PointXYZ>, negative: bool);
        fn get_filter_limits_negative_xyz(filter: &PassThrough_PointXYZ) -> bool;
        fn set_keep_organized_xyz(filter: Pin<&mut PassThrough_PointXYZ>, keep_organized: bool);
        fn get_keep_organized_xyz(filter: &PassThrough_PointXYZ) -> bool;
        fn filter_pass_xyz(
            filter: Pin<&mut PassThrough_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - PassThrough PointXYZRGB
        fn new_pass_through_xyzrgb() -> UniquePtr<PassThrough_PointXYZRGB>;
        fn set_input_cloud_pass_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_filter_field_name_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            field_name: &str,
        );
        fn get_filter_field_name_xyzrgb(filter: &PassThrough_PointXYZRGB) -> String;
        fn set_filter_limits_xyzrgb(filter: Pin<&mut PassThrough_PointXYZRGB>, min: f32, max: f32);
        fn set_filter_limits_negative_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            negative: bool,
        );
        fn get_filter_limits_negative_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        fn set_keep_organized_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
            keep_organized: bool,
        );
        fn get_keep_organized_xyzrgb(filter: &PassThrough_PointXYZRGB) -> bool;
        fn filter_pass_xyzrgb(
            filter: Pin<&mut PassThrough_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - VoxelGrid PointXYZ
        fn new_voxel_grid_xyz() -> UniquePtr<VoxelGrid_PointXYZ>;
        fn set_input_cloud_voxel_xyz(
            filter: Pin<&mut VoxelGrid_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_leaf_size_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>, lx: f32, ly: f32, lz: f32);
        fn filter_voxel_xyz(filter: Pin<&mut VoxelGrid_PointXYZ>)
        -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - VoxelGrid PointXYZRGB
        fn new_voxel_grid_xyzrgb() -> UniquePtr<VoxelGrid_PointXYZRGB>;
        fn set_input_cloud_voxel_xyzrgb(
            filter: Pin<&mut VoxelGrid_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_leaf_size_xyzrgb(filter: Pin<&mut VoxelGrid_PointXYZRGB>, lx: f32, ly: f32, lz: f32);
        fn filter_voxel_xyzrgb(
            filter: Pin<&mut VoxelGrid_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - StatisticalOutlierRemoval PointXYZ
        fn new_statistical_outlier_removal_xyz() -> UniquePtr<StatisticalOutlierRemoval_PointXYZ>;
        fn set_input_cloud_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_mean_k_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            mean_k: i32,
        );
        fn set_std_dev_mul_thresh_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            stddev_mult: f64,
        );
        fn set_negative_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
            negative: bool,
        );
        fn filter_statistical_xyz(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - StatisticalOutlierRemoval PointXYZRGB
        fn new_statistical_outlier_removal_xyzrgb()
        -> UniquePtr<StatisticalOutlierRemoval_PointXYZRGB>;
        fn set_input_cloud_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_mean_k_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            mean_k: i32,
        );
        fn set_std_dev_mul_thresh_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            stddev_mult: f64,
        );
        fn set_negative_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
            negative: bool,
        );
        fn filter_statistical_xyzrgb(
            filter: Pin<&mut StatisticalOutlierRemoval_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Filter functions - RadiusOutlierRemoval PointXYZ
        fn new_radius_outlier_removal_xyz() -> UniquePtr<RadiusOutlierRemoval_PointXYZ>;
        fn set_input_cloud_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_radius_search_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, radius: f64);
        fn set_min_neighbors_in_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
            min_neighbors: i32,
        );
        fn set_negative_radius_xyz(filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>, negative: bool);
        fn filter_radius_xyz(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // Filter functions - RadiusOutlierRemoval PointXYZRGB
        fn new_radius_outlier_removal_xyzrgb() -> UniquePtr<RadiusOutlierRemoval_PointXYZRGB>;
        fn set_input_cloud_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_radius_search_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            radius: f64,
        );
        fn set_min_neighbors_in_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            min_neighbors: i32,
        );
        fn set_negative_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
            negative: bool,
        );
        fn filter_radius_xyzrgb(
            filter: Pin<&mut RadiusOutlierRemoval_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;

        // Surface reconstruction types
        #[namespace = "pcl"]
        type MarchingCubesHoppe_PointXYZ;
        #[namespace = "pcl"]
        type MarchingCubesRBF_PointXYZ;
        #[namespace = "pcl"]
        type OrganizedFastMesh_PointXYZ;
        #[namespace = "pcl"]
        type PolygonMesh;

        // Marching Cubes Hoppe reconstruction functions
        fn new_marching_cubes_hoppe_xyz() -> UniquePtr<MarchingCubesHoppe_PointXYZ>;
        fn set_iso_level_hoppe_xyz(mc: Pin<&mut MarchingCubesHoppe_PointXYZ>, iso_level: f32);
        fn get_iso_level_hoppe_xyz(mc: Pin<&mut MarchingCubesHoppe_PointXYZ>) -> f32;
        fn set_grid_resolution_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            res_x: i32,
            res_y: i32,
            res_z: i32,
        );
        fn set_percentage_extend_grid_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            percentage: f32,
        );
        fn set_input_cloud_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn perform_reconstruction_hoppe_xyz(
            mc: Pin<&mut MarchingCubesHoppe_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Marching Cubes RBF reconstruction functions
        fn new_marching_cubes_rbf_xyz() -> UniquePtr<MarchingCubesRBF_PointXYZ>;
        fn set_iso_level_rbf_xyz(mc: Pin<&mut MarchingCubesRBF_PointXYZ>, iso_level: f32);
        fn get_iso_level_rbf_xyz(mc: Pin<&mut MarchingCubesRBF_PointXYZ>) -> f32;
        fn set_grid_resolution_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            res_x: i32,
            res_y: i32,
            res_z: i32,
        );
        fn set_percentage_extend_grid_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            percentage: f32,
        );
        fn set_off_surface_displacement_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            displacement: f32,
        );
        fn set_input_cloud_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn perform_reconstruction_rbf_xyz(
            mc: Pin<&mut MarchingCubesRBF_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Organized Fast Mesh functions
        fn new_organized_fast_mesh_xyz() -> UniquePtr<OrganizedFastMesh_PointXYZ>;
        fn set_triangle_pixel_size_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            triangle_size: i32,
        );
        fn get_triangle_pixel_size_xyz(ofm: &OrganizedFastMesh_PointXYZ) -> i32;
        fn set_triangulation_type_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            triangle_type: i32,
        );
        fn get_triangulation_type_xyz(ofm: &OrganizedFastMesh_PointXYZ) -> i32;
        fn set_input_cloud_ofm_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn perform_reconstruction_ofm_xyz(
            ofm: Pin<&mut OrganizedFastMesh_PointXYZ>,
            mesh: Pin<&mut PolygonMesh>,
        ) -> i32;

        // Polygon mesh utility functions
        fn new_polygon_mesh() -> UniquePtr<PolygonMesh>;
        fn get_polygon_count(mesh: &PolygonMesh) -> usize;
        fn get_vertex_count(mesh: &PolygonMesh) -> usize;
        fn is_valid_mesh(mesh: &PolygonMesh) -> bool;
        fn save_polygon_mesh_ply(mesh: &PolygonMesh, filename: &str) -> i32;
        fn save_polygon_mesh_obj(mesh: &PolygonMesh, filename: &str) -> i32;
        fn save_polygon_mesh_vtk(mesh: &PolygonMesh, filename: &str) -> i32;
    }
}
