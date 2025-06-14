//! Low-level FFI bindings for Point Cloud Library (PCL)
//!
//! This crate provides unsafe FFI bindings to PCL using the cxx crate.
//! For safe, idiomatic Rust APIs, use the `pcl` crate instead.

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

        // Sample consensus types
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

        // Filter types
        #[namespace = "pcl"]
        type PassThrough_PointXYZ;
        #[namespace = "pcl"]
        type PassThrough_PointXYZRGB;

        // Registration types
        #[namespace = "pcl"]
        type IterativeClosestPoint_PointXYZ;
        #[namespace = "pcl"]
        type IterativeClosestPoint_PointXYZRGB;

        // Segmentation types
        #[namespace = "pcl"]
        type PointCloud_Normal;
        #[namespace = "pcl"]
        type RegionGrowing_PointXYZ_Normal;
        #[namespace = "pcl"]
        type RegionGrowingRGB_PointXYZRGB;
        #[namespace = "pcl"]
        type EuclideanClusterExtraction_PointXYZ;
        #[namespace = "pcl"]
        type SACSegmentation_PointXYZ;

        // Features types
        #[namespace = "pcl"]
        type Normal;
        #[namespace = "pcl"]
        type FPFHSignature33;
        #[namespace = "pcl"]
        type PFHSignature125;
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
        type PointCloud_FPFHSignature33;
        #[namespace = "pcl"]
        type PointCloud_PFHSignature125;

        // Keypoints types
        #[namespace = "pcl"]
        type PointWithScale;
        #[namespace = "pcl"]
        type HarrisKeypoint3D_PointXYZ_PointXYZI;
        #[namespace = "pcl"]
        type ISSKeypoint3D_PointXYZ_PointXYZ;
        #[namespace = "pcl"]
        type SIFTKeypoint_PointXYZI_PointWithScale;
        #[namespace = "pcl"]
        type PointCloud_PointWithScale;

        // Point cloud functions
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

        // Sample consensus functions
        // RANSAC creation and configuration - PointXYZ
        fn new_ransac_plane_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;
        fn new_ransac_sphere_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;
        fn set_distance_threshold_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            threshold: f64,
        );
        fn get_distance_threshold_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> f64;
        fn set_max_iterations_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            max_iterations: i32,
        );
        fn get_max_iterations_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> i32;
        fn set_probability_xyz(ransac: Pin<&mut RandomSampleConsensus_PointXYZ>, probability: f64);
        fn get_probability_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> f64;
        fn compute_model_xyz(ransac: Pin<&mut RandomSampleConsensus_PointXYZ>) -> bool;
        fn refine_model_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            sigma: f64,
            max_iterations: u32,
        ) -> bool;
        fn get_inliers_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<i32>;
        fn get_model_coefficients_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<f32>;
        fn get_inliers_count_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> usize;

        // RANSAC creation and configuration - PointXYZRGB
        fn new_ransac_plane_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;
        fn new_ransac_sphere_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;
        fn set_distance_threshold_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            threshold: f64,
        );
        fn get_distance_threshold_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> f64;
        fn set_max_iterations_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            max_iterations: i32,
        );
        fn get_max_iterations_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> i32;
        fn set_probability_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            probability: f64,
        );
        fn get_probability_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> f64;
        fn compute_model_xyzrgb(ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>) -> bool;
        fn refine_model_xyzrgb(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZRGB>,
            sigma: f64,
            max_iterations: u32,
        ) -> bool;
        fn get_inliers_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> Vec<i32>;
        fn get_model_coefficients_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> Vec<f32>;
        fn get_inliers_count_xyzrgb(ransac: &RandomSampleConsensus_PointXYZRGB) -> usize;

        // Filter functions
        // PassThrough filter functions - PointXYZ
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

        // PassThrough filter functions - PointXYZRGB
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

        // Registration functions
        // ICP functions - PointXYZ
        fn new_icp_xyz() -> UniquePtr<IterativeClosestPoint_PointXYZ>;
        fn set_input_source_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_input_target_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_max_iterations_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            iterations: i32,
        );
        fn get_max_iterations_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> i32;
        fn set_transformation_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            epsilon: f64,
        );
        fn get_transformation_epsilon_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>)
        -> f64;
        fn set_euclidean_fitness_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            epsilon: f64,
        );
        fn get_euclidean_fitness_epsilon_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> f64;
        fn set_max_correspondence_distance_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            distance: f64,
        );
        fn get_max_correspondence_distance_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> f64;
        fn align_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        fn align_with_guess_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZ>;
        fn has_converged_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> bool;
        fn get_fitness_score_icp_xyz(icp: Pin<&mut IterativeClosestPoint_PointXYZ>) -> f64;
        fn get_final_transformation_icp_xyz(
            icp: Pin<&mut IterativeClosestPoint_PointXYZ>,
        ) -> Vec<f32>;

        // ICP functions - PointXYZRGB
        fn new_icp_xyzrgb() -> UniquePtr<IterativeClosestPoint_PointXYZRGB>;
        fn set_input_source_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_input_target_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_max_iterations_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            iterations: i32,
        );
        fn get_max_iterations_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> i32;
        fn set_transformation_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            epsilon: f64,
        );
        fn get_transformation_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        fn set_euclidean_fitness_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            epsilon: f64,
        );
        fn get_euclidean_fitness_epsilon_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        fn set_max_correspondence_distance_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            distance: f64,
        );
        fn get_max_correspondence_distance_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> f64;
        fn align_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        fn align_with_guess_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
            guess: &Vec<f32>,
        ) -> UniquePtr<PointCloud_PointXYZRGB>;
        fn has_converged_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> bool;
        fn get_fitness_score_icp_xyzrgb(icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>) -> f64;
        fn get_final_transformation_icp_xyzrgb(
            icp: Pin<&mut IterativeClosestPoint_PointXYZRGB>,
        ) -> Vec<f32>;

        // Segmentation functions
        // Region Growing segmentation - PointXYZ with Normal
        fn new_region_growing_xyz() -> UniquePtr<RegionGrowing_PointXYZ_Normal>;
        fn set_input_cloud_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_input_normals_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            normals: &PointCloud_Normal,
        );
        fn set_min_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            min_size: i32,
        );
        fn get_min_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
        ) -> i32;
        fn set_max_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            max_size: i32,
        );
        fn get_max_cluster_size_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
        ) -> i32;
        fn set_smoothness_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            threshold: f32,
        );
        fn get_smoothness_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
        ) -> f32;
        fn set_curvature_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            threshold: f32,
        );
        fn get_curvature_threshold_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
        ) -> f32;
        fn set_number_of_neighbours_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
            k: i32,
        );
        fn get_number_of_neighbours_region_growing_xyz(
            rg: Pin<&mut RegionGrowing_PointXYZ_Normal>,
        ) -> i32;
        fn extract_region_growing_xyz(rg: Pin<&mut RegionGrowing_PointXYZ_Normal>) -> Vec<i32>;

        // Region Growing RGB segmentation - PointXYZRGB
        fn new_region_growing_rgb_xyzrgb() -> UniquePtr<RegionGrowingRGB_PointXYZRGB>;
        fn set_input_cloud_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            cloud: &PointCloud_PointXYZRGB,
        );
        fn set_distance_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        fn get_distance_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> f32;
        fn set_point_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        fn get_point_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> f32;
        fn set_region_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            threshold: f32,
        );
        fn get_region_color_threshold_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> f32;
        fn set_min_cluster_size_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
            min_size: i32,
        );
        fn get_min_cluster_size_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> i32;
        fn extract_region_growing_rgb_xyzrgb(
            rg: Pin<&mut RegionGrowingRGB_PointXYZRGB>,
        ) -> Vec<i32>;

        // Euclidean Cluster Extraction - PointXYZ
        fn new_euclidean_cluster_extraction_xyz() -> UniquePtr<EuclideanClusterExtraction_PointXYZ>;
        fn set_input_cloud_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_cluster_tolerance_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            tolerance: f64,
        );
        fn get_cluster_tolerance_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> f64;
        fn set_min_cluster_size_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            min_size: i32,
        );
        fn get_min_cluster_size_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> i32;
        fn set_max_cluster_size_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
            max_size: i32,
        );
        fn get_max_cluster_size_euclidean_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> i32;
        fn extract_euclidean_clusters_xyz(
            ece: Pin<&mut EuclideanClusterExtraction_PointXYZ>,
        ) -> Vec<i32>;

        // SAC Segmentation - PointXYZ
        fn new_sac_segmentation_xyz() -> UniquePtr<SACSegmentation_PointXYZ>;
        fn set_input_cloud_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_model_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, model_type: i32);
        fn get_model_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>) -> i32;
        fn set_method_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, method_type: i32);
        fn get_method_type_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>) -> i32;
        fn set_distance_threshold_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, threshold: f64);
        fn get_distance_threshold_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>) -> f64;
        fn set_max_iterations_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>, max_iterations: i32);
        fn get_max_iterations_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>) -> i32;
        fn set_optimize_coefficients_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            optimize: bool,
        );
        fn get_optimize_coefficients_sac_xyz(sac: Pin<&mut SACSegmentation_PointXYZ>) -> bool;
        fn segment_sac_xyz(
            sac: Pin<&mut SACSegmentation_PointXYZ>,
            inliers: &mut Vec<i32>,
            coefficients: &mut Vec<f32>,
        ) -> bool;

        // Normal estimation helper
        fn estimate_normals_xyz(
            cloud: &PointCloud_PointXYZ,
            radius: f64,
        ) -> UniquePtr<PointCloud_Normal>;

        // Features functions
        // Normal estimation - PointXYZ
        fn new_normal_estimation_xyz() -> UniquePtr<NormalEstimation_PointXYZ_Normal>;
        fn set_input_cloud_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_search_method_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_search_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            radius: f64,
        );
        fn get_radius_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> f64;
        fn set_k_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>, k: i32);
        fn get_k_search_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> i32;
        fn set_view_point_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            vpx: f32,
            vpy: f32,
            vpz: f32,
        );
        fn get_view_point_normal_xyz(ne: Pin<&mut NormalEstimation_PointXYZ_Normal>) -> Vec<f32>;
        fn set_use_sensor_origin_normal_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
            use_sensor_origin: bool,
        );
        fn compute_normals_xyz(
            ne: Pin<&mut NormalEstimation_PointXYZ_Normal>,
        ) -> UniquePtr<PointCloud_Normal>;

        // Normal estimation OMP - PointXYZ
        fn new_normal_estimation_omp_xyz() -> UniquePtr<NormalEstimationOMP_PointXYZ_Normal>;
        fn set_input_cloud_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_search_method_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_search_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            radius: f64,
        );
        fn set_k_search_normal_omp_xyz(ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>, k: i32);
        fn set_number_of_threads_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
            threads: i32,
        );
        fn get_number_of_threads_normal_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
        ) -> i32;
        fn compute_normals_omp_xyz(
            ne: Pin<&mut NormalEstimationOMP_PointXYZ_Normal>,
        ) -> UniquePtr<PointCloud_Normal>;

        // FPFH feature estimation - PointXYZ
        fn new_fpfh_estimation_xyz() -> UniquePtr<FPFHEstimation_PointXYZ_Normal_FPFHSignature33>;
        fn set_input_cloud_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_input_normals_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            normals: &PointCloud_Normal,
        );
        fn set_search_method_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_search_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            radius: f64,
        );
        fn set_k_search_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
            k: i32,
        );
        fn compute_fpfh_xyz(
            fpfh: Pin<&mut FPFHEstimation_PointXYZ_Normal_FPFHSignature33>,
        ) -> UniquePtr<PointCloud_FPFHSignature33>;

        // FPFH OMP feature estimation - PointXYZ
        fn new_fpfh_estimation_omp_xyz()
        -> UniquePtr<FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>;
        fn set_input_cloud_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_input_normals_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            normals: &PointCloud_Normal,
        );
        fn set_search_method_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_search_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            radius: f64,
        );
        fn set_number_of_threads_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
            threads: i32,
        );
        fn compute_fpfh_omp_xyz(
            fpfh: Pin<&mut FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>,
        ) -> UniquePtr<PointCloud_FPFHSignature33>;

        // PFH feature estimation - PointXYZ
        fn new_pfh_estimation_xyz() -> UniquePtr<PFHEstimation_PointXYZ_Normal_PFHSignature125>;
        fn set_input_cloud_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_input_normals_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            normals: &PointCloud_Normal,
        );
        fn set_search_method_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_search_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            radius: f64,
        );
        fn set_k_search_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
            k: i32,
        );
        fn compute_pfh_xyz(
            pfh: Pin<&mut PFHEstimation_PointXYZ_Normal_PFHSignature125>,
        ) -> UniquePtr<PointCloud_PFHSignature125>;

        // Helper functions for features
        fn get_fpfh_histogram(signature: &FPFHSignature33) -> Vec<f32>;
        fn get_pfh_histogram(signature: &PFHSignature125) -> Vec<f32>;
        fn get_normal_vector(normal: &Normal) -> Vec<f32>;

        // Keypoints functions
        // Harris 3D keypoint detector
        fn new_harris_3d_xyz() -> UniquePtr<HarrisKeypoint3D_PointXYZ_PointXYZI>;
        fn set_input_cloud_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_search_method_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            tree: &KdTree_PointXYZ,
        );
        fn set_radius_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            radius: f64,
        );
        fn set_threshold_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            threshold: f32,
        );
        fn set_non_max_suppression_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            suppress: bool,
        );
        fn set_refine_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
            refine: bool,
        );
        fn compute_harris_3d_xyz(
            harris: Pin<&mut HarrisKeypoint3D_PointXYZ_PointXYZI>,
        ) -> UniquePtr<PointCloud_PointXYZI>;

        // ISS 3D keypoint detector
        fn new_iss_3d_xyz() -> UniquePtr<ISSKeypoint3D_PointXYZ_PointXYZ>;
        fn set_input_cloud_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            cloud: &PointCloud_PointXYZ,
        );
        fn set_search_method_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            tree: &KdTree_PointXYZ,
        );
        fn set_salient_radius_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            radius: f64,
        );
        fn set_non_max_radius_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            radius: f64,
        );
        fn set_threshold21_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            threshold: f64,
        );
        fn set_threshold32_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            threshold: f64,
        );
        fn set_min_neighbors_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
            min_neighbors: i32,
        );
        fn compute_iss_3d_xyz(
            iss: Pin<&mut ISSKeypoint3D_PointXYZ_PointXYZ>,
        ) -> UniquePtr<PointCloud_PointXYZ>;

        // SIFT keypoint detector - PointXYZI (SIFT requires intensity field)
        fn new_sift_keypoint_xyzi() -> UniquePtr<SIFTKeypoint_PointXYZI_PointWithScale>;
        fn set_input_cloud_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            cloud: &PointCloud_PointXYZI,
        );
        fn set_search_method_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            tree: &KdTree_PointXYZI,
        );
        fn set_scales_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            min_scale: f32,
            nr_octaves: f32,
            nr_scales_per_octave: i32,
        );
        fn set_minimum_contrast_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
            min_contrast: f32,
        );
        fn compute_sift_xyzi(
            sift: Pin<&mut SIFTKeypoint_PointXYZI_PointWithScale>,
        ) -> UniquePtr<PointCloud_PointWithScale>;

        // Helper functions for keypoints
        fn get_point_with_scale_coords(point: &PointWithScale) -> Vec<f32>;
        fn get_point_xyzi_coords(point: &PointXYZI) -> Vec<f32>;
    }
}

// Type aliases for convenience
pub type PointXYZ = ffi::PointXYZ;
pub type PointXYZI = ffi::PointXYZI;
pub type PointXYZRGB = ffi::PointXYZRGB;
pub type PointCloudXYZ = ffi::PointCloud_PointXYZ;
pub type PointCloudXYZI = ffi::PointCloud_PointXYZI;
pub type PointCloudXYZRGB = ffi::PointCloud_PointXYZRGB;
pub type KdTreeXYZ = ffi::KdTree_PointXYZ;
pub type KdTreeXYZI = ffi::KdTree_PointXYZI;
pub type KdTreeXYZRGB = ffi::KdTree_PointXYZRGB;
pub type OctreeSearchXYZ = ffi::OctreePointCloudSearch_PointXYZ;
pub type OctreeVoxelCentroidXYZ = ffi::OctreePointCloudVoxelCentroid_PointXYZ;
pub type RansacXYZ = ffi::RandomSampleConsensus_PointXYZ;
pub type RansacXYZRGB = ffi::RandomSampleConsensus_PointXYZRGB;
pub type PlaneModelXYZ = ffi::SampleConsensusModelPlane_PointXYZ;
pub type SphereModelXYZ = ffi::SampleConsensusModelSphere_PointXYZ;
pub type PlaneModelXYZRGB = ffi::SampleConsensusModelPlane_PointXYZRGB;
pub type SphereModelXYZRGB = ffi::SampleConsensusModelSphere_PointXYZRGB;
pub type PassThroughXYZ = ffi::PassThrough_PointXYZ;
pub type PassThroughXYZRGB = ffi::PassThrough_PointXYZRGB;
pub type IcpXYZ = ffi::IterativeClosestPoint_PointXYZ;
pub type IcpXYZRGB = ffi::IterativeClosestPoint_PointXYZRGB;
pub type PointCloudNormal = ffi::PointCloud_Normal;
pub type RegionGrowingXYZ = ffi::RegionGrowing_PointXYZ_Normal;
pub type RegionGrowingRgbXYZRGB = ffi::RegionGrowingRGB_PointXYZRGB;
pub type EuclideanClusterExtractionXYZ = ffi::EuclideanClusterExtraction_PointXYZ;
pub type SacSegmentationXYZ = ffi::SACSegmentation_PointXYZ;
pub type Normal = ffi::Normal;
pub type FpfhSignature33 = ffi::FPFHSignature33;
pub type PfhSignature125 = ffi::PFHSignature125;
pub type NormalEstimationXYZ = ffi::NormalEstimation_PointXYZ_Normal;
pub type NormalEstimationOmpXYZ = ffi::NormalEstimationOMP_PointXYZ_Normal;
pub type FpfhEstimationXYZ = ffi::FPFHEstimation_PointXYZ_Normal_FPFHSignature33;
pub type FpfhEstimationOmpXYZ = ffi::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33;
pub type PfhEstimationXYZ = ffi::PFHEstimation_PointXYZ_Normal_PFHSignature125;
pub type PointCloudFpfh33 = ffi::PointCloud_FPFHSignature33;
pub type PointCloudPfh125 = ffi::PointCloud_PFHSignature125;
pub type PointWithScale = ffi::PointWithScale;
pub type HarrisKeypoint3DXYZ = ffi::HarrisKeypoint3D_PointXYZ_PointXYZI;
pub type IssKeypoint3DXYZ = ffi::ISSKeypoint3D_PointXYZ_PointXYZ;
pub type SiftKeypointXYZI = ffi::SIFTKeypoint_PointXYZI_PointWithScale;
pub type PointCloudWithScale = ffi::PointCloud_PointWithScale;
