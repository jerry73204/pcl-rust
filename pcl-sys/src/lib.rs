//! Low-level FFI bindings for Point Cloud Library (PCL)
//!
//! This crate provides unsafe FFI bindings to PCL using the cxx crate.
//! For safe, idiomatic Rust APIs, use the `pcl` crate instead.

pub mod common;
pub mod io;
pub mod octree;
pub mod sample_consensus;
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
        type PointCloud_PointXYZI;
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
pub type KdTreeXYZRGB = ffi::KdTree_PointXYZRGB;
pub type OctreeSearchXYZ = ffi::OctreePointCloudSearch_PointXYZ;
pub type OctreeVoxelCentroidXYZ = ffi::OctreePointCloudVoxelCentroid_PointXYZ;
pub type RansacXYZ = ffi::RandomSampleConsensus_PointXYZ;
pub type RansacXYZRGB = ffi::RandomSampleConsensus_PointXYZRGB;
pub type PlaneModelXYZ = ffi::SampleConsensusModelPlane_PointXYZ;
pub type SphereModelXYZ = ffi::SampleConsensusModelSphere_PointXYZ;
pub type PlaneModelXYZRGB = ffi::SampleConsensusModelPlane_PointXYZRGB;
pub type SphereModelXYZRGB = ffi::SampleConsensusModelSphere_PointXYZRGB;
