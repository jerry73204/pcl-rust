//! RANSAC algorithm FFI bindings

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");

        // Type declarations
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;

        // RANSAC algorithm types
        #[namespace = "pcl"]
        type RandomSampleConsensus_PointXYZ;
        #[namespace = "pcl"]
        type RandomSampleConsensus_PointXYZRGB;

        // Model types
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZRGB;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZRGB;

        // RANSAC creation and configuration - PointXYZ
        fn new_ransac_plane_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;
        fn new_ransac_sphere_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZ>;

        // RANSAC creation and configuration - PointXYZRGB
        fn new_ransac_plane_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;
        fn new_ransac_sphere_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<RandomSampleConsensus_PointXYZRGB>;

        // Algorithm configuration - PointXYZ
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

        // Algorithm configuration - PointXYZRGB
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

        // Model computation - PointXYZ
        fn compute_model_xyz(ransac: Pin<&mut RandomSampleConsensus_PointXYZ>) -> bool;
        fn refine_model_xyz(
            ransac: Pin<&mut RandomSampleConsensus_PointXYZ>,
            sigma: f64,
            max_iterations: u32,
        ) -> bool;
        fn get_inliers_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<i32>;
        fn get_model_coefficients_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> Vec<f32>;
        fn get_inliers_count_xyz(ransac: &RandomSampleConsensus_PointXYZ) -> usize;

        // Model computation - PointXYZRGB
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
