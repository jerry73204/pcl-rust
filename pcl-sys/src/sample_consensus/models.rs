//! Geometric model FFI bindings

#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("cxx/functions.h");

        // Type declarations
        #[namespace = "pcl"]
        type PointCloud_PointXYZ;
        #[namespace = "pcl"]
        type PointCloud_PointXYZRGB;

        // Model types
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZ;
        #[namespace = "pcl"]
        type SampleConsensusModelPlane_PointXYZRGB;
        #[namespace = "pcl"]
        type SampleConsensusModelSphere_PointXYZRGB;

        // Model creation - PointXYZ
        fn new_plane_model_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<SampleConsensusModelPlane_PointXYZ>;
        fn new_sphere_model_xyz(
            cloud: &PointCloud_PointXYZ,
        ) -> UniquePtr<SampleConsensusModelSphere_PointXYZ>;

        // Model creation - PointXYZRGB
        fn new_plane_model_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<SampleConsensusModelPlane_PointXYZRGB>;
        fn new_sphere_model_xyzrgb(
            cloud: &PointCloud_PointXYZRGB,
        ) -> UniquePtr<SampleConsensusModelSphere_PointXYZRGB>;

        // Plane model methods - PointXYZ
        fn plane_compute_model_coefficients_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            samples: &[i32],
        ) -> Vec<f32>;
        fn plane_get_distances_to_model_xyz(
            model: &SampleConsensusModelPlane_PointXYZ,
            coefficients: &[f32],
        ) -> Vec<f64>;
        fn plane_select_within_distance_xyz(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZ>,
            coefficients: &[f32],
            threshold: f64,
        ) -> Vec<i32>;
        fn plane_count_within_distance_xyz(
            model: &SampleConsensusModelPlane_PointXYZ,
            coefficients: &[f32],
            threshold: f64,
        ) -> usize;
        fn plane_optimize_model_coefficients_xyz(
            model: &SampleConsensusModelPlane_PointXYZ,
            inliers: &[i32],
            coefficients: &[f32],
        ) -> Vec<f32>;

        // Sphere model methods - PointXYZ
        fn sphere_compute_model_coefficients_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            samples: &[i32],
        ) -> Vec<f32>;
        fn sphere_get_distances_to_model_xyz(
            model: &SampleConsensusModelSphere_PointXYZ,
            coefficients: &[f32],
        ) -> Vec<f64>;
        fn sphere_select_within_distance_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            coefficients: &[f32],
            threshold: f64,
        ) -> Vec<i32>;
        fn sphere_count_within_distance_xyz(
            model: &SampleConsensusModelSphere_PointXYZ,
            coefficients: &[f32],
            threshold: f64,
        ) -> usize;
        fn sphere_optimize_model_coefficients_xyz(
            model: &SampleConsensusModelSphere_PointXYZ,
            inliers: &[i32],
            coefficients: &[f32],
        ) -> Vec<f32>;
        fn sphere_set_radius_limits_xyz(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZ>,
            min_radius: f64,
            max_radius: f64,
        );

        // Plane model methods - PointXYZRGB
        fn plane_compute_model_coefficients_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            samples: &[i32],
        ) -> Vec<f32>;
        fn plane_get_distances_to_model_xyzrgb(
            model: &SampleConsensusModelPlane_PointXYZRGB,
            coefficients: &[f32],
        ) -> Vec<f64>;
        fn plane_select_within_distance_xyzrgb(
            model: Pin<&mut SampleConsensusModelPlane_PointXYZRGB>,
            coefficients: &[f32],
            threshold: f64,
        ) -> Vec<i32>;
        fn plane_count_within_distance_xyzrgb(
            model: &SampleConsensusModelPlane_PointXYZRGB,
            coefficients: &[f32],
            threshold: f64,
        ) -> usize;
        fn plane_optimize_model_coefficients_xyzrgb(
            model: &SampleConsensusModelPlane_PointXYZRGB,
            inliers: &[i32],
            coefficients: &[f32],
        ) -> Vec<f32>;

        // Sphere model methods - PointXYZRGB
        fn sphere_compute_model_coefficients_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            samples: &[i32],
        ) -> Vec<f32>;
        fn sphere_get_distances_to_model_xyzrgb(
            model: &SampleConsensusModelSphere_PointXYZRGB,
            coefficients: &[f32],
        ) -> Vec<f64>;
        fn sphere_select_within_distance_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            coefficients: &[f32],
            threshold: f64,
        ) -> Vec<i32>;
        fn sphere_count_within_distance_xyzrgb(
            model: &SampleConsensusModelSphere_PointXYZRGB,
            coefficients: &[f32],
            threshold: f64,
        ) -> usize;
        fn sphere_optimize_model_coefficients_xyzrgb(
            model: &SampleConsensusModelSphere_PointXYZRGB,
            inliers: &[i32],
            coefficients: &[f32],
        ) -> Vec<f32>;
        fn sphere_set_radius_limits_xyzrgb(
            model: Pin<&mut SampleConsensusModelSphere_PointXYZRGB>,
            min_radius: f64,
            max_radius: f64,
        );
    }
}
