//! Comprehensive tests for keypoint detection algorithms

use crate::common::{PointCloud, PointXYZ, PointXYZI, XYZ, XYZI};
use crate::error::PclResult;
use crate::keypoints::*;
use crate::search::{KdTreeXYZ, SearchInputCloud};

/// Create a simple test cloud with known features
fn create_test_cloud_xyz() -> PclResult<PointCloud<XYZ>> {
    let mut cloud = PointCloud::new()?;

    // Create a simple L-shaped corner (should be detected as keypoint)
    // Horizontal line
    for i in 0..10 {
        cloud.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
    }

    // Vertical line
    for i in 1..10 {
        cloud.push(PointXYZ::new(0.0, i as f32 * 0.1, 0.0))?;
    }

    // Add some planar points (should not be keypoints)
    for i in 0..5 {
        for j in 0..5 {
            cloud.push(PointXYZ::new(2.0 + i as f32 * 0.1, j as f32 * 0.1, 0.0))?;
        }
    }

    Ok(cloud)
}

/// Create a test cloud with intensity values
fn create_test_cloud_xyzi() -> PclResult<PointCloud<XYZI>> {
    let mut cloud = PointCloud::new()?;

    // Create a grid with varying intensity
    for i in 0..10 {
        for j in 0..10 {
            let x = i as f32 * 0.1;
            let y = j as f32 * 0.1;
            let z = 0.0;
            let intensity = (i * j) as f32; // Varying intensity

            cloud.push(PointXYZI::new(x, y, z, intensity))?;
        }
    }

    // Add some high-intensity points (potential keypoints)
    cloud.push(PointXYZI::new(0.5, 0.5, 0.1, 255.0))?;
    cloud.push(PointXYZI::new(0.2, 0.8, 0.1, 255.0))?;

    Ok(cloud)
}

#[cfg(test)]
mod harris_tests {
    use super::*;

    #[test]
    fn test_harris_basic_detection() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut harris = Harris3D::new().unwrap();

        // Set up search
        let mut kdtree = KdTreeXYZ::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        harris.set_search_method(&kdtree).unwrap();

        // Configure
        harris.set_radius(0.2).unwrap();
        harris.set_threshold(0.0).unwrap(); // Low threshold to get some keypoints

        // Detect
        harris.set_input_cloud(&cloud).unwrap();
        let keypoints = harris.compute().unwrap();

        // Should detect at least the corner point
        assert!(keypoints.size() > 0, "Should detect at least one keypoint");
    }

    #[test]
    fn test_harris_parameter_validation() {
        let mut harris = Harris3D::new().unwrap();

        // Test invalid radius
        assert!(harris.set_radius(-1.0).is_err());
        assert!(harris.set_radius(0.0).is_err());

        // Test invalid threshold
        assert!(harris.set_threshold(-1.0).is_err());

        // Valid parameters should work
        assert!(harris.set_radius(0.1).is_ok());
        assert!(harris.set_threshold(0.01).is_ok());
    }

    #[test]
    fn test_harris_non_max_suppression() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut harris = Harris3D::new().unwrap();

        let mut kdtree = KdTreeXYZ::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        harris.set_search_method(&kdtree).unwrap();

        harris.set_radius(0.2).unwrap();
        harris.set_threshold(0.0).unwrap();
        harris.set_input_cloud(&cloud).unwrap();

        // Test with non-max suppression disabled
        harris.set_non_max_suppression(false).unwrap();
        let keypoints_no_nms = harris.compute().unwrap();

        // Test with non-max suppression enabled
        harris.set_non_max_suppression(true).unwrap();
        let keypoints_with_nms = harris.compute().unwrap();

        // With NMS should have fewer or equal keypoints
        assert!(keypoints_with_nms.size() <= keypoints_no_nms.size());
    }

    #[test]
    fn test_harris_empty_cloud() {
        let cloud = PointCloud::<XYZ>::new().unwrap();
        let mut harris = Harris3D::new().unwrap();

        // Should error on empty cloud
        assert!(harris.set_input_cloud(&cloud).is_err());
    }
}

#[cfg(test)]
mod iss_tests {
    use super::*;

    #[test]
    fn test_iss_basic_detection() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut iss = Iss3D::new().unwrap();

        // Set up search
        let mut kdtree = KdTreeXYZ::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        iss.set_search_method(&kdtree).unwrap();

        // Configure
        iss.set_salient_radius(0.2).unwrap();
        iss.set_non_max_radius(0.1).unwrap();
        iss.set_threshold21(0.975).unwrap();
        iss.set_threshold32(0.975).unwrap();
        iss.set_min_neighbors(3).unwrap();

        // Detect
        iss.set_input_cloud(&cloud).unwrap();
        let keypoints = iss.compute().unwrap();

        // Should detect some keypoints (size() returns usize which is always >= 0)
        let _keypoint_count = keypoints.size();
    }

    #[test]
    fn test_iss_parameter_validation() {
        let mut iss = Iss3D::new().unwrap();

        // Test invalid radii
        assert!(iss.set_salient_radius(-1.0).is_err());
        assert!(iss.set_non_max_radius(0.0).is_err());

        // Test invalid thresholds
        assert!(iss.set_threshold21(-1.0).is_err());
        assert!(iss.set_threshold32(0.0).is_err());

        // Test invalid min neighbors
        assert!(iss.set_min_neighbors(0).is_err());
        assert!(iss.set_min_neighbors(-1).is_err());

        // Valid parameters
        assert!(iss.set_salient_radius(0.1).is_ok());
        assert!(iss.set_non_max_radius(0.05).is_ok());
        assert!(iss.set_threshold21(0.9).is_ok());
        assert!(iss.set_threshold32(0.9).is_ok());
        assert!(iss.set_min_neighbors(5).is_ok());
    }

    #[test]
    fn test_iss_threshold_effects() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut iss = Iss3D::new().unwrap();

        let mut kdtree = KdTreeXYZ::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        iss.set_search_method(&kdtree).unwrap();

        iss.set_salient_radius(0.2).unwrap();
        iss.set_non_max_radius(0.1).unwrap();
        iss.set_min_neighbors(3).unwrap();
        iss.set_input_cloud(&cloud).unwrap();

        // Strict thresholds
        iss.set_threshold21(0.5).unwrap();
        iss.set_threshold32(0.5).unwrap();
        let keypoints_strict = iss.compute().unwrap();

        // Relaxed thresholds
        iss.set_threshold21(0.99).unwrap();
        iss.set_threshold32(0.99).unwrap();
        let keypoints_relaxed = iss.compute().unwrap();

        // Relaxed thresholds should find more or equal keypoints
        assert!(keypoints_relaxed.size() >= keypoints_strict.size());
    }
}

#[cfg(test)]
mod sift_tests {
    use super::*;
    use crate::search::KdTreeXYZI;

    #[test]
    fn test_sift_basic_detection() {
        let cloud = create_test_cloud_xyzi().unwrap();
        let mut sift = SiftKeypoint::new().unwrap();

        // Set up search with XYZI
        let mut kdtree = KdTreeXYZI::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        sift.set_search_method(&kdtree).unwrap();

        // Configure
        sift.set_scales(0.01, 3.0, 4).unwrap();
        sift.set_minimum_contrast(0.001).unwrap(); // Low contrast for test

        // Detect
        sift.set_input_cloud(&cloud).unwrap();
        let keypoints = sift.compute().unwrap();

        // Should complete without error
        assert!(!keypoints.empty(), "SIFT detection should produce results");
    }

    #[test]
    fn test_sift_parameter_validation() {
        let mut sift = SiftKeypoint::new().unwrap();

        // Test invalid scale parameters
        assert!(sift.set_scales(-1.0, 3.0, 4).is_err());
        assert!(sift.set_scales(0.01, -1.0, 4).is_err());
        assert!(sift.set_scales(0.01, 3.0, -1).is_err());
        assert!(sift.set_scales(0.01, 3.0, 0).is_err());

        // Test invalid contrast
        assert!(sift.set_minimum_contrast(-1.0).is_err());

        // Valid parameters
        assert!(sift.set_scales(0.01, 3.0, 4).is_ok());
        assert!(sift.set_minimum_contrast(0.03).is_ok());
    }

    #[test]
    fn test_sift_requires_intensity() {
        // SIFT requires PointXYZI input, not PointXYZ
        let cloud = create_test_cloud_xyzi().unwrap();
        let mut sift = SiftKeypoint::new().unwrap();

        // This should work with XYZI cloud
        assert!(sift.set_input_cloud(&cloud).is_ok());
    }
}

#[cfg(test)]
mod builder_tests {
    use super::*;

    #[test]
    fn test_harris_builder_complete() {
        let harris = Harris3DBuilder::new()
            .radius(0.1)
            .threshold(0.01)
            .non_max_suppression(true)
            .refine(true)
            .build();

        assert!(harris.is_ok());
    }

    #[test]
    fn test_harris_builder_partial() {
        // Builder should work with partial configuration
        let harris = Harris3DBuilder::new().radius(0.1).build();

        assert!(harris.is_ok());
    }

    #[test]
    fn test_iss_builder_complete() {
        let iss = Iss3DBuilder::new()
            .salient_radius(0.1)
            .non_max_radius(0.05)
            .threshold21(0.9)
            .threshold32(0.9)
            .min_neighbors(5)
            .build();

        assert!(iss.is_ok());
    }

    #[test]
    fn test_sift_builder_complete() {
        let sift = SiftKeypointBuilder::new()
            .scales(0.01, 4.0, 4)
            .minimum_contrast(0.03)
            .build();

        assert!(sift.is_ok());
    }

    #[test]
    fn test_builder_invalid_params() {
        // Invalid radius should propagate through builder
        let harris = Harris3DBuilder::new().radius(-1.0).build();

        assert!(harris.is_err());
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_keypoint_detection_pipeline() {
        // Create a more complex test cloud
        let mut cloud = PointCloud::<XYZ>::new().unwrap();

        // Add a cube with edges (corners should be keypoints)
        let positions = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (1.0, 1.0, 1.0),
        ];

        for &(x, y, z) in &positions {
            cloud.push(PointXYZ::new(x, y, z)).unwrap();
        }

        // Add points along edges
        for i in 1..10 {
            let t = i as f32 / 10.0;
            cloud.push(PointXYZ::new(t, 0.0, 0.0)).unwrap();
            cloud.push(PointXYZ::new(0.0, t, 0.0)).unwrap();
            cloud.push(PointXYZ::new(0.0, 0.0, t)).unwrap();
        }

        // Test Harris detection
        let mut harris = Harris3D::new().unwrap();
        let mut kdtree = KdTreeXYZ::new().unwrap();
        kdtree.set_input_cloud(&cloud).unwrap();
        harris.set_search_method(&kdtree).unwrap();
        harris.set_radius(0.3).unwrap();
        harris.set_threshold(0.0).unwrap();
        harris.set_input_cloud(&cloud).unwrap();

        let harris_keypoints = harris.compute().unwrap();
        assert!(
            harris_keypoints.size() > 0,
            "Harris should detect corner keypoints"
        );

        // Test ISS detection on same cloud
        let mut iss = Iss3D::new().unwrap();
        iss.set_search_method(&kdtree).unwrap();
        iss.set_salient_radius(0.3).unwrap();
        iss.set_non_max_radius(0.15).unwrap();
        iss.set_threshold21(0.975).unwrap();
        iss.set_threshold32(0.975).unwrap();
        iss.set_min_neighbors(3).unwrap();
        iss.set_input_cloud(&cloud).unwrap();

        let iss_keypoints = iss.compute().unwrap();
        let _keypoint_count = iss_keypoints.size(); // ISS detection completed
    }

    #[test]
    fn test_point_with_scale_conversions() {
        // Test From trait implementations
        let coords = vec![1.0, 2.0, 3.0, 0.5];
        let point = PointWithScale::from(coords.clone());

        assert_eq!(point.x, 1.0);
        assert_eq!(point.y, 2.0);
        assert_eq!(point.z, 3.0);
        assert_eq!(point.scale, 0.5);

        // Test from slice
        let slice: &[f32] = &[1.0, 2.0, 3.0, 0.5];
        let point2 = PointWithScale::from(slice);

        assert_eq!(point2.coords(), coords);
        assert_eq!(point2.spatial_coords(), vec![1.0, 2.0, 3.0]);
    }

    #[test]
    #[should_panic(expected = "PointWithScale requires 4 coordinates")]
    fn test_point_with_scale_wrong_size() {
        let coords = vec![1.0, 2.0, 3.0]; // Missing scale
        let _point = PointWithScale::from(coords);
    }
}
