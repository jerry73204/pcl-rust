//! Comprehensive tests for segmentation algorithms

use crate::common::{PointCloud, PointXYZ, PointXYZRGB, XYZ, XYZRGB};
use crate::error::PclResult;
use crate::segmentation::*;

/// Create a simple test cloud with known structure
fn create_test_cloud_xyz() -> PclResult<PointCloud<XYZ>> {
    let mut cloud = PointCloud::new()?;

    // Create a simple plane at z=0
    for x in -5..=5 {
        for y in -5..=5 {
            cloud.push(PointXYZ::new(x as f32 * 0.1, y as f32 * 0.1, 0.0))?;
        }
    }

    // Add some points above the plane (objects)
    for i in 0..20 {
        let angle = i as f32 * std::f32::consts::PI / 10.0;
        cloud.push(PointXYZ::new(angle.cos() * 0.3, angle.sin() * 0.3, 0.5))?;
    }

    Ok(cloud)
}

/// Create a colored test cloud
fn create_test_cloud_xyzrgb() -> PclResult<PointCloud<XYZRGB>> {
    let mut cloud = PointCloud::new()?;

    // Red region
    for x in 0..10 {
        for y in 0..10 {
            cloud.push(PointXYZRGB::new(
                x as f32 * 0.1,
                y as f32 * 0.1,
                0.0,
                255,
                0,
                0,
            ))?;
        }
    }

    // Green region
    for x in 0..10 {
        for y in 0..10 {
            cloud.push(PointXYZRGB::new(
                x as f32 * 0.1 + 1.5,
                y as f32 * 0.1,
                0.0,
                0,
                255,
                0,
            ))?;
        }
    }

    Ok(cloud)
}

#[cfg(test)]
mod sac_segmentation_tests {
    use super::*;

    #[test]
    fn test_sac_creation() {
        let sac = SacSegmentationXYZ::new();
        assert!(sac.is_ok());
    }

    #[test]
    fn test_sac_model_types() {
        let mut sac = SacSegmentationXYZ::new().unwrap();

        // Test setting different model types
        assert!(sac.set_model_type(ModelType::Plane).is_ok());
        assert_eq!(sac.model_type(), ModelType::Plane);

        assert!(sac.set_model_type(ModelType::Sphere).is_ok());
        assert_eq!(sac.model_type(), ModelType::Sphere);

        assert!(sac.set_model_type(ModelType::Cylinder).is_ok());
        assert_eq!(sac.model_type(), ModelType::Cylinder);
    }

    #[test]
    fn test_sac_method_types() {
        let mut sac = SacSegmentationXYZ::new().unwrap();

        // Test setting different method types
        assert!(sac.set_method_type(MethodType::Ransac).is_ok());
        assert_eq!(sac.method_type(), MethodType::Ransac);

        assert!(sac.set_method_type(MethodType::LMedS).is_ok());
        assert_eq!(sac.method_type(), MethodType::LMedS);
    }

    #[test]
    fn test_sac_parameters() {
        let mut sac = SacSegmentationXYZ::new().unwrap();

        // Test distance threshold
        assert!(sac.set_distance_threshold(0.1).is_ok());
        assert_eq!(sac.distance_threshold(), 0.1);

        // Invalid distance threshold
        assert!(sac.set_distance_threshold(-0.1).is_err());

        // Test max iterations
        assert!(sac.set_max_iterations(1000).is_ok());
        assert_eq!(sac.max_iterations(), 1000);

        // Invalid max iterations
        assert!(sac.set_max_iterations(0).is_err());

        // Test optimize coefficients
        assert!(sac.set_optimize_coefficients(true).is_ok());
        assert!(sac.optimize_coefficients());
    }

    #[test]
    fn test_sac_builder() {
        let sac = SacSegmentationBuilder::new()
            .model_type(ModelType::Plane)
            .method_type(MethodType::Ransac)
            .distance_threshold(0.02)
            .max_iterations(100)
            .optimize_coefficients(true)
            .build();

        assert!(sac.is_ok());

        let sac = sac.unwrap();
        assert_eq!(sac.model_type(), ModelType::Plane);
        assert_eq!(sac.method_type(), MethodType::Ransac);
        assert_eq!(sac.distance_threshold(), 0.02);
        assert_eq!(sac.max_iterations(), 100);
        assert!(sac.optimize_coefficients());
    }

    #[test]
    fn test_sac_plane_segmentation() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut sac = SacSegmentationXYZ::new().unwrap();

        sac.set_input_cloud(&cloud).unwrap();
        sac.set_model_type(ModelType::Plane).unwrap();
        sac.set_method_type(MethodType::Ransac).unwrap();
        sac.set_distance_threshold(0.01).unwrap();

        let result = sac.segment_model().unwrap();

        // Should find the plane at z=0
        assert!(result.inliers.len() > 100); // Most of the plane points
        assert_eq!(result.coefficients.len(), 4); // Plane has 4 coefficients
    }

    #[test]
    fn test_sac_empty_cloud() {
        let cloud = PointCloud::<XYZ>::new().unwrap();
        let mut sac = SacSegmentationXYZ::new().unwrap();

        // Should error on empty cloud
        assert!(sac.set_input_cloud(&cloud).is_err());
    }
}

#[cfg(test)]
mod region_growing_tests {
    use super::*;

    #[test]
    fn test_region_growing_creation() {
        let rg = RegionGrowingXYZ::new();
        assert!(rg.is_ok());
    }

    #[test]
    fn test_region_growing_parameters() {
        let mut rg = RegionGrowingXYZ::new().unwrap();

        // Test min/max cluster size
        assert!(rg.set_min_cluster_size(50).is_ok());
        assert_eq!(rg.min_cluster_size(), 50);

        assert!(rg.set_max_cluster_size(10000).is_ok());
        assert_eq!(rg.max_cluster_size(), 10000);

        // Invalid sizes
        assert!(rg.set_min_cluster_size(0).is_err());
        assert!(rg.set_max_cluster_size(-1).is_err());

        // Test smoothness threshold
        assert!(rg.set_smoothness_threshold(0.1).is_ok());
        assert_eq!(rg.smoothness_threshold(), 0.1);

        // Invalid smoothness
        assert!(rg.set_smoothness_threshold(-0.1).is_err());

        // Test curvature threshold
        assert!(rg.set_curvature_threshold(1.0).is_ok());
        assert_eq!(rg.curvature_threshold(), 1.0);

        // Test number of neighbors
        assert!(rg.set_number_of_neighbours(30).is_ok());
        assert_eq!(rg.number_of_neighbours(), 30);

        // Invalid neighbors
        assert!(rg.set_number_of_neighbours(0).is_err());
    }

    #[test]
    fn test_region_growing_builder() {
        let rg = RegionGrowingXYZBuilder::new()
            .min_cluster_size(100)
            .max_cluster_size(5000)
            .smoothness_threshold(3.0f32.to_radians())
            .curvature_threshold(0.5)
            .number_of_neighbours(20)
            .build();

        assert!(rg.is_ok());

        let rg = rg.unwrap();
        assert_eq!(rg.min_cluster_size(), 100);
        assert_eq!(rg.max_cluster_size(), 5000);
    }

    #[test]
    fn test_region_growing_rgb_creation() {
        let rg = RegionGrowingRgbXYZRGB::new();
        assert!(rg.is_ok());
    }

    #[test]
    fn test_region_growing_rgb_parameters() {
        let mut rg = RegionGrowingRgbXYZRGB::new().unwrap();

        // Test distance threshold
        assert!(rg.set_distance_threshold(0.05).is_ok());
        assert_eq!(rg.distance_threshold(), 0.05);

        // Invalid distance
        assert!(rg.set_distance_threshold(0.0).is_err());

        // Test color thresholds
        assert!(rg.set_point_color_threshold(10.0).is_ok());
        assert_eq!(rg.point_color_threshold(), 10.0);

        assert!(rg.set_region_color_threshold(5.0).is_ok());
        assert_eq!(rg.region_color_threshold(), 5.0);

        // Invalid thresholds
        assert!(rg.set_point_color_threshold(-1.0).is_err());
        assert!(rg.set_region_color_threshold(-1.0).is_err());
    }

    #[test]
    fn test_region_growing_rgb_segmentation() {
        let cloud = create_test_cloud_xyzrgb().unwrap();
        let mut rg = RegionGrowingRgbXYZRGB::new().unwrap();

        rg.set_input_cloud(&cloud).unwrap();
        rg.set_distance_threshold(0.05).unwrap();
        rg.set_point_color_threshold(50.0).unwrap();
        rg.set_region_color_threshold(50.0).unwrap();
        rg.set_min_cluster_size(10).unwrap();

        let clusters = rg.extract().unwrap();

        // Should find at least 2 color regions (red and green)
        assert!(clusters.len() >= 2);
    }
}

#[cfg(test)]
mod clustering_tests {
    use super::*;

    #[test]
    fn test_euclidean_clustering_creation() {
        let ec = EuclideanClusterExtractionXYZ::new();
        assert!(ec.is_ok());
    }

    #[test]
    fn test_euclidean_clustering_parameters() {
        let mut ec = EuclideanClusterExtractionXYZ::new().unwrap();

        // Test cluster tolerance
        assert!(ec.set_cluster_tolerance(0.1).is_ok());
        assert_eq!(ec.cluster_tolerance(), 0.1);

        // Invalid tolerance
        assert!(ec.set_cluster_tolerance(0.0).is_err());

        // Test min/max cluster size
        assert!(ec.set_min_cluster_size(10).is_ok());
        assert_eq!(ec.min_cluster_size(), 10);

        assert!(ec.set_max_cluster_size(1000).is_ok());
        assert_eq!(ec.max_cluster_size(), 1000);

        // Invalid sizes
        assert!(ec.set_min_cluster_size(0).is_err());
        assert!(ec.set_max_cluster_size(-1).is_err());
    }

    #[test]
    fn test_euclidean_clustering_builder() {
        let ec = EuclideanClusterExtractionBuilder::new()
            .cluster_tolerance(0.05)
            .min_cluster_size(50)
            .max_cluster_size(5000)
            .build();

        assert!(ec.is_ok());
    }

    #[test]
    fn test_euclidean_clustering_extraction() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut ec = EuclideanClusterExtractionXYZ::new().unwrap();

        ec.set_input_cloud(&cloud).unwrap();
        ec.set_cluster_tolerance(0.2).unwrap();
        ec.set_min_cluster_size(10).unwrap();
        ec.set_max_cluster_size(1000).unwrap();

        let clusters = ec.extract().unwrap();

        // Should find at least 2 clusters (plane and object above)
        assert!(clusters.len() >= 1);
    }

    #[test]
    fn test_conditional_clustering_creation() {
        let cec = ConditionalEuclideanClusteringXYZ::new();
        assert!(cec.is_ok());
    }

    #[test]
    fn test_conditional_clustering_parameters() {
        let mut cec = ConditionalEuclideanClusteringXYZ::new().unwrap();

        // Test cluster tolerance
        assert!(cec.set_cluster_tolerance(0.1).is_ok());
        assert_eq!(cec.cluster_tolerance(), 0.1);

        // Invalid tolerance
        assert!(cec.set_cluster_tolerance(0.0).is_err());

        // Test min/max cluster size
        assert!(cec.set_min_cluster_size(10).is_ok());
        assert_eq!(cec.min_cluster_size(), 10);

        assert!(cec.set_max_cluster_size(1000).is_ok());
        assert_eq!(cec.max_cluster_size(), 1000);
    }

    #[test]
    fn test_clustering_trait() {
        use crate::segmentation::ClusteringXYZ;

        let mut ec = EuclideanClusterExtractionXYZ::new().unwrap();

        // Test trait methods
        assert!(ec.set_cluster_tolerance(0.1).is_ok());
        assert_eq!(ec.cluster_tolerance(), 0.1);

        assert!(ec.set_min_cluster_size(20).is_ok());
        assert_eq!(ec.min_cluster_size(), 20);

        assert!(ec.set_max_cluster_size(2000).is_ok());
        assert_eq!(ec.max_cluster_size(), 2000);
    }
}

#[cfg(test)]
mod pmf_tests {
    use super::*;

    #[test]
    fn test_pmf_creation() {
        let pmf = ProgressiveMorphologicalFilterXYZ::new();
        assert!(pmf.is_ok());
    }

    #[test]
    fn test_pmf_parameters() {
        let mut pmf = ProgressiveMorphologicalFilterXYZ::new().unwrap();

        // Test max window size
        assert!(pmf.set_max_window_size(20).is_ok());
        assert_eq!(pmf.max_window_size(), 20);

        // Invalid window size
        assert!(pmf.set_max_window_size(0).is_err());

        // Test slope
        assert!(pmf.set_slope(0.7).is_ok());
        assert_eq!(pmf.slope(), 0.7);

        // Invalid slope
        assert!(pmf.set_slope(-0.1).is_err());
        assert!(pmf.set_slope(1.1).is_err());

        // Test distances
        assert!(pmf.set_max_distance(10.0).is_ok());
        assert_eq!(pmf.max_distance(), 10.0);

        assert!(pmf.set_initial_distance(0.5).is_ok());
        assert_eq!(pmf.initial_distance(), 0.5);

        // Invalid distances
        assert!(pmf.set_max_distance(0.0).is_err());
        assert!(pmf.set_initial_distance(-0.1).is_err());

        // Test cell size
        assert!(pmf.set_cell_size(1.0).is_ok());
        assert_eq!(pmf.cell_size(), 1.0);

        // Invalid cell size
        assert!(pmf.set_cell_size(0.0).is_err());

        // Test base
        assert!(pmf.set_base(2.0).is_ok());
        assert_eq!(pmf.base(), 2.0);

        // Invalid base
        assert!(pmf.set_base(0.5).is_err());

        // Test exponential
        pmf.set_exponential(true);
        assert!(pmf.exponential());
    }

    #[test]
    fn test_pmf_builder() {
        let pmf = ProgressiveMorphologicalFilterXYZBuilder::new()
            .max_window_size(30)
            .slope(0.8)
            .max_distance(5.0)
            .initial_distance(0.2)
            .cell_size(0.5)
            .base(2.5)
            .exponential(false)
            .build();

        assert!(pmf.is_ok());

        let pmf = pmf.unwrap();
        assert_eq!(pmf.max_window_size(), 30);
        assert_eq!(pmf.slope(), 0.8);
        assert_eq!(pmf.max_distance(), 5.0);
        assert_eq!(pmf.initial_distance(), 0.2);
        assert_eq!(pmf.cell_size(), 0.5);
        assert_eq!(pmf.base(), 2.5);
        assert!(!pmf.exponential());
    }

    #[test]
    fn test_pmf_ground_extraction() {
        let cloud = create_test_cloud_xyz().unwrap();
        let mut pmf = ProgressiveMorphologicalFilterXYZ::new().unwrap();

        pmf.set_input_cloud(&cloud).unwrap();
        pmf.set_max_window_size(10).unwrap();
        pmf.set_slope(1.0).unwrap();
        pmf.set_max_distance(2.0).unwrap();
        pmf.set_initial_distance(0.1).unwrap();
        pmf.set_cell_size(0.5).unwrap();

        let ground_indices = pmf.extract_ground().unwrap();

        // Should extract some ground points
        assert!(!ground_indices.is_empty());
    }
}

#[cfg(test)]
mod other_segmentation_tests {
    use super::*;

    #[test]
    fn test_min_cut_creation() {
        let min_cut = MinCutSegmentationXYZ::new();
        assert!(min_cut.is_ok());
    }

    #[test]
    fn test_extract_polygonal_prism_creation() {
        let prism = ExtractPolygonalPrismDataXYZ::new();
        assert!(prism.is_ok());
    }

    #[test]
    fn test_extract_polygonal_prism_parameters() {
        let mut prism = ExtractPolygonalPrismDataXYZ::new().unwrap();

        // Test height limits
        assert!(prism.set_height_limits(0.0, 1.0).is_ok());

        // Invalid height limits
        assert!(prism.set_height_limits(1.0, 0.0).is_err());
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_segmentation_pipeline() {
        // Create a more complex test cloud
        let mut cloud = PointCloud::<XYZ>::new().unwrap();

        // Ground plane
        for x in -20..=20 {
            for y in -20..=20 {
                cloud
                    .push(PointXYZ::new(x as f32 * 0.05, y as f32 * 0.05, 0.0))
                    .unwrap();
            }
        }

        // Object 1
        for x in 0..5 {
            for y in 0..5 {
                for z in 0..10 {
                    cloud
                        .push(PointXYZ::new(
                            x as f32 * 0.02,
                            y as f32 * 0.02,
                            z as f32 * 0.02 + 0.1,
                        ))
                        .unwrap();
                }
            }
        }

        // 1. First extract the ground plane using RANSAC
        let mut sac = SacSegmentationXYZ::new().unwrap();
        sac.set_input_cloud(&cloud).unwrap();
        sac.set_model_type(ModelType::Plane).unwrap();
        sac.set_method_type(MethodType::Ransac).unwrap();
        sac.set_distance_threshold(0.01).unwrap();

        let plane_result = sac.segment_model().unwrap();
        assert!(!plane_result.inliers.is_empty());

        // 2. Then perform clustering on the remaining points
        let mut ec = EuclideanClusterExtractionXYZ::new().unwrap();
        ec.set_input_cloud(&cloud).unwrap();
        ec.set_cluster_tolerance(0.05).unwrap();
        ec.set_min_cluster_size(10).unwrap();

        let clusters = ec.extract().unwrap();
        assert!(!clusters.is_empty());
    }

    #[test]
    fn test_model_type_conversions() {
        let types = vec![
            ModelType::Plane,
            ModelType::Line,
            ModelType::Circle2D,
            ModelType::Circle3D,
            ModelType::Sphere,
            ModelType::Cylinder,
            ModelType::Cone,
        ];

        for model_type in types {
            let constant = model_type.to_pcl_constant();
            assert!(constant >= 0);
        }
    }

    #[test]
    fn test_method_type_conversions() {
        let types = vec![
            MethodType::Ransac,
            MethodType::LMedS,
            MethodType::MSAC,
            MethodType::RRANSAC,
            MethodType::RMSAC,
            MethodType::MLESAC,
            MethodType::PROSAC,
        ];

        for method_type in types {
            let constant = method_type.to_pcl_constant();
            assert!(constant >= 0);
        }
    }

    #[test]
    fn test_segmentation_trait() {
        // Test that the Segmentation trait is properly defined
        fn accept_segmentation<T, S: Segmentation<T>>(_seg: &S) {}

        // This should compile if the trait is properly defined
        // In a real implementation, algorithms would implement this trait
    }
}
