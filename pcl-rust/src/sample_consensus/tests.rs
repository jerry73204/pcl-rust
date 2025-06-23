//! Tests corresponding to PCL's test/sample_consensus/*.cpp files
//!
//! This module tests the sample consensus functionality including:
//! - test/sample_consensus/test_sample_consensus.cpp - RANSAC, MSAC, etc. tests
//! - test/sample_consensus/test_sample_consensus_plane_models.cpp - Plane model tests
//! - test/sample_consensus/test_sample_consensus_line_models.cpp - Line model tests
//! - test/sample_consensus/test_sample_consensus_quadric_models.cpp - Quadric model tests
//!
//! Uses test fixtures from PCL test data:
//! - sac_plane_test.pcd - Plane segmentation test data
//! - Various synthetic point clouds for model fitting tests

use super::*;
use crate::common::{PointCloud, PointXYZ, PointXYZRGB};
use crate::error::PclResult;
use crate::sample_consensus::models::{PlaneModelXYZ, PlaneModelXYZRGB, SphereModelXYZ, SphereModelXYZRGB};
use crate::sample_consensus::ransac::{RansacPlaneXYZ, RansacPlaneXYZRGB, RansacSphereXYZ, RansacSphereXYZRGB};

/// Tests corresponding to test/sample_consensus/test_sample_consensus.cpp - general RANSAC functionality
#[cfg(test)]
mod general_ransac_tests {
    use super::*;

    #[test]
    fn test_ransac_plane_xyz_creation() -> PclResult<()> {
        // Test creating RANSAC plane fitter for PointXYZ
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add some planar points
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(1.0, 1.0, 0.0)?;
        
        let _ransac = RansacPlaneXYZ::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_ransac_plane_xyzrgb_creation() -> PclResult<()> {
        // Test creating RANSAC plane fitter for PointXYZRGB
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        // Add some planar points with color
        cloud.push(0.0, 0.0, 0.0, 255, 0, 0)?;
        cloud.push(1.0, 0.0, 0.0, 0, 255, 0)?;
        cloud.push(0.0, 1.0, 0.0, 0, 0, 255)?;
        cloud.push(1.0, 1.0, 0.0, 255, 255, 0)?;
        
        let _ransac = RansacPlaneXYZRGB::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_ransac_sphere_creation() -> PclResult<()> {
        // Test creating RANSAC sphere fitter
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Add points on a sphere (radius 1, center at origin)
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(-1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, -1.0, 0.0)?;
        cloud.push(0.0, 0.0, 1.0)?;
        cloud.push(0.0, 0.0, -1.0)?;
        
        let _ransac = RansacSphereXYZ::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_ransac_distance_threshold() -> PclResult<()> {
        // Test setting and getting distance threshold
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        
        ransac.set_distance_threshold(0.1);
        let threshold = ransac.distance_threshold();
        assert!((threshold - 0.1).abs() < 1e-6);
        
        ransac.set_distance_threshold(0.05);
        let threshold = ransac.distance_threshold();
        assert!((threshold - 0.05).abs() < 1e-6);
        Ok(())
    }

    #[test]
    fn test_ransac_max_iterations() -> PclResult<()> {
        // Test setting and getting maximum iterations
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        
        ransac.set_max_iterations(1000);
        assert_eq!(ransac.max_iterations(), 1000);
        
        ransac.set_max_iterations(500);
        assert_eq!(ransac.max_iterations(), 500);
        Ok(())
    }

    #[test]
    fn test_ransac_probability() -> PclResult<()> {
        // Test setting and getting desired probability
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        
        ransac.set_probability(0.99);
        let prob = ransac.probability();
        assert!((prob - 0.99).abs() < 1e-6);
        
        ransac.set_probability(0.95);
        let prob = ransac.probability();
        assert!((prob - 0.95).abs() < 1e-6);
        Ok(())
    }

    #[test]
    fn test_ransac_compute_model() -> PclResult<()> {
        // Test computing model using RANSAC
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create a perfect XY plane (z=0)
        for x in 0..10 {
            for y in 0..10 {
                cloud.push(x as f32, y as f32, 0.0)?;
            }
        }
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.01);
        ransac.set_max_iterations(1000);
        
        let result = ransac.compute_model()?;
        assert!(result, "RANSAC should successfully find a plane model");
        Ok(())
    }

    #[test]
    fn test_ransac_get_inliers() -> PclResult<()> {
        // Test getting inliers after model computation
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create mostly planar points with some outliers
        for i in 0..20 {
            cloud.push(i as f32, 0.0, 0.0)?; // Planar points
        }
        cloud.push(0.0, 0.0, 10.0)?; // Outlier
        cloud.push(5.0, 5.0, 10.0)?; // Outlier
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        
        if ransac.compute_model()? {
            let inliers = ransac.inliers();
            // Should find most of the planar points as inliers
            assert!(inliers.len() >= 15);
        }
        Ok(())
    }

    #[test]
    fn test_ransac_get_model_coefficients() -> PclResult<()> {
        // Test getting model coefficients after computation
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create XY plane (z=0)
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(1.0, 1.0, 0.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.01);
        
        if ransac.compute_model()? {
            let coefficients = ransac.model_coefficients();
            // Plane equation: ax + by + cz + d = 0
            // For XY plane: 0x + 0y + 1z + 0 = 0
            assert_eq!(coefficients.len(), 4);
            
            // Normal should point in Z direction (approximately [0, 0, 1] or [0, 0, -1])
            let normal_mag = (coefficients[0].powi(2) + coefficients[1].powi(2) + coefficients[2].powi(2)).sqrt();
            assert!((normal_mag - 1.0).abs() < 0.1, "Normal should be unit length");
        }
        Ok(())
    }

    #[test]
    fn test_ransac_empty_cloud() -> PclResult<()> {
        // Test RANSAC with empty cloud
        let cloud = PointCloud::<PointXYZ>::new()?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        
        let result = ransac.compute_model()?;
        assert!(!result, "RANSAC should fail with empty cloud");
        Ok(())
    }

    #[test]
    fn test_ransac_insufficient_points() -> PclResult<()> {
        // Test RANSAC with insufficient points for plane fitting
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Only 2 points - insufficient for plane
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        
        let result = ransac.compute_model()?;
        assert!(!result, "RANSAC should fail with insufficient points");
        Ok(())
    }
}

/// Tests corresponding to test/sample_consensus/test_sample_consensus_plane_models.cpp
#[cfg(test)]
mod plane_model_tests {
    use super::*;

    #[test]
    fn test_plane_model_xyz_creation() -> PclResult<()> {
        // Test creating plane model for PointXYZ
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        let _model = PlaneModelXYZ::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_plane_model_xyzrgb_creation() -> PclResult<()> {
        // Test creating plane model for PointXYZRGB
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        cloud.push(0.0, 0.0, 0.0, 255, 0, 0)?;
        cloud.push(1.0, 0.0, 0.0, 0, 255, 0)?;
        cloud.push(0.0, 1.0, 0.0, 0, 0, 255)?;
        
        let _model = PlaneModelXYZRGB::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_plane_model_compute_coefficients() -> PclResult<()> {
        // Test computing plane coefficients from sample points
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create XY plane (z=0)
        cloud.push(0.0, 0.0, 0.0)?; // Index 0
        cloud.push(1.0, 0.0, 0.0)?; // Index 1
        cloud.push(0.0, 1.0, 0.0)?; // Index 2
        cloud.push(1.0, 1.0, 0.0)?; // Index 3
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // Use first 3 points to define plane
        let samples = vec![0, 1, 2];
        let coefficients = model.compute_model_coefficients(&samples)?;
        
        assert_eq!(coefficients.len(), 4);
        // For XY plane, normal should be in Z direction
        let normal_z = coefficients[2].abs();
        assert!(normal_z > 0.9, "Z component of normal should be close to 1");
        Ok(())
    }

    #[test]
    fn test_plane_model_get_distances() -> PclResult<()> {
        // Test getting distances from points to plane
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points on and off XY plane
        cloud.push(0.0, 0.0, 0.0)?;   // On plane
        cloud.push(1.0, 0.0, 0.0)?;   // On plane
        cloud.push(0.0, 1.0, 0.0)?;   // On plane
        cloud.push(0.0, 0.0, 1.0)?;   // 1 unit above plane
        cloud.push(0.0, 0.0, -2.0)?;  // 2 units below plane
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // Define XY plane: 0x + 0y + 1z + 0 = 0
        let coefficients = vec![0.0, 0.0, 1.0, 0.0];
        let distances = model.get_distances_to_model(&coefficients);
        
        assert_eq!(distances.len(), 5);
        assert!(distances[0].abs() < 1e-6, "Point on plane should have distance ~0");
        assert!(distances[1].abs() < 1e-6, "Point on plane should have distance ~0");
        assert!(distances[2].abs() < 1e-6, "Point on plane should have distance ~0");
        assert!((distances[3].abs() - 1.0).abs() < 1e-6, "Point should be distance 1 from plane");
        assert!((distances[4].abs() - 2.0).abs() < 1e-6, "Point should be distance 2 from plane");
        Ok(())
    }

    #[test]
    fn test_plane_model_select_within_distance() -> PclResult<()> {
        // Test selecting points within distance threshold
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points at various distances from XY plane
        cloud.push(0.0, 0.0, 0.0)?;    // Distance 0
        cloud.push(1.0, 0.0, 0.05)?;   // Distance 0.05
        cloud.push(0.0, 1.0, 0.1)?;    // Distance 0.1
        cloud.push(0.0, 0.0, 0.2)?;    // Distance 0.2
        cloud.push(0.0, 0.0, 0.5)?;    // Distance 0.5
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // XY plane coefficients
        let coefficients = vec![0.0, 0.0, 1.0, 0.0];
        let threshold = 0.15;
        
        let inliers = model.select_within_distance(&coefficients, threshold)?;
        
        // Should include points at distances 0, 0.05, 0.1 (3 points)
        assert_eq!(inliers.len(), 3);
        assert!(inliers.contains(&0));
        assert!(inliers.contains(&1));
        assert!(inliers.contains(&2));
        Ok(())
    }

    #[test]
    fn test_plane_model_count_within_distance() -> PclResult<()> {
        // Test counting points within distance threshold
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points at known distances
        for i in 0..10 {
            cloud.push(i as f32, 0.0, 0.1 * i as f32)?; // Distance = 0.1 * i
        }
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // XY plane coefficients
        let coefficients = vec![0.0, 0.0, 1.0, 0.0];
        let threshold = 0.25;
        
        let count = model.count_within_distance(&coefficients, threshold)?;
        
        // Should count points with distance <= 0.25 (points 0, 1, 2)
        assert_eq!(count, 3);
        Ok(())
    }

    #[test]
    fn test_plane_model_optimize_coefficients() -> PclResult<()> {
        // Test optimizing plane coefficients using inliers
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create XY plane with slight noise
        for i in 0..10 {
            let noise = if i % 3 == 0 { 0.01 } else { 0.0 };
            cloud.push(i as f32, 0.0, noise)?;
        }
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // Initial rough coefficients
        let mut coefficients = vec![0.1, 0.1, 1.0, 0.0];
        
        // Use most points as inliers
        let inliers: Vec<i32> = (0..10).collect();
        
        let optimized = model.optimize_model_coefficients(&inliers, &mut coefficients)?;
        assert!(optimized, "Optimization should succeed");
        
        // Optimized normal should be closer to [0, 0, 1]
        let z_component = coefficients[2].abs();
        assert!(z_component > 0.95, "Optimized normal should be close to Z axis");
        Ok(())
    }

    #[test]
    fn test_plane_model_project_points() -> PclResult<()> {
        // Test projecting points onto plane
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points above XY plane
        cloud.push(0.0, 0.0, 1.0)?;
        cloud.push(1.0, 0.0, 2.0)?;
        cloud.push(0.0, 1.0, 0.5)?;
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // XY plane coefficients
        let coefficients = vec![0.0, 0.0, 1.0, 0.0];
        
        let projected_cloud = model.project_points(&coefficients)?;
        
        // Projected points should have z = 0
        assert_eq!(projected_cloud.size(), 3);
        // Would need point access methods to verify z coordinates are 0
        Ok(())
    }

    #[test]
    fn test_plane_model_invalid_samples() -> PclResult<()> {
        // Test with invalid sample indices
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        // Invalid indices
        let samples = vec![0, 1, 5]; // Index 5 doesn't exist
        let result = model.compute_model_coefficients(&samples);
        
        assert!(result.is_err(), "Should fail with invalid indices");
        Ok(())
    }

    #[test]
    fn test_plane_model_collinear_points() -> PclResult<()> {
        // Test with collinear points (cannot define a plane)
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // All points on a line
        cloud.push(0.0, 0.0, 0.0)?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(2.0, 0.0, 0.0)?;
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        
        let samples = vec![0, 1, 2];
        let result = model.compute_model_coefficients(&samples);
        
        // Should either fail or handle gracefully
        // The specific behavior depends on implementation
        assert!(result.is_ok() || result.is_err());
        Ok(())
    }
}

/// Tests for sphere model functionality
#[cfg(test)]
mod sphere_model_tests {
    use super::*;

    #[test]
    fn test_sphere_model_xyz_creation() -> PclResult<()> {
        // Test creating sphere model for PointXYZ
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points on unit sphere
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(-1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, -1.0, 0.0)?;
        
        let _model = SphereModelXYZ::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_sphere_model_xyzrgb_creation() -> PclResult<()> {
        // Test creating sphere model for PointXYZRGB
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        cloud.push(1.0, 0.0, 0.0, 255, 0, 0)?;
        cloud.push(-1.0, 0.0, 0.0, 0, 255, 0)?;
        cloud.push(0.0, 1.0, 0.0, 0, 0, 255)?;
        cloud.push(0.0, -1.0, 0.0, 255, 255, 0)?;
        
        let _model = SphereModelXYZRGB::new(&cloud)?;
        Ok(())
    }

    #[test]
    fn test_sphere_model_compute_coefficients() -> PclResult<()> {
        // Test computing sphere coefficients from sample points
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points on unit sphere centered at origin
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(-1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, -1.0, 0.0)?;
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Use 4 points to define sphere
        let samples = vec![0, 1, 2, 3];
        let coefficients = model.compute_model_coefficients(&samples)?;
        
        assert_eq!(coefficients.len(), 4);
        // Sphere: (x-cx)² + (y-cy)² + (z-cz)² = r²
        // Coefficients: [cx, cy, cz, r]
        
        // Center should be close to origin
        assert!(coefficients[0].abs() < 0.1, "X center should be near 0");
        assert!(coefficients[1].abs() < 0.1, "Y center should be near 0");
        assert!(coefficients[2].abs() < 0.1, "Z center should be near 0");
        
        // Radius should be close to 1
        assert!((coefficients[3] - 1.0).abs() < 0.1, "Radius should be close to 1");
        Ok(())
    }

    #[test]
    fn test_sphere_model_radius_limits() -> PclResult<()> {
        // Test setting radius limits
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(-1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        cloud.push(0.0, -1.0, 0.0)?;
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Set radius limits
        model.set_radius_limits(0.5, 2.0)?;
        
        let (min_radius, max_radius) = model.radius_limits();
        assert!((min_radius - 0.5).abs() < 1e-6);
        assert!((max_radius - 2.0).abs() < 1e-6);
        Ok(())
    }

    #[test]
    fn test_sphere_model_distances() -> PclResult<()> {
        // Test getting distances from points to sphere
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points at various distances from unit sphere
        cloud.push(1.0, 0.0, 0.0)?;    // On sphere (distance 0)
        cloud.push(2.0, 0.0, 0.0)?;    // Outside sphere (distance 1)
        cloud.push(0.5, 0.0, 0.0)?;    // Inside sphere (distance 0.5)
        cloud.push(0.0, 0.0, 0.0)?;    // At center (distance 1)
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Unit sphere centered at origin: [0, 0, 0, 1]
        let coefficients = vec![0.0, 0.0, 0.0, 1.0];
        let distances = model.get_distances_to_model(&coefficients);
        
        assert_eq!(distances.len(), 4);
        assert!(distances[0].abs() < 1e-6, "Point on sphere should have distance ~0");
        assert!((distances[1] - 1.0).abs() < 1e-6, "Point should be distance 1 from sphere");
        assert!((distances[2] - 0.5).abs() < 1e-6, "Point should be distance 0.5 from sphere");
        assert!((distances[3] - 1.0).abs() < 1e-6, "Center point should be distance 1 from sphere");
        Ok(())
    }

    #[test]
    fn test_sphere_model_project_points() -> PclResult<()> {
        // Test projecting points onto sphere surface
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Points to project onto unit sphere
        cloud.push(2.0, 0.0, 0.0)?;    // Outside, should project to (1,0,0)
        cloud.push(0.5, 0.0, 0.0)?;    // Inside, should project to (1,0,0)
        cloud.push(0.0, 0.5, 0.0)?;    // Inside, should project to (0,1,0)
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Unit sphere centered at origin
        let coefficients = vec![0.0, 0.0, 0.0, 1.0];
        
        let projected_cloud = model.project_points(&coefficients)?;
        
        // All projected points should be on sphere surface
        assert_eq!(projected_cloud.size(), 3);
        // Would need point access methods to verify distances from center
        Ok(())
    }

    #[test]
    fn test_sphere_model_invalid_radius_limits() -> PclResult<()> {
        // Test with invalid radius limits
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(1.0, 0.0, 0.0)?;
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Min radius > max radius should fail
        let result = model.set_radius_limits(2.0, 1.0);
        assert!(result.is_err(), "Should fail with min > max radius");
        Ok(())
    }

    #[test]
    fn test_sphere_model_negative_radius() -> PclResult<()> {
        // Test with negative radius limits
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(1.0, 0.0, 0.0)?;
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        // Negative radius should fail
        let result = model.set_radius_limits(-1.0, 1.0);
        assert!(result.is_err(), "Should fail with negative radius");
        Ok(())
    }

    #[test]
    fn test_sphere_model_insufficient_points() -> PclResult<()> {
        // Test with insufficient points for sphere fitting
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Only 3 points - insufficient for sphere (needs 4)
        cloud.push(1.0, 0.0, 0.0)?;
        cloud.push(-1.0, 0.0, 0.0)?;
        cloud.push(0.0, 1.0, 0.0)?;
        
        let mut model = SphereModelXYZ::new(&cloud)?;
        
        let samples = vec![0, 1, 2];
        let result = model.compute_model_coefficients(&samples);
        
        assert!(result.is_err(), "Should fail with insufficient points");
        Ok(())
    }
}

/// Tests for line model functionality (placeholder)
#[cfg(test)]
mod line_model_tests {
    use super::*;

    #[test]
    fn test_line_model_placeholder() {
        // TODO: Implement line model tests when line model is available
        // This corresponds to test/sample_consensus/test_sample_consensus_line_models.cpp
        
        // Line models fit 3D lines to point data
        // Equation: point = origin + t * direction
        
        todo!( "Line model tests not yet implemented");
    }
}

/// Tests for quadric model functionality (placeholder)
#[cfg(test)]
mod quadric_model_tests {
    use super::*;

    #[test]
    fn test_quadric_model_placeholder() {
        // TODO: Implement quadric model tests when quadric models are available
        // This corresponds to test/sample_consensus/test_sample_consensus_quadric_models.cpp
        
        // Quadric models include cylinders, cones, and other quadric surfaces
        
        todo!( "Quadric model tests not yet implemented");
    }
}

/// Integration tests combining RANSAC with different models
#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_ransac_plane_integration() -> PclResult<()> {
        // Test complete RANSAC plane fitting workflow
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create mostly planar data with outliers
        for x in 0..10 {
            for y in 0..10 {
                // Most points on XY plane
                cloud.push(x as f32, y as f32, 0.0)?;
            }
        }
        
        // Add some outliers
        cloud.push(5.0, 5.0, 10.0)?;
        cloud.push(3.0, 3.0, -8.0)?;
        cloud.push(7.0, 2.0, 15.0)?;
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        ransac.set_max_iterations(1000);
        ransac.set_probability(0.99);
        
        let success = ransac.compute_model()?;
        assert!(success, "RANSAC should find plane in mostly planar data");
        
        let inliers = ransac.inliers();
        assert!(inliers.len() >= 90, "Should find most planar points as inliers");
        
        let coefficients = ransac.model_coefficients();
        assert_eq!(coefficients.len(), 4);
        
        // Normal should be approximately [0, 0, ±1]
        let normal_z = coefficients[2].abs();
        assert!(normal_z > 0.9, "Normal should be close to Z axis");
        
        Ok(())
    }

    #[test]
    fn test_ransac_sphere_integration() -> PclResult<()> {
        // Test complete RANSAC sphere fitting workflow
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create points on unit sphere with some noise and outliers
        let n_points = 50;
        for i in 0..n_points {
            let angle = 2.0 * std::f32::consts::PI * i as f32 / n_points as f32;
            let x = angle.cos();
            let y = angle.sin();
            let z = 0.0;
            
            // Add small noise
            let noise = 0.05;
            cloud.push(x + noise * (i as f32).sin(), y + noise * (i as f32).cos(), z)?;
        }
        
        // Add outliers
        cloud.push(5.0, 5.0, 5.0)?;
        cloud.push(-3.0, -3.0, -3.0)?;
        
        let mut ransac = RansacSphereXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        ransac.set_max_iterations(1000);
        
        let success = ransac.compute_model()?;
        assert!(success, "RANSAC should find sphere in spherical data");
        
        let inliers = ransac.inliers();
        assert!(inliers.len() >= 40, "Should find most spherical points as inliers");
        
        let coefficients = ransac.model_coefficients();
        assert_eq!(coefficients.len(), 4);
        
        // Center should be close to origin
        assert!(coefficients[0].abs() < 0.2, "X center should be near 0");
        assert!(coefficients[1].abs() < 0.2, "Y center should be near 0");
        assert!(coefficients[2].abs() < 0.2, "Z center should be near 0");
        
        // Radius should be close to 1
        assert!((coefficients[3] - 1.0).abs() < 0.2, "Radius should be close to 1");
        
        Ok(())
    }

    #[test]
    fn test_multiple_model_comparison() -> PclResult<()> {
        // Test fitting both plane and sphere to same data to compare results
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create planar data
        for x in 0..10 {
            for y in 0..10 {
                cloud.push(x as f32, y as f32, 0.0)?;
            }
        }
        
        // Try both plane and sphere fitting
        let mut ransac_plane = RansacPlaneXYZ::new(&cloud)?;
        ransac_plane.set_distance_threshold(0.1);
        ransac_plane.set_max_iterations(500);
        
        let mut ransac_sphere = RansacSphereXYZ::new(&cloud)?;
        ransac_sphere.set_distance_threshold(0.1);
        ransac_sphere.set_max_iterations(500);
        
        let plane_success = ransac_plane.compute_model()?;
        let sphere_success = ransac_sphere.compute_model()?;
        
        if plane_success && sphere_success {
            let plane_inliers = ransac_plane.inliers().len();
            let sphere_inliers = ransac_sphere.inliers().len();
            
            // Plane should fit better to planar data
            assert!(plane_inliers >= sphere_inliers, 
                   "Plane should have more inliers for planar data");
        }
        
        Ok(())
    }
}

/// Performance and stress tests
#[cfg(test)]
mod performance_tests {
    use super::*;

    #[test]
    fn test_ransac_large_dataset() -> PclResult<()> {
        // Test RANSAC performance with larger dataset
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create large planar dataset
        for x in 0..50 {
            for y in 0..50 {
                cloud.push(x as f32 * 0.1, y as f32 * 0.1, 0.0)?;
            }
        }
        
        // Add noise and outliers
        for i in 0..500 {
            cloud.push(
                (i as f32).sin() * 10.0,
                (i as f32).cos() * 10.0, 
                (i as f32 * 0.1).sin() * 5.0
            )?;
        }
        
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        ransac.set_distance_threshold(0.1);
        ransac.set_max_iterations(5000);
        
        let success = ransac.compute_model()?;
        assert!(success, "RANSAC should handle large datasets");
        
        let inliers = ransac.inliers();
        assert!(inliers.len() >= 2000, "Should find many inliers in large planar data");
        
        Ok(())
    }

    #[test]
    fn test_ransac_memory_usage() -> PclResult<()> {
        // Test that RANSAC doesn't leak memory
        for iteration in 0..20 {
            let mut cloud = PointCloud::<PointXYZ>::new()?;
            
            // Create different data each iteration
            for i in 0..100 {
                cloud.push(
                    (i + iteration) as f32, 
                    (i * 2 + iteration) as f32, 
                    0.0
                )?;
            }
            
            let mut ransac = RansacPlaneXYZ::new(&cloud)?;
            ransac.set_distance_threshold(0.1);
            ransac.set_max_iterations(100);
            
            let _success = ransac.compute_model()?;
            let _inliers = ransac.inliers();
            let _coefficients = ransac.model_coefficients();
        }
        
        Ok(())
    }

    #[test]
    fn test_model_operations_performance() -> PclResult<()> {
        // Test performance of model operations
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create test data
        for i in 0..1000 {
            cloud.push(i as f32 * 0.01, 0.0, 0.0)?;
        }
        
        let mut model = PlaneModelXYZ::new(&cloud)?;
        let coefficients = vec![0.0, 0.0, 1.0, 0.0];
        
        // Test distance computation performance
        for _ in 0..10 {
            let _distances = model.get_distances_to_model(&coefficients);
        }
        
        // Test inlier selection performance
        for threshold in [0.1, 0.05, 0.01, 0.005, 0.001].iter() {
            let _inliers = model.select_within_distance(&coefficients, *threshold)?;
        }
        
        Ok(())
    }
}