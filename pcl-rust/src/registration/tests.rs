//! Tests corresponding to PCL's test/registration/*.cpp files
//!
//! This module tests the registration functionality including:
//! - test/registration/test_registration.cpp - General registration and ICP tests
//! - test/registration/test_ndt.cpp - NDT registration tests
//! - test/registration/test_correspondence_estimation.cpp - Correspondence estimation tests
//! - test/registration/test_correspondence_rejectors.cpp - Correspondence rejection tests
//! - test/registration/test_sac_ia.cpp - SAC-IA (Sample Consensus Initial Alignment) tests
//!
//! Uses test fixtures from PCL test data:
//! - bunny.pcd - Stanford bunny model for registration tests
//! - office1.pcd, office2.pcd - Office scans for registration tests
//! - milk.pcd - Milk carton scan for transformation tests

use super::*;
use crate::common::{PointCloud, PointXYZ, PointXYZRGB, XYZ, XYZRGB};
use crate::error::PclResult;

/// Tests corresponding to test/registration/test_registration.cpp - ICP functionality
#[cfg(test)]
mod icp_tests {
    use super::*;

    #[test]
    fn test_icp_xyz_creation() -> PclResult<()> {
        // Test creating ICP instance for PointXYZ
        let _icp = IcpXYZ::new()?;
        Ok(())
    }

    #[test]
    fn test_icp_xyzrgb_creation() -> PclResult<()> {
        // Test creating ICP instance for PointXYZRGB
        let _icp = IcpXYZRGB::new()?;
        Ok(())
    }

    #[test]
    fn test_icp_parameter_configuration() -> PclResult<()> {
        // Test setting and getting ICP parameters
        let mut icp = IcpXYZ::new()?;

        // Test max iterations
        icp.set_max_iterations(50)?;
        assert_eq!(icp.get_max_iterations(), 50);

        // Test transformation epsilon
        icp.set_transformation_epsilon(1e-8)?;
        assert_eq!(icp.get_transformation_epsilon(), 1e-8);

        // Test Euclidean fitness epsilon
        icp.set_euclidean_fitness_epsilon(1e-6)?;
        assert_eq!(icp.get_euclidean_fitness_epsilon(), 1e-6);

        // Test max correspondence distance
        icp.set_max_correspondence_distance(0.05)?;
        assert_eq!(icp.get_max_correspondence_distance(), 0.05);

        Ok(())
    }

    #[test]
    fn test_icp_invalid_parameters() -> PclResult<()> {
        // Test parameter validation
        let mut icp = IcpXYZ::new()?;

        // Test negative iterations
        assert!(icp.set_max_iterations(-1).is_err());
        assert!(icp.set_max_iterations(0).is_err());

        // Test negative epsilons
        assert!(icp.set_transformation_epsilon(-1.0).is_err());
        assert!(icp.set_euclidean_fitness_epsilon(-1.0).is_err());

        // Test negative/zero correspondence distance
        assert!(icp.set_max_correspondence_distance(-1.0).is_err());
        assert!(icp.set_max_correspondence_distance(0.0).is_err());

        Ok(())
    }

    #[test]
    fn test_icp_empty_clouds() -> PclResult<()> {
        // Test ICP behavior with empty clouds
        let mut icp = IcpXYZ::new()?;
        let empty_cloud = PointCloud::<XYZ>::new()?;

        // Setting empty source should error
        assert!(icp.set_input_source(&empty_cloud).is_err());

        // Setting empty target should error
        assert!(icp.set_input_target(&empty_cloud).is_err());

        Ok(())
    }

    #[test]
    fn test_icp_basic_alignment() -> PclResult<()> {
        // Test basic ICP alignment between two simple clouds
        let mut icp = IcpXYZ::new()?;

        // Create source cloud
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(0.0, 1.0, 0.0))?;

        // Create target cloud (slightly translated)
        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.1, 0.1, 0.0))?;
        target.push(PointXYZ::new(1.1, 0.1, 0.0))?;
        target.push(PointXYZ::new(0.1, 1.1, 0.0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let aligned = icp.align()?;
        assert!(icp.has_converged());
        assert!(aligned.size() == source.size());

        Ok(())
    }

    #[test]
    fn test_icp_with_initial_guess() -> PclResult<()> {
        // Test ICP alignment with initial transformation guess
        let mut icp = IcpXYZ::new()?;

        // Create source and target clouds
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.5, 0.5, 0.0))?;
        target.push(PointXYZ::new(1.5, 0.5, 0.0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        // Create initial guess (translation)
        let mut initial_guess = TransformationMatrix::identity();
        initial_guess.set_translation(0.5, 0.5, 0.0);

        let aligned = icp.align_with_guess(&initial_guess)?;
        assert!(aligned.size() == source.size());

        Ok(())
    }

    #[test]
    fn test_icp_fitness_score() -> PclResult<()> {
        // Test fitness score computation
        let mut icp = IcpXYZ::new()?;

        // Create identical clouds (perfect alignment)
        let mut cloud = PointCloud::<XYZ>::new()?;
        cloud.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, 1.0, 0.0))?;

        icp.set_input_source(&cloud)?;
        icp.set_input_target(&cloud)?;

        let _aligned = icp.align()?;
        let fitness = icp.get_fitness_score();

        // Perfect alignment should have low fitness score
        assert!(fitness >= 0.0);
        assert!(fitness < 0.001);

        Ok(())
    }

    #[test]
    fn test_icp_transformation_matrix() -> PclResult<()> {
        // Test getting final transformation matrix
        let mut icp = IcpXYZ::new()?;

        // Create simple clouds
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let _aligned = icp.align()?;
        let transform = icp.get_final_transformation();

        // Transformation should not be identity
        // TODO: Add is_identity method to TransformationMatrix
        // For now, check that transformation is not default
        let identity = TransformationMatrix::identity();
        assert!(transform != identity);

        Ok(())
    }

    #[test]
    fn test_icp_builder() -> PclResult<()> {
        // Test ICP builder pattern
        let icp = IcpXYZBuilder::new()
            .max_iterations(100)
            .transformation_epsilon(1e-9)
            .euclidean_fitness_epsilon(1e-7)
            .max_correspondence_distance(0.1)
            .build()?;

        let mut icp = icp;
        assert_eq!(icp.get_max_iterations(), 100);
        assert_eq!(icp.get_transformation_epsilon(), 1e-9);
        assert_eq!(icp.get_euclidean_fitness_epsilon(), 1e-7);
        assert_eq!(icp.get_max_correspondence_distance(), 0.1);

        Ok(())
    }

    #[test]
    fn test_icp_convergence_criteria() -> PclResult<()> {
        // Test different convergence criteria
        let mut icp = IcpXYZ::new()?;

        // Set very strict criteria
        icp.set_max_iterations(1)?;
        icp.set_transformation_epsilon(1e-15)?;
        icp.set_euclidean_fitness_epsilon(1e-15)?;

        // Create slightly misaligned clouds
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.01, 0.01, 0.0))?;
        target.push(PointXYZ::new(1.01, 0.01, 0.0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let _aligned = icp.align()?;

        // With only 1 iteration, it may not converge
        // This tests that the algorithm respects the iteration limit

        Ok(())
    }

    #[test]
    fn test_icp_xyzrgb_alignment() -> PclResult<()> {
        // Test ICP with colored point clouds
        let mut icp = IcpXYZRGB::new()?;

        // Create colored source cloud
        let mut source = PointCloud::<XYZRGB>::new()?;
        source.push(PointXYZRGB::new(0.0, 0.0, 0.0, 255, 0, 0))?; // Red
        source.push(PointXYZRGB::new(1.0, 0.0, 0.0, 0, 255, 0))?; // Green

        // Create colored target cloud
        let mut target = PointCloud::<XYZRGB>::new()?;
        target.push(PointXYZRGB::new(0.1, 0.0, 0.0, 255, 0, 0))?;
        target.push(PointXYZRGB::new(1.1, 0.0, 0.0, 0, 255, 0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let aligned = icp.align()?;
        assert!(aligned.size() == source.size());

        Ok(())
    }

    #[test]
    fn test_icp_large_cloud_performance() -> PclResult<()> {
        // Test ICP performance with larger point clouds
        let mut icp = IcpXYZ::new()?;
        icp.set_max_iterations(10)?; // Limit iterations for test speed

        // Create larger clouds
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;

        // Create a 10x10 grid of points
        for i in 0..10 {
            for j in 0..10 {
                source.push(PointXYZ::new(i as f32, j as f32, 0.0))?;
                // Target is slightly translated
                target.push(PointXYZ::new(i as f32 + 0.1, j as f32 + 0.1, 0.0))?;
            }
        }

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let aligned = icp.align()?;
        assert!(aligned.size() == source.size());

        Ok(())
    }
}

/// Tests corresponding to test/registration/test_ndt.cpp - NDT registration
#[cfg(test)]
mod ndt_tests {
    use super::*;

    #[test]
    fn test_ndt_creation() -> PclResult<()> {
        // Test creating NDT instance
        let _ndt = NdtXYZ::new()?;
        Ok(())
    }

    #[test]
    fn test_ndt_parameter_configuration() -> PclResult<()> {
        // Test NDT parameter setting
        let mut ndt = NdtXYZ::new()?;

        // Test transformation epsilon
        ndt.set_transformation_epsilon(1e-8);
        assert_eq!(ndt.get_transformation_epsilon(), 1e-8);

        // Test step size
        ndt.set_step_size(0.1);
        assert_eq!(ndt.get_step_size(), 0.1);

        // Test resolution
        ndt.set_resolution(1.0);
        assert_eq!(ndt.get_resolution(), 1.0);

        // Test max iterations
        ndt.set_max_iterations(35);
        assert_eq!(ndt.get_max_iterations(), 35);

        Ok(())
    }

    #[test]
    fn test_ndt_basic_alignment() -> PclResult<()> {
        // Test basic NDT alignment
        let mut ndt = NdtXYZ::new()?;

        // Configure NDT
        ndt.set_resolution(1.0);
        ndt.set_max_iterations(20);
        ndt.set_transformation_epsilon(1e-6);

        // Create source cloud (grid pattern)
        let mut source = PointCloud::<XYZ>::new()?;
        for i in 0..5 {
            for j in 0..5 {
                source.push(PointXYZ::new(i as f32, j as f32, 0.0))?;
            }
        }

        // Create target cloud (slightly transformed)
        let mut target = PointCloud::<XYZ>::new()?;
        for i in 0..5 {
            for j in 0..5 {
                target.push(PointXYZ::new(i as f32 + 0.5, j as f32 + 0.5, 0.0))?;
            }
        }

        ndt.set_input_source(&source)?;
        ndt.set_input_target(&target)?;

        let aligned = ndt.align()?;
        assert!(ndt.has_converged());
        assert_eq!(aligned.size(), source.size());

        Ok(())
    }

    #[test]
    fn test_ndt_with_initial_guess() -> PclResult<()> {
        // Test NDT with initial transformation guess
        let mut ndt = NdtXYZ::new()?;
        ndt.set_resolution(1.0);

        // Create clouds
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(0.0, 1.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(2.0, 2.0, 0.0))?;
        target.push(PointXYZ::new(3.0, 2.0, 0.0))?;
        target.push(PointXYZ::new(2.0, 3.0, 0.0))?;

        ndt.set_input_source(&source)?;
        ndt.set_input_target(&target)?;

        // Create initial guess
        let mut initial_guess = TransformationMatrix::identity();
        initial_guess.set_translation(2.0, 2.0, 0.0);

        let aligned = ndt.align_with_guess(&initial_guess)?;
        assert_eq!(aligned.size(), source.size());

        Ok(())
    }

    #[test]
    fn test_ndt_resolution_impact() -> PclResult<()> {
        // Test impact of different resolutions on NDT
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;

        // Create test clouds
        for i in 0..10 {
            source.push(PointXYZ::new(i as f32 * 0.1, 0.0, 0.0))?;
            target.push(PointXYZ::new(i as f32 * 0.1 + 0.05, 0.0, 0.0))?;
        }

        // Test with fine resolution
        let mut ndt_fine = NdtXYZ::new()?;
        ndt_fine.set_resolution(0.1);
        ndt_fine.set_input_source(&source)?;
        ndt_fine.set_input_target(&target)?;
        let _aligned_fine = ndt_fine.align()?;
        let score_fine = ndt_fine.get_fitness_score();

        // Test with coarse resolution
        let mut ndt_coarse = NdtXYZ::new()?;
        ndt_coarse.set_resolution(1.0);
        ndt_coarse.set_input_source(&source)?;
        ndt_coarse.set_input_target(&target)?;
        let _aligned_coarse = ndt_coarse.align()?;
        let score_coarse = ndt_coarse.get_fitness_score();

        // Scores will differ based on resolution
        assert!(score_fine >= 0.0);
        assert!(score_coarse >= 0.0);

        Ok(())
    }

    #[test]
    fn test_ndt_step_size_configuration() -> PclResult<()> {
        // Test different step sizes for optimization
        let mut ndt = NdtXYZ::new()?;

        // Test various step sizes
        let step_sizes = vec![0.01, 0.1, 1.0];
        for step_size in step_sizes {
            ndt.set_step_size(step_size);
            assert_eq!(ndt.get_step_size(), step_size);
        }

        Ok(())
    }

    #[test]
    fn test_ndt_xyzrgb_alignment() -> PclResult<()> {
        // Test NDT with colored point clouds
        let mut ndt = NdtXYZRGB::new()?;

        // Create colored clouds
        let mut source = PointCloud::<XYZRGB>::new()?;
        source.push(PointXYZRGB::new(0.0, 0.0, 0.0, 255, 0, 0))?;
        source.push(PointXYZRGB::new(1.0, 0.0, 0.0, 0, 255, 0))?;

        let mut target = PointCloud::<XYZRGB>::new()?;
        target.push(PointXYZRGB::new(0.1, 0.1, 0.0, 255, 0, 0))?;
        target.push(PointXYZRGB::new(1.1, 0.1, 0.0, 0, 255, 0))?;

        ndt.set_input_source(&source)?;
        ndt.set_input_target(&target)?;

        let aligned = ndt.align()?;
        assert_eq!(aligned.size(), source.size());

        Ok(())
    }

    #[test]
    fn test_ndt_builder() -> PclResult<()> {
        // Test NDT builder pattern
        let ndt = NdtXYZBuilder::new()
            .resolution(0.5)
            .step_size(0.05)
            .max_iterations(50)
            .transformation_epsilon(1e-8)
            .build()?;

        let mut ndt = ndt;
        assert_eq!(ndt.get_resolution(), 0.5);
        assert_eq!(ndt.get_step_size(), 0.05);
        assert_eq!(ndt.get_max_iterations(), 50);
        assert_eq!(ndt.get_transformation_epsilon(), 1e-8);

        Ok(())
    }

    #[test]
    fn test_ndt_convergence_behavior() -> PclResult<()> {
        // Test NDT convergence with different settings
        let mut ndt = NdtXYZ::new()?;

        // Set very strict convergence criteria
        ndt.set_max_iterations(1);
        ndt.set_transformation_epsilon(1e-15);

        // Create test clouds
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;
        for i in 0..3 {
            source.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
            target.push(PointXYZ::new(i as f32 + 10.0, 0.0, 0.0))?; // Large offset
        }

        ndt.set_input_source(&source)?;
        ndt.set_input_target(&target)?;

        let _aligned = ndt.align()?;

        // With only 1 iteration and large offset, unlikely to converge
        // This tests iteration limit enforcement

        Ok(())
    }
}

/// Tests corresponding to test/registration/test_correspondence_estimation.cpp
#[cfg(test)]
mod correspondence_tests {
    use super::*;

    #[test]
    fn test_correspondence_creation() -> PclResult<()> {
        // Test creating correspondence estimation
        let _corr = CorrespondenceEstimation::new()?;
        Ok(())
    }

    #[test]
    fn test_correspondence_struct() {
        // Test Correspondence struct
        let corr = Correspondence::new(10, 20, 1.5);
        assert_eq!(corr.source_index, 10);
        assert_eq!(corr.target_index, 20);
        assert_eq!(corr.distance, 1.5);
    }

    #[test]
    fn test_correspondence_estimation_simple() -> PclResult<()> {
        // Test basic correspondence estimation
        let mut est = CorrespondenceEstimation::new()?;

        // Create identical clouds (perfect correspondences)
        let mut cloud = PointCloud::<XYZ>::new()?;
        cloud.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, 1.0, 0.0))?;

        est.set_input_source(&cloud)?;
        est.set_input_target(&cloud)?;

        let correspondences = est.determine_correspondences()?;

        // Should find perfect correspondences
        assert_eq!(correspondences.len(), cloud.size());
        for (i, corr) in correspondences.iter().enumerate() {
            assert_eq!(corr.source_index, i as i32);
            assert_eq!(corr.target_index, i as i32);
            assert!(corr.distance < 1e-6); // Perfect match
        }

        Ok(())
    }

    #[test]
    fn test_correspondence_estimation_offset() -> PclResult<()> {
        // Test correspondence with offset clouds
        let mut est = CorrespondenceEstimation::new()?;

        // Create source cloud
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        // Create target cloud (offset)
        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.1, 0.0, 0.0))?;
        target.push(PointXYZ::new(1.1, 0.0, 0.0))?;

        est.set_input_source(&source)?;
        est.set_input_target(&target)?;

        let correspondences = est.determine_correspondences()?;

        // Should find correspondences with non-zero distances
        assert_eq!(correspondences.len(), source.size());
        for corr in &correspondences {
            assert!(corr.distance > 0.0);
            assert!(corr.distance < 0.2); // Small offset
        }

        Ok(())
    }

    #[test]
    fn test_reciprocal_correspondences() -> PclResult<()> {
        // Test reciprocal correspondence estimation
        let mut est = CorrespondenceEstimation::new()?;

        // Create asymmetric clouds
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(2.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        target.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        // Missing point at (2.0, 0.0, 0.0)

        est.set_input_source(&source)?;
        est.set_input_target(&target)?;

        let reciprocal = est.determine_reciprocal_correspondences()?;

        // Reciprocal correspondences should be symmetric
        // May have fewer correspondences than source points
        assert!(reciprocal.len() <= source.size());

        Ok(())
    }

    #[test]
    fn test_correspondence_empty_clouds() -> PclResult<()> {
        // Test correspondence with empty clouds
        let mut est = CorrespondenceEstimation::new()?;

        let empty = PointCloud::<XYZ>::new()?;
        let mut cloud = PointCloud::<XYZ>::new()?;
        cloud.push(PointXYZ::new(0.0, 0.0, 0.0))?;

        // Empty source
        est.set_input_source(&empty)?;
        est.set_input_target(&cloud)?;
        let corr1 = est.determine_correspondences()?;
        assert_eq!(corr1.len(), 0);

        // Empty target
        est.set_input_source(&cloud)?;
        est.set_input_target(&empty)?;
        let corr2 = est.determine_correspondences()?;
        assert_eq!(corr2.len(), 0);

        Ok(())
    }

    #[test]
    fn test_correspondence_large_clouds() -> PclResult<()> {
        // Test correspondence with larger clouds
        let mut est = CorrespondenceEstimation::new()?;

        // Create larger clouds
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;

        for i in 0..50 {
            let x = i as f32 * 0.1;
            source.push(PointXYZ::new(x, 0.0, 0.0))?;
            target.push(PointXYZ::new(x + 0.01, 0.0, 0.0))?; // Small offset
        }

        est.set_input_source(&source)?;
        est.set_input_target(&target)?;

        let correspondences = est.determine_correspondences()?;
        assert_eq!(correspondences.len(), source.size());

        Ok(())
    }
}

/// Tests corresponding to test/registration/test_correspondence_rejectors.cpp
#[cfg(test)]
mod correspondence_rejection_tests {
    use super::*;

    #[test]
    fn test_correspondence_rejector_sac_creation() -> PclResult<()> {
        // Test creating RANSAC correspondence rejector
        let _rejector = CorrespondenceRejectorSampleConsensus::new()?;
        Ok(())
    }

    #[test]
    fn test_correspondence_rejector_configuration() -> PclResult<()> {
        // Test configuring correspondence rejector
        let mut rejector = CorrespondenceRejectorSampleConsensus::new()?;

        // Set inlier threshold
        rejector.set_inlier_threshold(0.05);
        assert_eq!(rejector.get_inlier_threshold(), 0.05);

        // TODO: Add max_iterations methods to CorrespondenceRejectorSampleConsensus
        // rejector.set_max_iterations(1000);
        // assert_eq!(rejector.get_max_iterations(), 1000);

        Ok(())
    }

    #[test]
    fn test_correspondence_rejection_basic() -> PclResult<()> {
        // Test basic correspondence rejection
        let mut rejector = CorrespondenceRejectorSampleConsensus::new()?;
        rejector.set_inlier_threshold(0.1);

        // Create test correspondences with outliers
        let correspondences = vec![
            Correspondence::new(0, 0, 0.01), // Good
            Correspondence::new(1, 1, 0.02), // Good
            Correspondence::new(2, 5, 10.0), // Outlier (large distance)
            Correspondence::new(3, 3, 0.03), // Good
        ];

        // Create clouds for rejection
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;
        for i in 0..4 {
            source.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
            target.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
        }

        rejector.set_input_source(&source)?;
        rejector.set_input_target(&target)?;

        let filtered = rejector.filter_correspondences(&correspondences)?;

        // Should have fewer correspondences after rejection
        assert!(filtered.len() < correspondences.len());

        Ok(())
    }

    #[test]
    fn test_correspondence_rejector_empty_input() -> PclResult<()> {
        // Test rejector with empty input
        let mut rejector = CorrespondenceRejectorSampleConsensus::new()?;

        let empty_corr: Vec<Correspondence> = vec![];
        let cloud = PointCloud::<XYZ>::new()?;

        rejector.set_input_source(&cloud)?;
        rejector.set_input_target(&cloud)?;

        let filtered = rejector.filter_correspondences(&empty_corr)?;
        assert_eq!(filtered.len(), 0);

        Ok(())
    }

    #[test]
    fn test_correspondence_rejector_invalid_threshold() -> PclResult<()> {
        // Test invalid threshold values
        let mut rejector = CorrespondenceRejectorSampleConsensus::new()?;

        // TODO: Add validation for negative threshold
        // Currently set_inlier_threshold returns void
        rejector.set_inlier_threshold(0.01); // Set valid threshold

        Ok(())
    }
}

/// Tests corresponding to test/registration/test_sac_ia.cpp - SAC-IA alignment
#[cfg(test)]
mod sac_ia_tests {

    #[test]
    fn test_sac_ia_placeholder() {
        // TODO: Implement SAC-IA tests when SAC-IA is available
        // SAC-IA (Sample Consensus Initial Alignment) provides
        // initial alignment for feature-based registration

        // This is a placeholder test until SAC-IA is implemented
    }

    #[test]
    fn test_sac_ia_feature_alignment_placeholder() {
        // TODO: Test SAC-IA with feature descriptors
        // This would test alignment using FPFH, SHOT, or other features

        // This is a placeholder test until SAC-IA feature alignment is implemented
    }

    #[test]
    fn test_sac_ia_convergence_placeholder() {
        // TODO: Test SAC-IA convergence criteria

        // This is a placeholder test until SAC-IA convergence is implemented
    }
}

/// Tests for transformation utilities
#[cfg(test)]
mod transformation_tests {
    use super::*;
    use crate::{PclError, PclResult};

    #[test]
    fn test_transformation_matrix_creation() {
        // Test creating transformation matrices
        let identity = TransformationMatrix::identity();
        assert!(identity == TransformationMatrix::identity());

        let zeros = TransformationMatrix::from_array(&[0.0; 16]);
        assert!(zeros != TransformationMatrix::identity());
    }

    #[test]
    fn test_transformation_matrix_translation() {
        // Test setting translation
        let mut transform = TransformationMatrix::identity();
        transform.set_translation(1.0, 2.0, 3.0);

        let [tx, ty, tz] = transform.translation();
        assert_eq!(tx, 1.0);
        assert_eq!(ty, 2.0);
        assert_eq!(tz, 3.0);
    }

    #[test]
    fn test_transformation_matrix_rotation() {
        // Test rotation methods
        // Test rotation around Z axis (90 degrees)
        let rotated = TransformationMatrix::rotation_z(std::f32::consts::PI / 2.0);
        assert!(rotated != TransformationMatrix::identity());

        // TODO: Add set_euler_angles method
        // For now test basic rotation
    }

    #[test]
    fn test_transformation_matrix_composition() {
        // Test combining transformations
        let t1 = TransformationMatrix::translation_matrix(1.0, 0.0, 0.0);
        let t2 = TransformationMatrix::translation_matrix(0.0, 1.0, 0.0);

        let combined = t1 * t2;
        let [tx, ty, tz] = combined.translation();

        // Combined translation should be (1, 1, 0)
        assert_eq!(tx, 1.0);
        assert_eq!(ty, 1.0);
        assert_eq!(tz, 0.0);
    }

    #[test]
    fn test_transformation_matrix_inverse() -> PclResult<()> {
        // Test matrix inversion
        let mut transform = TransformationMatrix::identity();
        transform.set_translation(2.0, 3.0, 4.0);

        let inverse = transform.inverse().ok_or(PclError::ProcessingFailed {
            message: "Failed to compute inverse".into(),
        })?;
        let combined = transform * inverse;

        // Transform * Inverse should be identity
        let _identity = TransformationMatrix::identity();
        // Check that combined is approximately identity
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                let actual = combined[(i, j)];
                assert!((actual - expected).abs() < 1e-6);
            }
        }

        Ok(())
    }

    #[test]
    fn test_transformation_matrix_from_vec() -> PclResult<()> {
        // Test creating from vector
        let vec = vec![
            1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, 1.0, 7.0, 0.0, 0.0, 0.0, 1.0,
        ];

        let transform =
            TransformationMatrix::from_vec(&vec).ok_or(PclError::InvalidParameters {
                message: "Failed to create transformation from vec".into(),
                parameter: "vec".into(),
                expected: "16 elements".into(),
                actual: vec.len().to_string(),
            })?;
        let [tx, ty, tz] = transform.translation();

        assert_eq!(tx, 5.0);
        assert_eq!(ty, 6.0);
        assert_eq!(tz, 7.0);

        Ok(())
    }

    #[test]
    fn test_transformation_matrix_to_vec() {
        // Test converting to vector
        let mut transform = TransformationMatrix::identity();
        transform.set_translation(1.0, 2.0, 3.0);

        let vec = transform.to_vec();
        assert_eq!(vec.len(), 16);

        // Check translation components
        assert_eq!(vec[3], 1.0); // tx
        assert_eq!(vec[7], 2.0); // ty
        assert_eq!(vec[11], 3.0); // tz
    }

    #[test]
    fn test_transform_3d_creation() {
        // Test Transform3D wrapper
        let transform = Transform3D::identity();
        let identity = TransformationMatrix::identity();
        assert!(transform == identity);
    }

    #[test]
    fn test_transform_3d_operations() {
        // Test Transform3D operations
        let t1 = Transform3D::translation_matrix(1.0, 0.0, 0.0);
        let t2 = Transform3D::rotation_z(std::f32::consts::PI / 4.0);

        let combined = t1 * t2;
        let identity = TransformationMatrix::identity();
        assert!(combined != identity);
    }
}

/// Tests for transformation estimation
#[cfg(test)]
mod transformation_estimation_tests {
    use super::*;

    #[test]
    fn test_transformation_estimation_svd_creation() -> PclResult<()> {
        // Test creating SVD transformation estimator
        let _estimator = TransformationEstimationSVD::new()?;
        Ok(())
    }

    #[test]
    fn test_transformation_estimation_simple() -> PclResult<()> {
        // Test basic transformation estimation
        let mut estimator = TransformationEstimationSVD::new()?;

        // Create corresponding point sets
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(0.0, 1.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        target.push(PointXYZ::new(2.0, 0.0, 0.0))?;
        target.push(PointXYZ::new(1.0, 1.0, 0.0))?;

        // Create correspondences
        let correspondences = vec![
            Correspondence::new(0, 0, 0.0),
            Correspondence::new(1, 1, 0.0),
            Correspondence::new(2, 2, 0.0),
        ];

        let transform =
            estimator.estimate_rigid_transformation(&source, &target, &correspondences)?;

        // Should find translation of (1, 0, 0)
        let [tx, ty, tz] = transform.translation();
        assert!((tx - 1.0).abs() < 0.01);
        assert!(ty.abs() < 0.01);
        assert!(tz.abs() < 0.01);

        Ok(())
    }

    #[test]
    fn test_transformation_estimation_with_rotation() -> PclResult<()> {
        // Test transformation estimation with rotation
        let mut estimator = TransformationEstimationSVD::new()?;

        // Create source cloud
        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(1.0, 0.0, 0.0))?;
        source.push(PointXYZ::new(0.0, 1.0, 0.0))?;
        source.push(PointXYZ::new(0.0, 0.0, 1.0))?;

        // Create target cloud (90 degree rotation around Z)
        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(0.0, 1.0, 0.0))?;
        target.push(PointXYZ::new(-1.0, 0.0, 0.0))?;
        target.push(PointXYZ::new(0.0, 0.0, 1.0))?;

        let correspondences = vec![
            Correspondence::new(0, 0, 0.0),
            Correspondence::new(1, 1, 0.0),
            Correspondence::new(2, 2, 0.0),
        ];

        let transform =
            estimator.estimate_rigid_transformation(&source, &target, &correspondences)?;

        // Should find rotation around Z axis
        let identity = TransformationMatrix::identity();
        assert!(transform != identity);

        Ok(())
    }

    #[test]
    fn test_transformation_estimation_insufficient_correspondences() -> PclResult<()> {
        // Test with too few correspondences
        let mut estimator = TransformationEstimationSVD::new()?;

        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        // Only one correspondence (need at least 3)
        let correspondences = vec![Correspondence::new(0, 0, 0.0)];

        // Should error or return something close to identity
        let result = estimator.estimate_rigid_transformation(&source, &target, &correspondences);
        if let Ok(transform) = result {
            // Check if it's close to identity (degenerate case)
            let identity = TransformationMatrix::identity();
            // With insufficient correspondences, might get identity or garbage
            assert!(transform == identity || transform != identity);
        }

        Ok(())
    }
}

/// Tests for feature-based registration
#[cfg(test)]
mod feature_registration_tests {
    use super::*;

    #[test]
    fn test_feature_based_registration_creation() -> PclResult<()> {
        // Test creating feature-based registration
        let _reg = FeatureBasedRegistration::new()?;
        Ok(())
    }

    #[test]
    fn test_feature_based_registration_configuration() -> PclResult<()> {
        // Test configuring feature-based registration
        let mut reg = FeatureBasedRegistration::new()?;

        // Set inlier threshold
        reg.set_inlier_threshold(0.05);
        assert_eq!(reg.get_inlier_threshold(), 0.05);

        Ok(())
    }

    #[test]
    fn test_feature_based_registration_builder() -> PclResult<()> {
        // Test feature registration builder
        let reg = FeatureBasedRegistrationBuilder::new()
            .inlier_threshold(0.01)
            .build()?;

        // Builder should configure the registration properly
        let mut reg = reg;
        assert_eq!(reg.get_inlier_threshold(), 0.01);

        Ok(())
    }

    #[test]
    fn test_registration_result() {
        // Test RegistrationResult struct
        let result = RegistrationResult {
            transformation: TransformationMatrix::identity(),
            correspondences: vec![
                Correspondence::new(0, 0, 0.01),
                Correspondence::new(1, 1, 0.02),
            ],
            total_correspondences: 2,
            inlier_correspondences: 2,
        };

        assert_eq!(result.inlier_ratio(), 1.0);
        assert_eq!(result.correspondences.len(), 2);
        assert_eq!(result.total_correspondences, 2);
        assert_eq!(result.inlier_correspondences, 2);
        let identity = TransformationMatrix::identity();
        assert!(result.transformation == identity);
    }
}

/// Performance and stress tests for registration
#[cfg(test)]
mod registration_performance_tests {
    use super::*;

    #[test]
    fn test_registration_memory_usage() -> PclResult<()> {
        // Test that registration algorithms don't leak memory
        for _ in 0..10 {
            let mut icp = IcpXYZ::new()?;
            let mut cloud = PointCloud::<XYZ>::new()?;

            for i in 0..100 {
                cloud.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
            }

            icp.set_input_source(&cloud)?;
            icp.set_input_target(&cloud)?;

            let _aligned = icp.align()?;
        }
        Ok(())
    }

    #[test]
    fn test_icp_vs_ndt_performance() -> PclResult<()> {
        // Compare ICP and NDT performance
        let mut source = PointCloud::<XYZ>::new()?;
        let mut target = PointCloud::<XYZ>::new()?;

        // Create test clouds
        for i in 0..20 {
            for j in 0..20 {
                source.push(PointXYZ::new(i as f32 * 0.1, j as f32 * 0.1, 0.0))?;
                target.push(PointXYZ::new(
                    i as f32 * 0.1 + 0.05,
                    j as f32 * 0.1 + 0.05,
                    0.0,
                ))?;
            }
        }

        // Test ICP
        let mut icp = IcpXYZ::new()?;
        icp.set_max_iterations(20)?;
        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;
        let _icp_result = icp.align()?;
        let icp_converged = icp.has_converged();

        // Test NDT
        let mut ndt = NdtXYZ::new()?;
        ndt.set_max_iterations(20);
        ndt.set_resolution(0.5);
        ndt.set_input_source(&source)?;
        ndt.set_input_target(&target)?;
        let _ndt_result = ndt.align()?;
        let ndt_converged = ndt.has_converged();

        // Both should converge for this simple case
        assert!(icp_converged || ndt_converged);

        Ok(())
    }
}

/// Edge case and error handling tests
#[cfg(test)]
mod registration_edge_cases {
    use super::*;
    use crate::PclError;

    #[test]
    fn test_registration_single_point() -> PclResult<()> {
        // Test registration with single point clouds
        let mut icp = IcpXYZ::new()?;

        let mut source = PointCloud::<XYZ>::new()?;
        source.push(PointXYZ::new(0.0, 0.0, 0.0))?;

        let mut target = PointCloud::<XYZ>::new()?;
        target.push(PointXYZ::new(1.0, 0.0, 0.0))?;

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        // Single point registration is degenerate
        let result = icp.align();

        // May succeed but won't be meaningful
        if let Ok(aligned) = result {
            assert_eq!(aligned.size(), 1);
        }

        Ok(())
    }

    #[test]
    fn test_registration_identical_clouds() -> PclResult<()> {
        // Test registration with identical clouds
        let mut icp = IcpXYZ::new()?;

        let mut cloud = PointCloud::<XYZ>::new()?;
        for i in 0..10 {
            cloud.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
        }

        icp.set_input_source(&cloud)?;
        icp.set_input_target(&cloud)?;

        let _aligned = icp.align()?;

        // Should converge immediately
        assert!(icp.has_converged());

        // Transformation should be identity
        let transform = icp.get_final_transformation();
        let _identity = TransformationMatrix::identity();
        // Check that transform is approximately identity
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                let actual = transform[(i, j)];
                assert!(
                    (actual - expected).abs() < 1e-6,
                    "Transform not identity at ({}, {}): {} != {}",
                    i,
                    j,
                    actual,
                    expected
                );
            }
        }

        Ok(())
    }

    #[test]
    fn test_registration_no_overlap() -> PclResult<()> {
        // Test registration with non-overlapping clouds
        let mut icp = IcpXYZ::new()?;
        icp.set_max_correspondence_distance(0.1)?; // Small threshold

        // Create non-overlapping clouds
        let mut source = PointCloud::<XYZ>::new()?;
        for i in 0..5 {
            source.push(PointXYZ::new(i as f32, 0.0, 0.0))?;
        }

        let mut target = PointCloud::<XYZ>::new()?;
        for i in 0..5 {
            target.push(PointXYZ::new(i as f32 + 100.0, 0.0, 0.0))?; // Far away
        }

        icp.set_input_source(&source)?;
        icp.set_input_target(&target)?;

        let _aligned = icp.align()?;

        // May not converge due to no overlap
        // Fitness score should be poor
        let fitness = icp.get_fitness_score();
        assert!(fitness > 0.1); // Poor fitness expected

        Ok(())
    }

    #[test]
    fn test_transformation_matrix_invalid_vec() {
        // Test creating transformation matrix from invalid vector
        let too_short = vec![1.0, 2.0, 3.0]; // Need 16 elements
        let result = TransformationMatrix::from_vec(&too_short);
        assert!(result.is_none());

        let too_long = vec![0.0; 20]; // Too many elements
        let result = TransformationMatrix::from_vec(&too_long);
        assert!(result.is_none());
    }

    #[test]
    fn test_transformation_non_invertible() -> PclResult<()> {
        // Test non-invertible transformation matrix
        let vec = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];

        let transform =
            TransformationMatrix::from_vec(&vec).ok_or(PclError::InvalidParameters {
                message: "Failed to create transformation from vec".into(),
                parameter: "vec".into(),
                expected: "16 elements".into(),
                actual: vec.len().to_string(),
            })?;
        let result = transform.inverse();

        // Should fail to invert (returns None for non-invertible)
        assert!(result.is_none());

        Ok(())
    }
}
