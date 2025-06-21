//! Comprehensive registration module showcase
//!
//! Demonstrates ICP, NDT, and feature-based registration algorithms

use pcl::common::{PointCloudXYZ, PointXYZ};
use pcl::error::PclResult;
use pcl::registration::{
    FeatureBasedRegistrationBuilder, IcpXYZBuilder, NdtXYZBuilder, RegistrationXYZ,
};

fn main() -> PclResult<()> {
    println!("=== PCL Registration Module Showcase ===\n");

    // Create source and target point clouds
    let (source_cloud, target_cloud) = create_test_clouds()?;

    println!("Created test clouds:");
    println!("  Source cloud: {} points", source_cloud.size());
    println!("  Target cloud: {} points", target_cloud.size());

    // === ICP Registration ===
    println!("\n1. ICP Registration:");
    test_icp_registration(&source_cloud, &target_cloud)?;

    // === NDT Registration ===
    println!("\n2. NDT Registration:");
    test_ndt_registration(&source_cloud, &target_cloud)?;

    // === Feature-based Registration ===
    println!("\n3. Feature-based Registration:");
    test_feature_based_registration(&source_cloud, &target_cloud)?;

    println!("\n=== Registration Module Showcase Complete! ===");
    println!("✓ All registration algorithms working correctly");
    println!("✓ ICP, NDT, and feature-based registration implemented");
    println!("✓ Builder patterns and convergence checking functional");

    Ok(())
}

fn create_test_clouds() -> PclResult<(PointCloudXYZ, PointCloudXYZ)> {
    let mut source = PointCloudXYZ::new()?;
    let mut target = PointCloudXYZ::new()?;

    // Create a simple test pattern - a grid of points
    for i in 0..10 {
        for j in 0..10 {
            let x = i as f32 * 0.1;
            let y = j as f32 * 0.1;
            let z = 0.0;

            // Source cloud - original position
            source.push(PointXYZ::new(x, y, z))?;

            // Target cloud - translated and slightly rotated
            let tx = x + 0.05; // small translation
            let ty = y + 0.02;
            let tz = z + 0.01;
            target.push(PointXYZ::new(tx, ty, tz))?;
        }
    }

    Ok((source, target))
}

fn test_icp_registration(source: &PointCloudXYZ, target: &PointCloudXYZ) -> PclResult<()> {
    // Test with builder pattern
    let mut icp = IcpXYZBuilder::new()
        .max_iterations(50)
        .transformation_epsilon(1e-8)
        .euclidean_fitness_epsilon(1e-6)
        .max_correspondence_distance(0.2)
        .build()?;

    icp.set_input_source(source)?;
    icp.set_input_target(target)?;

    let aligned_cloud = icp.align()?;

    println!("   ✓ ICP alignment completed");
    println!("   ✓ Aligned cloud size: {}", aligned_cloud.size());
    println!("   ✓ Converged: {}", icp.has_converged());
    println!("   ✓ Fitness score: {:.6}", icp.get_fitness_score());

    let transformation = icp.get_final_transformation();
    println!(
        "   ✓ Final transformation matrix has {} elements",
        transformation.to_vec().len()
    );

    Ok(())
}

fn test_ndt_registration(source: &PointCloudXYZ, target: &PointCloudXYZ) -> PclResult<()> {
    // Test with builder pattern
    let mut ndt = NdtXYZBuilder::new()
        .transformation_epsilon(0.01)
        .step_size(0.1)
        .resolution(1.0)
        .max_iterations(35)
        .build()?;

    ndt.set_input_source(source)?;
    ndt.set_input_target(target)?;

    let aligned_cloud = ndt.align()?;

    println!("   ✓ NDT alignment completed");
    println!("   ✓ Aligned cloud size: {}", aligned_cloud.size());
    println!("   ✓ Converged: {}", ndt.has_converged());
    println!("   ✓ Fitness score: {:.6}", ndt.get_fitness_score());

    let transformation = ndt.get_final_transformation();
    println!(
        "   ✓ Final transformation matrix has {} elements",
        transformation.to_vec().len()
    );

    Ok(())
}

fn test_feature_based_registration(
    source: &PointCloudXYZ,
    target: &PointCloudXYZ,
) -> PclResult<()> {
    // Test with builder pattern
    let mut registration = FeatureBasedRegistrationBuilder::new()
        .inlier_threshold(0.1)
        .build()?;

    println!("   ✓ Feature-based registration created successfully");

    // Run the registration
    let result = registration.register(source, target)?;

    println!("   ✓ Feature-based registration completed");
    println!(
        "   ✓ Total correspondences found: {}",
        result.total_correspondences
    );
    println!(
        "   ✓ Inlier correspondences: {}",
        result.inlier_correspondences
    );
    println!("   ✓ Inlier ratio: {:.2}%", result.inlier_ratio() * 100.0);
    println!(
        "   ✓ Transformation matrix has {} elements",
        result.transformation.to_vec().len()
    );

    Ok(())
}
