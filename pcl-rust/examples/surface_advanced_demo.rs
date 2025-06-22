//! Advanced surface reconstruction algorithms demonstration
//!
//! This example shows how to use the surface reconstruction algorithms that
//! require normals: Poisson reconstruction, Greedy Projection Triangulation,
//! and Moving Least Squares smoothing.

use pcl::common::{PointCloud, PointCloudXYZBuilder, XYZ};

#[cfg(feature = "surface")]
use pcl::surface::{
    GreedyProjectionTriangulationBuilder, MovingLeastSquaresBuilder, OrganizedFastMeshXYZ,
    PoissonReconstructionBuilder, TriangulationType, UpsampleMethod,
};

fn main() -> pcl::PclResult<()> {
    println!("Advanced Surface Reconstruction Demo");
    println!("====================================\n");

    // Create a sample point cloud
    let cloud = create_sample_cloud()?;
    println!("Created sample cloud with {} points", cloud.size());

    #[cfg(feature = "surface")]
    {
        // Note: In a real application, you would first estimate normals using
        // the features module, then use those normals for surface reconstruction
        println!("\nNote: This demo shows the API structure for surface algorithms");
        println!(
            "In practice, you would first estimate normals using normal estimation algorithms."
        );

        // Demonstrate Moving Least Squares smoothing
        moving_least_squares_demo(&cloud)?;

        // Demonstrate surface reconstruction algorithms (API structure only)
        poisson_reconstruction_demo()?;
        greedy_triangulation_demo()?;

        // Demonstrate OrganizedFastMesh (works with PointXYZ)
        organized_fast_mesh_demo(&cloud)?;

        println!("\nAdvanced surface reconstruction demo completed!");
        println!(
            "Note: Full functionality requires point clouds with normals from normal estimation."
        );
    }

    #[cfg(not(feature = "surface"))]
    {
        println!("Surface feature not enabled. Run with --features surface to see the full demo.");
    }

    Ok(())
}

/// Create a sample point cloud with some structure
fn create_sample_cloud() -> pcl::PclResult<PointCloud<XYZ>> {
    let mut points = Vec::new();

    // Create a sphere-like structure
    let radius = 1.0;
    let num_points = 500;

    for i in 0..num_points {
        let theta = (i as f32) * 2.0 * std::f32::consts::PI / (num_points as f32);
        let phi = (i as f32) * std::f32::consts::PI / (num_points as f32);

        let x = radius * phi.sin() * theta.cos();
        let y = radius * phi.sin() * theta.sin();
        let z = radius * phi.cos();

        points.push((x, y, z));
    }

    PointCloudXYZBuilder::new().add_points(points).build()
}

/// Demonstrate Moving Least Squares smoothing
#[cfg(feature = "surface")]
fn moving_least_squares_demo(_cloud: &PointCloud<XYZ>) -> pcl::PclResult<()> {
    println!("\n1. Moving Least Squares Smoothing");
    println!("---------------------------------");

    // Create MLS with builder pattern
    let mls = MovingLeastSquaresBuilder::new()
        .search_radius(0.1)
        .polynomial_order(2)
        .compute_normals(true)
        .upsample_method(UpsampleMethod::SampleLocalPlane)
        .upsampling_radius(0.05)
        .upsampling_step_size(0.02)
        .build()?;

    println!("✓ MovingLeastSquares configured:");
    println!("  Search radius: {}", mls.search_radius());
    println!("  Polynomial order: {}", mls.polynomial_order());
    println!("  Upsampling radius: {}", mls.upsampling_radius());
    println!("  Upsampling step size: {}", mls.upsampling_step_size());

    // In practice, you would:
    // let mut mls = mls;
    // mls.set_input_cloud(cloud)?;
    // let smoothed_cloud: PointCloudNormal = mls.process()?;
    println!("  Note: Process would return PointCloudNormal with computed normals");

    Ok(())
}

/// Demonstrate Poisson surface reconstruction API
#[cfg(feature = "surface")]
fn poisson_reconstruction_demo() -> pcl::PclResult<()> {
    println!("\n2. Poisson Surface Reconstruction");
    println!("----------------------------------");

    // Create Poisson reconstruction with builder pattern
    let mut poisson = PoissonReconstructionBuilder::new()
        .depth(8)
        .min_depth(2)
        .point_weight(4.0)
        .scale(1.1)
        .confidence(true)
        .manifold(false)
        .output_polygons(false)
        .build()?;

    println!("✓ PoissonReconstruction configured:");
    println!("  Depth: {}", poisson.depth());
    println!("  Min depth: {}", poisson.min_depth());
    println!("  Point weight: {}", poisson.point_weight());
    println!("  Scale: {}", poisson.scale());
    println!("  Confidence: {}", poisson.confidence());
    println!("  Manifold: {}", poisson.manifold());

    // In practice, you would:
    // let mut mesh = PolygonMesh::new()?;
    // TODO: PointCloudNormal is not yet implemented - requires PointNormal type with proper Send + Sync traits
    // poisson.set_input_cloud(&point_cloud_with_normals)?;
    // poisson.reconstruct(&mut mesh)?;
    println!("  Note: Requires PointCloudNormal input (coordinates + normals)");

    Ok(())
}

/// Demonstrate Greedy Projection Triangulation API
#[cfg(feature = "surface")]
fn greedy_triangulation_demo() -> pcl::PclResult<()> {
    println!("\n3. Greedy Projection Triangulation");
    println!("-----------------------------------");

    // Create Greedy Projection with builder pattern
    let mut gp3 = GreedyProjectionTriangulationBuilder::new()
        .mu(2.5)
        .search_radius(0.025)
        .minimum_angle(std::f64::consts::PI / 18.0) // 10 degrees
        .maximum_angle(2.0 * std::f64::consts::PI / 3.0) // 120 degrees
        .maximum_nearest_neighbors(100)
        .maximum_surface_angle(std::f64::consts::PI / 4.0) // 45 degrees
        .normal_consistency(false)
        .consistent_vertex_ordering(true)
        .build()?;

    println!("✓ GreedyProjectionTriangulation configured:");
    println!("  Mu: {}", gp3.mu());
    println!("  Search radius: {}", gp3.search_radius());
    println!(
        "  Maximum nearest neighbors: {}",
        gp3.maximum_nearest_neighbors()
    );
    println!("  Normal consistency: {}", gp3.normal_consistency());
    println!(
        "  Consistent vertex ordering: {}",
        gp3.consistent_vertex_ordering()
    );

    // In practice, you would:
    // let mut mesh = PolygonMesh::new()?;
    // TODO: PointCloudNormal is not yet implemented - requires PointNormal type with proper Send + Sync traits
    // gp3.set_input_cloud(&point_cloud_with_normals)?;
    // gp3.reconstruct(&mut mesh)?;
    println!("  Note: Requires PointCloudNormal input (coordinates + normals)");

    Ok(())
}

/// Demonstrate OrganizedFastMesh (works with PointXYZ)
#[cfg(feature = "surface")]
fn organized_fast_mesh_demo(_cloud: &PointCloud<XYZ>) -> pcl::PclResult<()> {
    println!("\n4. Organized Fast Mesh (PointXYZ compatible)");
    println!("---------------------------------------------");

    // Create and configure OrganizedFastMesh algorithm
    let mut organized_mesh = OrganizedFastMeshXYZ::new()?;
    organized_mesh.set_triangle_pixel_size(1)?;
    organized_mesh.set_triangulation_type(TriangulationType::TriangleMesh)?;

    println!("✓ OrganizedFastMesh configured:");
    println!(
        "  Triangle pixel size: {}",
        organized_mesh.triangle_pixel_size()
    );
    println!(
        "  Triangulation type: {:?}",
        organized_mesh.triangulation_type()
    );

    // In practice, for organized clouds (from depth sensors):
    // organized_mesh.set_input_cloud(cloud)?;
    // let mut mesh = PolygonMesh::new()?;
    // organized_mesh.reconstruct(&mut mesh)?;
    println!("  Note: Works with organized PointXYZ clouds (width > 1, height > 1)");

    Ok(())
}
