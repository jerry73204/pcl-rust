//! Demonstration of surface reconstruction algorithms
//!
//! This example shows how to use various surface reconstruction algorithms to:
//! - Create meshes from point clouds using Marching Cubes
//! - Generate organized meshes using OrganizedFastMesh
//! - Save meshes to various file formats

use pcl::{
    MarchingCubesHoppeXYZ, OrganizedFastMeshXYZ, PolygonMesh, SurfaceReconstruction,
    TriangulationType, common::PointCloudXYZBuilder,
};

fn main() -> pcl::PclResult<()> {
    println!("Surface Reconstruction Demo");
    println!("==========================\n");

    // Create a sample point cloud
    let cloud = create_sample_cloud()?;
    println!("Created sample cloud with {} points", cloud.size());

    // Demonstrate Marching Cubes surface reconstruction
    marching_cubes_demo(&cloud)?;

    // Create an organized point cloud for OrganizedFastMesh demo
    let organized_cloud = create_organized_cloud()?;
    println!(
        "Created organized cloud with {} points ({}x{})",
        organized_cloud.size(),
        organized_cloud.width(),
        organized_cloud.height()
    );

    // Demonstrate OrganizedFastMesh
    organized_fast_mesh_demo(&organized_cloud)?;

    println!("\nSurface reconstruction demo completed successfully!");

    Ok(())
}

/// Create a sample point cloud with some structure
fn create_sample_cloud() -> pcl::PclResult<pcl::PointCloudXYZ> {
    let mut points = Vec::new();

    // Create a simple 3D structure - a cube
    let resolution = 10;
    let step = 0.1;

    // Bottom face
    for i in 0..=resolution {
        for j in 0..=resolution {
            let x = i as f32 * step;
            let y = j as f32 * step;
            let z = 0.0;
            points.push((x, y, z));
        }
    }

    // Top face
    for i in 0..=resolution {
        for j in 0..=resolution {
            let x = i as f32 * step;
            let y = j as f32 * step;
            let z = 1.0;
            points.push((x, y, z));
        }
    }

    // Side faces (simplified)
    for i in 0..=resolution {
        for k in 1..resolution {
            let x = i as f32 * step;
            let z = k as f32 * step;
            // Left side (y = 0)
            points.push((x, 0.0, z));
            // Right side (y = 1)
            points.push((x, 1.0, z));
            // Front side (x = 0)
            points.push((0.0, x, z));
            // Back side (x = 1)
            points.push((1.0, x, z));
        }
    }

    PointCloudXYZBuilder::new().add_points(points).build()
}

/// Create an organized point cloud (like from a depth sensor)
fn create_organized_cloud() -> pcl::PclResult<pcl::PointCloudXYZ> {
    let width = 20;
    let height = 15;
    let mut points = Vec::new();

    // Create a simple organized point cloud representing a curved surface
    for row in 0..height {
        for col in 0..width {
            let x = col as f32 * 0.05; // 5cm spacing
            let y = row as f32 * 0.05;
            // Create a curved surface using a simple function
            let z = 0.1 * ((x * 2.0).sin() + (y * 2.0).cos());
            points.push((x, y, z));
        }
    }

    // Create organized cloud by setting width and height explicitly
    let mut cloud = PointCloudXYZBuilder::new().add_points(points).build()?;

    // For now, we can't set width/height directly through the builder
    // This would need to be implemented when we have proper organized cloud support
    // cloud.set_width_height(width, height)?;

    Ok(cloud)
}

/// Demonstrate Marching Cubes surface reconstruction
fn marching_cubes_demo(cloud: &pcl::PointCloudXYZ) -> pcl::PclResult<()> {
    println!("\n1. Marching Cubes Hoppe Reconstruction");
    println!("-------------------------------------");

    // Create and configure Marching Cubes Hoppe algorithm
    let mut marching_cubes = MarchingCubesHoppeXYZ::new()?;
    marching_cubes.set_iso_level(0.0)?;
    marching_cubes.set_grid_resolution(50, 50, 50)?;
    marching_cubes.set_percentage_extend_grid(0.1)?;

    // Set input cloud
    marching_cubes.set_input_cloud(cloud)?;

    // Create output mesh
    let mut mesh = PolygonMesh::new()?;

    // Perform reconstruction
    marching_cubes.reconstruct(&mut mesh)?;

    println!("Reconstruction completed!");
    println!("  Vertices: {}", mesh.vertex_count());
    println!("  Polygons: {}", mesh.polygon_count());
    println!("  Valid mesh: {}", mesh.is_valid());

    // Save mesh to file (commented out to avoid file I/O in demo)
    // mesh.save("marching_cubes_result.ply")?;
    // println!("  Saved to: marching_cubes_result.ply");

    Ok(())
}

/// Demonstrate OrganizedFastMesh surface reconstruction
fn organized_fast_mesh_demo(cloud: &pcl::PointCloudXYZ) -> pcl::PclResult<()> {
    println!("\n2. Organized Fast Mesh Reconstruction");
    println!("------------------------------------");

    // Create and configure OrganizedFastMesh algorithm
    let mut organized_mesh = OrganizedFastMeshXYZ::new()?;
    organized_mesh.set_triangle_pixel_size(1)?;
    organized_mesh.set_triangulation_type(TriangulationType::TriangleMesh)?;

    // For this demo, we'll skip the actual reconstruction since we need
    // a properly organized point cloud (with width > 1 and height > 1)
    println!("OrganizedFastMesh configured successfully!");
    println!(
        "  Triangle pixel size: {}",
        organized_mesh.triangle_pixel_size()
    );
    println!(
        "  Triangulation type: {:?}",
        organized_mesh.triangulation_type()
    );

    // Note: Actual reconstruction would require:
    // organized_mesh.set_input_cloud(cloud)?;
    // let mut mesh = PolygonMesh::new()?;
    // organized_mesh.reconstruct(&mut mesh)?;

    println!("  Note: Skipped reconstruction - requires properly organized cloud");

    Ok(())
}
