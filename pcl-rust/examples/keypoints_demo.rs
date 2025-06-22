//! Keypoint detection example demonstrating Harris 3D, ISS 3D, and SIFT algorithms
//!
//! This example shows how to use different keypoint detection algorithms to find
//! distinctive points in point clouds.

use pcl::{
    PclResult, PointCloud, PointXYZ, PointXYZI, XYZ, XYZI,
    keypoints::{Harris3D, Iss3D, KeypointDetector, SiftKeypoint},
    search::KdTree,
};

/// Create a sample point cloud with some distinctive features
fn create_sample_cloud() -> PclResult<PointCloud<XYZ>> {
    let mut cloud = PointCloud::new()?;

    // Create a cube with corners (these should be detected as keypoints)
    let cube_size = 1.0;
    let points_per_edge = 10;

    // Generate points along the edges of a cube
    for i in 0..points_per_edge {
        let t = i as f32 / (points_per_edge - 1) as f32;

        // Bottom square edges
        cloud.push(PointXYZ::new(t * cube_size, 0.0, 0.0))?;
        cloud.push(PointXYZ::new(0.0, t * cube_size, 0.0))?;
        cloud.push(PointXYZ::new(cube_size, t * cube_size, 0.0))?;
        cloud.push(PointXYZ::new(t * cube_size, cube_size, 0.0))?;

        // Top square edges
        cloud.push(PointXYZ::new(t * cube_size, 0.0, cube_size))?;
        cloud.push(PointXYZ::new(0.0, t * cube_size, cube_size))?;
        cloud.push(PointXYZ::new(cube_size, t * cube_size, cube_size))?;
        cloud.push(PointXYZ::new(t * cube_size, cube_size, cube_size))?;

        // Vertical edges
        cloud.push(PointXYZ::new(0.0, 0.0, t * cube_size))?;
        cloud.push(PointXYZ::new(cube_size, 0.0, t * cube_size))?;
        cloud.push(PointXYZ::new(0.0, cube_size, t * cube_size))?;
        cloud.push(PointXYZ::new(cube_size, cube_size, t * cube_size))?;
    }

    // Add some planar surfaces with noise
    for i in 0..20 {
        for j in 0..20 {
            let x = 2.0 + i as f32 * 0.05;
            let y = j as f32 * 0.05;
            let z = 0.5 + (rand::random::<f32>() - 0.5) * 0.01; // Small noise
            cloud.push(PointXYZ::new(x, y, z))?;
        }
    }

    // Add a sphere-like structure
    let sphere_center = (3.0, 2.0, 1.0);
    let sphere_radius = 0.5;
    for _ in 0..200 {
        let theta = rand::random::<f32>() * 2.0 * std::f32::consts::PI;
        let phi = rand::random::<f32>() * std::f32::consts::PI;

        let x = sphere_center.0 + sphere_radius * phi.sin() * theta.cos();
        let y = sphere_center.1 + sphere_radius * phi.sin() * theta.sin();
        let z = sphere_center.2 + sphere_radius * phi.cos();

        cloud.push(PointXYZ::new(x, y, z))?;
    }

    Ok(cloud)
}

/// Create a sample point cloud with intensity for SIFT
fn create_sample_cloud_with_intensity() -> PclResult<PointCloud<XYZI>> {
    let mut cloud = PointCloud::new()?;

    // Create a simple scene with varying intensity
    for i in 0..50 {
        for j in 0..50 {
            let x = i as f32 * 0.02;
            let y = j as f32 * 0.02;
            let z = 0.1 * ((x * 5.0).sin() + (y * 5.0).cos()); // Wavy surface

            // Intensity based on position - creates a gradient
            let intensity = ((x + y) * 50.0) as u8;

            cloud.push(PointXYZI::new(x, y, z, intensity as f32))?;
        }
    }

    // Add some high-intensity features
    for i in 0..5 {
        let angle = i as f32 * 2.0 * std::f32::consts::PI / 5.0;
        let x = 0.5 + 0.3 * angle.cos();
        let y = 0.5 + 0.3 * angle.sin();
        let z = 0.2;

        // High intensity points that should be detected
        cloud.push(PointXYZI::new(x, y, z, 255.0))?;
    }

    Ok(cloud)
}

/// Demonstrate Harris 3D keypoint detection
fn demo_harris3d(cloud: &PointCloud<XYZ>) -> PclResult<()> {
    println!("\n=== Harris 3D Keypoint Detection ===");

    // Create and configure Harris detector
    let mut harris = Harris3D::new()?;

    // Set up search method
    let mut kdtree = KdTree::<XYZ>::new()?;
    kdtree.set_input_cloud(cloud)?;
    harris.set_search_method(&kdtree)?;

    // Configure parameters
    harris.set_radius(0.1)?; // Search radius for computing Harris response
    harris.set_threshold(0.01)?; // Minimum Harris response for keypoint
    harris.set_non_max_suppression(true)?; // Enable non-maximum suppression
    harris.set_refine(false)?; // Disable refinement for speed

    // Set input cloud and compute keypoints
    harris.set_input_cloud(cloud)?;
    let keypoints = harris.compute()?;

    println!("Input cloud size: {} points", cloud.size());
    println!("Detected {} Harris keypoints", keypoints.size());

    // The keypoints are returned as PointXYZI where intensity is the Harris response
    if keypoints.size() > 0 {
        println!("\nDetected keypoints with Harris response values");
        // Note: Individual point access requires proper FFI implementation
    }

    Ok(())
}

/// Demonstrate ISS 3D keypoint detection
fn demo_iss3d(cloud: &PointCloud<XYZ>) -> PclResult<()> {
    println!("\n=== ISS 3D Keypoint Detection ===");

    // Create and configure ISS detector
    let mut iss = Iss3D::new()?;

    // Set up search method
    let mut kdtree = KdTree::<XYZ>::new()?;
    kdtree.set_input_cloud(cloud)?;
    iss.set_search_method(&kdtree)?;

    // Configure parameters
    iss.set_salient_radius(0.1)?; // Radius for computing eigenvalues
    iss.set_non_max_radius(0.05)?; // Non-maximum suppression radius
    iss.set_threshold21(0.975)?; // Ratio threshold λ2/λ1
    iss.set_threshold32(0.975)?; // Ratio threshold λ3/λ2
    iss.set_min_neighbors(5)?; // Minimum neighbors for stability

    // Set input cloud and compute keypoints
    iss.set_input_cloud(cloud)?;
    let keypoints = iss.compute()?;

    println!("Input cloud size: {} points", cloud.size());
    println!("Detected {} ISS keypoints", keypoints.size());

    if keypoints.size() > 0 {
        println!("\nDetected keypoints based on eigenvalue analysis");
        // Note: Individual point access requires proper FFI implementation
    }

    Ok(())
}

/// Demonstrate SIFT keypoint detection
fn demo_sift(cloud: &PointCloud<XYZI>) -> PclResult<()> {
    println!("\n=== SIFT Keypoint Detection ===");

    // Create and configure SIFT detector
    let mut sift = SiftKeypoint::new()?;

    // Set up search method with XYZI cloud
    let mut kdtree = KdTree::<XYZI>::new()?;
    kdtree.set_input_cloud(cloud)?;
    sift.set_search_method(&kdtree)?;

    // Configure scale parameters
    sift.set_scales(
        0.005, // Minimum scale
        6.0,   // Number of octaves
        4,     // Scales per octave
    )?;
    sift.set_minimum_contrast(0.05)?; // Minimum contrast for keypoint

    // Set input cloud and compute keypoints
    sift.set_input_cloud(cloud)?;
    let keypoints = sift.compute()?;

    println!("Input cloud size: {} points", cloud.size());
    println!("Detected SIFT keypoints with scale information");

    // Note: The keypoints include scale information
    // This would need proper FFI support to access individual points

    Ok(())
}

/// Demonstrate builder pattern for keypoint detectors
fn demo_builders() -> PclResult<()> {
    println!("\n=== Using Builder Pattern ===");

    use pcl::keypoints::{Harris3DBuilder, Iss3DBuilder, KeypointBuilder, SiftKeypointBuilder};

    // Build a Harris detector with custom configuration
    let harris = Harris3DBuilder::new()
        .radius(0.15)
        .threshold(0.005)
        .non_max_suppression(true)
        .refine(true)
        .build()?;
    println!("Created Harris detector with builder");

    // Build an ISS detector
    let iss = Iss3DBuilder::new()
        .salient_radius(0.1)
        .non_max_radius(0.05)
        .threshold21(0.95)
        .threshold32(0.95)
        .min_neighbors(10)
        .build()?;
    println!("Created ISS detector with builder");

    // Build a SIFT detector
    let sift = SiftKeypointBuilder::new()
        .scales(0.01, 4.0, 3)
        .minimum_contrast(0.03)
        .build()?;
    println!("Created SIFT detector with builder");

    Ok(())
}

fn main() -> PclResult<()> {
    println!("PCL Keypoint Detection Demo");
    println!("===========================");

    // Create sample point clouds
    let xyz_cloud = create_sample_cloud()?;
    let xyzi_cloud = create_sample_cloud_with_intensity()?;

    // Demonstrate different keypoint detectors
    demo_harris3d(&xyz_cloud)?;
    demo_iss3d(&xyz_cloud)?;
    demo_sift(&xyzi_cloud)?;
    demo_builders()?;

    println!("\nKeypoint detection demo completed!");

    Ok(())
}

// Add rand as a dev dependency for this example
// In Cargo.toml:
// [dev-dependencies]
// rand = "0.8"
