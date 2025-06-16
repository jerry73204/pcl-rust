//! Demonstration of point cloud segmentation algorithms
//!
//! This example shows how to use various segmentation algorithms to:
//! - Extract clusters using Euclidean clustering
//! - Find planes using RANSAC
//! - Perform region growing segmentation
//! - Extract ground points using progressive morphological filtering

use pcl::{
    ModelType, PointCloudXYZ, ProgressiveMorphologicalFilterXYZ, SacSegmentationXYZ,
    common::PointCloudXYZBuilder,
};
use rand::Rng;

fn main() -> pcl::PclResult<()> {
    println!("Point Cloud Segmentation Demo");
    println!("=============================\n");

    // Create a sample point cloud with multiple objects
    let cloud = create_sample_cloud()?;
    println!("Created sample cloud with {} points", cloud.size());

    // Demonstrate Euclidean clustering
    euclidean_clustering_demo(&cloud)?;

    // Demonstrate RANSAC plane segmentation
    ransac_plane_demo(&cloud)?;

    // Demonstrate progressive morphological filtering for ground extraction
    ground_extraction_demo(&cloud)?;

    Ok(())
}

/// Create a sample point cloud with some structure
fn create_sample_cloud() -> pcl::PclResult<PointCloudXYZ> {
    let mut rng = rand::thread_rng();
    let mut points = Vec::new();

    // Create a ground plane (z = 0)
    for x in -10..=10 {
        for y in -10..=10 {
            points.push((x as f32 * 0.5, y as f32 * 0.5, 0.0));
        }
    }

    // Create a vertical wall (x = 5)
    for y in -5..=5 {
        for z in 0..10 {
            points.push((5.0, y as f32 * 0.5, z as f32 * 0.5));
        }
    }

    // Create some clusters of points (objects)
    // Cluster 1: around (-2, -2, 1)
    for _ in 0..50 {
        let dx = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        let dy = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        let dz = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        points.push((-2.0 + dx, -2.0 + dy, 1.0 + dz));
    }

    // Cluster 2: around (2, 2, 1.5)
    for _ in 0..50 {
        let dx = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        let dy = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        let dz = (rng.gen_range(0.0..1.0) - 0.5) * 0.5;
        points.push((2.0 + dx, 2.0 + dy, 1.5 + dz));
    }

    // Add some noise
    for _ in 0..20 {
        let x = (rng.gen_range(0.0..1.0) - 0.5) * 10.0;
        let y = (rng.gen_range(0.0..1.0) - 0.5) * 10.0;
        let z = rng.gen_range(0.0..5.0);
        points.push((x, y, z));
    }

    PointCloudXYZBuilder::new().add_points(points).build()
}

/// Demonstrate Euclidean clustering
fn euclidean_clustering_demo(cloud: &PointCloudXYZ) -> pcl::PclResult<()> {
    println!("\n1. Euclidean Clustering");
    println!("----------------------");

    // Create and configure the clustering algorithm using builder
    let mut clustering = pcl::segmentation::clustering::EuclideanClusterExtractionBuilder::new()
        .cluster_tolerance(0.5) // 50cm tolerance
        .min_cluster_size(10) // Minimum 10 points per cluster
        .max_cluster_size(1000) // Maximum 1000 points per cluster
        .build()?;

    clustering.set_input_cloud(cloud)?;

    // Extract clusters
    let clusters = clustering.extract()?;
    println!("Found {} clusters", clusters.len());

    for (i, cluster) in clusters.iter().enumerate() {
        println!("  Cluster {}: {} points", i + 1, cluster.len());
    }

    Ok(())
}

/// Demonstrate RANSAC plane segmentation
fn ransac_plane_demo(cloud: &PointCloudXYZ) -> pcl::PclResult<()> {
    println!("\n2. RANSAC Plane Segmentation");
    println!("---------------------------");

    // Create and configure SAC segmentation
    let mut segmentation = SacSegmentationXYZ::new()?;
    segmentation.set_input_cloud(cloud)?;
    segmentation.set_model_type(ModelType::Plane)?;
    segmentation.set_method_type(pcl::MethodType::Ransac)?;
    segmentation.set_distance_threshold(0.1)?; // 10cm threshold
    segmentation.set_max_iterations(100)?;
    segmentation.set_optimize_coefficients(true)?;

    // Segment the largest plane
    let result = segmentation.segment_model()?;
    println!("Found plane with {} inliers", result.inliers.len());
    println!(
        "Plane equation: {}x + {}y + {}z + {} = 0",
        result.coefficients.get(0).unwrap_or(&0.0),
        result.coefficients.get(1).unwrap_or(&0.0),
        result.coefficients.get(2).unwrap_or(&0.0),
        result.coefficients.get(3).unwrap_or(&0.0)
    );

    Ok(())
}

/// Demonstrate ground extraction using progressive morphological filter
fn ground_extraction_demo(cloud: &PointCloudXYZ) -> pcl::PclResult<()> {
    println!("\n3. Progressive Morphological Filter (Ground Extraction)");
    println!("-----------------------------------------------------");

    // Create and configure the filter
    let mut pmf = ProgressiveMorphologicalFilterXYZ::new()?;
    pmf.set_input_cloud(cloud)?;
    pmf.set_max_window_size(20)?;
    pmf.set_slope(1.0)?;
    pmf.set_initial_distance(0.5)?;
    pmf.set_max_distance(3.0)?;
    pmf.set_cell_size(1.0)?;
    pmf.set_base(2.0)?;
    pmf.set_exponential(true);

    // Extract ground points
    let ground_indices = pmf.extract_ground()?;
    println!("Found {} ground points", ground_indices.len());

    let ground_percentage = (ground_indices.len() as f32 / cloud.size() as f32) * 100.0;
    println!(
        "Ground points represent {:.1}% of the cloud",
        ground_percentage
    );

    Ok(())
}
