//! Point Cloud Segmentation Demo
//!
//! This example demonstrates various segmentation algorithms available in PCL-Rust:
//! - Euclidean cluster extraction for separating objects
//! - Region growing segmentation using surface normals
//! - SAC segmentation for finding geometric models (planes, spheres, etc.)

use pcl::{
    PclResult, PointCloudXYZ,
    common::PointCloudXYZBuilder,
    segmentation::{
        ClusteringXYZ, EuclideanClusterExtractionXYZ, MethodType, ModelType, RegionGrowingXYZ,
        SacSegmentationXYZ, SegmentationXYZ,
    },
};

fn main() -> PclResult<()> {
    println!("Point Cloud Segmentation Demo");
    println!("=============================");

    // Create a sample point cloud with multiple clusters
    let cloud = create_sample_cloud()?;
    println!("Created sample cloud with {} points", cloud.size());

    // Demonstrate Euclidean cluster extraction
    println!("\n1. Euclidean Cluster Extraction");
    println!("--------------------------------");
    euclidean_clustering_demo(&cloud)?;

    // Demonstrate region growing segmentation
    println!("\n2. Region Growing Segmentation");
    println!("------------------------------");
    region_growing_demo(&cloud)?;

    // Demonstrate SAC segmentation for plane detection
    println!("\n3. SAC Segmentation (Plane Detection)");
    println!("--------------------------------------");
    sac_segmentation_demo(&cloud)?;

    println!("\nSegmentation demo completed successfully!");
    Ok(())
}

fn create_sample_cloud() -> PclResult<PointCloudXYZ> {
    let mut builder = PointCloudXYZBuilder::new();

    // Create first cluster (ground plane)
    for i in 0..50 {
        for j in 0..50 {
            let x = (i as f32 - 25.0) * 0.1;
            let y = (j as f32 - 25.0) * 0.1;
            let z = 0.0 + (rand::random::<f32>() - 0.5) * 0.02; // Small noise
            builder = builder.add_point(x, y, z);
        }
    }

    // Create second cluster (elevated box)
    for i in 0..20 {
        for j in 0..20 {
            let x = (i as f32 - 10.0) * 0.05 + 1.0;
            let y = (j as f32 - 10.0) * 0.05;
            let z = 0.5 + (rand::random::<f32>() - 0.5) * 0.02;
            builder = builder.add_point(x, y, z);
        }
    }

    // Create third cluster (cylinder-like)
    for i in 0..100 {
        let angle = (i as f32) * 2.0 * std::f32::consts::PI / 100.0;
        let radius = 0.3;
        let x = radius * angle.cos() - 1.0;
        let y = radius * angle.sin();
        let z = (i as f32) * 0.01 + (rand::random::<f32>() - 0.5) * 0.02;
        builder = builder.add_point(x, y, z);
    }

    builder.build()
}

fn euclidean_clustering_demo(cloud: &PointCloudXYZ) -> PclResult<()> {
    // Create Euclidean cluster extraction with builder pattern
    let mut clustering = EuclideanClusterExtractionXYZ::builder()
        .cluster_tolerance(0.1) // 10cm tolerance
        .min_cluster_size(10)
        .max_cluster_size(5000)
        .build()?;

    // Set input cloud
    clustering.set_input_cloud(cloud)?;

    // Perform clustering
    let clusters = clustering.segment()?;

    println!("Found {} clusters", clusters.len());
    for (i, cluster) in clusters.iter().enumerate() {
        println!("  Cluster {}: {} points", i + 1, cluster.len());
    }

    // Demonstrate configuration access
    println!("Configuration:");
    println!("  Cluster tolerance: {:.3}", clustering.cluster_tolerance());
    println!("  Min cluster size: {}", clustering.min_cluster_size());
    println!("  Max cluster size: {}", clustering.max_cluster_size());

    Ok(())
}

fn region_growing_demo(cloud: &PointCloudXYZ) -> PclResult<()> {
    // Estimate normals for region growing
    println!("Estimating surface normals...");
    let normals = RegionGrowingXYZ::estimate_normals(cloud, 0.05)?;

    // Create region growing segmentation
    let mut region_growing = RegionGrowingXYZ::new()?;

    // Configure the algorithm
    region_growing.set_input_cloud(cloud)?;
    region_growing.set_input_normals(&normals)?;
    region_growing.set_min_cluster_size(10)?;
    region_growing.set_max_cluster_size(5000)?;
    region_growing.set_smoothness_threshold(3.0 / 180.0 * std::f32::consts::PI)?; // 3 degrees
    region_growing.set_curvature_threshold(1.0)?;
    region_growing.set_number_of_neighbours(30)?;

    // Perform segmentation
    let regions = region_growing.segment()?;

    println!("Found {} regions", regions.len());
    for (i, region) in regions.iter().enumerate() {
        println!("  Region {}: {} points", i + 1, region.len());
    }

    // Show configuration
    println!("Configuration:");
    println!("  Min cluster size: {}", region_growing.min_cluster_size());
    println!("  Max cluster size: {}", region_growing.max_cluster_size());
    println!(
        "  Smoothness threshold: {:.3} rad",
        region_growing.smoothness_threshold()
    );
    println!(
        "  Curvature threshold: {:.3}",
        region_growing.curvature_threshold()
    );
    println!(
        "  Number of neighbours: {}",
        region_growing.number_of_neighbours()
    );

    Ok(())
}

fn sac_segmentation_demo(cloud: &PointCloudXYZ) -> PclResult<()> {
    // Create SAC segmentation for plane detection using builder pattern
    let mut sac_seg = SacSegmentationXYZ::builder()
        .model_type(ModelType::Plane)
        .method_type(MethodType::Ransac)
        .distance_threshold(0.02) // 2cm tolerance
        .max_iterations(1000)
        .optimize_coefficients(true)
        .build()?;

    // Set input cloud
    sac_seg.set_input_cloud(cloud)?;

    // Perform segmentation
    let result = match sac_seg.segment_model() {
        Ok(result) => result,
        Err(e) => {
            println!("  Plane detection failed: {}", e);
            println!("  This is normal when no clear plane is found in the data");
            return Ok(());
        }
    };

    println!("Plane detection results:");
    println!("  Found {} inlier points", result.inliers.len());

    if result.coefficients.len() >= 4 {
        println!(
            "  Plane equation: {:.3}x + {:.3}y + {:.3}z + {:.3} = 0",
            result.coefficients[0],
            result.coefficients[1],
            result.coefficients[2],
            result.coefficients[3]
        );
    }

    // Show configuration
    println!("Configuration:");
    println!("  Model type: {:?}", sac_seg.model_type());
    println!("  Method type: {:?}", sac_seg.method_type());
    println!("  Distance threshold: {:.3}", sac_seg.distance_threshold());
    println!("  Max iterations: {}", sac_seg.max_iterations());
    println!(
        "  Optimize coefficients: {}",
        sac_seg.optimize_coefficients()
    );

    // Demonstrate sphere detection
    println!("\nTrying sphere detection...");
    sac_seg.set_model_type(ModelType::Sphere)?;
    sac_seg.set_distance_threshold(0.05)?; // Larger tolerance for sphere

    let sphere_result = sac_seg.segment_model()?;
    println!("Sphere detection results:");
    println!("  Found {} inlier points", sphere_result.inliers.len());

    if sphere_result.coefficients.len() >= 4 {
        println!(
            "  Sphere center: ({:.3}, {:.3}, {:.3}), radius: {:.3}",
            sphere_result.coefficients[0],
            sphere_result.coefficients[1],
            sphere_result.coefficients[2],
            sphere_result.coefficients[3]
        );
    }

    Ok(())
}

// Simple random number generation for demo
mod rand {
    use std::cell::Cell;

    thread_local! {
        static SEED: Cell<u32> = Cell::new(1);
    }

    pub fn random<T>() -> T
    where
        T: From<f32>,
    {
        SEED.with(|seed| {
            let mut s = seed.get();
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            seed.set(s);
            T::from((s as f32) / (u32::MAX as f32))
        })
    }
}
