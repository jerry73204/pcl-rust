//! Example demonstrating the use of PCL filters
//!
//! This example shows how to use various filters including:
//! - VoxelGrid for downsampling
//! - PassThrough for filtering by coordinate ranges
//! - StatisticalOutlierRemoval for removing statistical outliers
//! - RadiusOutlierRemoval for removing isolated points

use pcl::{
    PointCloudXYZ,
    common::PointCloudXYZBuilder,
    filters::{
        FilterXYZ, PassThroughXYZBuilder, RadiusOutlierRemovalXYZBuilder,
        StatisticalOutlierRemovalXYZBuilder, VoxelGridXYZBuilder,
    },
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("PCL Filter Examples");
    println!("==================");

    // Create a sample point cloud with some noise
    let cloud = create_sample_cloud()?;
    println!("\nOriginal cloud size: {} points", cloud.size());

    // Example 1: VoxelGrid Filter
    println!("\n1. VoxelGrid Filter (Downsampling)");
    println!("   Reducing point cloud density by using a 3D voxel grid");

    let mut voxel_filter = VoxelGridXYZBuilder::new()
        .leaf_size(0.1, 0.1, 0.1) // 10cm voxels
        .build()?;

    voxel_filter.set_input_cloud(&cloud)?;
    let voxel_filtered = voxel_filter.filter()?;
    println!("   Filtered cloud size: {} points", voxel_filtered.size());
    println!(
        "   Reduction: {:.1}%",
        (1.0 - voxel_filtered.size() as f32 / cloud.size() as f32) * 100.0
    );

    // Example 2: PassThrough Filter
    println!("\n2. PassThrough Filter (Range filtering)");
    println!("   Keeping only points with Z coordinate between 0.5 and 2.0");

    let mut pass_filter = PassThroughXYZBuilder::new()
        .field_name("z")
        .limits(0.5, 2.0)
        .build()?;

    pass_filter.set_input_cloud(&cloud)?;
    let pass_filtered = pass_filter.filter()?;
    println!("   Filtered cloud size: {} points", pass_filtered.size());

    // Example 3: Statistical Outlier Removal
    println!("\n3. Statistical Outlier Removal");
    println!("   Removing points that are statistical outliers");

    let mut statistical_filter = StatisticalOutlierRemovalXYZBuilder::new()
        .mean_k(50) // Analyze 50 nearest neighbors
        .std_dev_mul_thresh(1.0) // 1 standard deviation threshold
        .build()?;

    statistical_filter.set_input_cloud(&cloud)?;
    let statistical_filtered = statistical_filter.filter()?;
    println!(
        "   Filtered cloud size: {} points",
        statistical_filtered.size()
    );
    println!(
        "   Outliers removed: {} points",
        cloud.size() - statistical_filtered.size()
    );

    // Example 4: Radius Outlier Removal
    println!("\n4. Radius Outlier Removal");
    println!("   Removing points with fewer than 10 neighbors within 0.2 units");

    let mut radius_filter = RadiusOutlierRemovalXYZBuilder::new()
        .radius_search(0.2)
        .min_neighbors_in_radius(10)
        .build()?;

    radius_filter.set_input_cloud(&cloud)?;
    let radius_filtered = radius_filter.filter()?;
    println!("   Filtered cloud size: {} points", radius_filtered.size());
    println!(
        "   Isolated points removed: {} points",
        cloud.size() - radius_filtered.size()
    );

    // Example 5: Chaining filters
    println!("\n5. Chaining Multiple Filters");
    println!("   Applying VoxelGrid → PassThrough → Statistical Outlier Removal");

    // Start with the original cloud
    let mut result = cloud;

    // Apply VoxelGrid
    let mut voxel = VoxelGridXYZBuilder::new()
        .leaf_size(0.05, 0.05, 0.05)
        .build()?;
    voxel.set_input_cloud(&result)?;
    result = voxel.filter()?;
    println!("   After VoxelGrid: {} points", result.size());

    // Apply PassThrough
    let mut pass = PassThroughXYZBuilder::new()
        .field_name("z")
        .limits(0.0, 3.0)
        .build()?;
    pass.set_input_cloud(&result)?;
    result = pass.filter()?;
    println!("   After PassThrough: {} points", result.size());

    // Apply Statistical Outlier Removal
    let mut statistical = StatisticalOutlierRemovalXYZBuilder::new()
        .mean_k(30)
        .std_dev_mul_thresh(2.0)
        .build()?;
    statistical.set_input_cloud(&result)?;
    result = statistical.filter()?;
    println!("   After Statistical: {} points", result.size());

    println!("\nFilter examples completed!");
    Ok(())
}

/// Create a sample point cloud with some structure and noise
fn create_sample_cloud() -> Result<PointCloudXYZ, Box<dyn std::error::Error>> {
    let mut points = Vec::new();

    // Create a structured grid of points
    for x in -50..=50 {
        for y in -50..=50 {
            for z in 0..=30 {
                let fx = x as f32 * 0.05;
                let fy = y as f32 * 0.05;
                let fz = z as f32 * 0.1;

                // Add the main structure
                points.push((fx, fy, fz));

                // Add some random noise points
                if x % 10 == 0 && y % 10 == 0 && z % 5 == 0 {
                    // Random outliers
                    let noise_x = fx + (x as f32 * 0.001) % 0.5;
                    let noise_y = fy + (y as f32 * 0.001) % 0.5;
                    let noise_z = fz + (z as f32 * 0.001) % 0.5 + 3.0;
                    points.push((noise_x, noise_y, noise_z));
                }
            }
        }
    }

    // Add some isolated noise points
    for i in 0..100 {
        let x = ((i * 7) % 100) as f32 * 0.05 - 2.5;
        let y = ((i * 13) % 100) as f32 * 0.05 - 2.5;
        let z = ((i * 23) % 50) as f32 * 0.1 + 5.0;
        points.push((x, y, z));
    }

    let cloud = PointCloudXYZBuilder::new().add_points(points).build()?;

    Ok(cloud)
}
