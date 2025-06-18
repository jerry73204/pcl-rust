//! Demonstration of generic KdTree and Filter implementations
//!
//! This example shows how to use the generic KdTree<T> and Filter<T> APIs
//! that work with any point type implementing the required traits.

use pcl::{Filter, KdTree, PclResult, PointCloud, PointXYZ, PointXYZRGB, VoxelGrid};

fn main() -> PclResult<()> {
    println!("Generic API Demonstration");
    println!("========================\n");

    // Demonstrate generic KdTree with different point types
    kdtree_demo()?;

    // Demonstrate generic filters
    filter_demo()?;

    Ok(())
}

/// Demonstrate generic KdTree that works with any point type
fn kdtree_demo() -> PclResult<()> {
    println!("1. Generic KdTree Demo");
    println!("---------------------");

    // Create a PointXYZ cloud
    let mut xyz_cloud = PointCloud::<PointXYZ>::new()?;
    xyz_cloud.push(1.0, 2.0, 3.0)?;
    xyz_cloud.push(4.0, 5.0, 6.0)?;
    xyz_cloud.push(7.0, 8.0, 9.0)?;

    // Create a generic KdTree for PointXYZ
    let mut xyz_kdtree = KdTree::<PointXYZ>::new()?;
    xyz_kdtree.set_input_cloud(&xyz_cloud)?;
    println!(
        "✓ Created KdTree<PointXYZ> with {} points",
        xyz_cloud.size()
    );

    // Create a PointXYZRGB cloud
    let mut rgb_cloud = PointCloud::<PointXYZRGB>::new()?;
    rgb_cloud.push(1.0, 2.0, 3.0, 255, 0, 0)?;
    rgb_cloud.push(4.0, 5.0, 6.0, 0, 255, 0)?;
    rgb_cloud.push(7.0, 8.0, 9.0, 0, 0, 255)?;

    // Create a generic KdTree for PointXYZRGB
    let mut rgb_kdtree = KdTree::<PointXYZRGB>::new()?;
    rgb_kdtree.set_input_cloud(&rgb_cloud)?;
    println!(
        "✓ Created KdTree<PointXYZRGB> with {} points",
        rgb_cloud.size()
    );

    // Note: Search methods would work once we implement point access
    // let neighbors = xyz_kdtree.nearest_k_search_coords(1.0, 2.0, 3.0, 5)?;

    println!();
    Ok(())
}

/// Demonstrate generic filters that work with any point type
fn filter_demo() -> PclResult<()> {
    println!("2. Generic Filter Demo");
    println!("---------------------");

    // Create a dense point cloud
    let mut cloud = PointCloud::<PointXYZ>::new()?;
    for i in 0..100 {
        for j in 0..100 {
            cloud.push(i as f32 * 0.01, j as f32 * 0.01, 0.0)?;
        }
    }
    println!("✓ Created dense cloud with {} points", cloud.size());

    // Use generic VoxelGrid filter
    let mut voxel_filter = VoxelGrid::<PointXYZ>::new()?;
    voxel_filter.set_leaf_size(0.05, 0.05, 0.05)?;
    voxel_filter.set_input_cloud(&cloud)?;

    let filtered_cloud = voxel_filter.filter()?;
    println!("✓ Applied VoxelGrid<PointXYZ> filter");
    println!("  Original size: {}", cloud.size());
    println!("  Filtered size: {}", filtered_cloud.size());
    println!(
        "  Reduction: {:.1}%",
        (1.0 - filtered_cloud.size() as f32 / cloud.size() as f32) * 100.0
    );

    // The same code works with PointXYZRGB
    let mut rgb_cloud = PointCloud::<PointXYZRGB>::new()?;
    for i in 0..50 {
        for j in 0..50 {
            rgb_cloud.push(
                i as f32 * 0.02,
                j as f32 * 0.02,
                0.0,
                (i * 5) as u8,
                (j * 5) as u8,
                128,
            )?;
        }
    }

    let mut rgb_voxel_filter = VoxelGrid::<PointXYZRGB>::new()?;
    rgb_voxel_filter.set_leaf_size(0.1, 0.1, 0.1)?;
    rgb_voxel_filter.set_input_cloud(&rgb_cloud)?;

    let filtered_rgb_cloud = rgb_voxel_filter.filter()?;
    println!("\n✓ Applied VoxelGrid<PointXYZRGB> filter");
    println!("  Original size: {}", rgb_cloud.size());
    println!("  Filtered size: {}", filtered_rgb_cloud.size());

    println!();
    Ok(())
}

// Generic function that works with any point cloud type
fn print_cloud_info<T: pcl::Point>(cloud: &PointCloud<T>)
where
    T::CloudType: cxx::memory::UniquePtrTarget,
{
    println!("Point cloud info:");
    println!("  Type: {}", T::type_name());
    println!("  Size: {}", cloud.size());
    println!("  Empty: {}", cloud.empty());
    println!("  Organized: {}", cloud.is_organized());
    println!("  Dense: {}", cloud.is_dense());
}
