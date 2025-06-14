//! Basic usage example for PCL Rust bindings
//!
//! This example demonstrates:
//! - Creating point clouds (PointXYZ and PointXYZRGB)
//! - Basic point cloud operations
//! - Creating search structures (KdTree)
//! - Creating octree structures
//! - Error handling

use pcl::octree::{OctreeSearchXYZ, OctreeVoxelCentroidXYZ};
use pcl::search::KdTreeXYZ;
use pcl::{PclResult, PointCloudXYZ, PointCloudXYZRGB};

fn main() -> PclResult<()> {
    println!("PCL Rust Basic Usage Example");
    println!("============================");

    // Demonstrate point cloud creation and basic operations
    demonstrate_point_clouds()?;

    // Demonstrate search structures
    demonstrate_search_structures()?;

    // Demonstrate octree structures
    demonstrate_octree_structures()?;

    println!("\n✅ All basic operations completed successfully!");
    println!("Note: Point creation and manipulation from Rust require additional");
    println!("      FFI functions that are not yet implemented due to cxx limitations.");

    Ok(())
}

fn demonstrate_point_clouds() -> PclResult<()> {
    println!("\n🔷 Point Cloud Operations:");

    // PointXYZ cloud
    let mut cloud_xyz = PointCloudXYZ::new()?;
    println!(
        "  Created PointXYZ cloud: {} points, empty: {}, organized: {}",
        cloud_xyz.size(),
        cloud_xyz.empty(),
        cloud_xyz.is_organized()
    );

    // Demonstrate new operations
    cloud_xyz.reserve(1000)?;
    println!("  Reserved capacity for 1000 points");

    cloud_xyz.resize(100)?;
    println!(
        "  After resize(100): {} points, width: {}, height: {}, dense: {}",
        cloud_xyz.size(),
        cloud_xyz.width(),
        cloud_xyz.height(),
        cloud_xyz.is_dense()
    );

    cloud_xyz.clear()?;
    println!(
        "  After clear: {} points, empty: {}",
        cloud_xyz.size(),
        cloud_xyz.empty()
    );

    // PointXYZRGB cloud
    let cloud_xyzrgb = PointCloudXYZRGB::new()?;
    println!(
        "  Created PointXYZRGB cloud: {} points, empty: {}",
        cloud_xyzrgb.size(),
        cloud_xyzrgb.empty()
    );

    // Demonstrate builder pattern
    println!("\n📦 Using Builder Pattern:");
    use pcl::common::PointCloudXYZBuilder;

    let cloud_from_builder = PointCloudXYZBuilder::new()
        .add_point(1.0, 2.0, 3.0)
        .add_point(4.0, 5.0, 6.0)
        .add_points(vec![(7.0, 8.0, 9.0), (10.0, 11.0, 12.0)])
        .build()?;

    println!(
        "  Built cloud with {} points (note: values are default-initialized due to FFI limitations)",
        cloud_from_builder.size()
    );

    Ok(())
}

fn demonstrate_search_structures() -> PclResult<()> {
    println!("\n🔍 Search Structures:");

    // KdTree for PointXYZ
    use pcl::search::{SearchConfiguration, SearchInputCloud};

    let mut kdtree = KdTreeXYZ::new()?;
    println!("  Created KdTreeXYZ successfully");
    println!("  Initial epsilon: {:.3}", kdtree.epsilon());

    // Configure search parameters
    kdtree.set_epsilon(0.1)?;
    println!("  Set epsilon to: {:.3}", kdtree.epsilon());

    // Create a point cloud and set it as input
    let mut cloud = PointCloudXYZ::new()?;
    cloud.resize(10)?; // Create 10 default points
    kdtree.set_input_cloud(&cloud)?;
    println!("  Set input cloud with {} points", cloud.size());

    // Demonstrate unified search interface
    use pcl::search::{SearchMethod, SearchXYZ};

    let mut unified_search = SearchXYZ::new(SearchMethod::KdTree)?;
    println!(
        "\n  Using unified search interface: {:?}",
        unified_search.method()
    );
    unified_search.set_input_cloud(&cloud)?;

    // Test KdTree for PointXYZRGB
    let mut kdtree_rgb = pcl::search::KdTreeXYZRGB::new()?;
    println!("\n  Created KdTreeXYZRGB successfully");

    let cloud_rgb = PointCloudXYZRGB::new()?;
    kdtree_rgb.set_input_cloud(&cloud_rgb)?;
    println!("  RGB KdTree configured with empty cloud");

    // Note: Actual search operations would require points with real data
    // which is not possible due to FFI limitations

    Ok(())
}

fn demonstrate_octree_structures() -> PclResult<()> {
    println!("\n🌳 Octree Structures:");

    // Octree for search operations
    let mut octree_search = OctreeSearchXYZ::new(0.1)?;
    println!("  Created OctreeSearchXYZ with resolution 0.1");
    println!("  Resolution: {:.2}", octree_search.resolution());
    println!("  Tree depth: {}", octree_search.tree_depth());
    println!("  Leaf count: {}", octree_search.leaf_count());
    println!("  Branch count: {}", octree_search.branch_count());

    // Octree for voxel centroid operations
    let _octree_voxel = OctreeVoxelCentroidXYZ::new(0.05)?;
    println!("  Created OctreeVoxelCentroidXYZ with resolution 0.05");

    // Test error handling
    match OctreeSearchXYZ::new(-1.0) {
        Ok(_) => println!("  ❌ This should not happen!"),
        Err(e) => println!("  ✅ Properly caught invalid resolution: {}", e),
    }

    Ok(())
}
