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
        "  Created PointXYZ cloud: {} points, empty: {}",
        cloud_xyz.size(),
        cloud_xyz.empty()
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

    Ok(())
}

fn demonstrate_search_structures() -> PclResult<()> {
    println!("\n🔍 Search Structures:");

    // KdTree for PointXYZ
    let kdtree = KdTreeXYZ::new()?;
    println!("  Created KdTreeXYZ successfully");

    // Note: Setting input cloud and performing searches would require
    // points to be added to the cloud first, which is not yet supported

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
