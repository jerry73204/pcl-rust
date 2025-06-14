//! Basic usage example for PCL Rust bindings
//!
//! This example demonstrates:
//! - Creating point clouds
//! - Basic point cloud operations

use pcl::{PclResult, PointCloudXYZ};

fn main() -> PclResult<()> {
    println!("PCL Rust Basic Usage Example");

    // Create a new point cloud
    let mut cloud = PointCloudXYZ::new()?;
    println!("Created empty point cloud with {} points", cloud.size());
    println!("Is cloud empty? {}", cloud.empty());

    // Clear the cloud (even though it's already empty)
    cloud.clear()?;
    println!("Cleared point cloud");

    // Create another point cloud
    let cloud2 = PointCloudXYZ::new()?;
    println!("Created second point cloud with {} points", cloud2.size());

    println!("Point cloud operations completed successfully!");
    println!("Note: Point creation and manipulation require additional");
    println!("      FFI functions that are not yet implemented.");

    Ok(())
}
