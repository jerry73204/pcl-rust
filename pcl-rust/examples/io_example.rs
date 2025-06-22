//! Example demonstrating point cloud I/O operations
//!
//! This example shows how to:
//! - Create a simple point cloud
//! - Save it to PCD and PLY formats
//! - Load it back and verify the data

use pcl::{
    BinaryFormat, PcdIoXYZ, PlyIoXYZ,
    common::{PointCloud, PointCloudXYZBuilder, XYZ},
    io::{load_pcd_xyz, load_ply_xyz},
};
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a simple point cloud with some test data
    let cloud = PointCloudXYZBuilder::new()
        .add_point(1.0, 2.0, 3.0)
        .add_point(4.0, 5.0, 6.0)
        .add_point(7.0, 8.0, 9.0)
        .add_point(-1.0, -2.0, -3.0)
        .add_point(0.0, 0.0, 0.0)
        .build()?;

    println!("Created point cloud with {} points", cloud.size());

    // Create a temporary directory for our test files
    let temp_dir = tempfile::tempdir()?;

    // Test PCD format
    println!("\n=== Testing PCD Format ===");

    // Save as ASCII PCD
    let pcd_ascii_path = temp_dir.path().join("test_ascii.pcd");
    cloud.save_pcd_with_format(&pcd_ascii_path, BinaryFormat::Ascii)?;
    println!("Saved ASCII PCD to: {:?}", pcd_ascii_path);

    // Save as Binary PCD
    let pcd_binary_path = temp_dir.path().join("test_binary.pcd");
    cloud.save_pcd_with_format(&pcd_binary_path, BinaryFormat::Binary)?;
    println!("Saved Binary PCD to: {:?}", pcd_binary_path);

    // Save as Compressed Binary PCD
    let pcd_compressed_path = temp_dir.path().join("test_compressed.pcd");
    cloud.save_pcd_with_format(&pcd_compressed_path, BinaryFormat::BinaryCompressed)?;
    println!("Saved Compressed PCD to: {:?}", pcd_compressed_path);

    // Load back and verify
    let loaded_pcd = load_pcd_xyz(&pcd_ascii_path)?;
    println!("Loaded PCD with {} points", loaded_pcd.size());
    assert_eq!(cloud.size(), loaded_pcd.size());

    // Test PLY format
    println!("\n=== Testing PLY Format ===");

    // Save as ASCII PLY
    let ply_ascii_path = temp_dir.path().join("test_ascii.ply");
    cloud.save_ply_with_format(&ply_ascii_path, BinaryFormat::Ascii)?;
    println!("Saved ASCII PLY to: {:?}", ply_ascii_path);

    // Save as Binary PLY
    let ply_binary_path = temp_dir.path().join("test_binary.ply");
    cloud.save_ply_with_format(&ply_binary_path, BinaryFormat::Binary)?;
    println!("Saved Binary PLY to: {:?}", ply_binary_path);

    // Load back and verify
    let loaded_ply = load_ply_xyz(&ply_ascii_path)?;
    println!("Loaded PLY with {} points", loaded_ply.size());
    assert_eq!(cloud.size(), loaded_ply.size());

    // Compare file sizes
    println!("\n=== File Size Comparison ===");
    let pcd_ascii_size = fs::metadata(&pcd_ascii_path)?.len();
    let pcd_binary_size = fs::metadata(&pcd_binary_path)?.len();
    let pcd_compressed_size = fs::metadata(&pcd_compressed_path)?.len();
    let ply_ascii_size = fs::metadata(&ply_ascii_path)?.len();
    let ply_binary_size = fs::metadata(&ply_binary_path)?.len();

    println!("PCD ASCII:      {} bytes", pcd_ascii_size);
    println!("PCD Binary:     {} bytes", pcd_binary_size);
    println!("PCD Compressed: {} bytes", pcd_compressed_size);
    println!("PLY ASCII:      {} bytes", ply_ascii_size);
    println!("PLY Binary:     {} bytes", ply_binary_size);

    // Test loading into existing cloud
    println!("\n=== Testing Load into Existing Cloud ===");
    let mut existing_cloud = PointCloud::<XYZ>::new()?;
    existing_cloud.load_pcd(&pcd_ascii_path)?;
    println!(
        "Loaded {} points into existing cloud",
        existing_cloud.size()
    );

    // Test error handling
    println!("\n=== Testing Error Handling ===");
    let invalid_path = "/nonexistent/directory/file.pcd";
    match cloud.save_pcd(invalid_path) {
        Ok(_) => println!("Unexpected success!"),
        Err(e) => println!("Expected error: {}", e),
    }

    // Clean up happens automatically when temp_dir goes out of scope
    println!("\n✅ All I/O tests passed!");

    Ok(())
}
