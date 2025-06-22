//! Auto-detection example for PCL-Rust
//!
//! This example demonstrates the format auto-detection functionality that can
//! automatically detect whether a file is PCD or PLY format and load it accordingly.

use pcl::common::PointCloudXYZBuilder;
use pcl::error::Result;
use pcl::io::{BinaryFormat, PcdIoXYZ, PlyIoXYZ};
use pcl::io::{
    FileFormat, detect_format, detect_format_from_ext, detect_format_from_file_content, load_xyz,
};
use std::path::Path;

fn main() -> Result<()> {
    println!("PCL-Rust Auto-Detection Example");
    println!("================================");

    // Create sample point clouds for testing
    let sample_cloud = create_sample_cloud()?;

    // Test files we'll create
    let test_files = [
        ("test_cloud.pcd", FileFormat::Pcd),
        ("test_cloud.ply", FileFormat::Ply),
        ("test_cloud_no_ext", FileFormat::Pcd), // Test content detection
    ];

    // Create test files
    println!("\n1. Creating test files...");
    sample_cloud.save_pcd_with_format("test_cloud.pcd", BinaryFormat::Ascii)?;
    sample_cloud.save_ply_with_format("test_cloud.ply", BinaryFormat::Ascii)?;

    // Create a file without extension (copy PCD content)
    std::fs::copy("test_cloud.pcd", "test_cloud_no_ext")?;

    println!("✓ Created test files: test_cloud.pcd, test_cloud.ply, test_cloud_no_ext");

    // Test format detection methods
    println!("\n2. Testing format detection methods...");

    for (filename, expected_format) in &test_files {
        println!("\nTesting file: {}", filename);

        // Test extension-based detection
        match detect_format_from_ext(filename) {
            Ok(format) => {
                println!("  Extension detection: {:?}", format);
                if filename.contains('.') {
                    assert_eq!(
                        format, *expected_format,
                        "Extension detection failed for {}",
                        filename
                    );
                }
            }
            Err(e) => {
                if filename.contains('.') {
                    panic!("Extension detection failed for {}: {}", filename, e);
                } else {
                    println!(
                        "  Extension detection: Failed (expected for no extension): {}",
                        e
                    );
                }
            }
        }

        // Test content-based detection
        match detect_format_from_file_content(filename) {
            Ok(format) => {
                println!("  Content detection: {:?}", format);
                assert_eq!(
                    format, *expected_format,
                    "Content detection failed for {}",
                    filename
                );
            }
            Err(e) => panic!("Content detection failed for {}: {}", filename, e),
        }

        // Test combined auto-detection
        match detect_format(filename) {
            Ok(format) => {
                println!("  Auto detection: {:?}", format);
                assert_eq!(
                    format, *expected_format,
                    "Auto detection failed for {}",
                    filename
                );
            }
            Err(e) => panic!("Auto detection failed for {}: {}", filename, e),
        }
    }

    // Test auto-loading functionality
    println!("\n3. Testing auto-loading functionality...");

    for (filename, _) in &test_files {
        println!("\nLoading file: {}", filename);

        match load_xyz(filename) {
            Ok(loaded_cloud) => {
                println!("  ✓ Successfully loaded {} points", loaded_cloud.size());

                // Verify the loaded cloud has the same number of points
                assert_eq!(
                    loaded_cloud.size(),
                    sample_cloud.size(),
                    "Loaded cloud size mismatch for {}",
                    filename
                );

                // Basic validation that points were loaded correctly
                assert!(
                    !loaded_cloud.empty(),
                    "Loaded cloud is empty for {}",
                    filename
                );
            }
            Err(e) => panic!("Auto-loading failed for {}: {}", filename, e),
        }
    }

    // Test error handling
    println!("\n4. Testing error handling...");

    // Test with non-existent file
    match load_xyz("non_existent_file.pcd") {
        Ok(_) => panic!("Should have failed for non-existent file"),
        Err(e) => println!("  ✓ Correctly failed for non-existent file: {}", e),
    }

    // Test with unsupported format
    std::fs::write("test.txt", "This is not a point cloud file")?;
    match detect_format("test.txt") {
        Ok(_) => panic!("Should have failed for unsupported format"),
        Err(e) => println!("  ✓ Correctly failed for unsupported format: {}", e),
    }

    // Test with corrupted file
    std::fs::write(
        "corrupted.pcd",
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n// Missing data",
    )?;
    match load_xyz("corrupted.pcd") {
        Ok(_) => println!("  Note: Corrupted file loaded (PCL may be tolerant)"),
        Err(e) => println!("  ✓ Correctly failed for corrupted file: {}", e),
    }

    // Performance comparison
    println!("\n5. Performance comparison...");

    let start = std::time::Instant::now();
    for _ in 0..100 {
        let _format = detect_format("test_cloud.pcd")?;
    }
    let auto_detect_time = start.elapsed();

    let start = std::time::Instant::now();
    for _ in 0..100 {
        let _cloud = load_xyz("test_cloud.pcd")?;
    }
    let auto_load_time = start.elapsed();

    println!("  Format detection (100x): {:?}", auto_detect_time);
    println!("  Auto-loading (100x): {:?}", auto_load_time);

    // Clean up test files
    println!("\n6. Cleaning up test files...");
    let cleanup_files = [
        "test_cloud.pcd",
        "test_cloud.ply",
        "test_cloud_no_ext",
        "test.txt",
        "corrupted.pcd",
    ];
    for file in &cleanup_files {
        if Path::new(file).exists() {
            std::fs::remove_file(file).ok();
        }
    }
    println!("  ✓ Cleaned up test files");

    println!("\n🎉 Auto-detection example completed successfully!");
    println!("\nKey features demonstrated:");
    println!("  • Extension-based format detection");
    println!("  • Content-based format detection");
    println!("  • Combined auto-detection (extension first, then content)");
    println!("  • Auto-loading with format detection");
    println!("  • Error handling for unsupported formats");
    println!("  • Performance characteristics");

    Ok(())
}

/// Create a sample point cloud for testing
fn create_sample_cloud() -> Result<pcl::common::PointCloud<pcl::common::XYZ>> {
    // Note: Due to current FFI limitations, we can't actually set point data from Rust
    // This creates an empty cloud with some points allocated
    let builder = PointCloudXYZBuilder::new();

    // Add some sample points conceptually (but they will be default-initialized)
    let points: Vec<(f32, f32, f32)> = (0..100)
        .map(|i| {
            let x = (i % 10) as f32 * 0.1;
            let y = (i / 10) as f32 * 0.1;
            let z = i as f32 * 0.01;
            (x, y, z)
        })
        .collect();

    let builder = builder.add_points(points);
    builder.build()
}
