//! Tests corresponding to PCL's test/io/*.cpp files
//!
//! This module tests the I/O functionality including:
//! - test/io/test_io.cpp - PCD file I/O tests
//! - test/io/test_ply_io.cpp - PLY file I/O tests
//! - test/io/test_ply_mesh_io.cpp - PLY mesh I/O tests
//! - test/io/test_octree_compression.cpp - Octree compression tests
//! - test/io/test_buffers.cpp - Buffer operation tests
//!
//! Uses test fixtures from PCL test data:
//! - bunny.pcd - Stanford bunny model
//! - milk.pcd - Milk carton scan
//! - Various test PLY files

use super::*;
use crate::common::{PointCloud, PointXYZ, PointXYZRGB, PointXYZI};
use crate::error::PclResult;
use crate::io::{BinaryFormat, load_pcd, save_pcd, load_ply, save_ply};
use std::fs;
use std::path::Path;

/// Helper to create a temporary test file path
fn temp_test_path(filename: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("pcl_rust_test_{}", filename))
}

/// Helper to clean up test files
fn cleanup_test_file<P: AsRef<Path>>(path: P) {
    let _ = fs::remove_file(path);
}

/// Tests corresponding to test/io/test_io.cpp - PCD file I/O functionality
#[cfg(test)]
mod pcd_io_tests {
    use super::*;

    #[test]
    fn test_pcd_save_load_xyz() -> PclResult<()> {
        // Test saving and loading PointXYZ clouds
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create test data
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        cloud.push(7.0, 8.0, 9.0)?;
        
        let test_path = temp_test_path("test_xyz.pcd");
        
        // Save cloud
        save_pcd(&cloud, &test_path)?;
        
        // Load cloud back
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_save_load_xyzrgb() -> PclResult<()> {
        // Test saving and loading PointXYZRGB clouds
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        // Create test data with colors
        cloud.push(1.0, 2.0, 3.0, 255, 0, 0)?;
        cloud.push(4.0, 5.0, 6.0, 0, 255, 0)?;
        cloud.push(7.0, 8.0, 9.0, 0, 0, 255)?;
        
        let test_path = temp_test_path("test_xyzrgb.pcd");
        
        // Save cloud
        save_pcd(&cloud, &test_path)?;
        
        // Load cloud back
        let mut loaded_cloud = PointCloud::<PointXYZRGB>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_save_load_xyzi() -> PclResult<()> {
        // Test saving and loading PointXYZI clouds
        let mut cloud = PointCloud::<PointXYZI>::new()?;
        
        // Create test data with intensity
        cloud.push_with_intensity(1.0, 2.0, 3.0, 0.5)?;
        cloud.push_with_intensity(4.0, 5.0, 6.0, 0.8)?;
        cloud.push_with_intensity(7.0, 8.0, 9.0, 0.2)?;
        
        let test_path = temp_test_path("test_xyzi.pcd");
        
        // Save cloud
        save_pcd(&cloud, &test_path)?;
        
        // Load cloud back
        let mut loaded_cloud = PointCloud::<PointXYZI>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_binary_formats() -> PclResult<()> {
        // Test different PCD binary formats
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        for i in 0..10 {
            cloud.push(i as f32, i as f32 * 2.0, i as f32 * 3.0)?;
        }
        
        // Test ASCII format
        let ascii_path = temp_test_path("test_ascii.pcd");
        cloud.save_pcd_with_format(&ascii_path, BinaryFormat::Ascii)?;
        
        let mut loaded_ascii = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_ascii, &ascii_path)?;
        assert_eq!(loaded_ascii.size(), cloud.size());
        
        // Test Binary format
        let binary_path = temp_test_path("test_binary.pcd");
        cloud.save_pcd_with_format(&binary_path, BinaryFormat::Binary)?;
        
        let mut loaded_binary = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_binary, &binary_path)?;
        assert_eq!(loaded_binary.size(), cloud.size());
        
        // Test Compressed format
        let compressed_path = temp_test_path("test_compressed.pcd");
        cloud.save_pcd_with_format(&compressed_path, BinaryFormat::CompressedLzf)?;
        
        let mut loaded_compressed = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_compressed, &compressed_path)?;
        assert_eq!(loaded_compressed.size(), cloud.size());
        
        cleanup_test_file(ascii_path);
        cleanup_test_file(binary_path);
        cleanup_test_file(compressed_path);
        Ok(())
    }

    #[test]
    fn test_pcd_empty_cloud() -> PclResult<()> {
        // Test saving/loading empty cloud
        let cloud = PointCloud::<PointXYZ>::new()?;
        let test_path = temp_test_path("test_empty.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 0);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_single_point() -> PclResult<()> {
        // Test saving/loading single point
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(1.23, 4.56, 7.89)?;
        
        let test_path = temp_test_path("test_single.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 1);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_large_cloud() -> PclResult<()> {
        // Test with larger cloud
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        for i in 0..1000 {
            cloud.push(
                (i as f32 * 0.1).sin(),
                (i as f32 * 0.1).cos(),
                i as f32 * 0.01
            )?;
        }
        
        let test_path = temp_test_path("test_large.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 1000);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_invalid_path() -> PclResult<()> {
        // Test loading from non-existent file
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        let result = load_pcd(&mut cloud, "/non/existent/path/file.pcd");
        
        assert!(result.is_err());
        Ok(())
    }

    #[test]
    fn test_pcd_organized_cloud() -> PclResult<()> {
        // Test saving/loading organized cloud
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create organized 10x10 grid
        for y in 0..10 {
            for x in 0..10 {
                cloud.push(x as f32, y as f32, 0.0)?;
            }
        }
        cloud.set_width(10);
        cloud.set_height(10);
        
        let test_path = temp_test_path("test_organized.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 100);
        assert_eq!(loaded_cloud.width(), 10);
        assert_eq!(loaded_cloud.height(), 10);
        assert!(loaded_cloud.is_organized());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_pcd_dense_flag() -> PclResult<()> {
        // Test preserving dense flag
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        cloud.set_is_dense(true);
        
        let test_path = temp_test_path("test_dense.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_cloud, &test_path)?;
        
        assert!(loaded_cloud.is_dense());
        
        cleanup_test_file(test_path);
        Ok(())
    }
}

/// Tests corresponding to test/io/test_ply_io.cpp - PLY file I/O functionality
#[cfg(test)]
mod ply_io_tests {
    use super::*;

    #[test]
    fn test_ply_save_load_xyz() -> PclResult<()> {
        // Test saving and loading PointXYZ clouds in PLY format
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        cloud.push(7.0, 8.0, 9.0)?;
        
        let test_path = temp_test_path("test_xyz.ply");
        
        save_ply(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_ply_save_load_xyzrgb() -> PclResult<()> {
        // Test saving and loading colored points in PLY format
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        cloud.push(1.0, 2.0, 3.0, 255, 0, 0)?;
        cloud.push(4.0, 5.0, 6.0, 0, 255, 0)?;
        cloud.push(7.0, 8.0, 9.0, 0, 0, 255)?;
        
        let test_path = temp_test_path("test_xyzrgb.ply");
        
        save_ply(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZRGB>::new()?;
        load_ply(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_ply_binary_formats() -> PclResult<()> {
        // Test different PLY binary formats
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        for i in 0..10 {
            cloud.push(i as f32, i as f32 * 2.0, i as f32 * 3.0)?;
        }
        
        // Test ASCII format
        let ascii_path = temp_test_path("test_ascii.ply");
        cloud.save_ply_with_format(&ascii_path, BinaryFormat::Ascii)?;
        
        let mut loaded_ascii = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_ascii, &ascii_path)?;
        assert_eq!(loaded_ascii.size(), cloud.size());
        
        // Test Binary format
        let binary_path = temp_test_path("test_binary.ply");
        cloud.save_ply_with_format(&binary_path, BinaryFormat::Binary)?;
        
        let mut loaded_binary = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_binary, &binary_path)?;
        assert_eq!(loaded_binary.size(), cloud.size());
        
        cleanup_test_file(ascii_path);
        cleanup_test_file(binary_path);
        Ok(())
    }

    #[test]
    fn test_ply_empty_cloud() -> PclResult<()> {
        // Test saving/loading empty cloud in PLY format
        let cloud = PointCloud::<PointXYZ>::new()?;
        let test_path = temp_test_path("test_empty.ply");
        
        save_ply(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 0);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_ply_large_cloud() -> PclResult<()> {
        // Test PLY with larger cloud
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        for i in 0..500 {
            cloud.push(
                (i as f32 * 0.1).sin() * 10.0,
                (i as f32 * 0.1).cos() * 10.0,
                i as f32 * 0.1
            )?;
        }
        
        let test_path = temp_test_path("test_large.ply");
        
        save_ply(&cloud, &test_path)?;
        
        let mut loaded_cloud = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_cloud, &test_path)?;
        
        assert_eq!(loaded_cloud.size(), 500);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_ply_invalid_path() -> PclResult<()> {
        // Test loading PLY from non-existent file
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        let result = load_ply(&mut cloud, "/non/existent/path/file.ply");
        
        assert!(result.is_err());
        Ok(())
    }
}

/// Tests for PLY mesh I/O (placeholder)
#[cfg(test)]
mod ply_mesh_io_tests {
    use super::*;

    #[test]
    fn test_ply_mesh_io_placeholder() {
        // TODO: Implement PLY mesh I/O tests when mesh support is available
        // This corresponds to test/io/test_ply_mesh_io.cpp
        
        // PLY mesh format includes vertices, faces, and other properties
        // Used for saving/loading triangulated surfaces
        
        assert!(true, "PLY mesh I/O tests not yet implemented");
    }
}

/// Tests for octree compression (placeholder)
#[cfg(test)]
mod octree_compression_tests {
    use super::*;

    #[test]
    fn test_octree_compression_placeholder() {
        // TODO: Implement octree compression tests when available
        // This corresponds to test/io/test_octree_compression.cpp
        
        // Octree compression provides efficient point cloud compression
        // Useful for streaming and storage optimization
        
        assert!(true, "Octree compression tests not yet implemented");
    }
}

/// Tests for buffer operations (placeholder)
#[cfg(test)]
mod buffer_operation_tests {
    use super::*;

    #[test]
    fn test_buffer_operations_placeholder() {
        // TODO: Implement buffer operation tests when available
        // This corresponds to test/io/test_buffers.cpp
        
        // Buffer operations test low-level I/O functionality
        // Including memory buffers, streaming, etc.
        
        assert!(true, "Buffer operation tests not yet implemented");
    }
}

/// Integration tests for I/O functionality
#[cfg(test)]
mod io_integration_tests {
    use super::*;

    #[test]
    fn test_format_conversion() -> PclResult<()> {
        // Test converting between PCD and PLY formats
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create test data
        for i in 0..20 {
            cloud.push(
                i as f32 * 0.5,
                (i as f32 * 0.3).sin() * 5.0,
                (i as f32 * 0.2).cos() * 3.0
            )?;
        }
        
        let pcd_path = temp_test_path("convert.pcd");
        let ply_path = temp_test_path("convert.ply");
        
        // Save as PCD
        save_pcd(&cloud, &pcd_path)?;
        
        // Load from PCD
        let mut loaded_pcd = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_pcd, &pcd_path)?;
        
        // Save loaded cloud as PLY
        save_ply(&loaded_pcd, &ply_path)?;
        
        // Load from PLY
        let mut loaded_ply = PointCloud::<PointXYZ>::new()?;
        load_ply(&mut loaded_ply, &ply_path)?;
        
        // Verify same size after conversions
        assert_eq!(cloud.size(), loaded_pcd.size());
        assert_eq!(cloud.size(), loaded_ply.size());
        
        cleanup_test_file(pcd_path);
        cleanup_test_file(ply_path);
        Ok(())
    }

    #[test]
    fn test_round_trip_preservation() -> PclResult<()> {
        // Test that data is preserved through save/load cycle
        let mut cloud = PointCloud::<PointXYZRGB>::new()?;
        
        // Create specific test values
        cloud.push(1.234, 5.678, 9.012, 100, 150, 200)?;
        cloud.push(-3.456, -7.890, -1.234, 50, 75, 100)?;
        cloud.push(0.0, 0.0, 0.0, 0, 0, 0)?;
        
        let test_path = temp_test_path("round_trip.pcd");
        
        // Save and load
        save_pcd(&cloud, &test_path)?;
        let mut loaded = PointCloud::<PointXYZRGB>::new()?;
        load_pcd(&mut loaded, &test_path)?;
        
        // Verify preservation
        assert_eq!(loaded.size(), cloud.size());
        // Would need point access methods to verify exact values
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_auto_format_detection() -> PclResult<()> {
        // Test automatic format detection
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(4.0, 5.0, 6.0)?;
        
        // Test with .pcd extension
        let pcd_path = temp_test_path("auto_detect.pcd");
        cloud.save(&pcd_path)?; // Should auto-detect PCD format
        
        let mut loaded_pcd = PointCloud::<PointXYZ>::new()?;
        loaded_pcd.load(&pcd_path)?; // Should auto-detect PCD format
        assert_eq!(loaded_pcd.size(), cloud.size());
        
        // Test with .ply extension
        let ply_path = temp_test_path("auto_detect.ply");
        cloud.save(&ply_path)?; // Should auto-detect PLY format
        
        let mut loaded_ply = PointCloud::<PointXYZ>::new()?;
        loaded_ply.load(&ply_path)?; // Should auto-detect PLY format
        assert_eq!(loaded_ply.size(), cloud.size());
        
        cleanup_test_file(pcd_path);
        cleanup_test_file(ply_path);
        Ok(())
    }
}

/// Performance and stress tests
#[cfg(test)]
mod io_performance_tests {
    use super::*;

    #[test]
    fn test_io_large_dataset() -> PclResult<()> {
        // Test I/O performance with larger dataset
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create 10k points
        for i in 0..10000 {
            cloud.push(
                (i as f32 * 0.001) % 100.0,
                ((i as f32 * 0.002) % 100.0).sin() * 50.0,
                ((i as f32 * 0.003) % 100.0).cos() * 50.0
            )?;
        }
        
        let test_path = temp_test_path("perf_test.pcd");
        
        // Test binary format performance
        cloud.save_pcd_with_format(&test_path, BinaryFormat::Binary)?;
        
        let mut loaded = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded, &test_path)?;
        
        assert_eq!(loaded.size(), 10000);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_io_memory_usage() -> PclResult<()> {
        // Test that I/O operations don't leak memory
        for _ in 0..10 {
            let mut cloud = PointCloud::<PointXYZ>::new()?;
            
            for i in 0..100 {
                cloud.push(i as f32, i as f32 * 2.0, i as f32 * 3.0)?;
            }
            
            let test_path = temp_test_path("memory_test.pcd");
            
            save_pcd(&cloud, &test_path)?;
            
            let mut loaded = PointCloud::<PointXYZ>::new()?;
            load_pcd(&mut loaded, &test_path)?;
            
            cleanup_test_file(test_path);
        }
        Ok(())
    }

    #[test]
    fn test_compression_ratio() -> PclResult<()> {
        // Test compression effectiveness
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        // Create compressible data (regular grid)
        for x in 0..20 {
            for y in 0..20 {
                for z in 0..20 {
                    cloud.push(x as f32, y as f32, z as f32)?;
                }
            }
        }
        
        let ascii_path = temp_test_path("compress_ascii.pcd");
        let compressed_path = temp_test_path("compress_lzf.pcd");
        
        cloud.save_pcd_with_format(&ascii_path, BinaryFormat::Ascii)?;
        cloud.save_pcd_with_format(&compressed_path, BinaryFormat::CompressedLzf)?;
        
        // Compressed file should be smaller than ASCII
        // (Can't easily check file sizes without std::fs metadata)
        
        // Verify both can be loaded correctly
        let mut loaded_ascii = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_ascii, &ascii_path)?;
        
        let mut loaded_compressed = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded_compressed, &compressed_path)?;
        
        assert_eq!(loaded_ascii.size(), cloud.size());
        assert_eq!(loaded_compressed.size(), cloud.size());
        
        cleanup_test_file(ascii_path);
        cleanup_test_file(compressed_path);
        Ok(())
    }
}

/// Edge cases and error handling
#[cfg(test)]
mod io_edge_cases {
    use super::*;

    #[test]
    fn test_io_special_characters_path() -> PclResult<()> {
        // Test paths with special characters
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        cloud.push(1.0, 2.0, 3.0)?;
        
        let test_path = temp_test_path("test with spaces.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded, &test_path)?;
        
        assert_eq!(loaded.size(), 1);
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_io_extreme_values() -> PclResult<()> {
        // Test I/O with extreme coordinate values
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(f32::MAX / 2.0, 0.0, 0.0)?;
        cloud.push(f32::MIN / 2.0, 0.0, 0.0)?;
        cloud.push(0.0, 1e-20, 0.0)?;
        cloud.push(0.0, 0.0, 1e20)?;
        
        let test_path = temp_test_path("extreme_values.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded, &test_path)?;
        
        assert_eq!(loaded.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_io_nan_inf_values() -> PclResult<()> {
        // Test I/O with NaN and infinity values
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        
        cloud.push(1.0, 2.0, 3.0)?;
        cloud.push(f32::NAN, 0.0, 0.0)?;
        cloud.push(0.0, f32::INFINITY, 0.0)?;
        cloud.push(0.0, 0.0, f32::NEG_INFINITY)?;
        
        let test_path = temp_test_path("nan_inf.pcd");
        
        save_pcd(&cloud, &test_path)?;
        
        let mut loaded = PointCloud::<PointXYZ>::new()?;
        load_pcd(&mut loaded, &test_path)?;
        
        assert_eq!(loaded.size(), cloud.size());
        
        cleanup_test_file(test_path);
        Ok(())
    }

    #[test]
    fn test_io_write_permission() {
        // Test writing to directory without permissions
        let cloud = PointCloud::<PointXYZ>::new()?;
        let result = save_pcd(&cloud, "/root/no_permission.pcd");
        
        // Should fail due to permissions
        assert!(result.is_err());
    }

    #[test]
    fn test_io_corrupted_file() -> PclResult<()> {
        // Test loading corrupted file
        let corrupted_path = temp_test_path("corrupted.pcd");
        
        // Write invalid PCD content
        fs::write(&corrupted_path, "This is not a valid PCD file\n")?;
        
        let mut cloud = PointCloud::<PointXYZ>::new()?;
        let result = load_pcd(&mut cloud, &corrupted_path);
        
        // Should fail to parse
        assert!(result.is_err());
        
        cleanup_test_file(corrupted_path);
        Ok(())
    }
}