//! Safe PCD (Point Cloud Data) file I/O operations
//!
//! This module provides safe, idiomatic Rust interfaces for reading and writing
//! PCD files with comprehensive error handling and format options.

use super::BinaryFormat;
use crate::common::{PointCloud, XYZ, XYZI, XYZRGB};
use crate::error::{PclError, PclResult};
use std::path::Path;

/// PCD file I/O operations for PointXYZ clouds
pub trait PcdIoXYZ {
    /// Load a PCD file into the point cloud
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file
    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file with specified format
    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

/// PCD file I/O operations for PointXYZI clouds
pub trait PcdIoXYZI {
    /// Load a PCD file into the point cloud
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file
    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file with specified format
    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

/// PCD file I/O operations for PointXYZRGB clouds
pub trait PcdIoXYZRGB {
    /// Load a PCD file into the point cloud
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file
    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PCD file with specified format
    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

impl PcdIoXYZ for PointCloud<XYZ> {
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_pcd_file_xyz(path_str, self.inner_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_pcd_with_format(path, BinaryFormat::Ascii)
    }

    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_pcd_file_ascii_xyz(path_str, self.inner()),
            BinaryFormat::Binary => pcl_sys::ffi::save_pcd_file_binary_xyz(path_str, self.inner()),
            BinaryFormat::BinaryCompressed => {
                pcl_sys::ffi::save_pcd_file_binary_compressed_xyz(path_str, self.inner())
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

impl PcdIoXYZI for PointCloud<XYZI> {
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_pcd_file_xyzi(path_str, self.inner_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_pcd_with_format(path, BinaryFormat::Ascii)
    }

    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_pcd_file_ascii_xyzi(path_str, self.inner()),
            BinaryFormat::Binary => pcl_sys::ffi::save_pcd_file_binary_xyzi(path_str, self.inner()),
            BinaryFormat::BinaryCompressed => {
                pcl_sys::ffi::save_pcd_file_binary_compressed_xyzi(path_str, self.inner())
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

impl PcdIoXYZRGB for PointCloud<XYZRGB> {
    fn load_pcd<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_pcd_file_xyzrgb(path_str, self.inner_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_pcd<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_pcd_with_format(path, BinaryFormat::Ascii)
    }

    fn save_pcd_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_pcd_file_ascii_xyzrgb(path_str, self.inner()),
            BinaryFormat::Binary => {
                pcl_sys::ffi::save_pcd_file_binary_xyzrgb(path_str, self.inner())
            }
            BinaryFormat::BinaryCompressed => {
                pcl_sys::ffi::save_pcd_file_binary_compressed_xyzrgb(path_str, self.inner())
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PCD file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

/// Convenience functions for loading PCD files
pub fn load_pcd_xyz<P: AsRef<Path>>(path: P) -> PclResult<PointCloud<XYZ>> {
    let mut cloud = PointCloud::<XYZ>::new()?;
    cloud.load_pcd(path)?;
    Ok(cloud)
}

/// Convenience functions for loading PCD files with PointXYZI
pub fn load_pcd_xyzi<P: AsRef<Path>>(path: P) -> PclResult<PointCloud<XYZI>> {
    let mut cloud = PointCloud::<XYZI>::new()?;
    cloud.load_pcd(path)?;
    Ok(cloud)
}

/// Convenience functions for loading PCD files with PointXYZRGB
pub fn load_pcd_xyzrgb<P: AsRef<Path>>(path: P) -> PclResult<PointCloud<XYZRGB>> {
    let mut cloud = PointCloud::<XYZRGB>::new()?;
    cloud.load_pcd(path)?;
    Ok(cloud)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_pcd_io_xyz() {
        let mut cloud = PointCloud::<XYZ>::new().unwrap();
        cloud.resize(10).unwrap();

        let temp_file = NamedTempFile::new().unwrap();
        let path = temp_file.path();

        // Test saving
        assert!(cloud.save_pcd(path).is_ok());
        assert!(path.exists());

        // Test loading
        let mut loaded_cloud = PointCloud::<XYZ>::new().unwrap();
        assert!(loaded_cloud.load_pcd(path).is_ok());
        assert_eq!(loaded_cloud.size(), cloud.size());
    }

    #[test]
    fn test_pcd_format_options() {
        let mut cloud = PointCloud::<XYZ>::new().unwrap();
        cloud.resize(5).unwrap();

        // Test ASCII format
        let temp_ascii = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_pcd_with_format(temp_ascii.path(), BinaryFormat::Ascii)
                .is_ok()
        );

        // Test Binary format
        let temp_binary = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_pcd_with_format(temp_binary.path(), BinaryFormat::Binary)
                .is_ok()
        );

        // Test Binary Compressed format
        let temp_compressed = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_pcd_with_format(temp_compressed.path(), BinaryFormat::BinaryCompressed)
                .is_ok()
        );
    }

    #[test]
    fn test_convenience_functions() {
        let mut cloud = PointCloud::<XYZ>::new().unwrap();
        cloud.resize(3).unwrap();

        let temp_file = NamedTempFile::new().unwrap();
        cloud.save_pcd(temp_file.path()).unwrap();

        // Test convenience loading function
        let loaded_cloud = load_pcd_xyz(temp_file.path()).unwrap();
        assert_eq!(loaded_cloud.size(), cloud.size());
    }

    #[test]
    fn test_invalid_path_error() {
        let cloud = PointCloud::<XYZ>::new().unwrap();

        // Test with invalid UTF-8 path (this would need platform-specific testing)
        // For now just test with non-existent directory
        let result = cloud.save_pcd("/nonexistent/directory/file.pcd");
        assert!(result.is_err());
    }
}
