//! Safe PLY (Polygon File Format) file I/O operations
//!
//! This module provides safe, idiomatic Rust interfaces for reading and writing
//! PLY files with comprehensive error handling and format options.

use super::BinaryFormat;
use crate::common::{PointCloudXYZ, PointCloudXYZI, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use std::path::Path;

/// PLY file I/O operations for PointXYZ clouds
pub trait PlyIoXYZ {
    /// Load a PLY file into the point cloud
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file
    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file with specified format
    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

/// PLY file I/O operations for PointXYZI clouds
pub trait PlyIoXYZI {
    /// Load a PLY file into the point cloud
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file
    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file with specified format
    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

/// PLY file I/O operations for PointXYZRGB clouds
pub trait PlyIoXYZRGB {
    /// Load a PLY file into the point cloud
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file
    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()>;

    /// Save the point cloud to a PLY file with specified format
    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()>;
}

impl PlyIoXYZ for PointCloudXYZ {
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_ply_file_xyz(path_str, self.inner.pin_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_ply_with_format(path, BinaryFormat::Ascii)
    }

    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_ply_file_ascii_xyz(path_str, &self.inner),
            BinaryFormat::Binary => pcl_sys::ffi::save_ply_file_binary_xyz(path_str, &self.inner),
            BinaryFormat::BinaryCompressed => {
                // PLY doesn't support compressed format, fall back to binary
                pcl_sys::ffi::save_ply_file_binary_xyz(path_str, &self.inner)
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

impl PlyIoXYZI for PointCloudXYZI {
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_ply_file_xyzi(path_str, self.inner.pin_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_ply_with_format(path, BinaryFormat::Ascii)
    }

    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_ply_file_ascii_xyzi(path_str, &self.inner),
            BinaryFormat::Binary => pcl_sys::ffi::save_ply_file_binary_xyzi(path_str, &self.inner),
            BinaryFormat::BinaryCompressed => {
                // PLY doesn't support compressed format, fall back to binary
                pcl_sys::ffi::save_ply_file_binary_xyzi(path_str, &self.inner)
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

impl PlyIoXYZRGB for PointCloudXYZRGB {
    fn load_ply<P: AsRef<Path>>(&mut self, path: P) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = pcl_sys::ffi::load_ply_file_xyzrgb(path_str, self.inner.pin_mut());

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to load PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }

    fn save_ply<P: AsRef<Path>>(&self, path: P) -> PclResult<()> {
        self.save_ply_with_format(path, BinaryFormat::Ascii)
    }

    fn save_ply_with_format<P: AsRef<Path>>(&self, path: P, format: BinaryFormat) -> PclResult<()> {
        let path_str = path.as_ref().to_str().ok_or_else(|| {
            PclError::invalid_parameters(
                "File path conversion failed",
                "path",
                "valid UTF-8 string",
                "non-UTF-8 path",
            )
        })?;

        let result = match format {
            BinaryFormat::Ascii => pcl_sys::ffi::save_ply_file_ascii_xyzrgb(path_str, &self.inner),
            BinaryFormat::Binary => {
                pcl_sys::ffi::save_ply_file_binary_xyzrgb(path_str, &self.inner)
            }
            BinaryFormat::BinaryCompressed => {
                // PLY doesn't support compressed format, fall back to binary
                pcl_sys::ffi::save_ply_file_binary_xyzrgb(path_str, &self.inner)
            }
        };

        if result == 0 {
            Ok(())
        } else {
            Err(PclError::IoError {
                message: format!("Failed to save PLY file: {}", path_str),
                path: Some(path.as_ref().to_path_buf()),
                source: None,
            })
        }
    }
}

/// Convenience functions for loading PLY files
pub fn load_ply_xyz<P: AsRef<Path>>(path: P) -> PclResult<PointCloudXYZ> {
    let mut cloud = PointCloudXYZ::new()?;
    cloud.load_ply(path)?;
    Ok(cloud)
}

/// Convenience functions for loading PLY files with PointXYZI
pub fn load_ply_xyzi<P: AsRef<Path>>(path: P) -> PclResult<PointCloudXYZI> {
    let mut cloud = PointCloudXYZI::new()?;
    cloud.load_ply(path)?;
    Ok(cloud)
}

/// Convenience functions for loading PLY files with PointXYZRGB
pub fn load_ply_xyzrgb<P: AsRef<Path>>(path: P) -> PclResult<PointCloudXYZRGB> {
    let mut cloud = PointCloudXYZRGB::new()?;
    cloud.load_ply(path)?;
    Ok(cloud)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_ply_io_xyz() {
        let mut cloud = PointCloudXYZ::new().unwrap();
        cloud.resize(10).unwrap();

        let temp_file = NamedTempFile::new().unwrap();
        let path = temp_file.path();

        // Test saving
        assert!(cloud.save_ply(path).is_ok());
        assert!(path.exists());

        // Test loading
        let mut loaded_cloud = PointCloudXYZ::new().unwrap();
        assert!(loaded_cloud.load_ply(path).is_ok());
        assert_eq!(loaded_cloud.size(), cloud.size());
    }

    #[test]
    fn test_ply_format_options() {
        let mut cloud = PointCloudXYZ::new().unwrap();
        cloud.resize(5).unwrap();

        // Test ASCII format
        let temp_ascii = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_ply_with_format(temp_ascii.path(), BinaryFormat::Ascii)
                .is_ok()
        );

        // Test Binary format
        let temp_binary = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_ply_with_format(temp_binary.path(), BinaryFormat::Binary)
                .is_ok()
        );

        // Test Binary Compressed format (should fall back to binary)
        let temp_compressed = NamedTempFile::new().unwrap();
        assert!(
            cloud
                .save_ply_with_format(temp_compressed.path(), BinaryFormat::BinaryCompressed)
                .is_ok()
        );
    }

    #[test]
    fn test_convenience_functions() {
        let mut cloud = PointCloudXYZ::new().unwrap();
        cloud.resize(3).unwrap();

        let temp_file = NamedTempFile::new().unwrap();
        cloud.save_ply(temp_file.path()).unwrap();

        // Test convenience loading function
        let loaded_cloud = load_ply_xyz(temp_file.path()).unwrap();
        assert_eq!(loaded_cloud.size(), cloud.size());
    }
}
