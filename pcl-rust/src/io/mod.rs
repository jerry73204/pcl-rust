//! Safe I/O operations for point cloud file formats
//!
//! This module provides safe, idiomatic Rust interfaces for reading and writing
//! point cloud files in various formats including PCD and PLY.

pub mod pcd;
pub mod ply;

#[cfg(test)]
mod tests;

// Re-export for convenience
pub use pcd::*;
pub use ply::*;

/// File format enumeration for point cloud files
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileFormat {
    /// Point Cloud Data format
    Pcd,
    /// Polygon File Format
    Ply,
}

/// Binary format options for file output
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BinaryFormat {
    /// ASCII text format
    #[default]
    Ascii,
    /// Binary format
    Binary,
    /// Compressed binary format (PCD only)
    BinaryCompressed,
}

/// Auto-detection functionality for point cloud file formats
pub mod auto_detect {
    use super::FileFormat;
    use crate::common::{PointCloud, XYZ};
    use crate::error::{PclError, Result};
    use pcl_sys::io::{
        detect_file_format, detect_format_from_content, detect_format_from_extension,
        load_point_cloud_auto_xyz,
    };
    use std::path::Path;

    /// Detect file format from file extension
    pub fn detect_format_from_ext<P: AsRef<Path>>(path: P) -> Result<FileFormat> {
        let path_str = path.as_ref().to_string_lossy();
        let format_code = detect_format_from_extension(&path_str);

        match format_code {
            1 => Ok(FileFormat::Pcd),
            2 => Ok(FileFormat::Ply),
            -1 => Err(PclError::IoFailed(format!(
                "Cannot access file: {}",
                path_str
            ))),
            _ => Err(PclError::UnsupportedFormat(format!(
                "Unknown format for file: {}",
                path_str
            ))),
        }
    }

    /// Detect file format from file content
    pub fn detect_format_from_file_content<P: AsRef<Path>>(path: P) -> Result<FileFormat> {
        let path_str = path.as_ref().to_string_lossy();
        let format_code = detect_format_from_content(&path_str);

        match format_code {
            1 => Ok(FileFormat::Pcd),
            2 => Ok(FileFormat::Ply),
            -1 => Err(PclError::IoFailed(format!(
                "Cannot access file: {}",
                path_str
            ))),
            _ => Err(PclError::UnsupportedFormat(format!(
                "Unknown format for file: {}",
                path_str
            ))),
        }
    }

    /// Auto-detect file format (tries extension first, then content)
    pub fn detect_format<P: AsRef<Path>>(path: P) -> Result<FileFormat> {
        let path_str = path.as_ref().to_string_lossy();
        let format_code = detect_file_format(&path_str);

        match format_code {
            1 => Ok(FileFormat::Pcd),
            2 => Ok(FileFormat::Ply),
            -1 => Err(PclError::IoFailed(format!(
                "Cannot access file: {}",
                path_str
            ))),
            _ => Err(PclError::UnsupportedFormat(format!(
                "Unknown format for file: {}",
                path_str
            ))),
        }
    }

    /// Load point cloud with automatic format detection for XYZ points
    pub fn load_xyz<P: AsRef<Path>>(path: P) -> Result<PointCloud<XYZ>> {
        let path_str = path.as_ref().to_string_lossy();
        let mut cloud = PointCloud::<XYZ>::new()?;

        let result = load_point_cloud_auto_xyz(&path_str, cloud.inner_mut());

        if result == 0 {
            Ok(cloud)
        } else {
            Err(PclError::IoFailed(format!(
                "Failed to load point cloud from: {}",
                path_str
            )))
        }
    }
}

// Re-export auto-detection functions for convenience
pub use auto_detect::{detect_format, detect_format_from_ext, detect_format_from_file_content};

/// Load point cloud with automatic format detection for PointXYZ
pub fn load_xyz<P: AsRef<std::path::Path>>(
    path: P,
) -> crate::error::Result<crate::common::PointCloud<crate::common::XYZ>> {
    auto_detect::load_xyz(path)
}

// Re-export PCD functions for convenience
pub use pcd::{load_pcd_xyz as load_pcd, load_pcd_xyzi, load_pcd_xyzrgb};

// Re-export PLY functions for convenience
pub use ply::{load_ply_xyz as load_ply, load_ply_xyzi, load_ply_xyzrgb};

/// Save PointCloud<XYZ> to PCD file
pub fn save_pcd<P: AsRef<std::path::Path>>(
    cloud: &crate::common::PointCloud<crate::common::XYZ>,
    path: P,
) -> crate::error::PclResult<()> {
    cloud.save_pcd(path)
}

/// Save PointCloud<XYZ> to PLY file
pub fn save_ply<P: AsRef<std::path::Path>>(
    cloud: &crate::common::PointCloud<crate::common::XYZ>,
    path: P,
) -> crate::error::PclResult<()> {
    cloud.save_ply(path)
}
