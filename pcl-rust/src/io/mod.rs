//! Safe I/O operations for point cloud file formats
//!
//! This module provides safe, idiomatic Rust interfaces for reading and writing
//! point cloud files in various formats including PCD and PLY.

pub mod pcd;
pub mod ply;

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinaryFormat {
    /// ASCII text format
    Ascii,
    /// Binary format
    Binary,
    /// Compressed binary format (PCD only)
    BinaryCompressed,
}

impl Default for BinaryFormat {
    fn default() -> Self {
        BinaryFormat::Ascii
    }
}
