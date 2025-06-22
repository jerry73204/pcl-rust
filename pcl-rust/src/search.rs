//! Search algorithms and spatial queries
//!
//! This module provides safe wrappers around PCL's search interfaces
//! for nearest neighbor and radius-based spatial queries.

pub mod generic;
pub mod traits;
pub mod unified;

// Point types are re-exported through generic module
// Unused imports removed

pub use generic::KdTree;
pub use traits::{NearestNeighborSearch, SearchConfiguration, SearchInputCloud, SearchMethod};
pub use unified::{SearchXYZ, SearchXYZRGB};

// Deprecated type aliases have been removed - use KdTree<PointType> directly
