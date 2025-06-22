//! Migration utilities for transitioning from cxx::UniquePtr to raw pointer FFI
//!
//! This module provides tools and utilities to help migrate from the problematic
//! cxx::UniquePtr approach to the memory-safe raw pointer approach that respects
//! PCL's EIGEN_ALIGN16 requirements.

use crate::{ffi, ptr::PclPtr};
use cxx::UniquePtr;

/// Test helpers for migration verification
pub mod test_helpers {
    use super::*;

    /// Compare a cxx::UniquePtr PointCloud with a PclPtr PointCloud to ensure they're equivalent
    pub fn compare_cloud_equivalence(
        old: &UniquePtr<ffi::PointCloud_PointXYZ>,
        new: &PclPtr<ffi::PointCloud_PointXYZ>,
    ) -> bool {
        let old_size = ffi::size(old);
        let new_size = unsafe { ffi::size(new.as_ref()) };

        old_size == new_size && ffi::empty(old) == new.empty()
    }
}

#[cfg(test)]
mod tests {
    use super::test_helpers::*;
    use super::*;
    use crate::{ptr::TransferOwnership, raw_ffi::pcl_pointcloud_xyz_new};

    #[test]
    fn test_migration_from_unique_ptr() {
        // Test that we can migrate from UniquePtr to PclPtr approach
        let old_cloud = ffi::new_point_cloud_xyz();

        unsafe {
            let raw_ptr = pcl_pointcloud_xyz_new();
            let new_cloud = PclPtr::take_ownership(raw_ptr).expect("Failed to take ownership");

            // Both should be empty and equivalent
            assert!(compare_cloud_equivalence(&old_cloud, &new_cloud));
        }
    }

    #[test]
    fn test_cloud_equivalence() {
        // Test the helper function itself
        let cloud1 = ffi::new_point_cloud_xyz();

        unsafe {
            let raw_ptr = pcl_pointcloud_xyz_new();
            let cloud2 = PclPtr::take_ownership(raw_ptr).expect("Failed to take ownership");

            assert!(compare_cloud_equivalence(&cloud1, &cloud2));
        }
    }

    #[test]
    fn test_new_approach_works() {
        // Test that the new approach works correctly
        unsafe {
            let raw_ptr = pcl_pointcloud_xyz_new();
            assert!(!raw_ptr.is_null());

            let cloud = PclPtr::take_ownership(raw_ptr).expect("Failed to take ownership");
            assert!(cloud.empty());
            assert_eq!(cloud.size(), 0);

            // PclPtr should clean up automatically when dropped
        }
    }
}
