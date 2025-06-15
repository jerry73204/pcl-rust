//! Comprehensive tests for error handling

#[cfg(test)]
mod tests {
    #[cfg(feature = "octree")]
    use crate::error::OctreeOperation;
    use crate::error::PclError;
    #[cfg(feature = "search")]
    use crate::error::SearchOperation;
    #[cfg(feature = "octree")]
    use crate::octree::{OctreeSearchXYZ, OctreeVoxelCentroidXYZ};
    #[cfg(feature = "search")]
    use crate::search::{KdTreeXYZ, SearchConfiguration};

    #[test]
    fn test_invalid_parameters_error() {
        let err = PclError::invalid_parameters("test message", "test_param", "positive", "-1");

        match &err {
            PclError::InvalidParameters {
                message,
                parameter,
                expected,
                actual,
            } => {
                assert_eq!(message, "test message");
                assert_eq!(parameter, "test_param");
                assert_eq!(expected, "positive");
                assert_eq!(actual, "-1");
            }
            _ => panic!("Expected InvalidParameters error"),
        }

        // Test suggestion
        assert_eq!(
            err.suggestion(),
            Some("Parameter 'test_param' should be positive".to_string())
        );
    }

    #[test]
    fn test_not_implemented_error() {
        let err = PclError::not_implemented("test feature", Some("use alternative".to_string()));

        match &err {
            PclError::NotImplemented {
                feature,
                workaround,
            } => {
                assert_eq!(feature, "test feature");
                assert_eq!(workaround, &Some("use alternative".to_string()));
            }
            _ => panic!("Expected NotImplemented error"),
        }

        // Test suggestion
        assert_eq!(err.suggestion(), Some("use alternative".to_string()));
    }

    #[test]
    fn test_invalid_state_error() {
        let err = PclError::invalid_state(
            "operation on uninitialized object",
            "initialized",
            "uninitialized",
        );

        match &err {
            PclError::InvalidState {
                message,
                expected_state,
                actual_state,
            } => {
                assert_eq!(message, "operation on uninitialized object");
                assert_eq!(expected_state, "initialized");
                assert_eq!(actual_state, "uninitialized");
            }
            _ => panic!("Expected InvalidState error"),
        }

        // Test suggestion
        assert_eq!(
            err.suggestion(),
            Some("Ensure the object is in 'initialized' state before this operation".to_string())
        );
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_search_failed_error() {
        let err =
            PclError::search_failed("search operation failed", SearchOperation::NearestKSearch);

        match err {
            PclError::SearchFailed {
                message, operation, ..
            } => {
                assert_eq!(message, "search operation failed");
                assert_eq!(operation, SearchOperation::NearestKSearch);
            }
            _ => panic!("Expected SearchFailed error"),
        }
    }

    #[test]
    #[cfg(feature = "octree")]
    fn test_octree_failed_error() {
        let err =
            PclError::octree_failed("octree construction failed", OctreeOperation::Construction);

        match err {
            PclError::OctreeFailed {
                message, operation, ..
            } => {
                assert_eq!(message, "octree construction failed");
                assert_eq!(operation, OctreeOperation::Construction);
            }
            _ => panic!("Expected OctreeFailed error"),
        }
    }

    #[test]
    #[cfg(feature = "octree")]
    fn test_octree_invalid_resolution() {
        let result = OctreeSearchXYZ::new(-1.0);
        assert!(result.is_err());

        let err = result.unwrap_err();
        match &err {
            PclError::InvalidParameters {
                parameter,
                expected,
                actual,
                ..
            } => {
                assert_eq!(parameter, "resolution");
                assert_eq!(expected, "positive value");
                assert_eq!(actual, "-1");
            }
            _ => panic!("Expected InvalidParameters error"),
        }

        assert!(err.suggestion().is_some());
    }

    #[test]
    #[cfg(feature = "octree")]
    fn test_octree_voxel_centroid_invalid_resolution() {
        let result = OctreeVoxelCentroidXYZ::new(0.0);
        assert!(result.is_err());

        let err = result.unwrap_err();
        match err {
            PclError::InvalidParameters { parameter, .. } => {
                assert_eq!(parameter, "resolution");
            }
            _ => panic!("Expected InvalidParameters error"),
        }
    }

    // Note: Tests requiring point creation are skipped due to cxx FFI limitations
    // Points must be created and managed by PCL C++ code

    #[test]
    #[cfg(feature = "octree")]
    fn test_voxel_centroid_operations_without_cloud() {
        let mut octree = OctreeVoxelCentroidXYZ::new(0.1).unwrap();

        // Test add_points_from_input_cloud without cloud
        let result = octree.add_points_from_input_cloud();
        assert!(result.is_err());

        let err = result.unwrap_err();
        match err {
            PclError::InvalidState {
                expected_state,
                actual_state,
                ..
            } => {
                assert_eq!(expected_state, "input cloud set");
                assert_eq!(actual_state, "no input cloud");
            }
            _ => panic!("Expected InvalidState error"),
        }
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_search_configuration_invalid_epsilon() {
        let mut searcher = KdTreeXYZ::new().unwrap();

        // Test negative epsilon
        let result = searcher.set_epsilon(-1.0);
        assert!(result.is_err());

        let err = result.unwrap_err();
        match &err {
            PclError::InvalidParameters {
                parameter,
                expected,
                actual,
                ..
            } => {
                assert_eq!(parameter, "epsilon");
                assert_eq!(expected, "non-negative value");
                assert_eq!(actual, "-1");
            }
            _ => panic!("Expected InvalidParameters error"),
        }
    }

    #[test]
    fn test_error_display() {
        let err = PclError::invalid_parameters("test message", "param", "positive", "negative");

        let display_str = format!("{}", err);
        assert!(display_str.contains("Invalid parameters"));
        assert!(display_str.contains("test message"));
    }

    #[test]
    fn test_error_debug() {
        let err = PclError::invalid_point_cloud("test cloud error");

        let debug_str = format!("{:?}", err);
        assert!(debug_str.contains("InvalidPointCloud"));
        assert!(debug_str.contains("test cloud error"));
    }

    #[test]
    fn test_memory_allocation_error() {
        let err = PclError::MemoryAllocation { size: 1024 };

        assert_eq!(
            err.suggestion(),
            Some(
                "Try reducing the allocation size or freeing memory. Requested: 1024 bytes"
                    .to_string()
            )
        );
    }

    #[test]
    #[cfg(feature = "search")]
    fn test_search_set_input_cloud_suggestion() {
        let err =
            PclError::search_failed("failed to set input cloud", SearchOperation::SetInputCloud);

        assert_eq!(
            err.suggestion(),
            Some("Ensure the point cloud is not empty and contains valid points".to_string())
        );
    }

    #[test]
    fn test_error_chaining() {
        let source_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let pcl_err = PclError::invalid_point_cloud("failed to load cloud").with_source(source_err);

        match pcl_err {
            PclError::InvalidPointCloud { source, .. } => {
                assert!(source.is_some());
            }
            _ => panic!("Expected InvalidPointCloud error"),
        }
    }

    #[test]
    fn test_io_error_conversion() {
        let io_err = std::io::Error::new(std::io::ErrorKind::PermissionDenied, "access denied");
        let pcl_err: PclError = io_err.into();

        match pcl_err {
            PclError::IoError {
                message, source, ..
            } => {
                assert!(message.contains("access denied"));
                assert!(source.is_some());
            }
            _ => panic!("Expected IoError"),
        }
    }
}
