//! Visualization module for Point Cloud Library
//!
//! This module provides direct FFI bindings to PCL's visualization functionality,
//! including the PCLVisualizer and CloudViewer classes.

#[cfg(feature = "visualization")]
pub use crate::ffi::{CloudViewer, PCLVisualizer};

// Re-export visualization functions from the FFI
#[cfg(feature = "visualization")]
pub use crate::ffi::{
    add_coordinate_system, add_point_cloud_xyz, add_point_cloud_xyzrgb, add_sphere_xyz, add_text,
    close, cloud_viewer_was_stopped, new_cloud_viewer, new_pcl_visualizer,
    register_keyboard_callback, remove_point_cloud, remove_shape, reset_camera, reset_stopped_flag,
    set_background_color, set_camera_position, set_point_cloud_color_xyz,
    set_point_cloud_render_properties_xyz, show_cloud_xyz, show_cloud_xyzrgb, spin, spin_once,
    update_point_cloud_xyz, update_point_cloud_xyzrgb, wait_for_cloud_viewer, was_stopped,
};

// Common PCL visualization property constants
// These correspond to pcl::visualization::PCL_VISUALIZER_* constants
pub mod properties {
    pub const POINT_SIZE: i32 = 0;
    pub const OPACITY: i32 = 1;
    pub const LINE_WIDTH: i32 = 2;
    pub const FONT_SIZE: i32 = 3;
    pub const COLOR: i32 = 4;
    pub const REPRESENTATION: i32 = 5;
    pub const IMMEDIATE_RENDERING: i32 = 6;
    pub const SHADING: i32 = 7;
}

// Representation modes
pub mod representation {
    pub const POINTS: i32 = 0;
    pub const WIREFRAME: i32 = 1;
    pub const SURFACE: i32 = 2;
}

// Shading modes
pub mod shading {
    pub const FLAT: i32 = 0;
    pub const GOURAUD: i32 = 1;
    pub const PHONG: i32 = 2;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_property_constants() {
        // Just verify the constants are accessible
        assert_eq!(properties::POINT_SIZE, 0);
        assert_eq!(properties::OPACITY, 1);
        assert_eq!(properties::LINE_WIDTH, 2);
    }

    #[test]
    fn test_representation_constants() {
        assert_eq!(representation::POINTS, 0);
        assert_eq!(representation::WIREFRAME, 1);
        assert_eq!(representation::SURFACE, 2);
    }

    #[test]
    fn test_shading_constants() {
        assert_eq!(shading::FLAT, 0);
        assert_eq!(shading::GOURAUD, 1);
        assert_eq!(shading::PHONG, 2);
    }
}
