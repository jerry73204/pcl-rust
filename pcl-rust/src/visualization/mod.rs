//! 3D Visualization for Point Clouds
//!
//! This module provides safe Rust interfaces for PCL's visualization functionality,
//! including the PCLVisualizer and CloudViewer classes for 3D point cloud visualization.
//!
//! ## Features
//!
//! - **PCLVisualizer**: Full-featured 3D visualization with camera controls, multiple viewports,
//!   and comprehensive rendering options
//! - **CloudViewer**: Simplified interface for quick point cloud visualization
//! - **Rendering Properties**: Control point size, opacity, colors, and representation modes
//! - **Interactive Controls**: Camera positioning, keyboard callbacks, and user interaction
//!
//! ## Usage Examples
//!
//! ### Basic Cloud Visualization with CloudViewer
//! ```rust,no_run
//! use pcl::{PointCloudXYZ, CloudViewer};
//!
//! let cloud = PointCloudXYZ::new()?;
//! let mut viewer = CloudViewer::new("Simple Viewer")?;
//! viewer.show_cloud(&cloud, "my_cloud")?;
//! viewer.wait_until_stopped();
//! # Ok::<(), pcl::PclError>(())
//! ```
//!
//! ### Advanced Visualization with PCLVisualizer
//! ```rust,no_run
//! use pcl::{PointCloudXYZ, PclVisualizer, RenderingProperties, Representation};
//!
//! let cloud = PointCloudXYZ::new()?;
//! let mut viewer = PclVisualizer::new("Advanced Viewer")?;
//!
//! // Add point cloud with custom properties
//! viewer.add_point_cloud(&cloud, "cloud1")?;
//! viewer.set_point_size("cloud1", 2.0)?;
//! viewer.set_point_color("cloud1", 1.0, 0.0, 0.0)?; // Red
//!
//! // Add coordinate system and text
//! viewer.add_coordinate_system(1.0, "coords")?;
//! viewer.add_text("My Point Cloud", 10, 10, 1.0, 1.0, 1.0, "title")?;
//!
//! // Set camera and background
//! viewer.set_background_color(0.1, 0.1, 0.1)?;
//! viewer.set_camera_position(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)?;
//!
//! // Start visualization loop
//! while !viewer.was_stopped() {
//!     viewer.spin_once(100)?;
//! }
//! # Ok::<(), pcl::PclError>(())
//! ```

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::PclResult;

pub mod cloud_viewer;
pub mod pcl_visualizer;
pub mod properties;

pub use cloud_viewer::{CloudViewer, CloudViewerBuilder};
pub use pcl_visualizer::{PclVisualizer, PclVisualizerBuilder};
pub use properties::{RenderingProperties, Representation, Shading};

/// Generic trait for point cloud viewers
pub trait ViewerXYZ {
    /// Display a PointXYZ cloud with the given name
    fn show_cloud(&mut self, cloud: &PointCloudXYZ, name: &str) -> PclResult<()>;
}

/// Generic trait for RGB point cloud viewers  
pub trait ViewerXYZRGB {
    /// Display a PointXYZRGB cloud with the given name
    fn show_cloud_rgb(&mut self, cloud: &PointCloudXYZRGB, name: &str) -> PclResult<()>;
}

/// Common camera control interface
pub trait CameraControl {
    /// Set the camera position and orientation
    #[allow(clippy::too_many_arguments)]
    fn set_camera_position(
        &mut self,
        pos_x: f64,
        pos_y: f64,
        pos_z: f64,
        view_x: f64,
        view_y: f64,
        view_z: f64,
        up_x: f64,
        up_y: f64,
        up_z: f64,
    ) -> PclResult<()>;

    /// Reset camera to default position
    fn reset_camera(&mut self) -> PclResult<()>;
}

/// Common visualization controls
pub trait VisualizationControl {
    /// Check if the viewer window was closed
    fn was_stopped(&self) -> bool;

    /// Set the background color (RGB values 0.0-1.0)
    fn set_background_color(&mut self, r: f64, g: f64, b: f64) -> PclResult<()>;

    /// Render one frame and process events
    fn spin_once(&mut self, time_ms: i32) -> PclResult<()>;

    /// Start the main visualization loop (blocking)
    fn spin(&mut self);

    /// Close the viewer window
    fn close(&mut self);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_property_constants() {
        // Test that property constants are accessible
        assert_eq!(RenderingProperties::PointSize as i32, 0);
        assert_eq!(RenderingProperties::Opacity as i32, 1);
        assert_eq!(RenderingProperties::LineWidth as i32, 2);
    }

    #[test]
    fn test_representation_constants() {
        assert_eq!(Representation::Points as i32, 0);
        assert_eq!(Representation::Wireframe as i32, 1);
        assert_eq!(Representation::Surface as i32, 2);
    }

    #[test]
    fn test_shading_constants() {
        assert_eq!(Shading::Flat as i32, 0);
        assert_eq!(Shading::Gouraud as i32, 1);
        assert_eq!(Shading::Phong as i32, 2);
    }
}
