//! Generic visualization traits and implementations
//!
//! This module provides generic traits for visualization that work with any point type,
//! allowing for unified visualization APIs across different point cloud types.

use crate::common::PointCloud;
use crate::common::point_types::PointType;
use crate::error::PclResult;
use crate::traits::Point;
use cxx::memory::UniquePtrTarget;

/// Generic trait for visualizing point clouds of any type
pub trait Viewer<T: PointType>
where
    T::CloudType: UniquePtrTarget,
{
    /// Display a point cloud with the given identifier
    fn show_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()>;

    /// Update an existing point cloud in the viewer
    fn update_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()>;

    /// Remove a point cloud from the viewer
    fn remove_cloud(&mut self, id: &str) -> PclResult<()>;
}

/// Trait for advanced visualization features
pub trait AdvancedViewer<T: PointType>: Viewer<T>
where
    T::CloudType: UniquePtrTarget,
{
    /// Set the point size for a specific cloud
    fn set_point_size(&mut self, id: &str, size: f64) -> PclResult<()>;

    /// Set the opacity for a specific cloud (0.0 = transparent, 1.0 = opaque)
    fn set_opacity(&mut self, id: &str, opacity: f64) -> PclResult<()>;

    /// Set the color for all points in a cloud (RGB values 0.0-1.0)
    fn set_cloud_color(&mut self, id: &str, r: f64, g: f64, b: f64) -> PclResult<()>;
}

/// Trait for adding geometric shapes to the visualization
pub trait ShapeVisualization {
    /// Add a line between two points
    fn add_line(&mut self, p1: (f32, f32, f32), p2: (f32, f32, f32), id: &str) -> PclResult<()>;

    /// Add a sphere at a specific position
    fn add_sphere(
        &mut self,
        center: (f32, f32, f32),
        radius: f64,
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()>;

    /// Add a cube/box shape
    fn add_cube(
        &mut self,
        min: (f32, f32, f32),
        max: (f32, f32, f32),
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()>;

    /// Add an arrow between two points
    fn add_arrow(
        &mut self,
        start: (f32, f32, f32),
        end: (f32, f32, f32),
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()>;
}

/// Trait for viewport management in multi-viewport visualizations
pub trait ViewportControl {
    /// Create a new viewport
    fn create_viewport(&mut self, xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> PclResult<i32>;

    /// Set the active viewport for subsequent operations
    fn set_viewport(&mut self, viewport_id: i32) -> PclResult<()>;

    /// Add a point cloud to a specific viewport
    fn add_cloud_to_viewport<T: Point + PointType>(
        &mut self,
        cloud: &PointCloud<T>,
        id: &str,
        viewport_id: i32,
    ) -> PclResult<()>
    where
        <T as PointType>::CloudType: UniquePtrTarget;
}

/// Trait for interactive features
pub trait InteractiveViewer {
    /// Register a callback for keyboard events
    fn register_keyboard_callback<F>(&mut self, callback: F) -> PclResult<()>
    where
        F: FnMut(char) + 'static;

    /// Register a callback for mouse events
    fn register_mouse_callback<F>(&mut self, callback: F) -> PclResult<()>
    where
        F: FnMut(f64, f64, i32) + 'static;

    /// Register a callback for point picking events
    fn register_point_picking_callback<F>(&mut self, callback: F) -> PclResult<()>
    where
        F: FnMut(f32, f32, f32, usize) + 'static;
}

/// Trait for handling point clouds with normals
pub trait NormalVisualization<T: Point + PointType>
where
    <T as PointType>::CloudType: UniquePtrTarget,
{
    /// Display normals as lines for each point
    fn add_normals(
        &mut self,
        cloud: &PointCloud<T>,
        normals: &PointCloud<T>,
        level: i32,
        scale: f64,
        id: &str,
    ) -> PclResult<()>;

    /// Remove normals visualization
    fn remove_normals(&mut self, id: &str) -> PclResult<()>;
}

/// Configuration for visualization properties
#[derive(Debug, Clone)]
pub struct VisualizationConfig {
    /// Window title
    pub window_name: String,
    /// Background color (RGB)
    pub background_color: (f64, f64, f64),
    /// Default point size
    pub point_size: f64,
    /// Enable coordinate system display
    pub show_coordinate_system: bool,
    /// Coordinate system scale
    pub coordinate_system_scale: f64,
    /// Enable FPS display
    pub show_fps: bool,
    /// Window size (width, height)
    pub window_size: Option<(i32, i32)>,
    /// Initial camera position
    pub camera_position: Option<CameraPosition>,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            window_name: "PCL Viewer".to_string(),
            background_color: (0.0, 0.0, 0.0),
            point_size: 1.0,
            show_coordinate_system: false,
            coordinate_system_scale: 1.0,
            show_fps: false,
            window_size: None,
            camera_position: None,
        }
    }
}

/// Camera position and orientation
#[derive(Debug, Clone, Copy)]
pub struct CameraPosition {
    /// Camera position (x, y, z)
    pub position: (f64, f64, f64),
    /// View direction/focal point (x, y, z)
    pub focal_point: (f64, f64, f64),
    /// Up vector (x, y, z)
    pub up_vector: (f64, f64, f64),
}

impl Default for CameraPosition {
    fn default() -> Self {
        Self {
            position: (0.0, 0.0, 2.0),
            focal_point: (0.0, 0.0, 0.0),
            up_vector: (0.0, 1.0, 0.0),
        }
    }
}

/// Builder for creating visualization configurations
pub struct VisualizationConfigBuilder {
    config: VisualizationConfig,
}

impl VisualizationConfigBuilder {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            config: VisualizationConfig::default(),
        }
    }

    /// Set the window name
    pub fn window_name(mut self, name: impl Into<String>) -> Self {
        self.config.window_name = name.into();
        self
    }

    /// Set the background color (RGB values 0.0-1.0)
    pub fn background_color(mut self, r: f64, g: f64, b: f64) -> Self {
        self.config.background_color = (r, g, b);
        self
    }

    /// Set the default point size
    pub fn point_size(mut self, size: f64) -> Self {
        self.config.point_size = size;
        self
    }

    /// Enable/disable coordinate system display
    pub fn show_coordinate_system(mut self, show: bool) -> Self {
        self.config.show_coordinate_system = show;
        self
    }

    /// Set the coordinate system scale
    pub fn coordinate_system_scale(mut self, scale: f64) -> Self {
        self.config.coordinate_system_scale = scale;
        self
    }

    /// Enable/disable FPS display
    pub fn show_fps(mut self, show: bool) -> Self {
        self.config.show_fps = show;
        self
    }

    /// Set the window size
    pub fn window_size(mut self, width: i32, height: i32) -> Self {
        self.config.window_size = Some((width, height));
        self
    }

    /// Set the initial camera position
    pub fn camera_position(mut self, camera: CameraPosition) -> Self {
        self.config.camera_position = Some(camera);
        self
    }

    /// Build the configuration
    pub fn build(self) -> VisualizationConfig {
        self.config
    }
}

impl Default for VisualizationConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_visualization_config_builder() {
        let config = VisualizationConfigBuilder::new()
            .window_name("Test Window")
            .background_color(0.1, 0.2, 0.3)
            .point_size(2.0)
            .show_coordinate_system(true)
            .coordinate_system_scale(0.5)
            .show_fps(true)
            .window_size(800, 600)
            .camera_position(CameraPosition {
                position: (1.0, 2.0, 3.0),
                focal_point: (0.0, 0.0, 0.0),
                up_vector: (0.0, 1.0, 0.0),
            })
            .build();

        assert_eq!(config.window_name, "Test Window");
        assert_eq!(config.background_color, (0.1, 0.2, 0.3));
        assert_eq!(config.point_size, 2.0);
        assert!(config.show_coordinate_system);
        assert_eq!(config.coordinate_system_scale, 0.5);
        assert!(config.show_fps);
        assert_eq!(config.window_size, Some((800, 600)));
        assert!(config.camera_position.is_some());
    }

    #[test]
    fn test_camera_position_default() {
        let camera = CameraPosition::default();
        assert_eq!(camera.position, (0.0, 0.0, 2.0));
        assert_eq!(camera.focal_point, (0.0, 0.0, 0.0));
        assert_eq!(camera.up_vector, (0.0, 1.0, 0.0));
    }
}
