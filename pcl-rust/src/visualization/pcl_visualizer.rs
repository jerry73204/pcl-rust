//! Advanced 3D visualization with full PCLVisualizer functionality
//!
//! PclVisualizer provides comprehensive 3D visualization capabilities including
//! multiple point clouds, shapes, text overlays, camera control, and interactive features.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB, PointXYZ};
use crate::error::{PclError, PclResult};
use crate::visualization::{
    CameraControl, RenderingProperties, ViewerXYZ, ViewerXYZRGB, VisualizationControl,
};
use pcl_sys::{UniquePtr, ffi};

/// Advanced 3D point cloud visualizer with full feature set
pub struct PclVisualizer {
    inner: UniquePtr<ffi::PCLVisualizer>,
}

impl PclVisualizer {
    /// Create a new PCLVisualizer with the specified window name
    pub fn new(window_name: &str) -> PclResult<Self> {
        let inner = ffi::new_pcl_visualizer(window_name);
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "PCLVisualizer".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Add a PointXYZ cloud to the viewer
    pub fn add_point_cloud_xyz(&mut self, cloud: &PointCloudXYZ, id: &str) -> PclResult<()> {
        let result = ffi::add_point_cloud_xyz(self.inner.pin_mut(), cloud.as_raw(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to add PointXYZ cloud '{}'", id),
                operation: "add_point_cloud_xyz".to_string(),
            });
        }
        Ok(())
    }

    /// Add a PointXYZRGB cloud to the viewer
    pub fn add_point_cloud_xyzrgb(&mut self, cloud: &PointCloudXYZRGB, id: &str) -> PclResult<()> {
        let result = ffi::add_point_cloud_xyzrgb(self.inner.pin_mut(), cloud.as_raw(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to add PointXYZRGB cloud '{}'", id),
                operation: "add_point_cloud_xyzrgb".to_string(),
            });
        }
        Ok(())
    }

    /// Update an existing PointXYZ cloud in the viewer
    pub fn update_point_cloud_xyz(&mut self, cloud: &PointCloudXYZ, id: &str) -> PclResult<()> {
        let result = ffi::update_point_cloud_xyz(self.inner.pin_mut(), cloud.as_raw(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to update PointXYZ cloud '{}'", id),
                operation: "update_point_cloud_xyz".to_string(),
            });
        }
        Ok(())
    }

    /// Update an existing PointXYZRGB cloud in the viewer
    pub fn update_point_cloud_xyzrgb(
        &mut self,
        cloud: &PointCloudXYZRGB,
        id: &str,
    ) -> PclResult<()> {
        let result = ffi::update_point_cloud_xyzrgb(self.inner.pin_mut(), cloud.as_raw(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to update PointXYZRGB cloud '{}'", id),
                operation: "update_point_cloud_xyzrgb".to_string(),
            });
        }
        Ok(())
    }

    /// Remove a point cloud from the viewer
    pub fn remove_point_cloud(&mut self, id: &str) -> PclResult<()> {
        let result = ffi::remove_point_cloud(self.inner.pin_mut(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to remove point cloud '{}'", id),
                operation: "remove_point_cloud".to_string(),
            });
        }
        Ok(())
    }

    /// Set rendering properties for a point cloud
    pub fn set_point_cloud_render_property(
        &mut self,
        property: RenderingProperties,
        value: f64,
        id: &str,
    ) -> PclResult<()> {
        let result = ffi::set_point_cloud_render_properties_xyz(
            self.inner.pin_mut(),
            property.as_i32(),
            value,
            id,
        );
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to set property {:?} for cloud '{}'", property, id),
                operation: "set_point_cloud_render_property".to_string(),
            });
        }
        Ok(())
    }

    /// Set the point size for a point cloud
    pub fn set_point_size(&mut self, id: &str, size: f64) -> PclResult<()> {
        self.set_point_cloud_render_property(RenderingProperties::PointSize, size, id)
    }

    /// Set the opacity for a point cloud (0.0 = transparent, 1.0 = opaque)
    pub fn set_opacity(&mut self, id: &str, opacity: f64) -> PclResult<()> {
        self.set_point_cloud_render_property(RenderingProperties::Opacity, opacity, id)
    }

    /// Set the color for a point cloud (RGB values 0.0-1.0)
    pub fn set_point_color(&mut self, id: &str, r: f64, g: f64, b: f64) -> PclResult<()> {
        let result = ffi::set_point_cloud_color_xyz(self.inner.pin_mut(), r, g, b, id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to set color for cloud '{}'", id),
                operation: "set_point_color".to_string(),
            });
        }
        Ok(())
    }

    /// Add a coordinate system to the viewer
    pub fn add_coordinate_system(&mut self, scale: f64, id: &str) -> PclResult<()> {
        let result = ffi::add_coordinate_system(self.inner.pin_mut(), scale, id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to add coordinate system '{}'", id),
                operation: "add_coordinate_system".to_string(),
            });
        }
        Ok(())
    }

    /// Add text overlay to the viewer
    #[allow(clippy::too_many_arguments)]
    pub fn add_text(
        &mut self,
        text: &str,
        x: i32,
        y: i32,
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()> {
        let result = ffi::add_text(self.inner.pin_mut(), text, x, y, r, g, b, id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to add text '{}'", id),
                operation: "add_text".to_string(),
            });
        }
        Ok(())
    }

    /// Add a sphere to the viewer
    pub fn add_sphere(
        &mut self,
        center: &PointXYZ,
        radius: f64,
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()> {
        let result = ffi::add_sphere_xyz(self.inner.pin_mut(), &center.inner, radius, r, g, b, id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to add sphere '{}'", id),
                operation: "add_sphere".to_string(),
            });
        }
        Ok(())
    }

    /// Remove a shape (text, sphere, coordinate system, etc.) from the viewer
    pub fn remove_shape(&mut self, id: &str) -> PclResult<()> {
        let result = ffi::remove_shape(self.inner.pin_mut(), id);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to remove shape '{}'", id),
                operation: "remove_shape".to_string(),
            });
        }
        Ok(())
    }

    /// Register keyboard callback for user interaction
    pub fn register_keyboard_callback(&mut self) -> PclResult<()> {
        let result = ffi::register_keyboard_callback(self.inner.pin_mut());
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: "Failed to register keyboard callback".to_string(),
                operation: "register_keyboard_callback".to_string(),
            });
        }
        Ok(())
    }

    /// Reset the stopped flag (allows restarting after stop)
    pub fn reset_stopped_flag(&mut self) {
        ffi::reset_stopped_flag(self.inner.pin_mut())
    }
}

impl ViewerXYZ for PclVisualizer {
    fn show_cloud(&mut self, cloud: &PointCloudXYZ, name: &str) -> PclResult<()> {
        self.add_point_cloud_xyz(cloud, name)
    }
}

impl ViewerXYZRGB for PclVisualizer {
    fn show_cloud_rgb(&mut self, cloud: &PointCloudXYZRGB, name: &str) -> PclResult<()> {
        self.add_point_cloud_xyzrgb(cloud, name)
    }
}

impl CameraControl for PclVisualizer {
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
    ) -> PclResult<()> {
        let result = ffi::set_camera_position(
            self.inner.pin_mut(),
            pos_x,
            pos_y,
            pos_z,
            view_x,
            view_y,
            view_z,
            up_x,
            up_y,
            up_z,
        );
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: "Failed to set camera position".to_string(),
                operation: "set_camera_position".to_string(),
            });
        }
        Ok(())
    }

    fn reset_camera(&mut self) -> PclResult<()> {
        let result = ffi::reset_camera(self.inner.pin_mut());
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: "Failed to reset camera".to_string(),
                operation: "reset_camera".to_string(),
            });
        }
        Ok(())
    }
}

impl VisualizationControl for PclVisualizer {
    fn was_stopped(&self) -> bool {
        ffi::was_stopped(&self.inner)
    }

    fn set_background_color(&mut self, r: f64, g: f64, b: f64) -> PclResult<()> {
        let result = ffi::set_background_color(self.inner.pin_mut(), r, g, b);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: "Failed to set background color".to_string(),
                operation: "set_background_color".to_string(),
            });
        }
        Ok(())
    }

    fn spin_once(&mut self, time_ms: i32) -> PclResult<()> {
        let result = ffi::spin_once(self.inner.pin_mut(), time_ms);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: "Failed to spin once".to_string(),
                operation: "spin_once".to_string(),
            });
        }
        Ok(())
    }

    fn spin(&mut self) {
        ffi::spin(self.inner.pin_mut())
    }

    fn close(&mut self) {
        ffi::close(self.inner.pin_mut())
    }
}

impl Default for PclVisualizer {
    fn default() -> Self {
        Self::new("PCLVisualizer").expect("Failed to create default PCLVisualizer")
    }
}

/// Builder for PclVisualizer configuration
pub struct PclVisualizerBuilder {
    window_name: String,
    background_color: Option<(f64, f64, f64)>,
    coordinate_system: Option<(f64, String)>,
}

impl PclVisualizerBuilder {
    /// Create a new PclVisualizer builder
    pub fn new() -> Self {
        Self {
            window_name: "PCLVisualizer".to_string(),
            background_color: None,
            coordinate_system: None,
        }
    }

    /// Set the window name
    pub fn window_name(mut self, name: &str) -> Self {
        self.window_name = name.to_string();
        self
    }

    /// Set the background color
    pub fn background_color(mut self, r: f64, g: f64, b: f64) -> Self {
        self.background_color = Some((r, g, b));
        self
    }

    /// Add a coordinate system with specified scale
    pub fn coordinate_system(mut self, scale: f64, id: &str) -> Self {
        self.coordinate_system = Some((scale, id.to_string()));
        self
    }

    /// Build the PclVisualizer with configured options
    pub fn build(self) -> PclResult<PclVisualizer> {
        let mut viewer = PclVisualizer::new(&self.window_name)?;

        if let Some((r, g, b)) = self.background_color {
            viewer.set_background_color(r, g, b)?;
        }

        if let Some((scale, id)) = self.coordinate_system {
            viewer.add_coordinate_system(scale, &id)?;
        }

        Ok(viewer)
    }
}

impl Default for PclVisualizerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pcl_visualizer_creation() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            eprintln!("Skipping visualization test - no display available");
            return;
        }

        // Note: This test checks object creation but doesn't open a window
        let viewer = PclVisualizer::new("Test Viewer");

        // Check if creation succeeds or fails due to missing display
        match viewer {
            Ok(_) => {
                println!("PclVisualizer created successfully");
            }
            Err(PclError::CreationFailed { .. }) => {
                println!("PclVisualizer creation failed (expected in headless environment)");
            }
            Err(e) => {
                panic!("Unexpected error: {:?}", e);
            }
        }
    }

    #[test]
    fn test_pcl_visualizer_builder() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            eprintln!("Skipping visualization test - no display available");
            return;
        }

        let builder = PclVisualizerBuilder::new()
            .window_name("Custom Window")
            .background_color(0.1, 0.1, 0.1)
            .coordinate_system(1.0, "coords");

        // Test that builder doesn't panic
        let _viewer = builder.build();
    }

    #[test]
    fn test_default_pcl_visualizer() {
        let _builder = PclVisualizerBuilder::default();
    }
}
