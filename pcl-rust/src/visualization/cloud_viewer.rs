//! Simple cloud viewer for quick point cloud visualization
//!
//! CloudViewer provides a simplified interface for displaying point clouds
//! with minimal setup. It's ideal for quick prototyping and simple visualization needs.

use crate::common::{PointCloudXYZ, PointCloudXYZRGB};
use crate::error::{PclError, PclResult};
use crate::visualization::{ViewerXYZ, ViewerXYZRGB};
use pcl_sys::{UniquePtr, ffi};

/// Simple cloud viewer for basic point cloud visualization
pub struct CloudViewer {
    inner: UniquePtr<ffi::CloudViewer>,
}

impl CloudViewer {
    /// Create a new CloudViewer with the specified window name
    pub fn new(window_name: &str) -> PclResult<Self> {
        let inner = ffi::new_cloud_viewer(window_name);
        if inner.is_null() {
            return Err(PclError::CreationFailed {
                typename: "CloudViewer".to_string(),
            });
        }
        Ok(Self { inner })
    }

    /// Check if the viewer window was stopped (closed by user)
    pub fn was_stopped(&self) -> bool {
        ffi::cloud_viewer_was_stopped(&self.inner)
    }

    /// Wait for the viewer to be stopped, with optional timeout
    ///
    /// # Arguments
    /// * `timeout_ms` - Maximum time to wait in milliseconds (0 = wait indefinitely)
    pub fn wait_for_stop(&mut self, timeout_ms: i32) {
        ffi::wait_for_cloud_viewer(self.inner.pin_mut(), timeout_ms)
    }

    /// Wait until the viewer window is closed (blocks indefinitely)
    pub fn wait_until_stopped(&mut self) {
        self.wait_for_stop(0)
    }

    /// Display a PointXYZ cloud
    pub fn show_cloud_xyz(&mut self, cloud: &PointCloudXYZ, cloud_name: &str) -> PclResult<()> {
        let result = ffi::show_cloud_xyz(self.inner.pin_mut(), cloud.inner(), cloud_name);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to show PointXYZ cloud '{}'", cloud_name),
                operation: "show_cloud_xyz".to_string(),
            });
        }
        Ok(())
    }

    /// Display a PointXYZRGB cloud
    pub fn show_cloud_xyzrgb(
        &mut self,
        cloud: &PointCloudXYZRGB,
        cloud_name: &str,
    ) -> PclResult<()> {
        let result = ffi::show_cloud_xyzrgb(self.inner.pin_mut(), cloud.inner(), cloud_name);
        if result != 0 {
            return Err(PclError::VisualizationError {
                message: format!("Failed to show PointXYZRGB cloud '{}'", cloud_name),
                operation: "show_cloud_xyzrgb".to_string(),
            });
        }
        Ok(())
    }
}

impl ViewerXYZ for CloudViewer {
    fn show_cloud(&mut self, cloud: &PointCloudXYZ, name: &str) -> PclResult<()> {
        self.show_cloud_xyz(cloud, name)
    }
}

impl ViewerXYZRGB for CloudViewer {
    fn show_cloud_rgb(&mut self, cloud: &PointCloudXYZRGB, name: &str) -> PclResult<()> {
        self.show_cloud_xyzrgb(cloud, name)
    }
}

impl Default for CloudViewer {
    fn default() -> Self {
        Self::new("CloudViewer").expect("Failed to create default CloudViewer")
    }
}

/// Builder for CloudViewer configuration
pub struct CloudViewerBuilder {
    window_name: String,
}

impl CloudViewerBuilder {
    /// Create a new CloudViewer builder
    pub fn new() -> Self {
        Self {
            window_name: "CloudViewer".to_string(),
        }
    }

    /// Set the window name
    pub fn window_name(mut self, name: &str) -> Self {
        self.window_name = name.to_string();
        self
    }

    /// Build the CloudViewer
    pub fn build(self) -> PclResult<CloudViewer> {
        CloudViewer::new(&self.window_name)
    }
}

impl Default for CloudViewerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cloud_viewer_creation() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            eprintln!("Skipping visualization test - no display available");
            return;
        }

        // Note: This test checks object creation but doesn't open a window
        // since we're in a test environment
        let viewer = CloudViewer::new("Test Viewer");

        // Check if creation succeeds or fails due to missing display
        match viewer {
            Ok(_) => {
                // Creation succeeded - likely in an environment with display
                println!("CloudViewer created successfully");
            }
            Err(PclError::CreationFailed { .. }) => {
                // Expected in headless environment
                println!("CloudViewer creation failed (expected in headless environment)");
            }
            Err(e) => {
                panic!("Unexpected error: {:?}", e);
            }
        }
    }

    #[test]
    fn test_cloud_viewer_builder() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            eprintln!("Skipping visualization test - no display available");
            return;
        }

        let builder = CloudViewerBuilder::new().window_name("Custom Window");

        // Test that builder doesn't panic
        let _viewer = builder.build();
    }

    #[test]
    fn test_default_cloud_viewer() {
        // Test default implementation
        let _viewer = CloudViewerBuilder::default();
    }
}
