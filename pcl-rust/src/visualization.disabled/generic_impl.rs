//! Generic implementations for visualization
//!
//! This module provides concrete implementations of generic visualization traits
//! for PCL's visualization classes.

use crate::common::PointCloud;
use crate::common::point_types::PointType;
use crate::error::{PclError, PclResult};
use crate::visualization::{
    cloud_viewer::CloudViewer,
    generic::{AdvancedViewer, ShapeVisualization, Viewer},
    pcl_visualizer::PclVisualizer,
};
use cxx::memory::UniquePtrTarget;

// Marker trait for points that can be visualized
pub trait VisualizablePoint: PointType {
    /// Add this point cloud to a PclVisualizer
    fn add_to_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()>
    where
        <Self as PointType>::CloudType: UniquePtrTarget;

    /// Update this point cloud in a PclVisualizer
    fn update_in_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()>
    where
        <Self as PointType>::CloudType: UniquePtrTarget;

    /// Add this point cloud to a CloudViewer
    fn add_to_cloud_viewer(
        viewer: &mut CloudViewer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()>
    where
        <Self as PointType>::CloudType: UniquePtrTarget;
}

// Implement VisualizablePoint for XYZ marker type
impl VisualizablePoint for crate::common::point_types::XYZ {
    fn add_to_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        visualizer.add_point_cloud_xyz(cloud, id)
    }

    fn update_in_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        visualizer.update_point_cloud_xyz(cloud, id)
    }

    fn add_to_cloud_viewer(
        viewer: &mut CloudViewer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        viewer.show_cloud_xyz(cloud, id)
    }
}

// Implement VisualizablePoint for XYZRGB marker type
impl VisualizablePoint for crate::common::point_types::XYZRGB {
    fn add_to_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        visualizer.add_point_cloud_xyzrgb(cloud, id)
    }

    fn update_in_visualizer(
        visualizer: &mut PclVisualizer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        visualizer.update_point_cloud_xyzrgb(cloud, id)
    }

    fn add_to_cloud_viewer(
        viewer: &mut CloudViewer,
        cloud: &PointCloud<Self>,
        id: &str,
    ) -> PclResult<()> {
        viewer.show_cloud_xyzrgb(cloud, id)
    }
}

// Implement generic Viewer trait for PclVisualizer
impl<T: VisualizablePoint> Viewer<T> for PclVisualizer
where
    <T as PointType>::CloudType: UniquePtrTarget,
{
    fn show_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()> {
        T::add_to_visualizer(self, cloud, id)
    }

    fn update_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()> {
        T::update_in_visualizer(self, cloud, id)
    }

    fn remove_cloud(&mut self, id: &str) -> PclResult<()> {
        self.remove_point_cloud(id)
    }
}

// Implement generic Viewer trait for CloudViewer
impl<T: VisualizablePoint> Viewer<T> for CloudViewer
where
    <T as PointType>::CloudType: UniquePtrTarget,
{
    fn show_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()> {
        T::add_to_cloud_viewer(self, cloud, id)
    }

    fn update_cloud(&mut self, cloud: &PointCloud<T>, id: &str) -> PclResult<()> {
        // CloudViewer doesn't support updating, so we just show again
        T::add_to_cloud_viewer(self, cloud, id)
    }

    fn remove_cloud(&mut self, _id: &str) -> PclResult<()> {
        // CloudViewer doesn't support removing individual clouds
        Err(PclError::NotImplemented {
            feature: "remove_cloud for CloudViewer".to_string(),
            workaround: Some("Create a new CloudViewer instance".to_string()),
        })
    }
}

// Implement AdvancedViewer for PclVisualizer
impl<T: VisualizablePoint> AdvancedViewer<T> for PclVisualizer
where
    <T as PointType>::CloudType: UniquePtrTarget,
{
    fn set_point_size(&mut self, id: &str, size: f64) -> PclResult<()> {
        self.set_point_size(id, size)
    }

    fn set_opacity(&mut self, id: &str, opacity: f64) -> PclResult<()> {
        self.set_opacity(id, opacity)
    }

    fn set_cloud_color(&mut self, id: &str, r: f64, g: f64, b: f64) -> PclResult<()> {
        self.set_point_color(id, r, g, b)
    }
}

// Implement ShapeVisualization for PclVisualizer
impl ShapeVisualization for PclVisualizer {
    fn add_line(&mut self, p1: (f32, f32, f32), p2: (f32, f32, f32), id: &str) -> PclResult<()> {
        // TODO: Implement line visualization when FFI bindings are available
        let _ = (p1, p2, id);
        Err(PclError::NotImplemented {
            feature: "add_line".to_string(),
            workaround: Some("Use add_sphere to mark endpoints".to_string()),
        })
    }

    fn add_sphere(
        &mut self,
        center: (f32, f32, f32),
        radius: f64,
        r: f64,
        g: f64,
        b: f64,
        id: &str,
    ) -> PclResult<()> {
        // TODO: Implement sphere visualization when center point creation is available
        let _ = (center, radius, r, g, b, id);
        Err(PclError::NotImplemented {
            feature: "add_sphere with coordinate tuple".to_string(),
            workaround: Some("Create a PointXYZ from a point cloud instead".to_string()),
        })
    }

    fn add_cube(
        &mut self,
        _min: (f32, f32, f32),
        _max: (f32, f32, f32),
        _r: f64,
        _g: f64,
        _b: f64,
        _id: &str,
    ) -> PclResult<()> {
        // TODO: Implement cube visualization when FFI bindings are available
        Err(PclError::NotImplemented {
            feature: "add_cube".to_string(),
            workaround: Some("Use multiple spheres to mark corners".to_string()),
        })
    }

    fn add_arrow(
        &mut self,
        _start: (f32, f32, f32),
        _end: (f32, f32, f32),
        _r: f64,
        _g: f64,
        _b: f64,
        _id: &str,
    ) -> PclResult<()> {
        // TODO: Implement arrow visualization when FFI bindings are available
        Err(PclError::NotImplemented {
            feature: "add_arrow".to_string(),
            workaround: Some("Use spheres to mark start and end points".to_string()),
        })
    }
}

/// Helper struct to manage multiple point cloud types in a single viewer
pub struct MultiCloudViewer {
    viewer: PclVisualizer,
}

impl MultiCloudViewer {
    /// Create a new multi-cloud viewer
    pub fn new(window_name: &str) -> PclResult<Self> {
        Ok(Self {
            viewer: PclVisualizer::new(window_name)?,
        })
    }

    /// Add any visualizable point cloud
    pub fn add_cloud<T: VisualizablePoint>(
        &mut self,
        cloud: &PointCloud<T>,
        id: &str,
    ) -> PclResult<()>
    where
        <T as PointType>::CloudType: UniquePtrTarget,
    {
        T::add_to_visualizer(&mut self.viewer, cloud, id)
    }

    /// Update any visualizable point cloud
    pub fn update_cloud<T: VisualizablePoint>(
        &mut self,
        cloud: &PointCloud<T>,
        id: &str,
    ) -> PclResult<()>
    where
        <T as PointType>::CloudType: UniquePtrTarget,
    {
        T::update_in_visualizer(&mut self.viewer, cloud, id)
    }

    /// Remove a cloud by id
    pub fn remove_cloud(&mut self, id: &str) -> PclResult<()> {
        self.viewer.remove_point_cloud(id)
    }

    /// Get a reference to the underlying PclVisualizer
    pub fn viewer(&self) -> &PclVisualizer {
        &self.viewer
    }

    /// Get a mutable reference to the underlying PclVisualizer
    pub fn viewer_mut(&mut self) -> &mut PclVisualizer {
        &mut self.viewer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multi_cloud_viewer() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            eprintln!("Skipping visualization test - no display available");
            return;
        }

        // Test creation
        let viewer = MultiCloudViewer::new("Test Multi Viewer");
        match viewer {
            Ok(_) => println!("MultiCloudViewer created successfully"),
            Err(PclError::CreationFailed { .. }) => {
                println!("MultiCloudViewer creation failed (expected in headless)")
            }
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
}
