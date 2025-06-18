//! Advanced visualization features
//!
//! This module provides advanced visualization capabilities including
//! histogram visualization, range images, and point cloud comparisons.

use crate::common::PointCloud;
use crate::error::{PclError, PclResult};
use crate::traits::Point;
use crate::visualization::{PclVisualizer, VisualizationControl};
use cxx::memory::UniquePtrTarget;

/// Histogram visualization for point cloud features
pub struct HistogramVisualizer {
    viewer: PclVisualizer,
    histogram_count: usize,
}

impl HistogramVisualizer {
    /// Create a new histogram visualizer
    pub fn new(window_name: &str) -> PclResult<Self> {
        Ok(Self {
            viewer: PclVisualizer::new(window_name)?,
            histogram_count: 0,
        })
    }

    /// Add a histogram to the visualization
    pub fn add_histogram(
        &mut self,
        values: &[f64],
        bin_count: usize,
        label: &str,
    ) -> PclResult<()> {
        if values.is_empty() {
            return Err(PclError::InvalidParameter {
                param: "values".to_string(),
                message: "cannot create histogram from empty data".to_string(),
            });
        }

        // TODO: Implement histogram visualization when FFI bindings are available
        let _ = (values, bin_count, label);
        self.histogram_count += 1;

        Err(PclError::NotImplemented {
            feature: "histogram visualization".to_string(),
            workaround: Some("Use external plotting libraries for now".to_string()),
        })
    }

    /// Update an existing histogram
    pub fn update_histogram(&mut self, values: &[f64], label: &str) -> PclResult<()> {
        if values.is_empty() {
            return Err(PclError::InvalidParameter {
                param: "values".to_string(),
                message: "cannot update histogram with empty data".to_string(),
            });
        }

        // TODO: Implement histogram update when FFI bindings are available
        let _ = (values, label);

        Err(PclError::NotImplemented {
            feature: "histogram update".to_string(),
            workaround: Some("Recreate the histogram instead".to_string()),
        })
    }

    /// Get the underlying PclVisualizer
    pub fn viewer(&self) -> &PclVisualizer {
        &self.viewer
    }

    /// Get a mutable reference to the underlying PclVisualizer
    pub fn viewer_mut(&mut self) -> &mut PclVisualizer {
        &mut self.viewer
    }
}

/// Side-by-side point cloud comparison viewer
pub struct ComparisonViewer {
    viewer: PclVisualizer,
    viewport_left: i32,
    viewport_right: i32,
}

impl ComparisonViewer {
    /// Create a new comparison viewer with two viewports
    pub fn new(window_name: &str) -> PclResult<Self> {
        let viewer = PclVisualizer::new(window_name)?;

        // TODO: Create viewports when FFI bindings are available
        let viewport_left = 0;
        let viewport_right = 1;

        Ok(Self {
            viewer,
            viewport_left,
            viewport_right,
        })
    }

    /// Add clouds to compare side by side
    pub fn add_comparison<T: Point>(
        &mut self,
        cloud_left: &PointCloud<T>,
        cloud_right: &PointCloud<T>,
        id_left: &str,
        id_right: &str,
    ) -> PclResult<()>
    where
        T::CloudType: UniquePtrTarget,
    {
        // TODO: Add clouds to specific viewports when FFI bindings are available
        let _ = (cloud_left, cloud_right, id_left, id_right);

        Err(PclError::NotImplemented {
            feature: "comparison visualization".to_string(),
            workaround: Some("Use multiple PclVisualizer instances".to_string()),
        })
    }

    /// Set titles for the viewports
    pub fn set_viewport_titles(&mut self, title_left: &str, title_right: &str) -> PclResult<()> {
        // Add text labels for each viewport
        self.viewer
            .add_text(title_left, 10, 10, 1.0, 1.0, 1.0, "title_left")?;

        self.viewer.add_text(
            title_right,
            410, // Assuming 800px window width, put on right half
            10,
            1.0,
            1.0,
            1.0,
            "title_right",
        )?;

        Ok(())
    }

    /// Get the underlying PclVisualizer
    pub fn viewer(&self) -> &PclVisualizer {
        &self.viewer
    }

    /// Get a mutable reference to the underlying PclVisualizer
    pub fn viewer_mut(&mut self) -> &mut PclVisualizer {
        &mut self.viewer
    }
}

/// Range image visualization
pub struct RangeImageVisualizer {
    viewer: PclVisualizer,
}

impl RangeImageVisualizer {
    /// Create a new range image visualizer
    pub fn new(window_name: &str) -> PclResult<Self> {
        Ok(Self {
            viewer: PclVisualizer::new(window_name)?,
        })
    }

    /// Display a range image from a point cloud
    pub fn show_range_image<T: Point>(
        &mut self,
        cloud: &PointCloud<T>,
        angular_resolution: f32,
        max_angle_width: f32,
        max_angle_height: f32,
        sensor_pose: Option<(f32, f32, f32, f32, f32, f32)>,
        id: &str,
    ) -> PclResult<()>
    where
        T::CloudType: UniquePtrTarget,
    {
        // Validate parameters
        if angular_resolution <= 0.0 {
            return Err(PclError::InvalidParameter {
                param: "angular_resolution".to_string(),
                message: "must be positive".to_string(),
            });
        }

        // TODO: Implement range image visualization when FFI bindings are available
        let _ = (
            cloud,
            angular_resolution,
            max_angle_width,
            max_angle_height,
            sensor_pose,
            id,
        );

        Err(PclError::NotImplemented {
            feature: "range image visualization".to_string(),
            workaround: Some("Convert to regular point cloud for visualization".to_string()),
        })
    }

    /// Get the underlying PclVisualizer
    pub fn viewer(&self) -> &PclVisualizer {
        &self.viewer
    }

    /// Get a mutable reference to the underlying PclVisualizer
    pub fn viewer_mut(&mut self) -> &mut PclVisualizer {
        &mut self.viewer
    }
}

/// Feature visualization utilities
pub struct FeatureVisualizer;

impl FeatureVisualizer {
    /// Visualize point cloud with feature values as colors
    pub fn visualize_features<T: Point>(
        viewer: &mut PclVisualizer,
        cloud: &PointCloud<T>,
        features: &[f32],
        id: &str,
        colormap: ColorMap,
    ) -> PclResult<()>
    where
        T::CloudType: UniquePtrTarget,
    {
        if features.len() != cloud.size() {
            return Err(PclError::InvalidParameter {
                param: "features".to_string(),
                message: format!(
                    "feature count {} doesn't match cloud size {}",
                    features.len(),
                    cloud.size()
                ),
            });
        }

        // Find min/max for normalization
        let (min_val, max_val) = features
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &val| {
                (min.min(val), max.max(val))
            });

        if (max_val - min_val).abs() < f32::EPSILON {
            return Err(PclError::InvalidParameter {
                param: "features".to_string(),
                message: "all feature values are the same".to_string(),
            });
        }

        // TODO: Create colored point cloud based on features when generic colored clouds are supported
        let _ = (viewer, cloud, id, colormap);

        Err(PclError::NotImplemented {
            feature: "feature visualization".to_string(),
            workaround: Some("Manually create colored point clouds".to_string()),
        })
    }
}

/// Color mapping schemes for feature visualization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorMap {
    /// Blue to red through green
    Jet,
    /// Blue to red
    Cool,
    /// Black to white
    Gray,
    /// Red to yellow
    Hot,
    /// Rainbow colors
    Hsv,
    /// Custom color map
    Custom,
}

impl ColorMap {
    /// Map a normalized value (0.0-1.0) to RGB color
    pub fn map_color(&self, value: f32) -> (f32, f32, f32) {
        let value = value.clamp(0.0, 1.0);

        match self {
            ColorMap::Jet => {
                // Simple jet colormap approximation
                if value < 0.25 {
                    (0.0, 4.0 * value, 1.0)
                } else if value < 0.5 {
                    (0.0, 1.0, 1.0 - 4.0 * (value - 0.25))
                } else if value < 0.75 {
                    (4.0 * (value - 0.5), 1.0, 0.0)
                } else {
                    (1.0, 1.0 - 4.0 * (value - 0.75), 0.0)
                }
            }
            ColorMap::Cool => (value, 0.0, 1.0 - value),
            ColorMap::Gray => (value, value, value),
            ColorMap::Hot => {
                if value < 0.33 {
                    (3.0 * value, 0.0, 0.0)
                } else if value < 0.66 {
                    (1.0, 3.0 * (value - 0.33), 0.0)
                } else {
                    (1.0, 1.0, 3.0 * (value - 0.66))
                }
            }
            ColorMap::Hsv => {
                // HSV to RGB conversion with full saturation and value
                let h = value * 360.0;
                let c = 1.0;
                let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
                let m = 0.0;

                let (r, g, b) = if h < 60.0 {
                    (c, x, 0.0)
                } else if h < 120.0 {
                    (x, c, 0.0)
                } else if h < 180.0 {
                    (0.0, c, x)
                } else if h < 240.0 {
                    (0.0, x, c)
                } else if h < 300.0 {
                    (x, 0.0, c)
                } else {
                    (c, 0.0, x)
                };

                (r + m, g + m, b + m)
            }
            ColorMap::Custom => (value, value, value), // Default to grayscale
        }
    }
}

/// Animation controller for time-series point cloud data
pub struct AnimationController {
    viewer: PclVisualizer,
    frame_delay_ms: i32,
    is_playing: bool,
    current_frame: usize,
    total_frames: usize,
}

impl AnimationController {
    /// Create a new animation controller
    pub fn new(window_name: &str, frame_delay_ms: i32) -> PclResult<Self> {
        Ok(Self {
            viewer: PclVisualizer::new(window_name)?,
            frame_delay_ms,
            is_playing: false,
            current_frame: 0,
            total_frames: 0,
        })
    }

    /// Play animation of point cloud sequence
    pub fn play_sequence<T: Point, F>(
        &mut self,
        clouds: &[PointCloud<T>],
        id: &str,
        mut frame_callback: F,
    ) -> PclResult<()>
    where
        T::CloudType: UniquePtrTarget,
        F: FnMut(usize, &PointCloud<T>),
    {
        if clouds.is_empty() {
            return Err(PclError::InvalidParameter {
                param: "clouds".to_string(),
                message: "cannot animate empty sequence".to_string(),
            });
        }

        self.total_frames = clouds.len();
        self.is_playing = true;

        // TODO: Implement animation loop when generic cloud support is complete
        for (i, cloud) in clouds.iter().enumerate() {
            if !self.is_playing {
                break;
            }

            self.current_frame = i;
            frame_callback(i, cloud);

            // Update display
            self.viewer.spin_once(self.frame_delay_ms)?;

            if self.viewer.was_stopped() {
                break;
            }
        }

        let _ = id;
        Ok(())
    }

    /// Stop the animation
    pub fn stop(&mut self) {
        self.is_playing = false;
    }

    /// Get the current frame index
    pub fn current_frame(&self) -> usize {
        self.current_frame
    }

    /// Get the total number of frames
    pub fn total_frames(&self) -> usize {
        self.total_frames
    }

    /// Get the underlying PclVisualizer
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
    fn test_colormap() {
        // Test Jet colormap
        let (_r, _g, b) = ColorMap::Jet.map_color(0.0);
        assert!((b - 1.0).abs() < f32::EPSILON);

        let (r, _g, _b) = ColorMap::Jet.map_color(1.0);
        assert!((r - 1.0).abs() < f32::EPSILON);

        // Test Gray colormap
        let (r, g, b) = ColorMap::Gray.map_color(0.5);
        assert!((r - 0.5).abs() < f32::EPSILON);
        assert!((g - 0.5).abs() < f32::EPSILON);
        assert!((b - 0.5).abs() < f32::EPSILON);

        // Test value clamping
        let (r, g, b) = ColorMap::Cool.map_color(-1.0);
        assert_eq!((r, g, b), (0.0, 0.0, 1.0));

        let (r, g, b) = ColorMap::Cool.map_color(2.0);
        assert_eq!((r, g, b), (1.0, 0.0, 0.0));
    }

    #[test]
    fn test_histogram_visualizer() {
        // Skip test if no display is available
        if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
            return;
        }

        let result = HistogramVisualizer::new("Test Histogram");
        match result {
            Ok(_) => println!("HistogramVisualizer created successfully"),
            Err(PclError::CreationFailed { .. }) => {
                println!("HistogramVisualizer creation failed (expected in headless)")
            }
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
}
