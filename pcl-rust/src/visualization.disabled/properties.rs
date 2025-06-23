//! Visualization properties and constants
//!
//! This module provides enums and constants for controlling the appearance
//! and behavior of point cloud visualizations.

/// Rendering properties for point clouds and shapes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum RenderingProperties {
    /// Point size for point cloud rendering
    PointSize = 0,
    /// Opacity/transparency (0.0 = transparent, 1.0 = opaque)
    Opacity = 1,
    /// Line width for wireframe rendering
    LineWidth = 2,
    /// Font size for text rendering
    FontSize = 3,
    /// Color property
    Color = 4,
    /// Representation mode (points, wireframe, surface)
    Representation = 5,
    /// Whether to use immediate rendering
    ImmediateRendering = 6,
    /// Shading mode (flat, Gouraud, Phong)
    Shading = 7,
}

/// Point cloud representation modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Representation {
    /// Render as individual points
    Points = 0,
    /// Render as wireframe mesh
    Wireframe = 1,
    /// Render as solid surface
    Surface = 2,
}

/// Shading modes for surface rendering
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Shading {
    /// Flat shading (constant color per face)
    Flat = 0,
    /// Gouraud shading (smooth color interpolation)
    Gouraud = 1,
    /// Phong shading (per-pixel lighting)
    Phong = 2,
}

impl RenderingProperties {
    /// Get the integer value for FFI calls
    pub fn as_i32(self) -> i32 {
        self as i32
    }
}

impl Representation {
    /// Get the integer value for FFI calls
    pub fn as_i32(self) -> i32 {
        self as i32
    }
}

impl Shading {
    /// Get the integer value for FFI calls
    pub fn as_i32(self) -> i32 {
        self as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rendering_properties() {
        assert_eq!(RenderingProperties::PointSize.as_i32(), 0);
        assert_eq!(RenderingProperties::Opacity.as_i32(), 1);
        assert_eq!(RenderingProperties::LineWidth.as_i32(), 2);
        assert_eq!(RenderingProperties::FontSize.as_i32(), 3);
        assert_eq!(RenderingProperties::Color.as_i32(), 4);
        assert_eq!(RenderingProperties::Representation.as_i32(), 5);
        assert_eq!(RenderingProperties::ImmediateRendering.as_i32(), 6);
        assert_eq!(RenderingProperties::Shading.as_i32(), 7);
    }

    #[test]
    fn test_representation() {
        assert_eq!(Representation::Points.as_i32(), 0);
        assert_eq!(Representation::Wireframe.as_i32(), 1);
        assert_eq!(Representation::Surface.as_i32(), 2);
    }

    #[test]
    fn test_shading() {
        assert_eq!(Shading::Flat.as_i32(), 0);
        assert_eq!(Shading::Gouraud.as_i32(), 1);
        assert_eq!(Shading::Phong.as_i32(), 2);
    }
}
