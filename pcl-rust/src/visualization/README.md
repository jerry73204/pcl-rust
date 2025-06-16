# PCL Visualization Module

This module provides safe Rust bindings for PCL's visualization capabilities, including 3D point cloud rendering, interactive controls, and comprehensive display options.

## Features

### Viewers
- **CloudViewer**: Simple interface for quick point cloud visualization
- **PCLVisualizer**: Full-featured 3D viewer with advanced rendering and controls

### Rendering Properties
- Point size, opacity, color control
- Multiple representation modes (points, wireframe, surface)
- Shading options (flat, Gouraud, Phong)

### Interactive Features
- Camera positioning and controls
- Text overlays and annotations
- Shape primitives (spheres, coordinate systems)
- Keyboard callbacks for user interaction

## Quick Start

### Basic Visualization
```rust
use pcl::{PointCloudXYZ, CloudViewer};

let cloud = PointCloudXYZ::new()?;
let mut viewer = CloudViewer::new("My Viewer")?;
viewer.show_cloud(&cloud, "cloud1")?;
viewer.wait_until_stopped();
```

### Advanced Visualization
```rust
use pcl::{PclVisualizer, RenderingProperties};

let mut viewer = PclVisualizer::new("Advanced Viewer")?;
viewer.add_point_cloud_xyz(&cloud, "cloud1")?;
viewer.set_point_size("cloud1", 3.0)?;
viewer.set_point_color("cloud1", 1.0, 0.0, 0.0)?; // Red
viewer.set_background_color(0.1, 0.1, 0.1)?;
viewer.add_coordinate_system(1.0, "coords")?;

while !viewer.was_stopped() {
    viewer.spin_once(100)?;
}
```

## Builder Patterns

Both viewers support builder patterns for easy configuration:

```rust
use pcl::visualization::{PclVisualizerBuilder, CloudViewerBuilder};

let viewer = PclVisualizerBuilder::new()
    .window_name("Custom Window")
    .background_color(0.2, 0.2, 0.4)
    .coordinate_system(1.0, "coords")
    .build()?;

let cloud_viewer = CloudViewerBuilder::new()
    .window_name("Simple Viewer")
    .build()?;
```

## Traits

The module provides traits for generic programming:
- `ViewerXYZ`: Display PointXYZ clouds
- `ViewerXYZRGB`: Display PointXYZRGB clouds  
- `CameraControl`: Camera positioning and reset
- `VisualizationControl`: Window lifecycle and rendering

## Headless Environments

The visualization module gracefully handles headless environments (CI, servers without display). All functions will return appropriate errors rather than crashing, making it safe to include visualization code in libraries that might run in various environments.

## Requirements

- VTK (Visualization Toolkit) must be installed
- X11 or Wayland display server (for actual rendering)
- OpenGL support (for hardware acceleration)

On Ubuntu/Debian:
```bash
sudo apt-get install libvtk9-dev
```

On macOS:
```bash
brew install vtk
```

## Examples

See `examples/visualization_demo.rs` for comprehensive usage examples demonstrating all features.