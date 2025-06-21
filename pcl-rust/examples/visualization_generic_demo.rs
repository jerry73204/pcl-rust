//! Demonstration of generic visualization API
//!
//! This example shows how to use the generic visualization traits and
//! advanced visualization features.

use pcl::{
    PclResult, PointCloud, PointXYZ, PointXYZRGB, XYZ, XYZRGB,
    visualization::{
        AdvancedViewer, AnimationController, CameraPosition, ColorMap, ComparisonViewer,
        MultiCloudViewer, VisualizationConfigBuilder, VisualizationControl,
    },
};

fn create_sample_clouds() -> PclResult<(PointCloud<XYZ>, PointCloud<XYZRGB>)> {
    // Create XYZ cloud - a simple spiral
    let mut xyz_cloud = PointCloud::<XYZ>::new()?;
    for i in 0..100 {
        let angle = i as f32 * 0.1;
        let x = angle.cos() * (i as f32 * 0.01);
        let y = angle.sin() * (i as f32 * 0.01);
        let z = i as f32 * 0.01;
        xyz_cloud.push(PointXYZ::new(x, y, z))?;
    }

    // Create XYZRGB cloud - a colorful grid
    let mut xyzrgb_cloud = PointCloud::<XYZRGB>::new()?;
    for x in 0..10 {
        for y in 0..10 {
            let r = (x * 25) as u8;
            let g = (y * 25) as u8;
            let b = 128;
            xyzrgb_cloud.push(PointXYZRGB::new(
                x as f32 * 0.1 - 0.5,
                y as f32 * 0.1 - 0.5,
                0.0,
                r,
                g,
                b,
            ))?;
        }
    }

    Ok((xyz_cloud, xyzrgb_cloud))
}

fn demo_generic_viewer() -> PclResult<()> {
    println!("=== Generic Viewer Demo ===");

    let (xyz_cloud, xyzrgb_cloud) = create_sample_clouds()?;

    // Create a multi-cloud viewer that can handle different point types
    let mut viewer = MultiCloudViewer::new("Generic Viewer Demo")?;

    // Add both cloud types using the generic interface
    viewer.add_cloud(&xyz_cloud, "spiral")?;
    viewer.add_cloud(&xyzrgb_cloud, "grid")?;

    // Configure visualization properties
    let pcl_viewer = viewer.viewer_mut();
    pcl_viewer.set_background_color(0.1, 0.1, 0.2)?;
    pcl_viewer.add_coordinate_system(0.5, "axes")?;

    // Use the AdvancedViewer trait methods
    AdvancedViewer::<XYZ>::set_point_size(pcl_viewer, "spiral", 3.0)?;
    AdvancedViewer::<XYZ>::set_cloud_color(pcl_viewer, "spiral", 1.0, 1.0, 0.0)?; // Yellow
    AdvancedViewer::<XYZ>::set_opacity(pcl_viewer, "spiral", 0.8)?;

    pcl_viewer.set_point_size("grid", 5.0)?;

    // Add labels
    pcl_viewer.add_text("XYZ Spiral", 10, 50, 1.0, 1.0, 0.0, "label1")?;
    pcl_viewer.add_text("XYZRGB Grid", 10, 30, 1.0, 1.0, 1.0, "label2")?;

    // Run for a few frames
    for _ in 0..5 {
        match pcl_viewer.spin_once(100) {
            Ok(()) => {}
            Err(_) => break,
        }
        if pcl_viewer.was_stopped() {
            break;
        }
    }

    println!("Generic viewer demo completed.");
    Ok(())
}

fn demo_comparison_viewer() -> PclResult<()> {
    println!("\n=== Comparison Viewer Demo ===");

    // Create two clouds to compare
    let mut cloud1 = PointCloud::<XYZ>::new()?;
    let mut cloud2 = PointCloud::<XYZ>::new()?;

    // Cloud 1: Original data
    for i in 0..50 {
        let x = (i as f32 - 25.0) * 0.02;
        let y = x * x;
        let z = 0.0;
        cloud1.push(PointXYZ::new(x, y, z))?;
    }

    // Cloud 2: Transformed data
    for i in 0..50 {
        let x = (i as f32 - 25.0) * 0.02;
        let y = x * x + 0.1; // Offset
        let z = 0.1;
        cloud2.push(PointXYZ::new(x, y, z))?;
    }

    // Create comparison viewer
    let mut viewer = ComparisonViewer::new("Comparison Demo")?;

    // Set viewport titles
    viewer.set_viewport_titles("Original", "Transformed")?;

    // Note: The actual side-by-side display requires viewport support
    // which needs FFI bindings. For now, we demonstrate the API structure.

    println!("Comparison viewer demo completed.");
    Ok(())
}

fn demo_animation() -> PclResult<()> {
    println!("\n=== Animation Demo ===");

    // Create a sequence of clouds for animation
    let mut cloud_sequence = Vec::new();

    for frame in 0..10 {
        let mut cloud = PointCloud::<XYZ>::new()?;

        // Create a rotating point pattern
        for i in 0..50 {
            let base_angle = i as f32 * 0.1;
            let rotation = frame as f32 * 0.2;
            let angle = base_angle + rotation;

            let x = angle.cos() * 0.5;
            let y = angle.sin() * 0.5;
            let z = i as f32 * 0.01;

            cloud.push(PointXYZ::new(x, y, z))?;
        }

        cloud_sequence.push(cloud);
    }

    // Create animation controller
    let mut animator = AnimationController::new("Animation Demo", 100)?;

    // Configure viewer
    let viewer = animator.viewer_mut();
    viewer.set_background_color(0.0, 0.0, 0.0)?;
    viewer.add_coordinate_system(0.3, "axes")?;

    // Play the animation
    animator.play_sequence(&cloud_sequence, "rotating_points", |frame, cloud| {
        println!(
            "Frame {}/{}: {} points",
            frame + 1,
            cloud_sequence.len(),
            cloud.size()
        );
    })?;

    println!("Animation demo completed.");
    Ok(())
}

fn demo_visualization_config() -> PclResult<()> {
    println!("\n=== Visualization Config Demo ===");

    // Build a custom visualization configuration
    let config = VisualizationConfigBuilder::new()
        .window_name("Configured Viewer")
        .background_color(0.2, 0.3, 0.4)
        .point_size(2.5)
        .show_coordinate_system(true)
        .coordinate_system_scale(0.25)
        .show_fps(true)
        .window_size(1024, 768)
        .camera_position(CameraPosition {
            position: (2.0, 2.0, 2.0),
            focal_point: (0.0, 0.0, 0.0),
            up_vector: (0.0, 0.0, 1.0),
        })
        .build();

    println!("Created visualization config:");
    println!("  Window: {}", config.window_name);
    println!("  Background: {:?}", config.background_color);
    println!("  Point size: {}", config.point_size);
    println!("  Show coords: {}", config.show_coordinate_system);
    println!("  Show FPS: {}", config.show_fps);

    // In a real implementation, this config would be used to initialize a viewer

    Ok(())
}

fn demo_colormap() -> PclResult<()> {
    println!("\n=== ColorMap Demo ===");

    // Demonstrate different color mapping schemes
    let colormaps = [
        (ColorMap::Jet, "Jet"),
        (ColorMap::Cool, "Cool"),
        (ColorMap::Hot, "Hot"),
        (ColorMap::Gray, "Gray"),
        (ColorMap::Hsv, "HSV"),
    ];

    println!("Color mapping examples (normalized value 0.0 to 1.0):");

    for (colormap, name) in &colormaps {
        println!("\n{}:", name);
        for i in 0..=4 {
            let value = i as f32 * 0.25;
            let (r, g, b) = colormap.map_color(value);
            println!("  {:.2} -> RGB({:.2}, {:.2}, {:.2})", value, r, g, b);
        }
    }

    Ok(())
}

fn main() -> PclResult<()> {
    println!("PCL Generic Visualization Demo");
    println!("==============================");

    // Check if we're in a headless environment
    if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
        println!("\nWarning: No display detected. Visualization may not work.");
        println!("This is normal in CI/headless environments.");
    }

    // Run demos
    match demo_generic_viewer() {
        Ok(()) => println!("✓ Generic viewer demo completed"),
        Err(e) => println!("✗ Generic viewer demo failed: {}", e),
    }

    match demo_comparison_viewer() {
        Ok(()) => println!("✓ Comparison viewer demo completed"),
        Err(e) => println!("✗ Comparison viewer demo failed: {}", e),
    }

    match demo_animation() {
        Ok(()) => println!("✓ Animation demo completed"),
        Err(e) => println!("✗ Animation demo failed: {}", e),
    }

    match demo_visualization_config() {
        Ok(()) => println!("✓ Visualization config demo completed"),
        Err(e) => println!("✗ Visualization config demo failed: {}", e),
    }

    match demo_colormap() {
        Ok(()) => println!("✓ ColorMap demo completed"),
        Err(e) => println!("✗ ColorMap demo failed: {}", e),
    }

    println!("\nGeneric visualization demo completed!");
    Ok(())
}
