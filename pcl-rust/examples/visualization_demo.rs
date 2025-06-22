//! Visualization demo showcasing PCL's 3D rendering capabilities
//!
//! This example demonstrates both CloudViewer and PCLVisualizer interfaces
//! for displaying point clouds. Note that in headless environments (CI),
//! the actual visualization will fail but the API structure is demonstrated.

use pcl::{
    CameraControl, CloudViewer, PclVisualizer, ViewerXYZ, VisualizationControl,
    common::{PointCloud, XYZ, XYZRGB},
    error::PclResult,
};

fn create_sample_xyz_cloud() -> PclResult<PointCloud<XYZ>> {
    let mut cloud = PointCloud::<XYZ>::new()?;
    // Just create an empty cloud for demonstration
    // In a real application, you would populate this with actual point data
    cloud.resize(100)?;
    Ok(cloud)
}

fn create_sample_xyzrgb_cloud() -> PclResult<PointCloud<XYZRGB>> {
    let mut cloud = PointCloud::<XYZRGB>::new()?;
    // Just create an empty cloud for demonstration
    cloud.resize(100)?;
    Ok(cloud)
}

fn demo_cloud_viewer() -> PclResult<()> {
    println!("=== CloudViewer Demo ===");
    println!("Creating sample point cloud...");

    let cloud = create_sample_xyz_cloud()?;
    println!("Created cloud with {} points", cloud.size());

    println!("Opening CloudViewer...");
    let mut viewer = CloudViewer::new("CloudViewer Demo")?;

    // Display the cloud
    viewer.show_cloud(&cloud, "sample_cloud")?;
    println!("Cloud displayed. In headless environment, this will fail gracefully.");

    // Wait for user to close window (with 2 second timeout for CI)
    viewer.wait_for_stop(2000);

    println!("CloudViewer demo completed.");
    Ok(())
}

fn demo_pcl_visualizer() -> PclResult<()> {
    println!("\n=== PCLVisualizer Demo ===");
    println!("Creating sample point clouds...");

    let xyz_cloud = create_sample_xyz_cloud()?;
    let xyzrgb_cloud = create_sample_xyzrgb_cloud()?;

    println!("Opening PCLVisualizer...");
    let mut viewer = PclVisualizer::new("PCLVisualizer Demo")?;

    // Set up the scene
    viewer.set_background_color(0.1, 0.1, 0.1)?;
    viewer.add_coordinate_system(0.2, "coordinate_system")?;

    // Add point clouds
    viewer.add_point_cloud_xyz(&xyz_cloud, "xyz_cloud")?;
    viewer.add_point_cloud_xyzrgb(&xyzrgb_cloud, "xyzrgb_cloud")?;

    // Configure rendering properties
    viewer.set_point_size("xyz_cloud", 3.0)?;
    viewer.set_point_color("xyz_cloud", 1.0, 0.0, 0.0)?; // Red
    viewer.set_opacity("xyz_cloud", 0.8)?;

    viewer.set_point_size("xyzrgb_cloud", 2.0)?;
    viewer.set_opacity("xyzrgb_cloud", 1.0)?;

    // Add text overlays
    viewer.add_text("PCL Visualization Demo", 10, 50, 1.0, 1.0, 1.0, "title")?;
    viewer.add_text("Red: PointXYZ", 10, 30, 1.0, 0.0, 0.0, "xyz_label")?;
    viewer.add_text(
        "Colored: PointXYZRGB",
        10,
        10,
        0.0,
        1.0,
        0.0,
        "xyzrgb_label",
    )?;

    // Set camera position
    viewer.set_camera_position(
        1.0, 1.0, 1.0, // position
        0.0, 0.0, 0.0, // view direction
        0.0, 0.0, 1.0, // up vector
    )?;

    println!("Visualization setup complete.");
    println!("Controls:");
    println!("- Mouse: Rotate view");
    println!("- Scroll: Zoom in/out");
    println!("- Close window to exit");

    // Run for a few frames to demonstrate the API
    for i in 0..10 {
        match viewer.spin_once(100) {
            Ok(()) => {}
            Err(_) => {
                println!("Spin failed (expected in headless environment)");
                break;
            }
        }

        if viewer.was_stopped() {
            println!("Viewer was stopped at frame {}", i);
            break;
        }
    }

    println!("PCLVisualizer demo completed.");
    Ok(())
}

fn demo_builder_patterns() -> PclResult<()> {
    println!("\n=== Builder Pattern Demo ===");

    use pcl::visualization::{CloudViewerBuilder, PclVisualizerBuilder};

    // CloudViewer with builder
    let cloud = create_sample_xyz_cloud()?;
    let mut cloud_viewer = CloudViewerBuilder::new()
        .window_name("Builder CloudViewer")
        .build()?;

    cloud_viewer.show_cloud(&cloud, "builder_cloud")?;
    println!("CloudViewer created with builder pattern");
    cloud_viewer.wait_for_stop(1000); // 1 second timeout

    // PCLVisualizer with builder
    let mut pcl_viewer = PclVisualizerBuilder::new()
        .window_name("Builder PCLVisualizer")
        .background_color(0.2, 0.2, 0.4)
        .coordinate_system(0.3, "builder_coords")
        .build()?;

    pcl_viewer.show_cloud(&cloud, "builder_cloud")?;
    pcl_viewer.set_point_size("builder_cloud", 4.0)?;
    pcl_viewer.set_point_color("builder_cloud", 1.0, 1.0, 0.0)?; // Yellow

    println!("PCLVisualizer created with builder pattern");

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

    println!("Builder pattern demo completed.");
    Ok(())
}

fn demo_api_features() -> PclResult<()> {
    println!("\n=== API Features Demo ===");

    // Demonstrate trait usage
    let cloud = create_sample_xyz_cloud()?;

    // Test different viewer types implementing the same trait
    fn use_viewer_trait<V: ViewerXYZ>(mut viewer: V, cloud: &PointCloud<XYZ>) -> PclResult<()> {
        viewer.show_cloud(cloud, "trait_cloud")?;
        Ok(())
    }

    println!("Testing trait-based interface...");

    match CloudViewer::new("Trait CloudViewer") {
        Ok(viewer) => {
            use_viewer_trait(viewer, &cloud)?;
            println!("CloudViewer trait usage successful");
        }
        Err(_) => println!("CloudViewer creation failed (expected in headless environment)"),
    }

    match PclVisualizer::new("Trait PCLVisualizer") {
        Ok(viewer) => {
            use_viewer_trait(viewer, &cloud)?;
            println!("PclVisualizer trait usage successful");
        }
        Err(_) => println!("PclVisualizer creation failed (expected in headless environment)"),
    }

    println!("API features demo completed.");
    Ok(())
}

fn main() -> PclResult<()> {
    println!("PCL Visualization Demo");
    println!("======================");

    // Check if we're in a headless environment
    if std::env::var("DISPLAY").is_err() && std::env::var("WAYLAND_DISPLAY").is_err() {
        println!("Warning: No display detected. Visualization may not work.");
        println!("This is normal in CI/headless environments.");
    }

    // Run demos - all errors are handled gracefully for headless environments
    match demo_cloud_viewer() {
        Ok(()) => println!("✓ CloudViewer demo completed successfully"),
        Err(e) => println!("✗ CloudViewer demo failed (expected in headless): {}", e),
    }

    match demo_pcl_visualizer() {
        Ok(()) => println!("✓ PCLVisualizer demo completed successfully"),
        Err(e) => println!("✗ PCLVisualizer demo failed (expected in headless): {}", e),
    }

    match demo_builder_patterns() {
        Ok(()) => println!("✓ Builder patterns demo completed successfully"),
        Err(e) => println!(
            "✗ Builder patterns demo failed (expected in headless): {}",
            e
        ),
    }

    match demo_api_features() {
        Ok(()) => println!("✓ API features demo completed successfully"),
        Err(e) => println!("✗ API features demo failed (expected in headless): {}", e),
    }

    println!("\nVisualization demo completed!");
    println!("Note: Errors are expected in headless environments (CI/no display).");
    println!("The API structure and safety are demonstrated even when display fails.");

    Ok(())
}
