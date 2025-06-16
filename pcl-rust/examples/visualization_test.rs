//! Interactive visualization test with synthetic point cloud data
//!
//! This example creates synthetic point clouds with actual data points
//! and demonstrates various visualization features.

use pcl::common::{PointCloudXYZBuilder, PointCloudXYZRGBBuilder};
use pcl::{
    CameraControl, PclError, PclResult, PclVisualizer, PointCloudXYZ, PointCloudXYZRGB, ViewerXYZ,
    ViewerXYZRGB, VisualizationControl,
};
use std::f64::consts::PI;

/// Generate a sphere point cloud
fn generate_sphere(radius: f64, num_points: usize) -> PclResult<PointCloudXYZ> {
    let mut builder = PointCloudXYZBuilder::new();

    // Generate points on sphere surface using spherical coordinates
    let golden_ratio = (1.0 + 5.0_f64.sqrt()) / 2.0;
    for i in 0..num_points {
        let theta = 2.0 * PI * i as f64 / golden_ratio;
        let phi = (1.0 - 2.0 * (i as f64 + 0.5) / num_points as f64).acos();

        let x = radius * phi.sin() * theta.cos();
        let y = radius * phi.sin() * theta.sin();
        let z = radius * phi.cos();

        builder = builder.add_point(x as f32, y as f32, z as f32);
    }

    builder.build()
}

/// Generate a colorful helix point cloud
fn generate_helix(
    radius: f64,
    height: f64,
    turns: f64,
    num_points: usize,
) -> PclResult<PointCloudXYZRGB> {
    let mut builder = PointCloudXYZRGBBuilder::new();

    for i in 0..num_points {
        let t = i as f64 / num_points as f64;
        let angle = 2.0 * PI * turns * t;

        let x = radius * angle.cos();
        let y = radius * angle.sin();
        let z = height * t - height / 2.0;

        // Color gradient along the helix
        let r = (255.0 * t) as u8;
        let g = (255.0 * (1.0 - t)) as u8;
        let b = (255.0 * (0.5 + 0.5 * angle.sin())) as u8;

        builder = builder.add_point_rgb(x as f32, y as f32, z as f32, r, g, b);
    }

    builder.build()
}

/// Generate a cubic grid of points
fn generate_grid(size: f64, density: usize) -> PclResult<PointCloudXYZ> {
    let mut builder = PointCloudXYZBuilder::new();
    let step = size / density as f64;
    let offset = size / 2.0;

    for i in 0..density {
        for j in 0..density {
            for k in 0..density {
                let x = i as f64 * step - offset;
                let y = j as f64 * step - offset;
                let z = k as f64 * step - offset;

                builder = builder.add_point(x as f32, y as f32, z as f32);
            }
        }
    }

    builder.build()
}

/// Generate a torus (donut) shape
fn generate_torus(
    major_radius: f64,
    minor_radius: f64,
    num_major: usize,
    num_minor: usize,
) -> PclResult<PointCloudXYZRGB> {
    let mut builder = PointCloudXYZRGBBuilder::new();

    for i in 0..num_major {
        let theta = 2.0 * PI * i as f64 / num_major as f64;

        // Color changes around major circumference
        let hue = i as f64 / num_major as f64;
        let (r, g, b) = hsv_to_rgb(hue * 360.0, 1.0, 1.0);

        for j in 0..num_minor {
            let phi = 2.0 * PI * j as f64 / num_minor as f64;

            let x = (major_radius + minor_radius * phi.cos()) * theta.cos();
            let y = (major_radius + minor_radius * phi.cos()) * theta.sin();
            let z = minor_radius * phi.sin();

            builder = builder.add_point_rgb(x as f32, y as f32, z as f32, r, g, b);
        }
    }

    builder.build()
}

/// Convert HSV to RGB color space
fn hsv_to_rgb(h: f64, s: f64, v: f64) -> (u8, u8, u8) {
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

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

    (
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
    )
}

fn visualize_all_shapes() -> PclResult<()> {
    println!("Creating visualization with synthetic point clouds...");

    // Generate all shapes
    let sphere = generate_sphere(1.0, 1000)?;
    println!("Generated sphere with {} points", sphere.size());

    let helix = generate_helix(0.5, 2.0, 3.0, 500)?;
    println!("Generated helix with {} points", helix.size());

    let grid = generate_grid(1.5, 10)?;
    println!("Generated grid with {} points", grid.size());

    let torus = generate_torus(1.0, 0.3, 50, 30)?;
    println!("Generated torus with {} points", torus.size());

    // Create visualizer
    let mut viewer = PclVisualizer::new("PCL-Rust Visualization Test")?;

    // Configure visualization
    viewer.set_background_color(0.05, 0.05, 0.1)?;
    viewer.add_coordinate_system(0.5, "world")?;

    // Add sphere (white points)
    viewer.show_cloud(&sphere, "sphere")?;
    viewer.set_point_size("sphere", 2.0)?;
    viewer.set_point_color("sphere", 1.0, 1.0, 1.0)?;

    // Add helix (uses per-point colors)
    viewer.show_cloud_rgb(&helix, "helix")?;
    viewer.set_point_size("helix", 3.0)?;

    // Add grid (cyan points)
    viewer.show_cloud(&grid, "grid")?;
    viewer.set_point_size("grid", 1.0)?;
    viewer.set_point_color("grid", 0.0, 1.0, 1.0)?;
    viewer.set_opacity("grid", 0.3)?;

    // Add torus (uses per-point colors)
    viewer.show_cloud_rgb(&torus, "torus")?;
    viewer.set_point_size("torus", 2.0)?;

    // Add labels
    viewer.add_text(
        "PCL-Rust Visualization Test",
        10,
        60,
        1.0,
        1.0,
        1.0,
        "title",
    )?;
    viewer.add_text("Sphere (1000 pts)", 10, 40, 1.0, 1.0, 1.0, "label1")?;
    viewer.add_text("Helix (500 pts)", 10, 25, 1.0, 0.5, 0.0, "label2")?;
    viewer.add_text("Grid (1000 pts)", 10, 10, 0.0, 1.0, 1.0, "label3")?;

    // Set initial camera position
    viewer.set_camera_position(
        3.0, 3.0, 3.0, // eye position
        0.0, 0.0, 0.0, // center point
        0.0, 0.0, 1.0, // up vector
    )?;

    println!("\nVisualization ready!");
    println!("Controls:");
    println!("  - Left mouse: Rotate");
    println!("  - Right mouse: Zoom");
    println!("  - Middle mouse: Pan");
    println!("  - Press 'q' to quit");
    println!("  - Press 'r' to reset camera");
    println!("  - Press 'g' to toggle grid");

    // Main visualization loop
    let mut frame_count = 0;
    let mut last_update = std::time::Instant::now();

    while !viewer.was_stopped() {
        viewer.spin_once(50)?;
        frame_count += 1;

        // Rotate the sphere slowly
        if frame_count % 2 == 0 {
            let angle = frame_count as f64 * 0.01;
            let rotated_sphere = rotate_cloud(&sphere, angle)?;
            viewer.update_point_cloud_xyz(&rotated_sphere, "sphere")?;
        }

        // Print FPS every second
        let now = std::time::Instant::now();
        if now.duration_since(last_update).as_secs() >= 1 {
            println!("FPS: {}", frame_count);
            frame_count = 0;
            last_update = now;
        }
    }

    println!("\nVisualization stopped.");
    Ok(())
}

/// Simple rotation around Z axis
fn rotate_cloud(_cloud: &PointCloudXYZ, angle: f64) -> PclResult<PointCloudXYZ> {
    let mut builder = PointCloudXYZBuilder::new();
    let _cos_a = angle.cos() as f32;
    let _sin_a = angle.sin() as f32;

    // Note: We can't directly access points, so we'll just create a new rotated sphere
    // In a real application with point access, you'd iterate through existing points
    let radius = 1.0;
    let num_points = 1000;
    let golden_ratio = (1.0 + 5.0_f64.sqrt()) / 2.0;

    for i in 0..num_points {
        let theta = 2.0 * PI * i as f64 / golden_ratio + angle;
        let phi = (1.0 - 2.0 * (i as f64 + 0.5) / num_points as f64).acos();

        let x = radius * phi.sin() * theta.cos();
        let y = radius * phi.sin() * theta.sin();
        let z = radius * phi.cos();

        builder = builder.add_point(x as f32, y as f32, z as f32);
    }

    builder.build()
}

fn main() -> PclResult<()> {
    println!("PCL-Rust Visualization Test");
    println!("===========================");

    // Check display environment
    let display = std::env::var("DISPLAY").unwrap_or_else(|_| "None".to_string());
    println!("DISPLAY: {}", display);

    match visualize_all_shapes() {
        Ok(()) => {
            println!("✓ Visualization completed successfully!");
        }
        Err(e) => {
            eprintln!("✗ Visualization failed: {}", e);
            match e {
                PclError::CreationFailed { typename } => {
                    eprintln!("  Failed to create: {}", typename);
                    eprintln!("  This usually means no display is available.");
                }
                PclError::VisualizationError { message, operation } => {
                    eprintln!("  Operation: {}", operation);
                    eprintln!("  Message: {}", message);
                }
                _ => {
                    eprintln!("  Error: {:?}", e);
                }
            }
            std::process::exit(1);
        }
    }

    Ok(())
}
