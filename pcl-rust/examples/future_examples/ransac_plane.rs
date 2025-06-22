//! Example demonstrating RANSAC plane fitting
//!
//! This example shows how to use the RANSAC algorithm to fit a plane
//! to a point cloud containing points and outliers.

use pcl::{common::{PointCloud, XYZ}, RansacPlaneXYZ};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("PCL Rust RANSAC Plane Fitting Example");
    println!("=====================================");

    // Create a point cloud (empty for now, as we don't have point creation yet)
    let cloud = PointCloud::<XYZ>::new()?;

    println!("Created point cloud with {} points", cloud.size());

    // For now, we'll just demonstrate the API with an empty cloud
    // In a real application, you would load points from a file or add them programmatically
    if !cloud.empty() {
        // Create RANSAC plane fitter
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;

        // Configure RANSAC parameters
        ransac.set_distance_threshold(0.01); // 1cm threshold
        ransac.set_max_iterations(1000);
        ransac.set_probability(0.99);

        println!("RANSAC Configuration:");
        println!("  Distance threshold: {:.3}", ransac.distance_threshold());
        println!("  Max iterations: {}", ransac.max_iterations());
        println!("  Probability: {:.2}", ransac.probability());

        // Compute the plane model
        match ransac.compute_model() {
            Ok(()) => {
                let coefficients = ransac.model_coefficients();
                let inliers = ransac.inliers();

                println!("\nPlane fitting successful!");
                println!(
                    "Plane coefficients: [{:.3}, {:.3}, {:.3}, {:.3}]",
                    coefficients.get(0).unwrap_or(&0.0),
                    coefficients.get(1).unwrap_or(&0.0),
                    coefficients.get(2).unwrap_or(&0.0),
                    coefficients.get(3).unwrap_or(&0.0)
                );
                println!(
                    "Number of inliers: {} out of {} points",
                    inliers.len(),
                    cloud.size()
                );

                // Optionally refine the model
                println!("\nRefining model...");
                ransac.refine_model(0.01, 100)?;

                let refined_coefficients = ransac.model_coefficients();
                println!(
                    "Refined coefficients: [{:.3}, {:.3}, {:.3}, {:.3}]",
                    refined_coefficients.get(0).unwrap_or(&0.0),
                    refined_coefficients.get(1).unwrap_or(&0.0),
                    refined_coefficients.get(2).unwrap_or(&0.0),
                    refined_coefficients.get(3).unwrap_or(&0.0)
                );
            }
            Err(e) => {
                println!("Plane fitting failed: {}", e);
            }
        }
    } else {
        println!("Point cloud is empty. In a real application, you would:");
        println!("1. Load points from a file using I/O functions");
        println!("2. Add points programmatically (when point creation is available)");
        println!("3. Then use RANSAC to fit geometric models");

        // Still demonstrate the API creation
        let mut ransac = RansacPlaneXYZ::new(&cloud)?;
        println!("\nRANSAC plane fitter created successfully!");
        println!("Default configuration:");
        println!("  Distance threshold: {:.3}", ransac.distance_threshold());
        println!("  Max iterations: {}", ransac.max_iterations());
        println!("  Probability: {:.2}", ransac.probability());

        // Test configuration changes
        ransac.set_distance_threshold(0.05);
        ransac.set_max_iterations(500);
        ransac.set_probability(0.95);

        println!("\nAfter configuration:");
        println!("  Distance threshold: {:.3}", ransac.distance_threshold());
        println!("  Max iterations: {}", ransac.max_iterations());
        println!("  Probability: {:.2}", ransac.probability());
    }

    Ok(())
}
