//! Example demonstrating feature extraction algorithms
//!
//! This example shows how to use PCL's feature extraction capabilities
//! for computing normal vectors, FPFH, and PFH descriptors.

use pcl::{FpfhEstimation, NormalEstimation, PclResult, PfhEstimation, PointCloudXYZ};

fn main() -> PclResult<()> {
    println!("PCL-Rust Feature Extraction Example");
    println!("===================================");

    // Create a new point cloud (normally you'd load this from a file)
    let cloud = PointCloudXYZ::new()?;
    println!("Created point cloud with {} points", cloud.size());

    // Example 1: Normal Estimation
    println!("\n1. Normal Estimation");
    println!("-------------------");

    let mut normal_estimator = NormalEstimation::new()?;
    println!("✓ Created normal estimation instance");

    // In a real implementation, you would:
    // normal_estimator.set_input_cloud(&cloud)?;
    // normal_estimator.set_radius_search(0.03);
    // let normals = normal_estimator.compute()?;

    normal_estimator.set_radius_search(0.03);
    println!("✓ Set radius search parameter to 0.03");

    normal_estimator.set_k_search(50);
    println!("✓ Set k-nearest neighbors to 50");

    normal_estimator.set_view_point(0.0, 0.0, 0.0);
    let vp = normal_estimator.get_view_point();
    println!("✓ Set view point to ({}, {}, {})", vp[0], vp[1], vp[2]);

    // Example 2: FPFH Feature Estimation
    println!("\n2. FPFH Feature Estimation");
    println!("--------------------------");

    let mut fpfh_estimator = FpfhEstimation::new()?;
    println!("✓ Created FPFH estimation instance");

    fpfh_estimator.set_radius_search(0.05);
    println!("✓ Set FPFH radius search to 0.05");

    fpfh_estimator.set_k_search(50);
    println!("✓ Set FPFH k-nearest neighbors to 50");

    // Example 3: PFH Feature Estimation
    println!("\n3. PFH Feature Estimation");
    println!("-------------------------");

    let mut pfh_estimator = PfhEstimation::new()?;
    println!("✓ Created PFH estimation instance");

    pfh_estimator.set_radius_search(0.05);
    println!("✓ Set PFH radius search to 0.05");

    pfh_estimator.set_k_search(50);
    println!("✓ Set PFH k-nearest neighbors to 50");

    // Example 4: Feature data structures
    println!("\n4. Feature Data Structures");
    println!("--------------------------");

    use pcl::{FpfhSignature, Normal, PfhSignature};

    // Create a normal vector
    let normal = Normal::new(0.0, 0.0, 1.0, 0.02);
    println!("✓ Created normal: {:?}", normal.vector());
    println!("  - Curvature: {:.3}", normal.curvature);
    println!("  - Magnitude: {:.3}", normal.magnitude());

    // Create feature signatures
    let fpfh_signature = FpfhSignature::new();
    println!(
        "✓ Created FPFH signature with {} bins",
        fpfh_signature.histogram().len()
    );

    let pfh_signature = PfhSignature::new();
    println!(
        "✓ Created PFH signature with {} bins",
        pfh_signature.histogram().len()
    );

    println!("\n✅ Feature extraction example completed successfully!");
    println!("\nNote: This example demonstrates the API interface.");
    println!("For actual feature computation, you would need to:");
    println!("  1. Load a point cloud from a file");
    println!("  2. Set input clouds and search methods");
    println!("  3. Call compute() methods to extract features");
    println!("  4. Process the resulting feature descriptors");

    Ok(())
}
