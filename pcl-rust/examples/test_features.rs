//! Test example for the features module

use pcl::common::PointCloudXYZ;
use pcl::error::PclResult;
use pcl::features::{FpfhEstimation, NormalEstimation, PfhEstimation};

fn main() -> PclResult<()> {
    println!("Testing PCL features module...");

    // Create a simple test point cloud
    let mut cloud = PointCloudXYZ::new()?;

    // Add some test points in a simple pattern
    for i in 0..10 {
        for j in 0..10 {
            let x = i as f32 * 0.1;
            let y = j as f32 * 0.1;
            let z = ((x * x + y * y) * 10.0).sin() * 0.05; // Simple surface
            cloud.push(x, y, z)?;
        }
    }

    println!("Created point cloud with {} points", cloud.size());

    // Test Normal Estimation
    println!("\nTesting Normal Estimation...");
    let mut normal_est = NormalEstimation::new()?;
    normal_est.set_input_cloud(&cloud)?;
    normal_est.set_radius_search(0.05);

    let normals = normal_est.compute()?;
    println!("Computed normals for {} points", normals.size());

    // Test FPFH Estimation
    println!("\nTesting FPFH Estimation...");
    let mut fpfh_est = FpfhEstimation::new()?;
    fpfh_est.set_input_cloud(&cloud)?;
    fpfh_est.set_input_normals(&normals)?;
    fpfh_est.set_radius_search(0.08);

    let fpfh_features = fpfh_est.compute()?;
    println!("Computed FPFH features for {} points", fpfh_features.size());

    // Test PFH Estimation
    println!("\nTesting PFH Estimation...");
    let mut pfh_est = PfhEstimation::new()?;
    pfh_est.set_input_cloud(&cloud)?;
    pfh_est.set_input_normals(&normals)?;
    pfh_est.set_radius_search(0.08);

    let pfh_features = pfh_est.compute()?;
    println!("Computed PFH features for {} points", pfh_features.size());

    println!("\nAll feature tests completed successfully!");
    Ok(())
}
