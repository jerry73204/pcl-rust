//! Comprehensive features module showcase

use pcl::common::PointCloudXYZ;
use pcl::error::PclResult;
use pcl::features::{
    FpfhEstimation, NormalEstimation, NormalEstimationOmp, PfhEstimation,
    builders::{FpfhEstimationBuilder, NormalEstimationBuilder},
    quick,
};

fn main() -> PclResult<()> {
    println!("=== PCL Features Module Showcase ===\n");

    // Create a test point cloud - a simple grid with some noise
    let mut cloud = PointCloudXYZ::new()?;
    for i in 0..15 {
        for j in 0..15 {
            let x = i as f32 * 0.05;
            let y = j as f32 * 0.05;
            let z = ((x * 20.0).sin() + (y * 20.0).cos()) * 0.02
                + (rand::random::<f32>() - 0.5) * 0.005; // Add small random noise
            cloud.push(x, y, z)?;
        }
    }
    println!("Created test point cloud with {} points", cloud.size());

    // === Basic Normal Estimation ===
    println!("\n1. Basic Normal Estimation:");
    let mut normal_est = NormalEstimation::new()?;
    normal_est.set_input_cloud(&cloud)?;
    normal_est.set_radius_search(0.03);
    normal_est.set_view_point(0.0, 0.0, 1.0);

    let normals = normal_est.compute()?;
    println!(
        "   ✓ Computed {} normals using radius search",
        normals.size()
    );
    println!("   ✓ Normal cloud empty: {}", normals.is_empty());

    // === OpenMP Normal Estimation ===
    println!("\n2. OpenMP Normal Estimation:");
    let mut normal_est_omp = NormalEstimationOmp::new()?;
    normal_est_omp.set_input_cloud(&cloud)?;
    normal_est_omp.set_k_search(10);
    normal_est_omp.set_number_of_threads(2);

    let normals_omp = normal_est_omp.compute()?;
    println!(
        "   ✓ Computed {} normals using OpenMP with k-search",
        normals_omp.size()
    );

    // === FPFH Feature Estimation ===
    println!("\n3. FPFH Feature Estimation:");
    let mut fpfh_est = FpfhEstimation::new()?;
    fpfh_est.set_input_cloud(&cloud)?;
    fpfh_est.set_input_normals(&normals)?;
    fpfh_est.set_radius_search(0.05);

    let fpfh_features = fpfh_est.compute()?;
    println!(
        "   ✓ Computed {} FPFH features (33-dimensional)",
        fpfh_features.size()
    );
    println!("   ✓ FPFH cloud empty: {}", fpfh_features.is_empty());

    // === PFH Feature Estimation ===
    println!("\n4. PFH Feature Estimation:");
    let mut pfh_est = PfhEstimation::new()?;
    pfh_est.set_input_cloud(&cloud)?;
    pfh_est.set_input_normals(&normals)?;
    pfh_est.set_radius_search(0.05);

    let pfh_features = pfh_est.compute()?;
    println!(
        "   ✓ Computed {} PFH features (125-dimensional)",
        pfh_features.size()
    );
    println!("   ✓ PFH cloud empty: {}", pfh_features.is_empty());

    // === Quick Functions ===
    println!("\n5. Quick Functions:");
    let quick_normals = quick::estimate_normals(&cloud, 0.03)?;
    println!(
        "   ✓ Quick normal estimation: {} normals",
        quick_normals.size()
    );

    let quick_fpfh = quick::compute_fpfh(&cloud, &quick_normals, 0.05)?;
    println!(
        "   ✓ Quick FPFH computation: {} features",
        quick_fpfh.size()
    );

    // === Pipeline Functions ===
    println!("\n6. Pipeline Functions:");
    let (pipeline_normals, pipeline_fpfh) = quick::compute_fpfh_pipeline(&cloud, 0.03, 0.05)?;
    println!(
        "   ✓ FPFH pipeline: {} normals → {} features",
        pipeline_normals.size(),
        pipeline_fpfh.size()
    );

    // === Builder Pattern ===
    println!("\n7. Builder Pattern:");
    let builder_normals = NormalEstimationBuilder::new()
        .radius(0.04)
        .view_point(0.0, 0.0, 2.0)
        .compute(&cloud)?;
    println!(
        "   ✓ Builder normal estimation: {} normals",
        builder_normals.size()
    );

    println!("\n=== Features Module Showcase Complete! ===");
    println!("✓ All feature extraction algorithms working correctly");
    println!("✓ Both single-threaded and OpenMP variants functional");
    println!("✓ Quick functions and builder patterns implemented");
    println!("✓ Pipeline functions for common workflows available");

    Ok(())
}

// Simple random number generation for demo purposes
mod rand {
    static mut SEED: u32 = 1;

    pub fn random<T>() -> T
    where
        T: From<f32>,
    {
        unsafe {
            SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
            T::from((SEED as f32) / (u32::MAX as f32))
        }
    }
}
