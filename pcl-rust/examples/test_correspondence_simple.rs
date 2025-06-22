//! Simple test for correspondence estimation

use pcl::common::{PointCloud, PointXYZ, XYZ};
use pcl::error::PclResult;
use pcl::registration::correspondence::CorrespondenceEstimation;

fn main() -> PclResult<()> {
    println!("=== Simple Correspondence Test ===\n");

    // Create test clouds with more points
    let mut source = PointCloud::<XYZ>::new()?;
    let mut target = PointCloud::<XYZ>::new()?;

    // Create a 3x3 grid of points
    for i in 0..3 {
        for j in 0..3 {
            let x = i as f32 * 0.1;
            let y = j as f32 * 0.1;
            source.push(PointXYZ::new(x, y, 0.0))?;
            // Target is translated by (0.05, 0.05, 0.0)
            target.push(PointXYZ::new(x + 0.05, y + 0.05, 0.0))?;
        }
    }

    println!("Created test clouds:");
    println!("  Source: {} points", source.size());
    println!("  Target: {} points", target.size());

    // Test correspondence estimation
    println!("\nTesting CorrespondenceEstimation:");
    let mut ce = CorrespondenceEstimation::new()?;
    println!("  ✓ Created");

    ce.set_input_source(&source)?;
    println!("  ✓ Set source");

    ce.set_input_target(&target)?;
    println!("  ✓ Set target");

    println!("  Determining correspondences...");
    let correspondences = ce.determine_correspondences()?;
    println!("  ✓ Found {} correspondences", correspondences.len());

    // Show all correspondences
    for (i, corr) in correspondences.iter().enumerate() {
        println!(
            "    {}: source[{}] -> target[{}], dist: {:.6}",
            i, corr.source_index, corr.target_index, corr.distance
        );
    }

    println!("\nTesting reciprocal correspondences:");
    let reciprocal = ce.determine_reciprocal_correspondences()?;
    println!("  ✓ Found {} reciprocal correspondences", reciprocal.len());

    println!("\n=== Test Complete ===");
    Ok(())
}
