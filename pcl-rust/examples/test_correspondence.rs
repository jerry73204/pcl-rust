//! Test correspondence estimation functionality

use pcl::common::PointCloudXYZ;
use pcl::error::PclResult;
use pcl::registration::correspondence::{
    Correspondence, CorrespondenceEstimation, CorrespondenceRejectorSampleConsensus,
    TransformationEstimationSVD,
};

fn main() -> PclResult<()> {
    println!("=== Testing Correspondence Estimation ===\n");

    // Create simple test clouds
    let mut source = PointCloudXYZ::new()?;
    let mut target = PointCloudXYZ::new()?;

    // Add some test points - create a simple pattern
    for i in 0..5 {
        let x = i as f32 * 0.1;
        source.push(x, 0.0, 0.0)?;
        // Target is slightly translated
        target.push(x + 0.05, 0.0, 0.0)?;
    }

    println!("Created test clouds:");
    println!("  Source: {} points", source.size());
    println!("  Target: {} points", target.size());

    // Test correspondence estimation
    println!("\n1. Testing CorrespondenceEstimation:");
    let mut ce = CorrespondenceEstimation::new()?;
    println!("   ✓ Created CorrespondenceEstimation");

    ce.set_input_source(&source)?;
    println!("   ✓ Set input source");

    ce.set_input_target(&target)?;
    println!("   ✓ Set input target");

    let correspondences = ce.determine_correspondences()?;
    println!("   ✓ Found {} correspondences", correspondences.len());

    // Print first few correspondences
    for (i, corr) in correspondences.iter().take(3).enumerate() {
        println!(
            "     Correspondence {}: source[{}] -> target[{}], distance: {:.4}",
            i, corr.source_index, corr.target_index, corr.distance
        );
    }

    // Test correspondence rejection
    println!("\n2. Testing CorrespondenceRejectorSampleConsensus:");
    let mut rejector = CorrespondenceRejectorSampleConsensus::new()?;
    println!("   ✓ Created CorrespondenceRejectorSampleConsensus");

    rejector.set_input_source(&source)?;
    rejector.set_input_target(&target)?;
    rejector.set_inlier_threshold(0.1);
    println!("   ✓ Configured rejector (threshold: 0.1)");

    let filtered = rejector.filter_correspondences(&correspondences)?;
    println!(
        "   ✓ Filtered correspondences: {} -> {} inliers",
        correspondences.len(),
        filtered.len()
    );

    // Test transformation estimation
    println!("\n3. Testing TransformationEstimationSVD:");
    let mut te = TransformationEstimationSVD::new()?;
    println!("   ✓ Created TransformationEstimationSVD");

    // Create manual correspondences for testing
    let test_corrs = vec![
        Correspondence::new(0, 0, 0.05),
        Correspondence::new(1, 1, 0.05),
        Correspondence::new(2, 2, 0.05),
    ];

    let transform = te.estimate_rigid_transformation(&source, &target, &test_corrs)?;
    println!("   ✓ Estimated transformation");
    println!(
        "   ✓ Transform matrix has {} elements",
        transform.to_vec().len()
    );

    // Print first row of transformation matrix
    let data = transform.to_vec();
    println!(
        "     First row: [{:.4}, {:.4}, {:.4}, {:.4}]",
        data[0], data[1], data[2], data[3]
    );

    println!("\n=== All Tests Passed! ===");
    Ok(())
}
