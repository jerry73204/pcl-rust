//! Example demonstrating ICP (Iterative Closest Point) registration
//!
//! This example shows how to align two point clouds using the ICP algorithm.

use pcl::registration::RegistrationXYZ;
use pcl::{IcpXYZ, PclResult, PointCloudXYZ, TransformationMatrix};

fn main() -> PclResult<()> {
    println!("ICP Registration Demo");
    println!("====================");

    // Create source and target point clouds
    // In a real application, these would be loaded from files
    let mut source = PointCloudXYZ::new()?;
    let mut target = PointCloudXYZ::new()?;

    // For demonstration, create some dummy points
    // (In practice, you would load these from PCD/PLY files)
    source.resize(100)?; // Create 100 points
    target.resize(100)?; // Create 100 points

    println!("Source cloud size: {}", source.size());
    println!("Target cloud size: {}", target.size());

    // Create an ICP instance
    let mut icp = IcpXYZ::new()?;

    // Configure ICP parameters
    icp.set_max_iterations(50)?;
    icp.set_transformation_epsilon(1e-8)?;
    icp.set_euclidean_fitness_epsilon(1e-6)?;
    icp.set_max_correspondence_distance(0.05)?; // 5cm

    println!("\nICP Configuration:");
    println!("Max iterations: {}", icp.get_max_iterations());
    println!(
        "Transformation epsilon: {}",
        icp.get_transformation_epsilon()
    );
    println!(
        "Euclidean fitness epsilon: {}",
        icp.get_euclidean_fitness_epsilon()
    );
    println!(
        "Max correspondence distance: {}",
        icp.get_max_correspondence_distance()
    );

    // Set input clouds
    icp.set_input_source(&source)?;
    icp.set_input_target(&target)?;

    // Perform alignment
    println!("\nPerforming ICP alignment...");
    let aligned = icp.align()?;

    // Check results
    println!("\nAlignment Results:");
    println!("Converged: {}", icp.has_converged());
    println!("Fitness score: {}", icp.get_fitness_score());
    println!("Aligned cloud size: {}", aligned.size());

    // Get the transformation matrix
    let transform = icp.get_final_transformation();
    println!("\nFinal transformation matrix:");
    print_transformation_matrix(&transform);

    // Example with initial guess
    println!("\n\nTesting ICP with initial transformation guess:");

    // Create an initial guess (small translation)
    let mut initial_guess = TransformationMatrix::identity();
    initial_guess.set_translation(0.1, 0.0, 0.0);

    println!("Initial guess:");
    print_transformation_matrix(&initial_guess);

    // Align with guess
    let aligned_with_guess = icp.align_with_guess(&initial_guess)?;

    println!("\nAlignment with guess results:");
    println!("Converged: {}", icp.has_converged());
    println!("Fitness score: {}", icp.get_fitness_score());
    println!("Aligned cloud size: {}", aligned_with_guess.size());

    // Demonstrate builder pattern
    println!("\n\nUsing ICP builder pattern:");
    use pcl::registration::icp::IcpXYZBuilder;

    let _icp2 = IcpXYZBuilder::new()
        .max_iterations(100)
        .transformation_epsilon(1e-10)
        .euclidean_fitness_epsilon(1e-8)
        .max_correspondence_distance(0.1)
        .build()?;

    println!("ICP created with builder successfully!");

    // Demonstrate transformation utilities
    println!("\n\nTransformation utilities:");

    // Create rotation around Z axis
    let rotation = TransformationMatrix::rotation_z(std::f32::consts::PI / 4.0); // 45 degrees
    println!("45° rotation around Z:");
    print_transformation_matrix(&rotation);

    // Create translation
    let translation = TransformationMatrix::translation_matrix(1.0, 2.0, 3.0);
    println!("\nTranslation (1, 2, 3):");
    print_transformation_matrix(&translation);

    // Combine transformations
    let combined = rotation * translation;
    println!("\nCombined (rotation then translation):");
    print_transformation_matrix(&combined);

    // Transform a point
    let (x, y, z) = combined.transform_point(1.0, 0.0, 0.0);
    println!(
        "\nPoint (1, 0, 0) transformed to: ({:.3}, {:.3}, {:.3})",
        x, y, z
    );

    println!("\nICP registration demo completed successfully!");
    Ok(())
}

fn print_transformation_matrix(t: &TransformationMatrix) {
    for i in 0..4 {
        print!("  [");
        for j in 0..4 {
            print!("{:7.3}", t[(i, j)]);
            if j < 3 {
                print!(", ");
            }
        }
        println!("]");
    }
}
