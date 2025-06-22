//! Example demonstrating PCL filters usage
//!
//! This example shows how to use the PassThrough filter to crop point clouds
//! along specified dimensions.

use pcl::{
    PclResult,
    common::{PointCloud, XYZ},
    filters::Filter,
    filters::PassThroughXYZ,
};

fn main() -> PclResult<()> {
    println!("PCL Filters Demo");
    println!("================");

    // Create a point cloud (normally this would be loaded from a file)
    let mut cloud = PointCloud::<XYZ>::new()?;
    cloud.resize(100)?; // Create 100 points

    println!("Original cloud size: {}", cloud.size());
    println!("Cloud is empty: {}", cloud.empty());

    // Create a PassThrough filter
    let mut filter = PassThroughXYZ::new()?;

    // Configure the filter
    filter.set_filter_field_name("z")?; // Filter along Z-axis
    filter.set_filter_limits(0.0, 2.0)?; // Keep points between 0 and 2 meters
    filter.set_negative(false)?; // Keep points inside the limits
    filter.set_keep_organized(false)?; // Don't maintain NaN points

    // Set the input cloud
    filter.set_input_cloud(&cloud)?;

    // Apply the filter
    let filtered_cloud = filter.filter()?;

    println!("Filtered cloud size: {}", filtered_cloud.size());
    println!("Filter field: {}", filter.get_filter_field_name()?);
    println!("Negative filter: {}", filter.get_negative()?);
    println!("Keep organized: {}", filter.get_keep_organized()?);

    println!("\nTesting filter builder pattern:");

    // Demonstrate the builder pattern
    use pcl::filters::pass_through::PassThroughXYZBuilder;

    let filter2 = PassThroughXYZBuilder::new()
        .field_name("x")
        .limits(-1.0, 1.0)
        .negative(true) // Keep points OUTSIDE the limits
        .keep_organized(false)
        .build()?;

    println!("Built filter field: {}", filter2.get_filter_field_name()?);
    println!("Built filter negative: {}", filter2.get_negative()?);

    println!("\nFilters demo completed successfully!");
    Ok(())
}
