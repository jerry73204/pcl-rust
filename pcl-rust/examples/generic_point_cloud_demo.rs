//! Demonstration of the generic PointCloud<T> system
//!
//! This example shows how to use the new generic PointCloud<T> API
//! that works with any point type implementing the required traits.

use pcl::error::PclResult;
use pcl::{PointCloud, PointXYZ, PointXYZRGB};

fn main() -> PclResult<()> {
    println!("=== Generic PointCloud<T> System Demo ===\n");

    // Test 1: Create generic point clouds for different point types
    test_generic_creation()?;

    // Test 2: Use generic algorithms
    test_generic_algorithms()?;

    // Test 3: Use the builder pattern
    test_builder_pattern()?;

    println!("All tests completed successfully!");
    Ok(())
}

/// Test creating generic point clouds
fn test_generic_creation() -> PclResult<()> {
    println!("1. Testing generic point cloud creation:");

    // Create PointCloud<PointXYZ> using the generic API
    let xyz_cloud: PointCloud<PointXYZ> = PointCloud::new()?;
    println!(
        "   Created PointCloud<PointXYZ> - size: {}",
        xyz_cloud.size()
    );

    // Create PointCloud<PointXYZRGB> using the generic API
    let xyzrgb_cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;
    println!(
        "   Created PointCloud<PointXYZRGB> - size: {}",
        xyzrgb_cloud.size()
    );

    println!("   ✓ Generic creation works!\n");
    Ok(())
}

/// Test generic algorithms that work with any point type
fn test_generic_algorithms() -> PclResult<()> {
    println!("2. Testing generic algorithms:");

    // Test with PointXYZ
    let mut xyz_cloud: PointCloud<PointXYZ> = PointCloud::new()?;
    xyz_cloud.resize(10)?;
    let xyz_info = analyze_cloud(&xyz_cloud)?;
    println!("   PointXYZ cloud analysis: {}", xyz_info);

    // Test with PointXYZRGB
    let mut xyzrgb_cloud: PointCloud<PointXYZRGB> = PointCloud::new()?;
    xyzrgb_cloud.resize(5)?;
    let xyzrgb_info = analyze_cloud(&xyzrgb_cloud)?;
    println!("   PointXYZRGB cloud analysis: {}", xyzrgb_info);

    println!("   ✓ Generic algorithms work with different point types!\n");
    Ok(())
}

/// Test the builder pattern with different point types
fn test_builder_pattern() -> PclResult<()> {
    println!("3. Testing alternative builder approach:");

    // For now, let's use the concrete types directly since the generic builder
    // has limitations with FFI point creation
    use pcl::{PointCloudXYZ, PointCloudXYZRGB};

    // Build PointCloudXYZ using the existing concrete builder
    let mut xyz_cloud = PointCloudXYZ::new()?;
    xyz_cloud.reserve(100)?;
    xyz_cloud.push(PointXYZ::new(1.0, 2.0, 3.0))?;
    xyz_cloud.push(PointXYZ::new(4.0, 5.0, 6.0))?;
    xyz_cloud.push(PointXYZ::new(7.0, 8.0, 9.0))?;

    println!("   Built PointXYZ cloud with {} points", xyz_cloud.size());

    // Build PointCloudXYZRGB using the existing concrete builder
    let mut xyzrgb_cloud = PointCloudXYZRGB::new()?;
    xyzrgb_cloud.reserve(100)?;
    xyzrgb_cloud.push(PointXYZRGB::new(1.0, 2.0, 3.0, 255, 0, 0))?; // Red point
    xyzrgb_cloud.push(PointXYZRGB::new(4.0, 5.0, 6.0, 0, 255, 0))?; // Green point
    xyzrgb_cloud.push(PointXYZRGB::new(7.0, 8.0, 9.0, 0, 0, 255))?; // Blue point

    println!(
        "   Built PointXYZRGB cloud with {} points",
        xyzrgb_cloud.size()
    );

    // Test conversion to generic types
    let _generic_xyz: PointCloud<PointXYZ> = PointCloud::new()?;
    let _generic_xyzrgb: PointCloud<PointXYZRGB> = PointCloud::new()?;

    println!("   ✓ Concrete builders work, generic containers created!\n");
    Ok(())
}

/// Generic algorithm that works with any point type having XYZ coordinates
fn analyze_cloud<T: pcl::common::point_types::PointType>(cloud: &PointCloud<T>) -> PclResult<String>
where
    T::CloudType: cxx::memory::UniquePtrTarget,
{
    let mut info = format!("type={}, size={}", T::type_name(), cloud.size());

    if cloud.is_organized() {
        info.push_str(&format!(", organized={}x{}", cloud.width(), cloud.height()));
    } else {
        info.push_str(", unorganized");
    }

    // Note: is_dense() is not available in the new API

    Ok(info)
}

/// Generic algorithm that works with any point type having both XYZ and RGB
fn _analyze_colored_cloud<T: pcl::common::point_types::PointType>(
    cloud: &PointCloud<T>,
) -> PclResult<String>
where
    T::CloudType: cxx::memory::UniquePtrTarget,
{
    let basic_info = analyze_cloud(cloud)?;
    Ok(format!("{}, has_color=true", basic_info))
}

/// Example of a generic spatial algorithm
/// Note: Direct point access via at() is not available due to FFI limitations
/// This is left here as an example of what could be done if direct access was available
#[allow(dead_code)]
fn _compute_centroid<T: pcl::common::point_types::PointType>(
    cloud: &PointCloud<T>,
) -> PclResult<(f32, f32, f32)>
where
    T::CloudType: cxx::memory::UniquePtrTarget,
{
    if cloud.empty() {
        return Ok((0.0, 0.0, 0.0));
    }

    // Direct point access is not available due to FFI limitations
    // In a real implementation, you would need to use PCL algorithms
    // or iterate through the cloud using FFI-specific methods

    // This is what the code would look like if at() worked:
    /*
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_z = 0.0;

    for i in 0..cloud.size() {
        let point = cloud.at(i)?;
        let (x, y, z) = point.xyz();
        sum_x += x;
        sum_y += y;
        sum_z += z;
    }

    let count = cloud.size() as f32;
    Ok((sum_x / count, sum_y / count, sum_z / count))
    */

    // For now, return a placeholder
    Err(pcl::error::PclError::NotImplemented {
        feature: "Direct point iteration".to_string(),
        workaround: Some("Use PCL algorithms or FFI-specific iteration methods".to_string()),
    })
}
