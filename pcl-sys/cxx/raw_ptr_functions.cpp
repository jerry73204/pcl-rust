#include "raw_ptr_functions.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <memory>
#include <string>

extern "C" {

// ============================================================================
// PointCloud creation and destruction functions
// ============================================================================

pcl::PointCloud<pcl::PointXYZ>* pcl_pointcloud_xyz_new() {
    try {
        // Use new directly to respect PCL_MAKE_ALIGNED_OPERATOR_NEW
        return new pcl::PointCloud<pcl::PointXYZ>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_pointcloud_xyz_delete(pcl::PointCloud<pcl::PointXYZ>* cloud) {
    // Use delete directly to respect PCL's overloaded delete operator
    delete cloud;
}

pcl::PointCloud<pcl::PointXYZI>* pcl_pointcloud_xyzi_new() {
    try {
        return new pcl::PointCloud<pcl::PointXYZI>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_pointcloud_xyzi_delete(pcl::PointCloud<pcl::PointXYZI>* cloud) {
    delete cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>* pcl_pointcloud_xyzrgb_new() {
    try {
        return new pcl::PointCloud<pcl::PointXYZRGB>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_pointcloud_xyzrgb_delete(pcl::PointCloud<pcl::PointXYZRGB>* cloud) {
    delete cloud;
}

pcl::PointCloud<pcl::PointNormal>* pcl_pointcloud_normal_new() {
    try {
        return new pcl::PointCloud<pcl::PointNormal>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_pointcloud_normal_delete(pcl::PointCloud<pcl::PointNormal>* cloud) {
    delete cloud;
}

// ============================================================================
// Filter creation and destruction functions
// ============================================================================

pcl::PassThrough<pcl::PointXYZ>* pcl_passthrough_xyz_new() {
    try {
        // Note: Filter objects typically don't have PCL_MAKE_ALIGNED_OPERATOR_NEW
        // but we use new for consistency
        return new pcl::PassThrough<pcl::PointXYZ>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_passthrough_xyz_delete(pcl::PassThrough<pcl::PointXYZ>* filter) {
    delete filter;
}

pcl::PassThrough<pcl::PointXYZRGB>* pcl_passthrough_xyzrgb_new() {
    try {
        return new pcl::PassThrough<pcl::PointXYZRGB>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_passthrough_xyzrgb_delete(pcl::PassThrough<pcl::PointXYZRGB>* filter) {
    delete filter;
}

pcl::VoxelGrid<pcl::PointXYZ>* pcl_voxelgrid_xyz_new() {
    try {
        return new pcl::VoxelGrid<pcl::PointXYZ>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_voxelgrid_xyz_delete(pcl::VoxelGrid<pcl::PointXYZ>* filter) {
    delete filter;
}

pcl::VoxelGrid<pcl::PointXYZRGB>* pcl_voxelgrid_xyzrgb_new() {
    try {
        return new pcl::VoxelGrid<pcl::PointXYZRGB>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_voxelgrid_xyzrgb_delete(pcl::VoxelGrid<pcl::PointXYZRGB>* filter) {
    delete filter;
}

pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* pcl_statistical_xyz_new() {
    try {
        return new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_statistical_xyz_delete(pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter) {
    delete filter;
}

pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>* pcl_statistical_xyzrgb_new() {
    try {
        return new pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_statistical_xyzrgb_delete(pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>* filter) {
    delete filter;
}

pcl::RadiusOutlierRemoval<pcl::PointXYZ>* pcl_radius_xyz_new() {
    try {
        return new pcl::RadiusOutlierRemoval<pcl::PointXYZ>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_radius_xyz_delete(pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter) {
    delete filter;
}

pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>* pcl_radius_xyzrgb_new() {
    try {
        return new pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>();
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_radius_xyzrgb_delete(pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>* filter) {
    delete filter;
}

// ============================================================================
// Filter operation functions (using raw pointers)
// ============================================================================

void pcl_passthrough_xyz_set_input_cloud(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    if (filter && cloud) {
        // Create a shared pointer from the raw pointer
        // Note: This creates a shared_ptr that does NOT own the memory
        auto cloud_ptr = std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>(
            cloud, [](const pcl::PointCloud<pcl::PointXYZ>*){} // no-op deleter
        );
        filter->setInputCloud(cloud_ptr);
    }
}

void pcl_passthrough_xyz_set_filter_field_name(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    const char* field_name) {
    if (filter && field_name) {
        filter->setFilterFieldName(std::string(field_name));
    }
}

void pcl_passthrough_xyz_set_filter_limits(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    float min, float max) {
    if (filter) {
        filter->setFilterLimits(min, max);
    }
}

void pcl_passthrough_xyz_set_negative(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    bool negative) {
    if (filter) {
        filter->setNegative(negative);
    }
}

pcl::PointCloud<pcl::PointXYZ>* pcl_passthrough_xyz_filter(
    pcl::PassThrough<pcl::PointXYZ>* filter) {
    if (!filter) {
        return nullptr;
    }
    
    try {
        // Create output cloud using aligned allocation
        pcl::PointCloud<pcl::PointXYZ>* output = new pcl::PointCloud<pcl::PointXYZ>();
        filter->filter(*output);
        return output;
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_voxelgrid_xyz_set_input_cloud(
    pcl::VoxelGrid<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    if (filter && cloud) {
        auto cloud_ptr = std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>(
            cloud, [](const pcl::PointCloud<pcl::PointXYZ>*){} // no-op deleter
        );
        filter->setInputCloud(cloud_ptr);
    }
}

void pcl_voxelgrid_xyz_set_leaf_size(
    pcl::VoxelGrid<pcl::PointXYZ>* filter,
    float lx, float ly, float lz) {
    if (filter) {
        filter->setLeafSize(lx, ly, lz);
    }
}

pcl::PointCloud<pcl::PointXYZ>* pcl_voxelgrid_xyz_filter(
    pcl::VoxelGrid<pcl::PointXYZ>* filter) {
    if (!filter) {
        return nullptr;
    }
    
    try {
        pcl::PointCloud<pcl::PointXYZ>* output = new pcl::PointCloud<pcl::PointXYZ>();
        filter->filter(*output);
        return output;
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_statistical_xyz_set_input_cloud(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    if (filter && cloud) {
        auto cloud_ptr = std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>(
            cloud, [](const pcl::PointCloud<pcl::PointXYZ>*){} // no-op deleter
        );
        filter->setInputCloud(cloud_ptr);
    }
}

void pcl_statistical_xyz_set_mean_k(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    int mean_k) {
    if (filter) {
        filter->setMeanK(mean_k);
    }
}

void pcl_statistical_xyz_set_stddev_mul_thresh(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    double stddev_mult) {
    if (filter) {
        filter->setStddevMulThresh(stddev_mult);
    }
}

pcl::PointCloud<pcl::PointXYZ>* pcl_statistical_xyz_filter(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter) {
    if (!filter) {
        return nullptr;
    }
    
    try {
        pcl::PointCloud<pcl::PointXYZ>* output = new pcl::PointCloud<pcl::PointXYZ>();
        filter->filter(*output);
        return output;
    } catch (const std::exception&) {
        return nullptr;
    }
}

void pcl_radius_xyz_set_input_cloud(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    if (filter && cloud) {
        auto cloud_ptr = std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>(
            cloud, [](const pcl::PointCloud<pcl::PointXYZ>*){} // no-op deleter
        );
        filter->setInputCloud(cloud_ptr);
    }
}

void pcl_radius_xyz_set_radius_search(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    double radius) {
    if (filter) {
        filter->setRadiusSearch(radius);
    }
}

void pcl_radius_xyz_set_min_neighbors_in_radius(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    int min_neighbors) {
    if (filter) {
        filter->setMinNeighborsInRadius(min_neighbors);
    }
}

pcl::PointCloud<pcl::PointXYZ>* pcl_radius_xyz_filter(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter) {
    if (!filter) {
        return nullptr;
    }
    
    try {
        pcl::PointCloud<pcl::PointXYZ>* output = new pcl::PointCloud<pcl::PointXYZ>();
        filter->filter(*output);
        return output;
    } catch (const std::exception&) {
        return nullptr;
    }
}

// ============================================================================
// PointCloud basic operations (raw pointer versions)
// ============================================================================

size_t pcl_pointcloud_xyz_size(const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    return cloud ? cloud->size() : 0;
}

bool pcl_pointcloud_xyz_empty(const pcl::PointCloud<pcl::PointXYZ>* cloud) {
    return cloud ? cloud->empty() : true;
}

void pcl_pointcloud_xyz_clear(pcl::PointCloud<pcl::PointXYZ>* cloud) {
    if (cloud) {
        cloud->clear();
    }
}

void pcl_pointcloud_xyz_reserve(pcl::PointCloud<pcl::PointXYZ>* cloud, size_t n) {
    if (cloud) {
        cloud->reserve(n);
    }
}

void pcl_pointcloud_xyz_resize(pcl::PointCloud<pcl::PointXYZ>* cloud, size_t n) {
    if (cloud) {
        cloud->resize(n);
    }
}

void pcl_pointcloud_xyz_push_back(
    pcl::PointCloud<pcl::PointXYZ>* cloud,
    float x, float y, float z) {
    if (cloud) {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);
        cloud->width = cloud->points.size();
        cloud->height = 1;
    }
}

void pcl_pointcloud_xyz_get_point(
    const pcl::PointCloud<pcl::PointXYZ>* cloud,
    size_t index,
    float* x, float* y, float* z) {
    if (cloud && index < cloud->size() && x && y && z) {
        const auto& point = cloud->points[index];
        *x = point.x;
        *y = point.y;
        *z = point.z;
    }
}

void pcl_pointcloud_xyz_set_point(
    pcl::PointCloud<pcl::PointXYZ>* cloud,
    size_t index,
    float x, float y, float z) {
    if (cloud && index < cloud->size()) {
        auto& point = cloud->points[index];
        point.x = x;
        point.y = y;
        point.z = z;
    }
}

} // extern "C"