#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// Raw pointer FFI functions for PCL objects
// These functions manage memory using PCL's aligned allocation/deallocation

extern "C" {

// ============================================================================
// PointCloud creation and destruction functions
// ============================================================================

// PointCloud<PointXYZ>
pcl::PointCloud<pcl::PointXYZ>* pcl_pointcloud_xyz_new();
void pcl_pointcloud_xyz_delete(pcl::PointCloud<pcl::PointXYZ>* cloud);

// PointCloud<PointXYZI>
pcl::PointCloud<pcl::PointXYZI>* pcl_pointcloud_xyzi_new();
void pcl_pointcloud_xyzi_delete(pcl::PointCloud<pcl::PointXYZI>* cloud);

// PointCloud<PointXYZRGB>
pcl::PointCloud<pcl::PointXYZRGB>* pcl_pointcloud_xyzrgb_new();
void pcl_pointcloud_xyzrgb_delete(pcl::PointCloud<pcl::PointXYZRGB>* cloud);

// PointCloud<PointNormal>
pcl::PointCloud<pcl::PointNormal>* pcl_pointcloud_normal_new();
void pcl_pointcloud_normal_delete(pcl::PointCloud<pcl::PointNormal>* cloud);

// ============================================================================
// Filter creation and destruction functions
// ============================================================================

// PassThrough filters
pcl::PassThrough<pcl::PointXYZ>* pcl_passthrough_xyz_new();
void pcl_passthrough_xyz_delete(pcl::PassThrough<pcl::PointXYZ>* filter);

pcl::PassThrough<pcl::PointXYZRGB>* pcl_passthrough_xyzrgb_new();
void pcl_passthrough_xyzrgb_delete(pcl::PassThrough<pcl::PointXYZRGB>* filter);

// VoxelGrid filters
pcl::VoxelGrid<pcl::PointXYZ>* pcl_voxelgrid_xyz_new();
void pcl_voxelgrid_xyz_delete(pcl::VoxelGrid<pcl::PointXYZ>* filter);

pcl::VoxelGrid<pcl::PointXYZRGB>* pcl_voxelgrid_xyzrgb_new();
void pcl_voxelgrid_xyzrgb_delete(pcl::VoxelGrid<pcl::PointXYZRGB>* filter);

// StatisticalOutlierRemoval filters
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* pcl_statistical_xyz_new();
void pcl_statistical_xyz_delete(pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter);

pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>* pcl_statistical_xyzrgb_new();
void pcl_statistical_xyzrgb_delete(pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>* filter);

// RadiusOutlierRemoval filters
pcl::RadiusOutlierRemoval<pcl::PointXYZ>* pcl_radius_xyz_new();
void pcl_radius_xyz_delete(pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter);

pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>* pcl_radius_xyzrgb_new();
void pcl_radius_xyzrgb_delete(pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>* filter);

// ============================================================================
// Filter operation functions (using raw pointers)
// ============================================================================

// PassThrough filter operations
void pcl_passthrough_xyz_set_input_cloud(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud);

void pcl_passthrough_xyz_set_filter_field_name(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    const char* field_name);

void pcl_passthrough_xyz_set_filter_limits(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    float min, float max);

void pcl_passthrough_xyz_set_negative(
    pcl::PassThrough<pcl::PointXYZ>* filter,
    bool negative);

pcl::PointCloud<pcl::PointXYZ>* pcl_passthrough_xyz_filter(
    pcl::PassThrough<pcl::PointXYZ>* filter);

// VoxelGrid filter operations
void pcl_voxelgrid_xyz_set_input_cloud(
    pcl::VoxelGrid<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud);

void pcl_voxelgrid_xyz_set_leaf_size(
    pcl::VoxelGrid<pcl::PointXYZ>* filter,
    float lx, float ly, float lz);

pcl::PointCloud<pcl::PointXYZ>* pcl_voxelgrid_xyz_filter(
    pcl::VoxelGrid<pcl::PointXYZ>* filter);

// Statistical filter operations
void pcl_statistical_xyz_set_input_cloud(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud);

void pcl_statistical_xyz_set_mean_k(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    int mean_k);

void pcl_statistical_xyz_set_stddev_mul_thresh(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter,
    double stddev_mult);

pcl::PointCloud<pcl::PointXYZ>* pcl_statistical_xyz_filter(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* filter);

// Radius filter operations
void pcl_radius_xyz_set_input_cloud(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    const pcl::PointCloud<pcl::PointXYZ>* cloud);

void pcl_radius_xyz_set_radius_search(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    double radius);

void pcl_radius_xyz_set_min_neighbors_in_radius(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter,
    int min_neighbors);

pcl::PointCloud<pcl::PointXYZ>* pcl_radius_xyz_filter(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>* filter);

// ============================================================================
// PointCloud basic operations (raw pointer versions)
// ============================================================================

size_t pcl_pointcloud_xyz_size(const pcl::PointCloud<pcl::PointXYZ>* cloud);
bool pcl_pointcloud_xyz_empty(const pcl::PointCloud<pcl::PointXYZ>* cloud);
void pcl_pointcloud_xyz_clear(pcl::PointCloud<pcl::PointXYZ>* cloud);
void pcl_pointcloud_xyz_reserve(pcl::PointCloud<pcl::PointXYZ>* cloud, size_t n);
void pcl_pointcloud_xyz_resize(pcl::PointCloud<pcl::PointXYZ>* cloud, size_t n);

// Point access
void pcl_pointcloud_xyz_push_back(
    pcl::PointCloud<pcl::PointXYZ>* cloud,
    float x, float y, float z);

void pcl_pointcloud_xyz_get_point(
    const pcl::PointCloud<pcl::PointXYZ>* cloud,
    size_t index,
    float* x, float* y, float* z);

void pcl_pointcloud_xyz_set_point(
    pcl::PointCloud<pcl::PointXYZ>* cloud,
    size_t index,
    float x, float y, float z);

} // extern "C"