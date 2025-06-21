#pragma once

#include "rust/cxx.h"
#include "types.h"

// Point clone functions - always available
std::unique_ptr<pcl::PointXYZ> clone_point_xyz(const pcl::PointXYZ &point);
std::unique_ptr<pcl::PointXYZI> clone_point_xyzi(const pcl::PointXYZI &point);
std::unique_ptr<pcl::PointXYZRGB>
clone_point_xyzrgb(const pcl::PointXYZRGB &point);
std::unique_ptr<pcl::PointNormal>
clone_point_normal(const pcl::PointNormal &point);

// Point creation from values - for new owned types
std::unique_ptr<pcl::PointXYZ> create_point_xyz(float x, float y, float z);
std::unique_ptr<pcl::PointXYZI> create_point_xyzi(float x, float y, float z,
                                                  float intensity);
std::unique_ptr<pcl::PointXYZRGB>
create_point_xyzrgb(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
std::unique_ptr<pcl::PointNormal>
create_point_normal(float x, float y, float z, float nx, float ny, float nz);

// Transform operations - always available
void transform_point_cloud_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                               pcl::PointCloud<pcl::PointXYZ> &cloud_out,
                               const rust::Vec<float> &transform_matrix);
void transform_point_cloud_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                                const rust::Vec<float> &transform_matrix);
void transform_point_cloud_xyzrgb(
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB> &cloud_out,
    const rust::Vec<float> &transform_matrix);
void transform_point_cloud_normal(
    const pcl::PointCloud<pcl::PointNormal> &cloud_in,
    pcl::PointCloud<pcl::PointNormal> &cloud_out,
    const rust::Vec<float> &transform_matrix);
