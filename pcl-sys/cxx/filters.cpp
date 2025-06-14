#include "cxx/functions.h"
#include <memory>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PassThrough filter functions - PointXYZ
std::unique_ptr<pcl::PassThrough<pcl::PointXYZ>> new_pass_through_xyz() {
  try {
    return std::make_unique<pcl::PassThrough<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_pass_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                              const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_filter_field_name_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                               rust::Str field_name) {
  filter.setFilterFieldName(std::string(field_name));
}

rust::String
get_filter_field_name_xyz(const pcl::PassThrough<pcl::PointXYZ> &filter) {
  return rust::String(filter.getFilterFieldName());
}

void set_filter_limits_xyz(pcl::PassThrough<pcl::PointXYZ> &filter, float min,
                           float max) {
  filter.setFilterLimits(min, max);
}

void set_filter_limits_negative_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                                    bool negative) {
  filter.setNegative(negative); // Use new method instead of deprecated one
}

bool get_filter_limits_negative_xyz(
    const pcl::PassThrough<pcl::PointXYZ> &filter) {
  return filter.getNegative();
}

void set_keep_organized_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                            bool keep_organized) {
  filter.setKeepOrganized(keep_organized);
}

bool get_keep_organized_xyz(const pcl::PassThrough<pcl::PointXYZ> &filter) {
  return filter.getKeepOrganized();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
filter_pass_xyz(pcl::PassThrough<pcl::PointXYZ> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// PassThrough filter functions - PointXYZRGB
std::unique_ptr<pcl::PassThrough<pcl::PointXYZRGB>> new_pass_through_xyzrgb() {
  try {
    return std::make_unique<pcl::PassThrough<pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_pass_xyzrgb(
    pcl::PassThrough<pcl::PointXYZRGB> &filter,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_filter_field_name_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                                  rust::Str field_name) {
  filter.setFilterFieldName(std::string(field_name));
}

rust::String
get_filter_field_name_xyzrgb(const pcl::PassThrough<pcl::PointXYZRGB> &filter) {
  return rust::String(filter.getFilterFieldName());
}

void set_filter_limits_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                              float min, float max) {
  filter.setFilterLimits(min, max);
}

void set_filter_limits_negative_xyzrgb(
    pcl::PassThrough<pcl::PointXYZRGB> &filter, bool negative) {
  filter.setNegative(negative); // Use new method instead of deprecated one
}

bool get_filter_limits_negative_xyzrgb(
    const pcl::PassThrough<pcl::PointXYZRGB> &filter) {
  return filter.getNegative();
}

void set_keep_organized_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                               bool keep_organized) {
  filter.setKeepOrganized(keep_organized);
}

bool get_keep_organized_xyzrgb(
    const pcl::PassThrough<pcl::PointXYZRGB> &filter) {
  return filter.getKeepOrganized();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
filter_pass_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// VoxelGrid filter functions - PointXYZ
std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZ>> new_voxel_grid_xyz() {
  try {
    return std::make_unique<pcl::VoxelGrid<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_voxel_xyz(pcl::VoxelGrid<pcl::PointXYZ> &filter,
                               const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_leaf_size_xyz(pcl::VoxelGrid<pcl::PointXYZ> &filter, float lx,
                       float ly, float lz) {
  filter.setLeafSize(lx, ly, lz);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
filter_voxel_xyz(pcl::VoxelGrid<pcl::PointXYZ> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// VoxelGrid filter functions - PointXYZRGB
std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZRGB>> new_voxel_grid_xyzrgb() {
  try {
    return std::make_unique<pcl::VoxelGrid<pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_voxel_xyzrgb(
    pcl::VoxelGrid<pcl::PointXYZRGB> &filter,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_leaf_size_xyzrgb(pcl::VoxelGrid<pcl::PointXYZRGB> &filter, float lx,
                          float ly, float lz) {
  filter.setLeafSize(lx, ly, lz);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
filter_voxel_xyzrgb(pcl::VoxelGrid<pcl::PointXYZRGB> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// StatisticalOutlierRemoval filter functions - PointXYZ
std::unique_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ>>
new_statistical_outlier_removal_xyz() {
  try {
    return std::make_unique<pcl::StatisticalOutlierRemoval<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_statistical_xyz(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> &filter,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_mean_k_statistical_xyz(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> &filter, int mean_k) {
  filter.setMeanK(mean_k);
}

void set_std_dev_mul_thresh_statistical_xyz(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> &filter, double stddev_mult) {
  filter.setStddevMulThresh(stddev_mult);
}

void set_negative_statistical_xyz(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> &filter, bool negative) {
  filter.setNegative(negative);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
filter_statistical_xyz(pcl::StatisticalOutlierRemoval<pcl::PointXYZ> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// StatisticalOutlierRemoval filter functions - PointXYZRGB
std::unique_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>>
new_statistical_outlier_removal_xyzrgb() {
  try {
    return std::make_unique<pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> &filter,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_mean_k_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> &filter, int mean_k) {
  filter.setMeanK(mean_k);
}

void set_std_dev_mul_thresh_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> &filter,
    double stddev_mult) {
  filter.setStddevMulThresh(stddev_mult);
}

void set_negative_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> &filter, bool negative) {
  filter.setNegative(negative);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> filter_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// RadiusOutlierRemoval filter functions - PointXYZ
std::unique_ptr<pcl::RadiusOutlierRemoval<pcl::PointXYZ>>
new_radius_outlier_removal_xyz() {
  try {
    return std::make_unique<pcl::RadiusOutlierRemoval<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_radius_xyz(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> &filter,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_radius_search_xyz(pcl::RadiusOutlierRemoval<pcl::PointXYZ> &filter,
                           double radius) {
  filter.setRadiusSearch(radius);
}

void set_min_neighbors_in_radius_xyz(
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> &filter, int min_neighbors) {
  filter.setMinNeighborsInRadius(min_neighbors);
}

void set_negative_radius_xyz(pcl::RadiusOutlierRemoval<pcl::PointXYZ> &filter,
                             bool negative) {
  filter.setNegative(negative);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
filter_radius_xyz(pcl::RadiusOutlierRemoval<pcl::PointXYZ> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// RadiusOutlierRemoval filter functions - PointXYZRGB
std::unique_ptr<pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>>
new_radius_outlier_removal_xyzrgb() {
  try {
    return std::make_unique<pcl::RadiusOutlierRemoval<pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_radius_xyzrgb(
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> &filter,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  filter.setInputCloud(cloud.makeShared());
}

void set_radius_search_xyzrgb(
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> &filter, double radius) {
  filter.setRadiusSearch(radius);
}

void set_min_neighbors_in_radius_xyzrgb(
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> &filter, int min_neighbors) {
  filter.setMinNeighborsInRadius(min_neighbors);
}

void set_negative_radius_xyzrgb(
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> &filter, bool negative) {
  filter.setNegative(negative);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
filter_radius_xyzrgb(pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> &filter) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    filter.filter(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}
