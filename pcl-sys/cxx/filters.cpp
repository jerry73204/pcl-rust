#include "cxx/functions.h"
#include <memory>
#include <pcl/filters/passthrough.h>
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
