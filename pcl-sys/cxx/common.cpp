#include "pcl/common/common.h"
#include "cxx/functions.h"
#include <memory>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/point_types.h>

// Point field access functions
float get_x(const pcl::PointXYZ &point) { return point.x; }
float get_y(const pcl::PointXYZ &point) { return point.y; }
float get_z(const pcl::PointXYZ &point) { return point.z; }

float get_x_xyzi(const pcl::PointXYZI &point) { return point.x; }
float get_y_xyzi(const pcl::PointXYZI &point) { return point.y; }
float get_z_xyzi(const pcl::PointXYZI &point) { return point.z; }
float get_intensity(const pcl::PointXYZI &point) { return point.intensity; }

float get_x_xyzrgb(const pcl::PointXYZRGB &point) { return point.x; }
float get_y_xyzrgb(const pcl::PointXYZRGB &point) { return point.y; }
float get_z_xyzrgb(const pcl::PointXYZRGB &point) { return point.z; }
uint8_t get_r(const pcl::PointXYZRGB &point) { return point.r; }
uint8_t get_g(const pcl::PointXYZRGB &point) { return point.g; }
uint8_t get_b(const pcl::PointXYZRGB &point) { return point.b; }

// PointCloud functions
std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> new_point_cloud_xyz() {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> new_point_cloud_xyzi() {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> new_point_cloud_xyzrgb() {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> new_point_cloud_xyzrgba() {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZRGBA>>();
}

size_t size(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.size();
}

size_t size_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.size();
}

size_t size_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.size();
}

void clear(pcl::PointCloud<pcl::PointXYZ> &cloud) { cloud.clear(); }

void clear_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud) { cloud.clear(); }

void clear_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud) { cloud.clear(); }

bool empty(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.empty();
}

bool empty_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.empty();
}

bool empty_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.empty();
}

void reserve_xyz(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t n) {
  cloud.reserve(n);
}

void reserve_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, size_t n) {
  cloud.reserve(n);
}

void reserve_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud, size_t n) {
  cloud.reserve(n);
}

void resize_xyz(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t n) {
  cloud.resize(n);
}

void resize_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, size_t n) {
  cloud.resize(n);
}

void resize_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud, size_t n) {
  cloud.resize(n);
}

uint32_t width(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.width;
}

uint32_t height(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.height;
}

uint32_t width_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.width;
}

uint32_t height_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.height;
}

uint32_t width_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.width;
}

uint32_t height_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.height;
}

bool is_dense(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.is_dense;
}

bool is_dense_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.is_dense;
}

bool is_dense_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.is_dense;
}

// Point manipulation functions
rust::Vec<float> get_point_coords(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                  size_t index) {
  if (index >= cloud.size()) {
    return rust::Vec<float>();
  }
  const auto &point = cloud.points[index];
  rust::Vec<float> coords;
  coords.push_back(point.x);
  coords.push_back(point.y);
  coords.push_back(point.z);
  return coords;
}

rust::Vec<float>
get_point_coords_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                      size_t index) {
  if (index >= cloud.size()) {
    return rust::Vec<float>();
  }
  const auto &point = cloud.points[index];
  rust::Vec<float> coords;
  coords.push_back(point.x);
  coords.push_back(point.y);
  coords.push_back(point.z);
  coords.push_back(point.intensity);
  return coords;
}

rust::Vec<float>
get_point_coords_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                        size_t index) {
  if (index >= cloud.size()) {
    return rust::Vec<float>();
  }
  const auto &point = cloud.points[index];
  rust::Vec<float> coords;
  coords.push_back(point.x);
  coords.push_back(point.y);
  coords.push_back(point.z);
  coords.push_back(static_cast<float>(point.r));
  coords.push_back(static_cast<float>(point.g));
  coords.push_back(static_cast<float>(point.b));
  return coords;
}

void set_point_coords(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t index,
                      rust::Slice<const float> coords) {
  if (index >= cloud.size() || coords.size() < 3) {
    return;
  }
  auto &point = cloud.points[index];
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
}

void set_point_coords_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, size_t index,
                           rust::Slice<const float> coords) {
  if (index >= cloud.size() || coords.size() < 4) {
    return;
  }
  auto &point = cloud.points[index];
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  point.intensity = coords[3];
}

void set_point_coords_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                             size_t index, rust::Slice<const float> coords) {
  if (index >= cloud.size() || coords.size() < 6) {
    return;
  }
  auto &point = cloud.points[index];
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  point.r = static_cast<uint8_t>(coords[3]);
  point.g = static_cast<uint8_t>(coords[4]);
  point.b = static_cast<uint8_t>(coords[5]);
}

void push_back_xyz(pcl::PointCloud<pcl::PointXYZ> &cloud,
                   rust::Slice<const float> coords) {
  if (coords.size() < 3) {
    return;
  }
  pcl::PointXYZ point;
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  cloud.points.push_back(point);
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void push_back_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud,
                    rust::Slice<const float> coords) {
  if (coords.size() < 4) {
    return;
  }
  pcl::PointXYZI point;
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  point.intensity = coords[3];
  cloud.points.push_back(point);
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void push_back_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                      rust::Slice<const float> coords) {
  if (coords.size() < 6) {
    return;
  }
  pcl::PointXYZRGB point;
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  point.r = static_cast<uint8_t>(coords[3]);
  point.g = static_cast<uint8_t>(coords[4]);
  point.b = static_cast<uint8_t>(coords[5]);
  cloud.points.push_back(point);
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

// Feature cloud functions
size_t size_normal(const pcl::PointCloud<pcl::Normal> &cloud) {
  return cloud.size();
}

bool empty_normal(const pcl::PointCloud<pcl::Normal> &cloud) {
  return cloud.empty();
}

size_t size_fpfh(const pcl::PointCloud<pcl::FPFHSignature33> &cloud) {
  return cloud.size();
}

bool empty_fpfh(const pcl::PointCloud<pcl::FPFHSignature33> &cloud) {
  return cloud.empty();
}

size_t size_pfh(const pcl::PointCloud<pcl::PFHSignature125> &cloud) {
  return cloud.size();
}

bool empty_pfh(const pcl::PointCloud<pcl::PFHSignature125> &cloud) {
  return cloud.empty();
}

// Individual feature access functions
rust::Vec<float> get_normal_at(const pcl::PointCloud<pcl::Normal> &cloud,
                               size_t index) {
  rust::Vec<float> result;
  if (index >= cloud.size()) {
    return result; // Return empty vector for invalid index
  }

  const auto &normal = cloud.points[index];
  result.reserve(4);
  result.push_back(normal.normal_x);
  result.push_back(normal.normal_y);
  result.push_back(normal.normal_z);
  result.push_back(normal.curvature);

  return result;
}

rust::Vec<float>
get_fpfh_signature_at(const pcl::PointCloud<pcl::FPFHSignature33> &cloud,
                      size_t index) {
  rust::Vec<float> result;
  if (index >= cloud.size()) {
    return result; // Return empty vector for invalid index
  }

  const auto &signature = cloud.points[index];
  result.reserve(33);
  for (int i = 0; i < 33; ++i) {
    result.push_back(signature.histogram[i]);
  }

  return result;
}

rust::Vec<float>
get_pfh_signature_at(const pcl::PointCloud<pcl::PFHSignature125> &cloud,
                     size_t index) {
  rust::Vec<float> result;
  if (index >= cloud.size()) {
    return result; // Return empty vector for invalid index
  }

  const auto &signature = cloud.points[index];
  result.reserve(125);
  for (int i = 0; i < 125; ++i) {
    result.push_back(signature.histogram[i]);
  }

  return result;
}

// Note: Bulk histogram access functions removed due to cxx Vec<Vec<T>>
// limitation Use individual access functions get_fpfh_signature_at and
// get_pfh_signature_at instead
