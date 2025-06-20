#include "pcl/common/common.h"
#include "cxx/common_functions.h"
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

float get_x_point_normal(const pcl::PointNormal &point) { return point.x; }
float get_y_point_normal(const pcl::PointNormal &point) { return point.y; }
float get_z_point_normal(const pcl::PointNormal &point) { return point.z; }
float get_normal_x_point_normal(const pcl::PointNormal &point) {
  return point.normal_x;
}
float get_normal_y_point_normal(const pcl::PointNormal &point) {
  return point.normal_y;
}
float get_normal_z_point_normal(const pcl::PointNormal &point) {
  return point.normal_z;
}

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

std::unique_ptr<pcl::PointCloud<pcl::PointNormal>>
new_point_cloud_point_normal() {
  return std::make_unique<pcl::PointCloud<pcl::PointNormal>>();
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

size_t size_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
  return cloud.size();
}

void clear(pcl::PointCloud<pcl::PointXYZ> &cloud) { cloud.clear(); }

void clear_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud) { cloud.clear(); }

void clear_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud) { cloud.clear(); }

void clear_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud) {
  cloud.clear();
}

bool empty(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.empty();
}

bool empty_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return cloud.empty();
}

bool empty_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.empty();
}

bool empty_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
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

void reserve_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud, size_t n) {
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

void resize_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud, size_t n) {
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

uint32_t width_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
  return cloud.width;
}

uint32_t height_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
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

bool is_dense_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
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

void push_back_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud,
                            rust::Slice<const float> coords) {
  if (coords.size() < 6) {
    return;
  }
  pcl::PointNormal point;
  point.x = coords[0];
  point.y = coords[1];
  point.z = coords[2];
  point.normal_x = coords[3];
  point.normal_y = coords[4];
  point.normal_z = coords[5];
  cloud.points.push_back(point);
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

// Point field access via at()
std::unique_ptr<pcl::PointXYZ>
get_point_at_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud, size_t index) {
  if (index >= cloud.size()) {
    return nullptr;
  }
  auto point = std::make_unique<pcl::PointXYZ>();
  *point = cloud.points[index];
  return point;
}

std::unique_ptr<pcl::PointXYZI>
get_point_at_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud, size_t index) {
  if (index >= cloud.size()) {
    return nullptr;
  }
  auto point = std::make_unique<pcl::PointXYZI>();
  *point = cloud.points[index];
  return point;
}

std::unique_ptr<pcl::PointXYZRGB>
get_point_at_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                    size_t index) {
  if (index >= cloud.size()) {
    return nullptr;
  }
  auto point = std::make_unique<pcl::PointXYZRGB>();
  *point = cloud.points[index];
  return point;
}

std::unique_ptr<pcl::PointNormal>
get_point_at_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud,
                          size_t index) {
  if (index >= cloud.size()) {
    return nullptr;
  }
  auto point = std::make_unique<pcl::PointNormal>();
  *point = cloud.points[index];
  return point;
}

// Point modification functions
void set_point_at_xyz(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t index,
                      const pcl::PointXYZ &point) {
  if (index >= cloud.size()) {
    return;
  }
  cloud.points[index] = point;
}

void set_point_at_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, size_t index,
                       const pcl::PointXYZI &point) {
  if (index >= cloud.size()) {
    return;
  }
  cloud.points[index] = point;
}

void set_point_at_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud, size_t index,
                         const pcl::PointXYZRGB &point) {
  if (index >= cloud.size()) {
    return;
  }
  cloud.points[index] = point;
}

void set_point_at_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud,
                               size_t index, const pcl::PointNormal &point) {
  if (index >= cloud.size()) {
    return;
  }
  cloud.points[index] = point;
}

// Set point fields
void set_x(pcl::PointXYZ &point, float x) { point.x = x; }
void set_y(pcl::PointXYZ &point, float y) { point.y = y; }
void set_z(pcl::PointXYZ &point, float z) { point.z = z; }

void set_x_xyzi(pcl::PointXYZI &point, float x) { point.x = x; }
void set_y_xyzi(pcl::PointXYZI &point, float y) { point.y = y; }
void set_z_xyzi(pcl::PointXYZI &point, float z) { point.z = z; }
void set_intensity(pcl::PointXYZI &point, float intensity) {
  point.intensity = intensity;
}

void set_x_xyzrgb(pcl::PointXYZRGB &point, float x) { point.x = x; }
void set_y_xyzrgb(pcl::PointXYZRGB &point, float y) { point.y = y; }
void set_z_xyzrgb(pcl::PointXYZRGB &point, float z) { point.z = z; }
void set_r(pcl::PointXYZRGB &point, uint8_t r) { point.r = r; }
void set_g(pcl::PointXYZRGB &point, uint8_t g) { point.g = g; }
void set_b(pcl::PointXYZRGB &point, uint8_t b) { point.b = b; }

void set_x_point_normal(pcl::PointNormal &point, float x) { point.x = x; }
void set_y_point_normal(pcl::PointNormal &point, float y) { point.y = y; }
void set_z_point_normal(pcl::PointNormal &point, float z) { point.z = z; }
void set_normal_x_point_normal(pcl::PointNormal &point, float nx) {
  point.normal_x = nx;
}
void set_normal_y_point_normal(pcl::PointNormal &point, float ny) {
  point.normal_y = ny;
}
void set_normal_z_point_normal(pcl::PointNormal &point, float nz) {
  point.normal_z = nz;
}

// Point creation
std::unique_ptr<pcl::PointXYZ> new_point_xyz(float x, float y, float z) {
  auto point = std::make_unique<pcl::PointXYZ>();
  point->x = x;
  point->y = y;
  point->z = z;
  return point;
}

// Point clone functions
std::unique_ptr<pcl::PointXYZ> clone_point_xyz(const pcl::PointXYZ &point) {
  return std::make_unique<pcl::PointXYZ>(point);
}

std::unique_ptr<pcl::PointXYZI> new_point_xyzi(float x, float y, float z,
                                               float intensity) {
  auto point = std::make_unique<pcl::PointXYZI>();
  point->x = x;
  point->y = y;
  point->z = z;
  point->intensity = intensity;
  return point;
}

std::unique_ptr<pcl::PointXYZI> clone_point_xyzi(const pcl::PointXYZI &point) {
  return std::make_unique<pcl::PointXYZI>(point);
}

std::unique_ptr<pcl::PointXYZRGB>
new_point_xyzrgb(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
  auto point = std::make_unique<pcl::PointXYZRGB>();
  point->x = x;
  point->y = y;
  point->z = z;
  point->r = r;
  point->g = g;
  point->b = b;
  return point;
}

std::unique_ptr<pcl::PointXYZRGB>
clone_point_xyzrgb(const pcl::PointXYZRGB &point) {
  return std::make_unique<pcl::PointXYZRGB>(point);
}

std::unique_ptr<pcl::PointNormal>
new_point_normal(float x, float y, float z, float nx, float ny, float nz) {
  auto point = std::make_unique<pcl::PointNormal>();
  point->x = x;
  point->y = y;
  point->z = z;
  point->normal_x = nx;
  point->normal_y = ny;
  point->normal_z = nz;
  return point;
}

std::unique_ptr<pcl::PointNormal>
clone_point_normal(const pcl::PointNormal &point) {
  return std::make_unique<pcl::PointNormal>(point);
}

// Point cloud clone
std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
clone_point_cloud_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZ>>(cloud);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>>
clone_point_cloud_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZI>>(cloud);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
clone_point_cloud_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>(cloud);
}

std::unique_ptr<pcl::PointCloud<pcl::PointNormal>>
clone_point_cloud_point_normal(const pcl::PointCloud<pcl::PointNormal> &cloud) {
  return std::make_unique<pcl::PointCloud<pcl::PointNormal>>(cloud);
}

// Set cloud width/height
void set_width(pcl::PointCloud<pcl::PointXYZ> &cloud, uint32_t width) {
  cloud.width = width;
}
void set_height(pcl::PointCloud<pcl::PointXYZ> &cloud, uint32_t height) {
  cloud.height = height;
}

void set_width_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, uint32_t width) {
  cloud.width = width;
}
void set_height_xyzi(pcl::PointCloud<pcl::PointXYZI> &cloud, uint32_t height) {
  cloud.height = height;
}

void set_width_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                      uint32_t width) {
  cloud.width = width;
}
void set_height_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                       uint32_t height) {
  cloud.height = height;
}

void set_width_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud,
                            uint32_t width) {
  cloud.width = width;
}
void set_height_point_normal(pcl::PointCloud<pcl::PointNormal> &cloud,
                             uint32_t height) {
  cloud.height = height;
}

// Check if point is finite
bool is_finite_xyz(const pcl::PointXYZ &point) { return pcl::isFinite(point); }
bool is_finite_xyzi(const pcl::PointXYZI &point) {
  return pcl::isFinite(point);
}
bool is_finite_xyzrgb(const pcl::PointXYZRGB &point) {
  return pcl::isFinite(point);
}
bool is_finite_point_normal(const pcl::PointNormal &point) {
  return pcl::isFinite(point);
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

// Transform operations
#include <pcl/common/transforms.h>

void transform_point_cloud_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                               pcl::PointCloud<pcl::PointXYZ> &cloud_out,
                               const rust::Vec<float> &transform_matrix) {
  if (transform_matrix.size() != 16) {
    return; // Invalid matrix size
  }

  Eigen::Matrix4f transform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      transform(i, j) = transform_matrix[i * 4 + j];
    }
  }

  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

void transform_point_cloud_xyzi(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                                const rust::Vec<float> &transform_matrix) {
  if (transform_matrix.size() != 16) {
    return; // Invalid matrix size
  }

  Eigen::Matrix4f transform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      transform(i, j) = transform_matrix[i * 4 + j];
    }
  }

  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

void transform_point_cloud_xyzrgb(
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB> &cloud_out,
    const rust::Vec<float> &transform_matrix) {
  if (transform_matrix.size() != 16) {
    return; // Invalid matrix size
  }

  Eigen::Matrix4f transform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      transform(i, j) = transform_matrix[i * 4 + j];
    }
  }

  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

void transform_point_cloud_normal(
    const pcl::PointCloud<pcl::PointNormal> &cloud_in,
    pcl::PointCloud<pcl::PointNormal> &cloud_out,
    const rust::Vec<float> &transform_matrix) {
  if (transform_matrix.size() != 16) {
    return; // Invalid matrix size
  }

  Eigen::Matrix4f transform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      transform(i, j) = transform_matrix[i * 4 + j];
    }
  }

  pcl::transformPointCloudWithNormals(cloud_in, cloud_out, transform);
}
