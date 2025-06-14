#include "pcl/common/common.h"
#include "cxx/functions.h"
#include <memory>

// Point field access functions
float get_x(const pcl::PointXYZ &point) { return point.x; }
float get_y(const pcl::PointXYZ &point) { return point.y; }
float get_z(const pcl::PointXYZ &point) { return point.z; }

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

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> new_point_cloud_xyzrgb() {
  return std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
}

size_t size(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.size();
}

size_t size_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.size();
}

void clear(pcl::PointCloud<pcl::PointXYZ> &cloud) { cloud.clear(); }

void clear_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB> &cloud) { cloud.clear(); }

bool empty(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  return cloud.empty();
}

bool empty_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return cloud.empty();
}

// Search functions
std::unique_ptr<pcl::search::KdTree<pcl::PointXYZ>> new_kdtree_xyz() {
  return std::make_unique<pcl::search::KdTree<pcl::PointXYZ>>();
}

std::unique_ptr<pcl::search::KdTree<pcl::PointXYZRGB>> new_kdtree_xyzrgb() {
  return std::make_unique<pcl::search::KdTree<pcl::PointXYZRGB>>();
}

rust::Vec<int32_t>
nearest_k_search_xyz(const pcl::search::KdTree<pcl::PointXYZ> &searcher,
                     const pcl::PointXYZ &point, int32_t k) {
  std::vector<int> indices;
  std::vector<float> distances;
  searcher.nearestKSearch(point, k, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<int32_t>
radius_search_xyz(const pcl::search::KdTree<pcl::PointXYZ> &searcher,
                  const pcl::PointXYZ &point, double radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  searcher.radiusSearch(point, radius, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

void set_input_cloud_xyz(pcl::search::KdTree<pcl::PointXYZ> &searcher,
                         const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  searcher.setInputCloud(cloud.makeShared());
}

void set_input_cloud_xyzrgb(pcl::search::KdTree<pcl::PointXYZRGB> &searcher,
                            const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  searcher.setInputCloud(cloud.makeShared());
}

// Octree functions
std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>
new_octree_search_xyz(double resolution) {
  return std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
      resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>
new_octree_search_xyzrgb(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>>
new_octree_voxel_centroid_xyz(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>>(resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>>
new_octree_voxel_centroid_xyzrgb(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>>(resolution);
}

void set_input_cloud_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  octree.setInputCloud(cloud.makeShared());
}

void add_points_from_input_cloud_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  octree.addPointsFromInputCloud();
}

rust::Vec<int32_t> nearest_k_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point, int32_t k) {
  std::vector<int> indices;
  std::vector<float> distances;
  octree.nearestKSearch(point, k, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<int32_t> radius_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point, double radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  octree.radiusSearch(point, radius, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<int32_t> voxel_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point) {
  std::vector<int> indices;
  octree.voxelSearch(point, indices);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

double
get_resolution(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getResolution();
}

uint32_t
get_tree_depth(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getTreeDepth();
}

size_t
get_leaf_count(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getLeafCount();
}

size_t
get_branch_count(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getBranchCount();
}
