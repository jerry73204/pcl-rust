#include "cxx/functions.h"
#include <memory>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

rust::Vec<int32_t>
nearest_k_search_xyzrgb(const pcl::search::KdTree<pcl::PointXYZRGB> &searcher,
                        const pcl::PointXYZRGB &point, int32_t k) {
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
radius_search_xyzrgb(const pcl::search::KdTree<pcl::PointXYZRGB> &searcher,
                     const pcl::PointXYZRGB &point, double radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  searcher.radiusSearch(point, radius, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

float get_epsilon_xyz(const pcl::search::KdTree<pcl::PointXYZ> &searcher) {
  return searcher.getEpsilon();
}

void set_epsilon_xyz(pcl::search::KdTree<pcl::PointXYZ> &searcher,
                     float epsilon) {
  searcher.setEpsilon(epsilon);
}

float get_epsilon_xyzrgb(
    const pcl::search::KdTree<pcl::PointXYZRGB> &searcher) {
  return searcher.getEpsilon();
}

void set_epsilon_xyzrgb(pcl::search::KdTree<pcl::PointXYZRGB> &searcher,
                        float epsilon) {
  searcher.setEpsilon(epsilon);
}

// KdTree PointXYZI functions
std::unique_ptr<pcl::search::KdTree<pcl::PointXYZI>> new_kdtree_xyzi() {
  return std::make_unique<pcl::search::KdTree<pcl::PointXYZI>>();
}

rust::Vec<int32_t>
nearest_k_search_xyzi(const pcl::search::KdTree<pcl::PointXYZI> &searcher,
                      const pcl::PointXYZI &point, int32_t k) {
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
radius_search_xyzi(const pcl::search::KdTree<pcl::PointXYZI> &searcher,
                   const pcl::PointXYZI &point, double radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  searcher.radiusSearch(point, radius, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

void set_input_cloud_xyzi(pcl::search::KdTree<pcl::PointXYZI> &searcher,
                          const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  searcher.setInputCloud(cloud.makeShared());
}

float get_epsilon_xyzi(const pcl::search::KdTree<pcl::PointXYZI> &searcher) {
  return searcher.getEpsilon();
}

void set_epsilon_xyzi(pcl::search::KdTree<pcl::PointXYZI> &searcher,
                      float epsilon) {
  searcher.setEpsilon(epsilon);
}