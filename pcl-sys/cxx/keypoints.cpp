#include "cxx/functions.h"
#include "cxx/keypoints_impl.h"
#include <memory>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// Harris 3D keypoint detector - PointXYZ
std::unique_ptr<pcl::HarrisKeypoint3D_PointXYZ_PointXYZI> new_harris_3d_xyz() {
  try {
    return std::make_unique<pcl::HarrisKeypoint3D_PointXYZ_PointXYZI>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
    const pcl::PointCloud_PointXYZ &cloud) {
  harris.setInputCloud(cloud.makeShared());
}

void set_search_method_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
    const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  harris.setSearchMethod(tree_ptr);
}

void set_radius_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
                              double radius) {
  harris.setRadius(radius);
}

// Note: Removed getRadius method as it doesn't exist in PCL Harris3D

void set_threshold_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris, float threshold) {
  harris.setThreshold(threshold);
}

// Note: Removed getThreshold method as it doesn't exist in PCL Harris3D

void set_non_max_suppression_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris, bool suppress) {
  harris.setNonMaxSupression(
      suppress); // Note: PCL has a typo in the method name
}

void set_refine_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
                              bool refine) {
  harris.setRefine(refine);
}

// Note: Removed getRefine method as it doesn't exist in PCL Harris3D

std::unique_ptr<pcl::PointCloud_PointXYZI>
compute_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris) {
  try {
    auto output = std::make_unique<pcl::PointCloud_PointXYZI>();
    harris.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// ISS 3D keypoint detector - PointXYZ
std::unique_ptr<pcl::ISSKeypoint3D_PointXYZ_PointXYZ> new_iss_3d_xyz() {
  try {
    return std::make_unique<pcl::ISSKeypoint3D_PointXYZ_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                const pcl::PointCloud_PointXYZ &cloud) {
  iss.setInputCloud(cloud.makeShared());
}

void set_search_method_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                  const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  iss.setSearchMethod(tree_ptr);
}

void set_salient_radius_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                   double radius) {
  iss.setSalientRadius(radius);
}

// Note: Removed getSalientRadius method as it doesn't exist in PCL ISS3D

void set_non_max_radius_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                   double radius) {
  iss.setNonMaxRadius(radius);
}

// Note: Removed getNonMaxRadius method as it doesn't exist in PCL ISS3D

void set_threshold21_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                double threshold) {
  iss.setThreshold21(threshold);
}

// Note: Removed getThreshold21 method as it doesn't exist in PCL ISS3D

void set_threshold32_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                double threshold) {
  iss.setThreshold32(threshold);
}

// Note: Removed getThreshold32 method as it doesn't exist in PCL ISS3D

void set_min_neighbors_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                  int32_t min_neighbors) {
  iss.setMinNeighbors(min_neighbors);
}

// Note: Removed getMinNeighbors method as it doesn't exist in PCL ISS3D

std::unique_ptr<pcl::PointCloud_PointXYZ>
compute_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss) {
  try {
    auto output = std::make_unique<pcl::PointCloud_PointXYZ>();
    iss.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// SIFT keypoint detector - PointXYZI (SIFT requires intensity field)
std::unique_ptr<pcl::SIFTKeypoint_PointXYZI_PointWithScale>
new_sift_keypoint_xyzi() {
  try {
    return std::make_unique<pcl::SIFTKeypoint_PointXYZI_PointWithScale>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
                               const pcl::PointCloud_PointXYZI &cloud) {
  sift.setInputCloud(cloud.makeShared());
}

void set_search_method_sift_xyzi(
    pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
    const pcl::search::KdTree_PointXYZI &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZI>(tree);
  sift.setSearchMethod(tree_ptr);
}

void set_scales_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
                          float min_scale, float nr_octaves,
                          int32_t nr_scales_per_octave) {
  sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
}

void set_minimum_contrast_sift_xyzi(
    pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift, float min_contrast) {
  sift.setMinimumContrast(min_contrast);
}

// Note: Removed getMinimumContrast method as it doesn't exist in PCL SIFT

std::unique_ptr<pcl::PointCloud_PointWithScale>
compute_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift) {
  try {
    auto output = std::make_unique<pcl::PointCloud_PointWithScale>();
    sift.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Helper functions to access keypoint data
rust::Vec<float> get_point_with_scale_coords(const pcl::PointWithScale &point) {
  rust::Vec<float> coords;
  coords.reserve(4);

  coords.push_back(point.x);
  coords.push_back(point.y);
  coords.push_back(point.z);
  coords.push_back(point.scale);

  return coords;
}

rust::Vec<float> get_point_xyzi_coords(const pcl::PointXYZI &point) {
  rust::Vec<float> coords;
  coords.reserve(4);

  coords.push_back(point.x);
  coords.push_back(point.y);
  coords.push_back(point.z);
  coords.push_back(point.intensity);

  return coords;
}
