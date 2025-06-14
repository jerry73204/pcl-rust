#include "cxx/functions.h"
#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>

// Region Growing segmentation - PointXYZ
std::unique_ptr<pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>>
new_region_growing_xyz() {
  try {
    return std::make_unique<pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  rg.setInputCloud(cloud.makeShared());
}

void set_input_normals_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg,
    const pcl::PointCloud<pcl::Normal> &normals) {
  rg.setInputNormals(normals.makeShared());
}

void set_min_cluster_size_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg, int32_t min_size) {
  rg.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  return rg.getMinClusterSize();
}

void set_max_cluster_size_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg, int32_t max_size) {
  rg.setMaxClusterSize(max_size);
}

int32_t get_max_cluster_size_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  return rg.getMaxClusterSize();
}

void set_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg, float threshold) {
  rg.setSmoothnessThreshold(threshold);
}

float get_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  return rg.getSmoothnessThreshold();
}

void set_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg, float threshold) {
  rg.setCurvatureThreshold(threshold);
}

float get_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  return rg.getCurvatureThreshold();
}

void set_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg, int32_t k) {
  rg.setNumberOfNeighbours(k);
}

int32_t get_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  return rg.getNumberOfNeighbours();
}

rust::Vec<int32_t>
extract_region_growing_xyz(pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &rg) {
  try {
    std::vector<pcl::PointIndices> cluster_indices;
    rg.extract(cluster_indices);

    rust::Vec<int32_t> result;

    // Format: [cluster_count, cluster1_size, cluster1_indices...,
    // cluster2_size, cluster2_indices..., ...]
    result.push_back(static_cast<int32_t>(cluster_indices.size()));

    for (const auto &cluster : cluster_indices) {
      result.push_back(static_cast<int32_t>(cluster.indices.size()));
      for (int idx : cluster.indices) {
        result.push_back(idx);
      }
    }

    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

// Region Growing RGB segmentation - PointXYZRGB
std::unique_ptr<pcl::RegionGrowingRGB<pcl::PointXYZRGB>>
new_region_growing_rgb_xyzrgb() {
  try {
    return std::make_unique<pcl::RegionGrowingRGB<pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  rg.setInputCloud(cloud.makeShared());
}

void set_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg, float threshold) {
  rg.setDistanceThreshold(threshold);
}

float get_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg) {
  return rg.getDistanceThreshold();
}

void set_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg, float threshold) {
  rg.setPointColorThreshold(threshold);
}

float get_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg) {
  return rg.getPointColorThreshold();
}

void set_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg, float threshold) {
  rg.setRegionColorThreshold(threshold);
}

float get_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg) {
  return rg.getRegionColorThreshold();
}

void set_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg, int32_t min_size) {
  rg.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg) {
  return rg.getMinClusterSize();
}

rust::Vec<int32_t>
extract_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB<pcl::PointXYZRGB> &rg) {
  try {
    std::vector<pcl::PointIndices> cluster_indices;
    rg.extract(cluster_indices);

    rust::Vec<int32_t> result;

    // Format: [cluster_count, cluster1_size, cluster1_indices...,
    // cluster2_size, cluster2_indices..., ...]
    result.push_back(static_cast<int32_t>(cluster_indices.size()));

    for (const auto &cluster : cluster_indices) {
      result.push_back(static_cast<int32_t>(cluster.indices.size()));
      for (int idx : cluster.indices) {
        result.push_back(idx);
      }
    }

    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

// Euclidean Cluster Extraction - PointXYZ
std::unique_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZ>>
new_euclidean_cluster_extraction_xyz() {
  try {
    return std::make_unique<pcl::EuclideanClusterExtraction<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  ece.setInputCloud(cloud.makeShared());
}

void set_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece, double tolerance) {
  ece.setClusterTolerance(tolerance);
}

double get_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece) {
  return ece.getClusterTolerance();
}

void set_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece, int32_t min_size) {
  ece.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece) {
  return ece.getMinClusterSize();
}

void set_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece, int32_t max_size) {
  ece.setMaxClusterSize(max_size);
}

int32_t get_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece) {
  return ece.getMaxClusterSize();
}

rust::Vec<int32_t> extract_euclidean_clusters_xyz(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> &ece) {
  try {
    std::vector<pcl::PointIndices> cluster_indices;
    ece.extract(cluster_indices);

    rust::Vec<int32_t> result;

    // Format: [cluster_count, cluster1_size, cluster1_indices...,
    // cluster2_size, cluster2_indices..., ...]
    result.push_back(static_cast<int32_t>(cluster_indices.size()));

    for (const auto &cluster : cluster_indices) {
      result.push_back(static_cast<int32_t>(cluster.indices.size()));
      for (int idx : cluster.indices) {
        result.push_back(idx);
      }
    }

    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

// SAC Segmentation - PointXYZ
std::unique_ptr<pcl::SACSegmentation<pcl::PointXYZ>>
new_sac_segmentation_xyz() {
  try {
    return std::make_unique<pcl::SACSegmentation<pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                             const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  sac.setInputCloud(cloud.makeShared());
}

void set_model_type_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                            int32_t model_type) {
  sac.setModelType(model_type);
}

int32_t get_model_type_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac) {
  return sac.getModelType();
}

void set_method_type_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                             int32_t method_type) {
  sac.setMethodType(method_type);
}

int32_t get_method_type_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac) {
  return sac.getMethodType();
}

void set_distance_threshold_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                                    double threshold) {
  sac.setDistanceThreshold(threshold);
}

double
get_distance_threshold_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac) {
  return sac.getDistanceThreshold();
}

void set_max_iterations_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                                int32_t max_iterations) {
  sac.setMaxIterations(max_iterations);
}

int32_t get_max_iterations_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac) {
  return sac.getMaxIterations();
}

void set_optimize_coefficients_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                                       bool optimize) {
  sac.setOptimizeCoefficients(optimize);
}

bool get_optimize_coefficients_sac_xyz(
    pcl::SACSegmentation<pcl::PointXYZ> &sac) {
  return sac.getOptimizeCoefficients();
}

bool segment_sac_xyz(pcl::SACSegmentation<pcl::PointXYZ> &sac,
                     rust::Vec<int32_t> &inliers,
                     rust::Vec<float> &coefficients) {
  try {
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);

    sac.segment(*inlier_indices, *model_coefficients);

    // Convert inliers
    inliers.clear();
    inliers.reserve(inlier_indices->indices.size());
    for (int idx : inlier_indices->indices) {
      inliers.push_back(idx);
    }

    // Convert coefficients
    coefficients.clear();
    coefficients.reserve(model_coefficients->values.size());
    for (float coeff : model_coefficients->values) {
      coefficients.push_back(coeff);
    }

    return !inlier_indices->indices.empty();
  } catch (const std::exception &e) {
    return false;
  }
}

// Normal estimation helper for region growing
std::unique_ptr<pcl::PointCloud<pcl::Normal>>
estimate_normals_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                     double radius) {
  try {
    auto normals = std::make_unique<pcl::PointCloud<pcl::Normal>>();

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud.makeShared());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);

    ne.compute(*normals);

    return normals;
  } catch (const std::exception &e) {
    return nullptr;
  }
}
