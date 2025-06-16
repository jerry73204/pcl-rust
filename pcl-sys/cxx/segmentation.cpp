#include "cxx/functions.h"
#include "cxx/segmentation_impl.h"
#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// Region Growing segmentation - PointXYZ
std::unique_ptr<pcl::RegionGrowing_PointXYZ_Normal> new_region_growing_xyz() {
  try {
    return std::make_unique<pcl::RegionGrowing_PointXYZ_Normal>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                        const pcl::PointCloud_PointXYZ &cloud) {
  rg.setInputCloud(cloud.makeShared());
}

void set_input_normals_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg,
    const pcl::PointCloud_Normal &normals) {
  rg.setInputNormals(normals.makeShared());
}

void set_min_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t min_size) {
  rg.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg) {
  return const_cast<pcl::RegionGrowing_PointXYZ_Normal &>(rg)
      .getMinClusterSize();
}

void set_max_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t max_size) {
  rg.setMaxClusterSize(max_size);
}

int32_t get_max_cluster_size_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg) {
  return const_cast<pcl::RegionGrowing_PointXYZ_Normal &>(rg)
      .getMaxClusterSize();
}

void set_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold) {
  rg.setSmoothnessThreshold(threshold);
}

float get_smoothness_threshold_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg) {
  return const_cast<pcl::RegionGrowing_PointXYZ_Normal &>(rg)
      .getSmoothnessThreshold();
}

void set_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold) {
  rg.setCurvatureThreshold(threshold);
}

float get_curvature_threshold_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg) {
  return const_cast<pcl::RegionGrowing_PointXYZ_Normal &>(rg)
      .getCurvatureThreshold();
}

void set_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t k) {
  rg.setNumberOfNeighbours(k);
}

int32_t get_number_of_neighbours_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg) {
  return const_cast<pcl::RegionGrowing_PointXYZ_Normal &>(rg)
      .getNumberOfNeighbours();
}

rust::Vec<int32_t>
extract_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg) {
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
std::unique_ptr<pcl::RegionGrowingRGB_PointXYZRGB>
new_region_growing_rgb_xyzrgb() {
  try {
    return std::make_unique<pcl::RegionGrowingRGB_PointXYZRGB>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg,
    const pcl::PointCloud_PointXYZRGB &cloud) {
  rg.setInputCloud(cloud.makeShared());
}

void set_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold) {
  rg.setDistanceThreshold(threshold);
}

float get_distance_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg) {
  return const_cast<pcl::RegionGrowingRGB_PointXYZRGB &>(rg)
      .getDistanceThreshold();
}

void set_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold) {
  rg.setPointColorThreshold(threshold);
}

float get_point_color_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg) {
  return const_cast<pcl::RegionGrowingRGB_PointXYZRGB &>(rg)
      .getPointColorThreshold();
}

void set_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold) {
  rg.setRegionColorThreshold(threshold);
}

float get_region_color_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg) {
  return const_cast<pcl::RegionGrowingRGB_PointXYZRGB &>(rg)
      .getRegionColorThreshold();
}

void set_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, int32_t min_size) {
  rg.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg) {
  return const_cast<pcl::RegionGrowingRGB_PointXYZRGB &>(rg)
      .getMinClusterSize();
}

rust::Vec<int32_t>
extract_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg) {
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
std::unique_ptr<pcl::EuclideanClusterExtraction_PointXYZ>
new_euclidean_cluster_extraction_xyz() {
  try {
    return std::make_unique<pcl::EuclideanClusterExtraction_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece,
    const pcl::PointCloud_PointXYZ &cloud) {
  ece.setInputCloud(cloud.makeShared());
}

void set_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, double tolerance) {
  ece.setClusterTolerance(tolerance);
}

double get_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece) {
  return ece.getClusterTolerance();
}

void set_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, int32_t min_size) {
  ece.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_euclidean_xyz(
    const pcl::EuclideanClusterExtraction_PointXYZ &ece) {
  return const_cast<pcl::EuclideanClusterExtraction_PointXYZ &>(ece)
      .getMinClusterSize();
}

void set_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, int32_t max_size) {
  ece.setMaxClusterSize(max_size);
}

int32_t get_max_cluster_size_euclidean_xyz(
    const pcl::EuclideanClusterExtraction_PointXYZ &ece) {
  return const_cast<pcl::EuclideanClusterExtraction_PointXYZ &>(ece)
      .getMaxClusterSize();
}

rust::Vec<int32_t>
extract_euclidean_clusters_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece) {
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
std::unique_ptr<pcl::SACSegmentation_PointXYZ> new_sac_segmentation_xyz() {
  try {
    return std::make_unique<pcl::SACSegmentation_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             const pcl::PointCloud_PointXYZ &cloud) {
  sac.setInputCloud(cloud.makeShared());
}

void set_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                            int32_t model_type) {
  sac.setModelType(model_type);
}

int32_t get_model_type_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac) {
  return const_cast<pcl::SACSegmentation_PointXYZ &>(sac).getModelType();
}

void set_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             int32_t method_type) {
  sac.setMethodType(method_type);
}

int32_t get_method_type_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac) {
  return const_cast<pcl::SACSegmentation_PointXYZ &>(sac).getMethodType();
}

void set_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                    double threshold) {
  sac.setDistanceThreshold(threshold);
}

double
get_distance_threshold_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac) {
  return const_cast<pcl::SACSegmentation_PointXYZ &>(sac)
      .getDistanceThreshold();
}

void set_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                int32_t max_iterations) {
  sac.setMaxIterations(max_iterations);
}

int32_t get_max_iterations_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac) {
  return const_cast<pcl::SACSegmentation_PointXYZ &>(sac).getMaxIterations();
}

void set_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                       bool optimize) {
  sac.setOptimizeCoefficients(optimize);
}

bool get_optimize_coefficients_sac_xyz(
    const pcl::SACSegmentation_PointXYZ &sac) {
  return const_cast<pcl::SACSegmentation_PointXYZ &>(sac)
      .getOptimizeCoefficients();
}

bool segment_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
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
std::unique_ptr<pcl::PointCloud_Normal>
estimate_normals_xyz(const pcl::PointCloud_PointXYZ &cloud, double radius) {
  try {
    auto normals = std::make_unique<pcl::PointCloud_Normal>();

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

// Min-Cut Segmentation - PointXYZ
std::unique_ptr<pcl::MinCutSegmentation_PointXYZ>
new_min_cut_segmentation_xyz() {
  try {
    return std::make_unique<pcl::MinCutSegmentation_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                 const pcl::PointCloud_PointXYZ &cloud) {
  mc.setInputCloud(cloud.makeShared());
}

void set_foreground_points_min_cut_xyz(
    pcl::MinCutSegmentation_PointXYZ &mc,
    const pcl::PointCloud_PointXYZ &foreground_points) {
  mc.setForegroundPoints(foreground_points.makeShared());
}

void set_sigma_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc, double sigma) {
  mc.setSigma(sigma);
}

double get_sigma_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc) {
  return const_cast<pcl::MinCutSegmentation_PointXYZ &>(mc).getSigma();
}

void set_radius_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                            double radius) {
  mc.setRadius(radius);
}

double get_radius_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc) {
  return const_cast<pcl::MinCutSegmentation_PointXYZ &>(mc).getRadius();
}

void set_number_of_neighbours_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                          int32_t k) {
  mc.setNumberOfNeighbours(k);
}

int32_t get_number_of_neighbours_min_cut_xyz(
    const pcl::MinCutSegmentation_PointXYZ &mc) {
  return const_cast<pcl::MinCutSegmentation_PointXYZ &>(mc)
      .getNumberOfNeighbours();
}

void set_source_weight_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                   double weight) {
  mc.setSourceWeight(weight);
}

double
get_source_weight_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc) {
  return const_cast<pcl::MinCutSegmentation_PointXYZ &>(mc).getSourceWeight();
}

rust::Vec<int32_t> extract_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc) {
  try {
    std::vector<pcl::PointIndices> clusters;
    mc.extract(clusters);

    rust::Vec<int32_t> result;

    // Format: [cluster_count, cluster1_size, cluster1_indices...,
    // cluster2_size, cluster2_indices..., ...]
    result.push_back(static_cast<int32_t>(clusters.size()));

    for (const auto &cluster : clusters) {
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

// Extract Polygonal Prism Data - PointXYZ
std::unique_ptr<pcl::ExtractPolygonalPrismData_PointXYZ>
new_extract_polygonal_prism_xyz() {
  try {
    return std::make_unique<pcl::ExtractPolygonalPrismData_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &cloud) {
  prism.setInputCloud(cloud.makeShared());
}

void set_input_planar_hull_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &hull) {
  prism.setInputPlanarHull(hull.makeShared());
}

void set_height_limits_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism, double height_min,
    double height_max) {
  prism.setHeightLimits(height_min, height_max);
}

void get_height_limits_polygonal_prism_xyz(
    const pcl::ExtractPolygonalPrismData_PointXYZ &prism, double &height_min,
    double &height_max) {
  prism.getHeightLimits(height_min, height_max);
}

rust::Vec<int32_t>
segment_polygonal_prism_xyz(pcl::ExtractPolygonalPrismData_PointXYZ &prism) {
  try {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    prism.segment(*inliers);

    rust::Vec<int32_t> result;
    result.reserve(inliers->indices.size());
    for (int idx : inliers->indices) {
      result.push_back(idx);
    }

    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

// Progressive Morphological Filter - PointXYZ
std::unique_ptr<pcl::ProgressiveMorphologicalFilter_PointXYZ>
new_progressive_morphological_filter_xyz() {
  try {
    return std::make_unique<pcl::ProgressiveMorphologicalFilter_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             const pcl::PointCloud_PointXYZ &cloud) {
  pmf.setInputCloud(cloud.makeShared());
}

void set_max_window_size_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
    int32_t max_window_size) {
  pmf.setMaxWindowSize(max_window_size);
}

int32_t get_max_window_size_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getMaxWindowSize();
}

void set_slope_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                       float slope) {
  pmf.setSlope(slope);
}

float get_slope_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getSlope();
}

void set_max_distance_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                              float max_distance) {
  pmf.setMaxDistance(max_distance);
}

float get_max_distance_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getMaxDistance();
}

void set_initial_distance_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf, float initial_distance) {
  pmf.setInitialDistance(initial_distance);
}

float get_initial_distance_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getInitialDistance();
}

void set_cell_size_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                           float cell_size) {
  pmf.setCellSize(cell_size);
}

float get_cell_size_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getCellSize();
}

void set_base_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                      float base) {
  pmf.setBase(base);
}

float get_base_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getBase();
}

void set_exponential_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             bool exponential) {
  pmf.setExponential(exponential);
}

bool get_exponential_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  return const_cast<pcl::ProgressiveMorphologicalFilter_PointXYZ &>(pmf)
      .getExponential();
}

rust::Vec<int32_t>
extract_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf) {
  try {
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pmf.extract(ground_indices->indices);

    rust::Vec<int32_t> result;
    result.reserve(ground_indices->indices.size());
    for (int idx : ground_indices->indices) {
      result.push_back(idx);
    }

    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

// Conditional Euclidean Clustering - PointXYZ
// Note: This requires a custom condition function which is complex to expose
// via FFI For now, we'll provide a simplified version with built-in conditions

std::unique_ptr<pcl::ConditionalEuclideanClustering_PointXYZ>
new_conditional_euclidean_clustering_xyz() {
  try {
    return std::make_unique<pcl::ConditionalEuclideanClustering_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec,
    const pcl::PointCloud_PointXYZ &cloud) {
  cec.setInputCloud(cloud.makeShared());
}

void set_cluster_tolerance_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, float tolerance) {
  cec.setClusterTolerance(tolerance);
}

float get_cluster_tolerance_conditional_euclidean_xyz(
    const pcl::ConditionalEuclideanClustering_PointXYZ &cec) {
  return const_cast<pcl::ConditionalEuclideanClustering_PointXYZ &>(cec)
      .getClusterTolerance();
}

void set_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t min_size) {
  cec.setMinClusterSize(min_size);
}

int32_t get_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec) {
  return cec.getMinClusterSize();
}

void set_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t max_size) {
  cec.setMaxClusterSize(max_size);
}

int32_t get_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec) {
  return cec.getMaxClusterSize();
}

// For conditional euclidean clustering, we need to set up a condition function
// This is a simplified example that clusters based on distance only
rust::Vec<int32_t> segment_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec) {
  try {
    // Set a simple distance-based condition
    std::function<bool(const pcl::PointXYZ &, const pcl::PointXYZ &, float)>
        distance_condition = [](const pcl::PointXYZ & /*point_a*/,
                                const pcl::PointXYZ & /*point_b*/,
                                float /*squared_distance*/) {
          // Accept points if they are within the squared distance threshold
          return true; // This makes it behave like regular Euclidean clustering
        };

    cec.setConditionFunction(distance_condition);

    std::vector<pcl::PointIndices> cluster_indices;
    cec.segment(cluster_indices);

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
