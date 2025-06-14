#include "cxx/functions.h"
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// RANSAC creation and configuration - PointXYZ
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZ>>
new_ransac_plane_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    auto model =
        std::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(
            cloud.makeShared());
    return std::make_unique<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZ>>
new_ransac_sphere_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    auto model =
        std::make_shared<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>(
            cloud.makeShared());
    return std::make_unique<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_distance_threshold_xyz(
    pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac, double threshold) {
  ransac.setDistanceThreshold(threshold);
}

double get_distance_threshold_xyz(
    const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  return ransac.getDistanceThreshold();
}

void set_max_iterations_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                            int32_t max_iterations) {
  ransac.setMaxIterations(max_iterations);
}

int32_t get_max_iterations_xyz(
    const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  return ransac.getMaxIterations();
}

void set_probability_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                         double probability) {
  ransac.setProbability(probability);
}

double
get_probability_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  return ransac.getProbability();
}

bool compute_model_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  try {
    return ransac.computeModel();
  } catch (const std::exception &e) {
    return false;
  }
}

bool refine_model_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                      double sigma, uint32_t max_iterations) {
  try {
    return ransac.refineModel(sigma, max_iterations);
  } catch (const std::exception &e) {
    return false;
  }
}

rust::Vec<int32_t>
get_inliers_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  pcl::Indices inliers;
  ransac.getInliers(inliers);
  rust::Vec<int32_t> result;
  for (auto idx : inliers) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<float> get_model_coefficients_xyz(
    const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);
  rust::Vec<float> result;
  for (int i = 0; i < coefficients.size(); ++i) {
    result.push_back(coefficients[i]);
  }
  return result;
}

size_t
get_inliers_count_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac) {
  pcl::Indices inliers;
  ransac.getInliers(inliers);
  return inliers.size();
}

// RANSAC creation and configuration - PointXYZRGB
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>
new_ransac_plane_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    auto model =
        std::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>(
            cloud.makeShared());
    return std::make_unique<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>(
        model);
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>
new_ransac_sphere_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    auto model =
        std::make_shared<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>(
            cloud.makeShared());
    return std::make_unique<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>(
        model);
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_distance_threshold_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac, double threshold) {
  ransac.setDistanceThreshold(threshold);
}

double get_distance_threshold_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  return ransac.getDistanceThreshold();
}

void set_max_iterations_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac,
    int32_t max_iterations) {
  ransac.setMaxIterations(max_iterations);
}

int32_t get_max_iterations_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  return ransac.getMaxIterations();
}

void set_probability_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac, double probability) {
  ransac.setProbability(probability);
}

double get_probability_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  return ransac.getProbability();
}

bool compute_model_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  try {
    return ransac.computeModel();
  } catch (const std::exception &e) {
    return false;
  }
}

bool refine_model_xyzrgb(pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac,
                         double sigma, uint32_t max_iterations) {
  try {
    return ransac.refineModel(sigma, max_iterations);
  } catch (const std::exception &e) {
    return false;
  }
}

rust::Vec<int32_t>
get_inliers_xyzrgb(const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  pcl::Indices inliers;
  ransac.getInliers(inliers);
  rust::Vec<int32_t> result;
  for (auto idx : inliers) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<float> get_model_coefficients_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);
  rust::Vec<float> result;
  for (int i = 0; i < coefficients.size(); ++i) {
    result.push_back(coefficients[i]);
  }
  return result;
}

size_t get_inliers_count_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac) {
  pcl::Indices inliers;
  ransac.getInliers(inliers);
  return inliers.size();
}

// Model creation functions - PointXYZ
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>
new_plane_model_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>
new_sphere_model_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Model creation functions - PointXYZRGB
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>
new_plane_model_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>
new_sphere_model_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Plane model methods - PointXYZ
std::vector<float> plane_compute_model_coefficients_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> samples) {
  try {
    pcl::Indices sample_indices(samples.begin(), samples.end());
    Eigen::VectorXf coefficients;
    if (model.computeModelCoefficients(sample_indices, coefficients)) {
      return std::vector<float>(coefficients.data(),
                                coefficients.data() + coefficients.size());
    }
    return {};
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<double> plane_get_distances_to_model_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    std::vector<double> distances;
    model.getDistancesToModel(coeff_eigen, distances);
    return distances;
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<int32_t> plane_select_within_distance_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    pcl::Indices inliers;
    model.selectWithinDistance(coeff_eigen, threshold, inliers);
    return std::vector<int32_t>(inliers.begin(), inliers.end());
  } catch (const std::exception &e) {
    return {};
  }
}

size_t plane_count_within_distance_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    return model.countWithinDistance(coeff_eigen, threshold);
  } catch (const std::exception &e) {
    return 0;
  }
}

std::vector<float> plane_optimize_model_coefficients_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients) {
  try {
    pcl::Indices inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    Eigen::VectorXf optimized_coefficients;
    model.optimizeModelCoefficients(inlier_indices, coeff_eigen,
                                    optimized_coefficients);
    return std::vector<float>(optimized_coefficients.data(),
                              optimized_coefficients.data() +
                                  optimized_coefficients.size());
  } catch (const std::exception &e) {
    return {};
  }
}

// Sphere model methods - PointXYZ (similar pattern as plane)
std::vector<float> sphere_compute_model_coefficients_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> samples) {
  try {
    pcl::Indices sample_indices(samples.begin(), samples.end());
    Eigen::VectorXf coefficients;
    if (model.computeModelCoefficients(sample_indices, coefficients)) {
      return std::vector<float>(coefficients.data(),
                                coefficients.data() + coefficients.size());
    }
    return {};
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<double> sphere_get_distances_to_model_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    std::vector<double> distances;
    model.getDistancesToModel(coeff_eigen, distances);
    return distances;
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<int32_t> sphere_select_within_distance_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    pcl::Indices inliers;
    model.selectWithinDistance(coeff_eigen, threshold, inliers);
    return std::vector<int32_t>(inliers.begin(), inliers.end());
  } catch (const std::exception &e) {
    return {};
  }
}

size_t sphere_count_within_distance_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    return model.countWithinDistance(coeff_eigen, threshold);
  } catch (const std::exception &e) {
    return 0;
  }
}

std::vector<float> sphere_optimize_model_coefficients_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients) {
  try {
    pcl::Indices inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    Eigen::VectorXf optimized_coefficients;
    model.optimizeModelCoefficients(inlier_indices, coeff_eigen,
                                    optimized_coefficients);
    return std::vector<float>(optimized_coefficients.data(),
                              optimized_coefficients.data() +
                                  optimized_coefficients.size());
  } catch (const std::exception &e) {
    return {};
  }
}

void sphere_set_radius_limits_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model, double min_radius,
    double max_radius) {
  model.setRadiusLimits(min_radius, max_radius);
}

// Plane model methods - PointXYZRGB (similar implementations for XYZRGB)
std::vector<float> plane_compute_model_coefficients_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> samples) {
  try {
    pcl::Indices sample_indices(samples.begin(), samples.end());
    Eigen::VectorXf coefficients;
    if (model.computeModelCoefficients(sample_indices, coefficients)) {
      return std::vector<float>(coefficients.data(),
                                coefficients.data() + coefficients.size());
    }
    return {};
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<double> plane_get_distances_to_model_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    std::vector<double> distances;
    model.getDistancesToModel(coeff_eigen, distances);
    return distances;
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<int32_t> plane_select_within_distance_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    pcl::Indices inliers;
    model.selectWithinDistance(coeff_eigen, threshold, inliers);
    return std::vector<int32_t>(inliers.begin(), inliers.end());
  } catch (const std::exception &e) {
    return {};
  }
}

size_t plane_count_within_distance_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    return model.countWithinDistance(coeff_eigen, threshold);
  } catch (const std::exception &e) {
    return 0;
  }
}

std::vector<float> plane_optimize_model_coefficients_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients) {
  try {
    pcl::Indices inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    Eigen::VectorXf optimized_coefficients;
    model.optimizeModelCoefficients(inlier_indices, coeff_eigen,
                                    optimized_coefficients);
    return std::vector<float>(optimized_coefficients.data(),
                              optimized_coefficients.data() +
                                  optimized_coefficients.size());
  } catch (const std::exception &e) {
    return {};
  }
}

// Sphere model methods - PointXYZRGB
std::vector<float> sphere_compute_model_coefficients_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> samples) {
  try {
    pcl::Indices sample_indices(samples.begin(), samples.end());
    Eigen::VectorXf coefficients;
    if (model.computeModelCoefficients(sample_indices, coefficients)) {
      return std::vector<float>(coefficients.data(),
                                coefficients.data() + coefficients.size());
    }
    return {};
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<double> sphere_get_distances_to_model_xyzrgb(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    std::vector<double> distances;
    model.getDistancesToModel(coeff_eigen, distances);
    return distances;
  } catch (const std::exception &e) {
    return {};
  }
}

std::vector<int32_t> sphere_select_within_distance_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    pcl::Indices inliers;
    model.selectWithinDistance(coeff_eigen, threshold, inliers);
    return std::vector<int32_t>(inliers.begin(), inliers.end());
  } catch (const std::exception &e) {
    return {};
  }
}

size_t sphere_count_within_distance_xyzrgb(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    return model.countWithinDistance(coeff_eigen, threshold);
  } catch (const std::exception &e) {
    return 0;
  }
}

std::vector<float> sphere_optimize_model_coefficients_xyzrgb(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients) {
  try {
    pcl::Indices inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff_eigen(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); ++i) {
      coeff_eigen[i] = coefficients[i];
    }
    Eigen::VectorXf optimized_coefficients;
    model.optimizeModelCoefficients(inlier_indices, coeff_eigen,
                                    optimized_coefficients);
    return std::vector<float>(optimized_coefficients.data(),
                              optimized_coefficients.data() +
                                  optimized_coefficients.size());
  } catch (const std::exception &e) {
    return {};
  }
}

void sphere_set_radius_limits_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model, double min_radius,
    double max_radius) {
  model.setRadiusLimits(min_radius, max_radius);
}
