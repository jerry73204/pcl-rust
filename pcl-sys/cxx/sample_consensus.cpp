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

// Standalone model creation and configuration - PointXYZ
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>
new_sac_model_plane_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>
new_sac_model_sphere_xyz(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Model-specific functions for plane - PointXYZ
bool compute_model_coefficients_plane_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients) {
  try {
    std::vector<int> indices(sample_indices.begin(), sample_indices.end());
    Eigen::VectorXf coeff;
    bool result = model.computeModelCoefficients(indices, coeff);
    if (result) {
      coefficients.clear();
      for (int i = 0; i < coeff.size(); ++i) {
        coefficients.push_back(coeff[i]);
      }
    }
    return result;
  } catch (const std::exception &e) {
    return false;
  }
}

void get_distances_to_model_plane_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<double> dist;
    model.getDistancesToModel(coeff, dist);
    distances.clear();
    distances.reserve(dist.size());
    for (double d : dist) {
      distances.push_back(d);
    }
  } catch (const std::exception &e) {
    distances.clear();
  }
}

rust::Vec<int32_t> select_within_distance_plane_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<int> inliers;
    model.selectWithinDistance(coeff, threshold, inliers);
    rust::Vec<int32_t> result;
    result.reserve(inliers.size());
    for (int idx : inliers) {
      result.push_back(idx);
    }
    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

int32_t count_within_distance_plane_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    return static_cast<int32_t>(model.countWithinDistance(coeff, threshold));
  } catch (const std::exception &e) {
    return 0;
  }
}

bool optimize_model_coefficients_plane_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    Eigen::VectorXf optimized_coeff;
    model.optimizeModelCoefficients(inlier_indices, coeff, optimized_coeff);

    optimized_coefficients.clear();
    for (int i = 0; i < optimized_coeff.size(); ++i) {
      optimized_coefficients.push_back(optimized_coeff[i]);
    }
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
project_points_plane_xyz(pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
                         const rust::Vec<int32_t> &inliers,
                         const rust::Vec<float> &model_coefficients,
                         bool copy_data_fields) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }

    auto projected_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    model.projectPoints(inlier_indices, coeff, *projected_cloud,
                        copy_data_fields);
    return projected_cloud;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Model-specific functions for sphere - PointXYZ
void set_radius_limits_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model, double min_radius,
    double max_radius) {
  model.setRadiusLimits(min_radius, max_radius);
}

void get_radius_limits_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model, double &min_radius,
    double &max_radius) {
  model.getRadiusLimits(min_radius, max_radius);
}

bool compute_model_coefficients_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients) {
  try {
    std::vector<int> indices(sample_indices.begin(), sample_indices.end());
    Eigen::VectorXf coeff;
    bool result = model.computeModelCoefficients(indices, coeff);
    if (result) {
      coefficients.clear();
      for (int i = 0; i < coeff.size(); ++i) {
        coefficients.push_back(coeff[i]);
      }
    }
    return result;
  } catch (const std::exception &e) {
    return false;
  }
}

void get_distances_to_model_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<double> dist;
    model.getDistancesToModel(coeff, dist);
    distances.clear();
    distances.reserve(dist.size());
    for (double d : dist) {
      distances.push_back(d);
    }
  } catch (const std::exception &e) {
    distances.clear();
  }
}

rust::Vec<int32_t> select_within_distance_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<int> inliers;
    model.selectWithinDistance(coeff, threshold, inliers);
    rust::Vec<int32_t> result;
    result.reserve(inliers.size());
    for (int idx : inliers) {
      result.push_back(idx);
    }
    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

int32_t count_within_distance_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    return static_cast<int32_t>(model.countWithinDistance(coeff, threshold));
  } catch (const std::exception &e) {
    return 0;
  }
}

bool optimize_model_coefficients_sphere_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    Eigen::VectorXf optimized_coeff;
    model.optimizeModelCoefficients(inlier_indices, coeff, optimized_coeff);

    optimized_coefficients.clear();
    for (int i = 0; i < optimized_coeff.size(); ++i) {
      optimized_coefficients.push_back(optimized_coeff[i]);
    }
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
project_points_sphere_xyz(pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
                          const rust::Vec<int32_t> &inliers,
                          const rust::Vec<float> &model_coefficients,
                          bool copy_data_fields) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }

    auto projected_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    model.projectPoints(inlier_indices, coeff, *projected_cloud,
                        copy_data_fields);
    return projected_cloud;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Standalone model creation and configuration - PointXYZRGB
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>
new_sac_model_plane_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>
new_sac_model_sphere_xyzrgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  try {
    return std::make_unique<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>(
        cloud.makeShared());
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Model-specific functions for plane - PointXYZRGB
bool compute_model_coefficients_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients) {
  try {
    std::vector<int> indices(sample_indices.begin(), sample_indices.end());
    Eigen::VectorXf coeff;
    bool result = model.computeModelCoefficients(indices, coeff);
    if (result) {
      coefficients.clear();
      for (int i = 0; i < coeff.size(); ++i) {
        coefficients.push_back(coeff[i]);
      }
    }
    return result;
  } catch (const std::exception &e) {
    return false;
  }
}

void get_distances_to_model_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<double> dist;
    model.getDistancesToModel(coeff, dist);
    distances.clear();
    distances.reserve(dist.size());
    for (double d : dist) {
      distances.push_back(d);
    }
  } catch (const std::exception &e) {
    distances.clear();
  }
}

rust::Vec<int32_t> select_within_distance_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<int> inliers;
    model.selectWithinDistance(coeff, threshold, inliers);
    rust::Vec<int32_t> result;
    result.reserve(inliers.size());
    for (int idx : inliers) {
      result.push_back(idx);
    }
    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

int32_t count_within_distance_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    return static_cast<int32_t>(model.countWithinDistance(coeff, threshold));
  } catch (const std::exception &e) {
    return 0;
  }
}

bool optimize_model_coefficients_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    Eigen::VectorXf optimized_coeff;
    model.optimizeModelCoefficients(inlier_indices, coeff, optimized_coeff);

    optimized_coefficients.clear();
    for (int i = 0; i < optimized_coeff.size(); ++i) {
      optimized_coefficients.push_back(optimized_coeff[i]);
    }
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> project_points_plane_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients, bool copy_data_fields) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }

    auto projected_cloud =
        std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    model.projectPoints(inlier_indices, coeff, *projected_cloud,
                        copy_data_fields);
    return projected_cloud;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Model-specific functions for sphere - PointXYZRGB
void set_radius_limits_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model, double min_radius,
    double max_radius) {
  model.setRadiusLimits(min_radius, max_radius);
}

void get_radius_limits_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    double &min_radius, double &max_radius) {
  model.getRadiusLimits(min_radius, max_radius);
}

bool compute_model_coefficients_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients) {
  try {
    std::vector<int> indices(sample_indices.begin(), sample_indices.end());
    Eigen::VectorXf coeff;
    bool result = model.computeModelCoefficients(indices, coeff);
    if (result) {
      coefficients.clear();
      for (int i = 0; i < coeff.size(); ++i) {
        coefficients.push_back(coeff[i]);
      }
    }
    return result;
  } catch (const std::exception &e) {
    return false;
  }
}

void get_distances_to_model_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<double> dist;
    model.getDistancesToModel(coeff, dist);
    distances.clear();
    distances.reserve(dist.size());
    for (double d : dist) {
      distances.push_back(d);
    }
  } catch (const std::exception &e) {
    distances.clear();
  }
}

rust::Vec<int32_t> select_within_distance_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    std::vector<int> inliers;
    model.selectWithinDistance(coeff, threshold, inliers);
    rust::Vec<int32_t> result;
    result.reserve(inliers.size());
    for (int idx : inliers) {
      result.push_back(idx);
    }
    return result;
  } catch (const std::exception &e) {
    return rust::Vec<int32_t>();
  }
}

int32_t count_within_distance_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<float> &model_coefficients, double threshold) {
  try {
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    return static_cast<int32_t>(model.countWithinDistance(coeff, threshold));
  } catch (const std::exception &e) {
    return 0;
  }
}

bool optimize_model_coefficients_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }
    Eigen::VectorXf optimized_coeff;
    model.optimizeModelCoefficients(inlier_indices, coeff, optimized_coeff);

    optimized_coefficients.clear();
    for (int i = 0; i < optimized_coeff.size(); ++i) {
      optimized_coefficients.push_back(optimized_coeff[i]);
    }
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> project_points_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients, bool copy_data_fields) {
  try {
    std::vector<int> inlier_indices(inliers.begin(), inliers.end());
    Eigen::VectorXf coeff(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size(); ++i) {
      coeff[i] = model_coefficients[i];
    }

    auto projected_cloud =
        std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    model.projectPoints(inlier_indices, coeff, *projected_cloud,
                        copy_data_fields);
    return projected_cloud;
  } catch (const std::exception &e) {
    return nullptr;
  }
}
