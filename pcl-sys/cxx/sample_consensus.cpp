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
