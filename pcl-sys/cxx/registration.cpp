#include "cxx/functions.h"
#include <Eigen/Core>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// ICP functions - PointXYZ
std::unique_ptr<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>
new_icp_xyz() {
  try {
    return std::make_unique<
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  icp.setInputSource(cloud.makeShared());
}

void set_input_target_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  icp.setInputTarget(cloud.makeShared());
}

void set_max_iterations_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    int32_t iterations) {
  icp.setMaximumIterations(iterations);
}

int32_t get_max_iterations_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.getMaximumIterations();
}

void set_transformation_epsilon_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    double epsilon) {
  icp.setTransformationEpsilon(epsilon);
}

double get_transformation_epsilon_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.getTransformationEpsilon();
}

void set_euclidean_fitness_epsilon_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    double epsilon) {
  icp.setEuclideanFitnessEpsilon(epsilon);
}

double get_euclidean_fitness_epsilon_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.getEuclideanFitnessEpsilon();
}

void set_max_correspondence_distance_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    double distance) {
  icp.setMaxCorrespondenceDistance(distance);
}

double get_max_correspondence_distance_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.getMaxCorrespondenceDistance();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
align_icp_xyz(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    icp.align(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> align_with_guess_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp,
    const rust::Vec<float> &guess) {
  try {
    if (guess.size() != 16) {
      return nullptr; // Expecting 4x4 matrix as flat array
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        transform(i, j) = guess[i * 4 + j];
      }
    }

    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    icp.align(*output, transform);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

bool has_converged_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.hasConverged();
}

double get_fitness_score_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  return icp.getFitnessScore();
}

rust::Vec<float> get_final_transformation_icp_xyz(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> &icp) {
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  rust::Vec<float> result;
  result.reserve(16);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result.push_back(transform(i, j));
    }
  }

  return result;
}

// ICP functions - PointXYZRGB
std::unique_ptr<pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>
new_icp_xyzrgb() {
  try {
    return std::make_unique<
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  icp.setInputSource(cloud.makeShared());
}

void set_input_target_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  icp.setInputTarget(cloud.makeShared());
}

void set_max_iterations_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    int32_t iterations) {
  icp.setMaximumIterations(iterations);
}

int32_t get_max_iterations_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.getMaximumIterations();
}

void set_transformation_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    double epsilon) {
  icp.setTransformationEpsilon(epsilon);
}

double get_transformation_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.getTransformationEpsilon();
}

void set_euclidean_fitness_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    double epsilon) {
  icp.setEuclideanFitnessEpsilon(epsilon);
}

double get_euclidean_fitness_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.getEuclideanFitnessEpsilon();
}

void set_max_correspondence_distance_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    double distance) {
  icp.setMaxCorrespondenceDistance(distance);
}

double get_max_correspondence_distance_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.getMaxCorrespondenceDistance();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> align_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    icp.align(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> align_with_guess_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp,
    const rust::Vec<float> &guess) {
  try {
    if (guess.size() != 16) {
      return nullptr; // Expecting 4x4 matrix as flat array
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        transform(i, j) = guess[i * 4 + j];
      }
    }

    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    icp.align(*output, transform);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

bool has_converged_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.hasConverged();
}

double get_fitness_score_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  return icp.getFitnessScore();
}

rust::Vec<float> get_final_transformation_icp_xyzrgb(
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> &icp) {
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  rust::Vec<float> result;
  result.reserve(16);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result.push_back(transform(i, j));
    }
  }

  return result;
}
