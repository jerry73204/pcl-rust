#include "cxx/functions.h"
#include <Eigen/Core>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_svd.h>

// Type aliases for cleaner code
using CorrespondenceEstimation_PointXYZ =
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>;
using CorrespondenceRejectorSampleConsensus_PointXYZ =
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>;
using TransformationEstimationSVD_PointXYZ =
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                   pcl::PointXYZ>;

// NDT functions - PointXYZ
std::unique_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>
new_ndt_xyz() {
  try {
    return std::make_unique<
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  ndt.setInputSource(cloud.makeShared());
}

void set_input_target_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  ndt.setInputTarget(cloud.makeShared());
}

void set_resolution_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    double resolution) {
  ndt.setResolution(resolution);
}

double get_resolution_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getResolution();
}

void set_max_iterations_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    int32_t iterations) {
  ndt.setMaximumIterations(iterations);
}

int32_t get_max_iterations_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getMaximumIterations();
}

void set_transformation_epsilon_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    double epsilon) {
  ndt.setTransformationEpsilon(epsilon);
}

double get_transformation_epsilon_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getTransformationEpsilon();
}

void set_step_size_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
    double step_size) {
  ndt.setStepSize(step_size);
}

double get_step_size_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getStepSize();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> align_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    ndt.align(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> align_with_guess_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
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
    ndt.align(*output, transform);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

bool has_converged_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.hasConverged();
}

double get_fitness_score_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getFitnessScore();
}

rust::Vec<float> get_final_transformation_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  Eigen::Matrix4f transform = ndt.getFinalTransformation();
  rust::Vec<float> result;
  result.reserve(16);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result.push_back(transform(i, j));
    }
  }

  return result;
}

double get_transformation_probability_ndt_xyz(
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt) {
  return ndt.getTransformationProbability();
}

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

// NDT functions - PointXYZRGB
std::unique_ptr<
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>>
new_ndt_xyzrgb() {
  try {
    return std::make_unique<pcl::NormalDistributionsTransform<
        pcl::PointXYZRGB, pcl::PointXYZRGB>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  ndt.setInputSource(cloud.makeShared());
}

void set_input_target_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  ndt.setInputTarget(cloud.makeShared());
}

void set_resolution_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    double resolution) {
  ndt.setResolution(resolution);
}

double get_resolution_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getResolution();
}

void set_max_iterations_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    int32_t iterations) {
  ndt.setMaximumIterations(iterations);
}

int32_t get_max_iterations_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getMaximumIterations();
}

void set_transformation_epsilon_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    double epsilon) {
  ndt.setTransformationEpsilon(epsilon);
}

double get_transformation_epsilon_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getTransformationEpsilon();
}

void set_step_size_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
    double step_size) {
  ndt.setStepSize(step_size);
}

double get_step_size_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getStepSize();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> align_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
    ndt.align(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> align_with_guess_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> &ndt,
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
    ndt.align(*output, transform);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

bool has_converged_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.hasConverged();
}

double get_fitness_score_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getFitnessScore();
}

rust::Vec<float> get_final_transformation_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  Eigen::Matrix4f transform = ndt.getFinalTransformation();
  rust::Vec<float> result;
  result.reserve(16);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result.push_back(transform(i, j));
    }
  }

  return result;
}

double get_transformation_probability_ndt_xyzrgb(
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB>
        &ndt) {
  return ndt.getTransformationProbability();
}

// Correspondence Estimation functions - PointXYZ
std::unique_ptr<CorrespondenceEstimation_PointXYZ>
new_correspondence_estimation_xyz() {
  try {
    return std::make_unique<CorrespondenceEstimation_PointXYZ>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_correspondence_xyz(
    CorrespondenceEstimation_PointXYZ &ce,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  ce.setInputSource(cloud.makeShared());
}

void set_input_target_correspondence_xyz(
    CorrespondenceEstimation_PointXYZ &ce,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  ce.setInputTarget(cloud.makeShared());
}

// Sample Consensus Initial Alignment (SAC-IA) functions - PointXYZ with FPFH
std::unique_ptr<pcl::SampleConsensusInitialAlignment<
    pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>>
new_sac_ia_xyz() {
  try {
    return std::make_unique<pcl::SampleConsensusInitialAlignment<
        pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  sac_ia.setInputSource(cloud.makeShared());
}

void set_input_target_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  sac_ia.setInputTarget(cloud.makeShared());
}

void set_source_features_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    const pcl::PointCloud<pcl::FPFHSignature33> &features) {
  sac_ia.setSourceFeatures(features.makeShared());
}

void set_target_features_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    const pcl::PointCloud<pcl::FPFHSignature33> &features) {
  sac_ia.setTargetFeatures(features.makeShared());
}

void set_min_sample_distance_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    float distance) {
  sac_ia.setMinSampleDistance(distance);
}

float get_min_sample_distance_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.getMinSampleDistance();
}

void set_number_of_samples_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    int nr_samples) {
  sac_ia.setNumberOfSamples(nr_samples);
}

int get_number_of_samples_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.getNumberOfSamples();
}

void set_correspondence_randomness_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    int k) {
  sac_ia.setCorrespondenceRandomness(k);
}

int get_correspondence_randomness_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.getCorrespondenceRandomness();
}

void set_max_iterations_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia,
    int iterations) {
  sac_ia.setMaximumIterations(iterations);
}

int get_max_iterations_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.getMaximumIterations();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> align_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  try {
    auto output = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    sac_ia.align(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

bool has_converged_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.hasConverged();
}

double get_fitness_score_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  return sac_ia.getFitnessScore();
}

rust::Vec<float> get_final_transformation_sac_ia_xyz(
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                         pcl::FPFHSignature33> &sac_ia) {
  Eigen::Matrix4f transform = sac_ia.getFinalTransformation();
  rust::Vec<float> result;
  result.reserve(16);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result.push_back(transform(i, j));
    }
  }

  return result;
}

// Correspondence estimation functions using flattened representation
void determine_correspondences_xyz(CorrespondenceEstimation_PointXYZ &ce,
                                   rust::Vec<int32_t> &correspondences,
                                   rust::Vec<float> &distances) {
  try {
    pcl::Correspondences pcl_corrs;
    ce.determineCorrespondences(pcl_corrs);

    correspondences.clear();
    distances.clear();
    correspondences.reserve(pcl_corrs.size() * 2);
    distances.reserve(pcl_corrs.size());

    for (const auto &corr : pcl_corrs) {
      correspondences.push_back(corr.index_query);
      correspondences.push_back(corr.index_match);
      distances.push_back(corr.distance);
    }
  } catch (const std::exception &e) {
    // Handle error
  }
}

void determine_reciprocal_correspondences_xyz(
    CorrespondenceEstimation_PointXYZ &ce, rust::Vec<int32_t> &correspondences,
    rust::Vec<float> &distances) {
  try {
    pcl::Correspondences pcl_corrs;
    ce.determineReciprocalCorrespondences(pcl_corrs);

    correspondences.clear();
    distances.clear();
    correspondences.reserve(pcl_corrs.size() * 2);
    distances.reserve(pcl_corrs.size());

    for (const auto &corr : pcl_corrs) {
      correspondences.push_back(corr.index_query);
      correspondences.push_back(corr.index_match);
      distances.push_back(corr.distance);
    }
  } catch (const std::exception &e) {
    // Handle error
  }
}

// Correspondence rejection RANSAC functions
std::unique_ptr<CorrespondenceRejectorSampleConsensus_PointXYZ>
new_correspondence_rejector_sac_xyz() {
  try {
    return std::make_unique<
        pcl::registration::CorrespondenceRejectorSampleConsensus<
            pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_source_rejector_xyz(
    CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  rejector.setInputSource(cloud.makeShared());
}

void set_input_target_rejector_xyz(
    CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  rejector.setInputTarget(cloud.makeShared());
}

void set_inlier_threshold_rejector_xyz(
    CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    double threshold) {
  rejector.setInlierThreshold(threshold);
}

double get_inlier_threshold_rejector_xyz(
    CorrespondenceRejectorSampleConsensus_PointXYZ &rejector) {
  return rejector.getInlierThreshold();
}

void get_correspondences_rejector_xyz(
    CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const rust::Vec<int32_t> &correspondences,
    const rust::Vec<float> &distances,
    rust::Vec<int32_t> &remaining_correspondences,
    rust::Vec<float> &remaining_distances) {
  try {
    // Convert flattened arrays to pcl::Correspondences
    pcl::Correspondences pcl_corrs;
    size_t num_corrs = correspondences.size() / 2;
    pcl_corrs.reserve(num_corrs);

    for (size_t i = 0; i < num_corrs; ++i) {
      pcl::Correspondence corr;
      corr.index_query = correspondences[i * 2];
      corr.index_match = correspondences[i * 2 + 1];
      corr.distance = distances[i];
      pcl_corrs.push_back(corr);
    }

    // Set input correspondences and get filtered result
    rejector.setInputCorrespondences(
        std::make_shared<const pcl::Correspondences>(pcl_corrs));
    pcl::Correspondences filtered;
    rejector.getCorrespondences(filtered);

    // Convert back to flattened arrays
    remaining_correspondences.clear();
    remaining_distances.clear();
    remaining_correspondences.reserve(filtered.size() * 2);
    remaining_distances.reserve(filtered.size());

    for (const auto &corr : filtered) {
      remaining_correspondences.push_back(corr.index_query);
      remaining_correspondences.push_back(corr.index_match);
      remaining_distances.push_back(corr.distance);
    }
  } catch (const std::exception &e) {
    // Handle error
  }
}

// Transformation estimation SVD functions
std::unique_ptr<TransformationEstimationSVD_PointXYZ>
new_transformation_estimation_svd_xyz() {
  try {
    return std::make_unique<pcl::registration::TransformationEstimationSVD<
        pcl::PointXYZ, pcl::PointXYZ>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void estimate_rigid_transformation_xyz(
    TransformationEstimationSVD_PointXYZ &est,
    const pcl::PointCloud<pcl::PointXYZ> &source,
    const pcl::PointCloud<pcl::PointXYZ> &target,
    const rust::Vec<int32_t> &correspondences,
    const rust::Vec<float> &distances, rust::Vec<float> &transformation) {
  try {
    // Convert flattened arrays to pcl::Correspondences
    pcl::Correspondences pcl_corrs;
    size_t num_corrs = correspondences.size() / 2;
    pcl_corrs.reserve(num_corrs);

    for (size_t i = 0; i < num_corrs; ++i) {
      pcl::Correspondence corr;
      corr.index_query = correspondences[i * 2];
      corr.index_match = correspondences[i * 2 + 1];
      corr.distance = distances[i];
      pcl_corrs.push_back(corr);
    }

    // Estimate transformation
    Eigen::Matrix4f transform_matrix;
    est.estimateRigidTransformation(source, target, pcl_corrs,
                                    transform_matrix);

    // Convert to rust::Vec
    transformation.clear();
    transformation.reserve(16);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        transformation.push_back(transform_matrix(i, j));
      }
    }
  } catch (const std::exception &e) {
    // Handle error
  }
}
