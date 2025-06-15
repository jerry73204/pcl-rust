#include "cxx/features_impl.h"
#include "cxx/functions.h"
#include <memory>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// Normal estimation - PointXYZ
std::unique_ptr<pcl::NormalEstimation_PointXYZ_Normal>
new_normal_estimation_xyz() {
  try {
    return std::make_unique<pcl::NormalEstimation_PointXYZ_Normal>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                const pcl::PointCloud_PointXYZ &cloud) {
  ne.setInputCloud(cloud.makeShared());
}

void set_search_method_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                  const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  ne.setSearchMethod(tree_ptr);
}

void set_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                  double radius) {
  ne.setRadiusSearch(radius);
}

double get_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne) {
  return ne.getRadiusSearch();
}

void set_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                             int32_t k) {
  ne.setKSearch(k);
}

int32_t get_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne) {
  return ne.getKSearch();
}

void set_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                               float vpx, float vpy, float vpz) {
  ne.setViewPoint(vpx, vpy, vpz);
}

rust::Vec<float>
get_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne) {
  rust::Vec<float> result;
  result.reserve(3);

  float vpx, vpy, vpz;
  ne.getViewPoint(vpx, vpy, vpz);

  result.push_back(vpx);
  result.push_back(vpy);
  result.push_back(vpz);

  return result;
}

void set_use_sensor_origin_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                      bool use_sensor_origin) {
  if (use_sensor_origin) {
    ne.useSensorOriginAsViewPoint();
  }
  // PCL doesn't provide a way to disable sensor origin, only to set it
}

std::unique_ptr<pcl::PointCloud_Normal>
compute_normals_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne) {
  try {
    auto output = std::make_unique<pcl::PointCloud_Normal>();
    ne.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Normal estimation OMP - PointXYZ
std::unique_ptr<pcl::NormalEstimationOMP_PointXYZ_Normal>
new_normal_estimation_omp_xyz() {
  try {
    return std::make_unique<pcl::NormalEstimationOMP_PointXYZ_Normal>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
    const pcl::PointCloud_PointXYZ &cloud) {
  ne.setInputCloud(cloud.makeShared());
}

void set_search_method_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
    const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  ne.setSearchMethod(tree_ptr);
}

void set_radius_search_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne, double radius) {
  ne.setRadiusSearch(radius);
}

void set_k_search_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
                                 int32_t k) {
  ne.setKSearch(k);
}

void set_number_of_threads_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne, int32_t threads) {
  ne.setNumberOfThreads(threads);
}

int32_t get_number_of_threads_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne) {
  // PCL's NormalEstimationOMP doesn't have a getNumberOfThreads method
  // We'll return -1 to indicate this method is not available
  (void)ne; // Silence unused parameter warning
  return -1;
}

std::unique_ptr<pcl::PointCloud_Normal>
compute_normals_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne) {
  try {
    auto output = std::make_unique<pcl::PointCloud_Normal>();
    ne.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// FPFH feature estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33>
new_fpfh_estimation_xyz() {
  try {
    return std::make_unique<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal,
                                                pcl::FPFHSignature33>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_PointXYZ &cloud) {
  fpfh.setInputCloud(cloud.makeShared());
}

void set_input_normals_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_Normal &normals) {
  fpfh.setInputNormals(normals.makeShared());
}

void set_search_method_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  fpfh.setSearchMethod(tree_ptr);
}

void set_radius_search_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, double radius) {
  fpfh.setRadiusSearch(radius);
}

void set_k_search_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, int32_t k) {
  fpfh.setKSearch(k);
}

std::unique_ptr<pcl::PointCloud_FPFHSignature33>
compute_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh) {
  try {
    auto output = std::make_unique<pcl::PointCloud_FPFHSignature33>();
    fpfh.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// FPFH OMP feature estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>
new_fpfh_estimation_omp_xyz() {
  try {
    return std::make_unique<pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal,
                                                   pcl::FPFHSignature33>>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_PointXYZ &cloud) {
  fpfh.setInputCloud(cloud.makeShared());
}

void set_input_normals_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_Normal &normals) {
  fpfh.setInputNormals(normals.makeShared());
}

void set_search_method_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  fpfh.setSearchMethod(tree_ptr);
}

void set_radius_search_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    double radius) {
  fpfh.setRadiusSearch(radius);
}

void set_number_of_threads_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    int32_t threads) {
  fpfh.setNumberOfThreads(threads);
}

std::unique_ptr<pcl::PointCloud_FPFHSignature33> compute_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh) {
  try {
    auto output = std::make_unique<pcl::PointCloud_FPFHSignature33>();
    fpfh.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// PFH feature estimation - PointXYZ
std::unique_ptr<pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125>
new_pfh_estimation_xyz() {
  try {
    return std::make_unique<
        pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}

void set_input_cloud_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::PointCloud_PointXYZ &cloud) {
  pfh.setInputCloud(cloud.makeShared());
}

void set_input_normals_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::PointCloud_Normal &normals) {
  pfh.setInputNormals(normals.makeShared());
}

void set_search_method_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::search::KdTree_PointXYZ &tree) {
  // Create a shared pointer from the const reference
  auto tree_ptr = std::make_shared<pcl::search::KdTree_PointXYZ>(tree);
  pfh.setSearchMethod(tree_ptr);
}

void set_radius_search_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, double radius) {
  pfh.setRadiusSearch(radius);
}

void set_k_search_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, int32_t k) {
  pfh.setKSearch(k);
}

std::unique_ptr<pcl::PointCloud_PFHSignature125>
compute_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh) {
  try {
    auto output = std::make_unique<pcl::PointCloud_PFHSignature125>();
    pfh.compute(*output);
    return output;
  } catch (const std::exception &e) {
    return nullptr;
  }
}

// Helper function to get FPFH histogram data
rust::Vec<float> get_fpfh_histogram(const pcl::FPFHSignature33 &signature) {
  rust::Vec<float> histogram;
  histogram.reserve(33);

  for (int i = 0; i < 33; ++i) {
    histogram.push_back(signature.histogram[i]);
  }

  return histogram;
}

// Helper function to get PFH histogram data
rust::Vec<float> get_pfh_histogram(const pcl::PFHSignature125 &signature) {
  rust::Vec<float> histogram;
  histogram.reserve(125);

  for (int i = 0; i < 125; ++i) {
    histogram.push_back(signature.histogram[i]);
  }

  return histogram;
}

// Helper function to get normal vector data
rust::Vec<float> get_normal_vector(const pcl::Normal &normal) {
  rust::Vec<float> vector;
  vector.reserve(4);

  vector.push_back(normal.normal_x);
  vector.push_back(normal.normal_y);
  vector.push_back(normal.normal_z);
  vector.push_back(normal.curvature);

  return vector;
}
