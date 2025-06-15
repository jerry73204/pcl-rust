#pragma once

#include "rust/cxx.h"
#include "types.h"
#include <memory>

// Feature function implementations
// These declarations ensure the functions are visible to the cxx bridge

// Normal estimation - PointXYZ
std::unique_ptr<pcl::NormalEstimation_PointXYZ_Normal>
new_normal_estimation_xyz();

void set_input_cloud_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                const pcl::PointCloud_PointXYZ &cloud);

void set_search_method_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                  const pcl::search::KdTree_PointXYZ &tree);

void set_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                  double radius);

double get_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);

void set_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                             int32_t k);

int32_t get_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);

void set_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                               float vpx, float vpy, float vpz);

rust::Vec<float>
get_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);

void set_use_sensor_origin_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                      bool use_sensor_origin);

std::unique_ptr<pcl::PointCloud_Normal>
compute_normals_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);

// Normal estimation OMP - PointXYZ
std::unique_ptr<pcl::NormalEstimationOMP_PointXYZ_Normal>
new_normal_estimation_omp_xyz();

void set_input_cloud_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
    const pcl::PointCloud_PointXYZ &cloud);

void set_search_method_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
    const pcl::search::KdTree_PointXYZ &tree);

void set_radius_search_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne, double radius);

void set_k_search_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
                                 int32_t k);

void set_number_of_threads_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne, int32_t threads);

int32_t get_number_of_threads_normal_omp_xyz(
    pcl::NormalEstimationOMP_PointXYZ_Normal &ne);

std::unique_ptr<pcl::PointCloud_Normal>
compute_normals_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne);

// FPFH estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33>
new_fpfh_estimation_xyz();

void set_input_cloud_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_PointXYZ &cloud);

void set_input_normals_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_Normal &normals);

void set_search_method_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::search::KdTree_PointXYZ &tree);

void set_radius_search_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, double radius);

void set_k_search_fpfh_xyz(
    pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, int32_t k);

std::unique_ptr<pcl::PointCloud_FPFHSignature33>
compute_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh);

// FPFH OMP estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33>
new_fpfh_estimation_omp_xyz();

void set_input_cloud_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_PointXYZ &cloud);

void set_input_normals_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::PointCloud_Normal &normals);

void set_search_method_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    const pcl::search::KdTree_PointXYZ &tree);

void set_radius_search_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    double radius);

void set_number_of_threads_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
    int32_t threads);

std::unique_ptr<pcl::PointCloud_FPFHSignature33> compute_fpfh_omp_xyz(
    pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh);

// PFH estimation - PointXYZ
std::unique_ptr<pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125>
new_pfh_estimation_xyz();

void set_input_cloud_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::PointCloud_PointXYZ &cloud);

void set_input_normals_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::PointCloud_Normal &normals);

void set_search_method_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
    const pcl::search::KdTree_PointXYZ &tree);

void set_radius_search_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, double radius);

void set_k_search_pfh_xyz(
    pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, int32_t k);

std::unique_ptr<pcl::PointCloud_PFHSignature125>
compute_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh);

// Helper functions
rust::Vec<float> get_fpfh_histogram(const pcl::FPFHSignature33 &signature);

rust::Vec<float> get_pfh_histogram(const pcl::PFHSignature125 &signature);

rust::Vec<float> get_normal_vector(const pcl::Normal &normal);