#pragma once

#include "rust/cxx.h"
#include "types.h"
#include <memory>

// Keypoint function implementations
// These declarations ensure the functions are visible to the cxx bridge

// Harris 3D keypoint detector functions
std::unique_ptr<pcl::HarrisKeypoint3D_PointXYZ_PointXYZI> new_harris_3d_xyz();
void set_input_cloud_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
    const pcl::PointCloud_PointXYZ &cloud);
void set_search_method_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
    const pcl::search::KdTree_PointXYZ &tree);
void set_radius_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
                              double radius);
void set_threshold_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris, float threshold);
void set_non_max_suppression_harris_3d_xyz(
    pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris, bool suppress);
void set_refine_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris,
                              bool refine);
std::unique_ptr<pcl::PointCloud_PointXYZI>
compute_harris_3d_xyz(pcl::HarrisKeypoint3D_PointXYZ_PointXYZI &harris);

// ISS 3D keypoint detector functions
std::unique_ptr<pcl::ISSKeypoint3D_PointXYZ_PointXYZ> new_iss_3d_xyz();
void set_input_cloud_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                const pcl::PointCloud_PointXYZ &cloud);
void set_search_method_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                  const pcl::search::KdTree_PointXYZ &tree);
void set_salient_radius_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                   double radius);
void set_non_max_radius_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                   double radius);
void set_threshold21_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                double threshold);
void set_threshold32_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                double threshold);
void set_min_neighbors_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss,
                                  int32_t min_neighbors);
std::unique_ptr<pcl::PointCloud_PointXYZ>
compute_iss_3d_xyz(pcl::ISSKeypoint3D_PointXYZ_PointXYZ &iss);

// SIFT keypoint detector functions
std::unique_ptr<pcl::SIFTKeypoint_PointXYZI_PointWithScale>
new_sift_keypoint_xyzi();
void set_input_cloud_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
                               const pcl::PointCloud_PointXYZI &cloud);
void set_search_method_sift_xyzi(
    pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
    const pcl::search::KdTree_PointXYZI &tree);
void set_scales_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift,
                          float min_scale, float nr_octaves,
                          int32_t nr_scales_per_octave);
void set_minimum_contrast_sift_xyzi(
    pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift, float min_contrast);
std::unique_ptr<pcl::PointCloud_PointWithScale>
compute_sift_xyzi(pcl::SIFTKeypoint_PointXYZI_PointWithScale &sift);

// Helper functions for keypoint data access
rust::Vec<float> get_point_with_scale_coords(const pcl::PointWithScale &point);
rust::Vec<float> get_point_xyzi_coords(const pcl::PointXYZI &point);
