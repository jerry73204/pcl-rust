#pragma once

// CRITICAL: When visualization is enabled, ensure VTK config is included first
#ifdef PCL_RUST_ENABLE_VISUALIZATION
#include "vtk_config.h"
#endif

#include "common_functions.h"
#include "common_impl.h"
#include "features_impl.h"
#include "keypoints_impl.h"
#include "rust/cxx.h"
#include "segmentation_impl.h"
#include "surface_impl.h"
#include "types.h"

// VTK includes have been moved to visualization-specific files
// to prevent unconditional inclusion via cxx bridge

// Point field access functions
float get_x(const pcl::PointXYZ &point);
float get_y(const pcl::PointXYZ &point);
float get_z(const pcl::PointXYZ &point);

float get_x_xyzi(const pcl::PointXYZI &point);
float get_y_xyzi(const pcl::PointXYZI &point);
float get_z_xyzi(const pcl::PointXYZI &point);
float get_intensity(const pcl::PointXYZI &point);

float get_x_xyzrgb(const pcl::PointXYZRGB &point);
float get_y_xyzrgb(const pcl::PointXYZRGB &point);
float get_z_xyzrgb(const pcl::PointXYZRGB &point);
uint8_t get_r(const pcl::PointXYZRGB &point);
uint8_t get_g(const pcl::PointXYZRGB &point);
uint8_t get_b(const pcl::PointXYZRGB &point);

float get_x_point_normal(const pcl::PointNormal &point);
float get_y_point_normal(const pcl::PointNormal &point);
float get_z_point_normal(const pcl::PointNormal &point);
float get_normal_x_point_normal(const pcl::PointNormal &point);
float get_normal_y_point_normal(const pcl::PointNormal &point);
float get_normal_z_point_normal(const pcl::PointNormal &point);

// Registration function declarations - ICP for PointXYZ
std::unique_ptr<pcl::IterativeClosestPoint_PointXYZ> new_icp_xyz();
void set_input_source_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_input_target_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_max_iterations_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                                int32_t max_iter);
int32_t get_max_iterations_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_transformation_epsilon_icp_xyz(
    pcl::IterativeClosestPoint_PointXYZ &icp, double epsilon);
double
get_transformation_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_euclidean_fitness_epsilon_icp_xyz(
    pcl::IterativeClosestPoint_PointXYZ &icp, double epsilon);
double
get_euclidean_fitness_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_max_correspondence_distance_icp_xyz(
    pcl::IterativeClosestPoint_PointXYZ &icp, double distance);
double get_max_correspondence_distance_icp_xyz(
    pcl::IterativeClosestPoint_PointXYZ &icp);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_with_guess_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                         const rust::Vec<float> &guess);
bool has_converged_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
double get_fitness_score_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
rust::Vec<float>
get_final_transformation_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);

// Registration function declarations - ICP for PointXYZRGB
std::unique_ptr<pcl::IterativeClosestPoint_PointXYZRGB> new_icp_xyzrgb();
void set_input_source_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                 const pcl::PointCloud_PointXYZRGB &cloud);
void set_input_target_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                 const pcl::PointCloud_PointXYZRGB &cloud);
void set_max_iterations_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                   int32_t max_iter);
int32_t
get_max_iterations_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_transformation_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp, double epsilon);
double get_transformation_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_euclidean_fitness_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp, double epsilon);
double get_euclidean_fitness_epsilon_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_max_correspondence_distance_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp, double distance);
double get_max_correspondence_distance_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_with_guess_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                            const rust::Vec<float> &guess);
bool has_converged_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
double
get_fitness_score_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
rust::Vec<float> get_final_transformation_icp_xyzrgb(
    pcl::IterativeClosestPoint_PointXYZRGB &icp);

// Correspondence estimation functions
std::unique_ptr<pcl::registration::CorrespondenceEstimation_PointXYZ>
new_correspondence_estimation_xyz();
void set_input_source_correspondence_xyz(
    pcl::registration::CorrespondenceEstimation_PointXYZ &ce,
    const pcl::PointCloud_PointXYZ &cloud);
void set_input_target_correspondence_xyz(
    pcl::registration::CorrespondenceEstimation_PointXYZ &ce,
    const pcl::PointCloud_PointXYZ &cloud);
void determine_correspondences_xyz(
    pcl::registration::CorrespondenceEstimation_PointXYZ &ce,
    rust::Vec<int32_t> &correspondences, rust::Vec<float> &distances);
void determine_reciprocal_correspondences_xyz(
    pcl::registration::CorrespondenceEstimation_PointXYZ &ce,
    rust::Vec<int32_t> &correspondences, rust::Vec<float> &distances);

// Correspondence rejection functions
std::unique_ptr<
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ>
new_correspondence_rejector_sac_xyz();
void set_input_source_rejector_xyz(
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const pcl::PointCloud_PointXYZ &cloud);
void set_input_target_rejector_xyz(
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const pcl::PointCloud_PointXYZ &cloud);
void set_inlier_threshold_rejector_xyz(
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    double threshold);
double get_inlier_threshold_rejector_xyz(
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ
        &rejector);
void get_correspondences_rejector_xyz(
    pcl::registration::CorrespondenceRejectorSampleConsensus_PointXYZ &rejector,
    const rust::Vec<int32_t> &correspondences,
    const rust::Vec<float> &distances,
    rust::Vec<int32_t> &remaining_correspondences,
    rust::Vec<float> &remaining_distances);

// Transformation estimation functions
std::unique_ptr<pcl::registration::TransformationEstimationSVD_PointXYZ>
new_transformation_estimation_svd_xyz();
void estimate_rigid_transformation_xyz(
    pcl::registration::TransformationEstimationSVD_PointXYZ &est,
    const pcl::PointCloud_PointXYZ &source,
    const pcl::PointCloud_PointXYZ &target,
    const rust::Vec<int32_t> &correspondences,
    const rust::Vec<float> &distances, rust::Vec<float> &transformation);

// NDT registration functions
std::unique_ptr<pcl::NormalDistributionsTransform_PointXYZ> new_ndt_xyz();
void set_input_source_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_input_target_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_resolution_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                            double resolution);
double get_resolution_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
void set_max_iterations_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                                int32_t iterations);
int32_t
get_max_iterations_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
void set_transformation_epsilon_ndt_xyz(
    pcl::NormalDistributionsTransform_PointXYZ &ndt, double epsilon);
double get_transformation_epsilon_ndt_xyz(
    pcl::NormalDistributionsTransform_PointXYZ &ndt);
void set_step_size_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                           double step_size);
double get_step_size_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_with_guess_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt,
                         const rust::Vec<float> &guess);
bool has_converged_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
double
get_fitness_score_ndt_xyz(pcl::NormalDistributionsTransform_PointXYZ &ndt);
rust::Vec<float> get_final_transformation_ndt_xyz(
    pcl::NormalDistributionsTransform_PointXYZ &ndt);

// NDT registration functions for PointXYZRGB
std::unique_ptr<pcl::NormalDistributionsTransform_PointXYZRGB> new_ndt_xyzrgb();
void set_input_source_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_input_target_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_resolution_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt, double resolution);
double
get_resolution_ndt_xyzrgb(pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
void set_max_iterations_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt, int32_t iterations);
int32_t get_max_iterations_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
void set_transformation_epsilon_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt, double epsilon);
double get_transformation_epsilon_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
void set_step_size_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt, double step_size);
double
get_step_size_ndt_xyzrgb(pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_ndt_xyzrgb(pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_with_guess_ndt_xyzrgb(pcl::NormalDistributionsTransform_PointXYZRGB &ndt,
                            const rust::Vec<float> &guess);
bool has_converged_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
double get_fitness_score_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt);
rust::Vec<float> get_final_transformation_ndt_xyzrgb(
    pcl::NormalDistributionsTransform_PointXYZRGB &ndt);

// PointCloud functions
std::unique_ptr<pcl::PointCloud_PointXYZ> new_point_cloud_xyz();
std::unique_ptr<pcl::PointCloud_PointXYZI> new_point_cloud_xyzi();
std::unique_ptr<pcl::PointCloud_PointXYZRGB> new_point_cloud_xyzrgb();
std::unique_ptr<pcl::PointCloud_PointXYZRGBA> new_point_cloud_xyzrgba();
std::unique_ptr<pcl::PointCloud_PointNormal> new_point_cloud_point_normal();

size_t size(const pcl::PointCloud_PointXYZ &cloud);
size_t size_xyzi(const pcl::PointCloud_PointXYZI &cloud);
size_t size_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
size_t size_point_normal(const pcl::PointCloud_PointNormal &cloud);

bool empty(const pcl::PointCloud_PointXYZ &cloud);
bool empty_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool empty_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
bool empty_point_normal(const pcl::PointCloud_PointNormal &cloud);

void clear(pcl::PointCloud_PointXYZ &cloud);
void clear_xyzi(pcl::PointCloud_PointXYZI &cloud);
void clear_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud);
void clear_point_normal(pcl::PointCloud_PointNormal &cloud);

void reserve(pcl::PointCloud_PointXYZ &cloud, size_t size);
void reserve_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t size);
void reserve_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t size);
void reserve_point_normal(pcl::PointCloud_PointNormal &cloud, size_t size);

void resize(pcl::PointCloud_PointXYZ &cloud, size_t size);
void resize_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t size);
void resize_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t size);
void resize_point_normal(pcl::PointCloud_PointNormal &cloud, size_t size);

rust::Vec<float> get_point_coords(const pcl::PointCloud_PointXYZ &cloud,
                                  size_t index);
rust::Vec<float> get_point_coords_xyzi(const pcl::PointCloud_PointXYZI &cloud,
                                       size_t index);
rust::Vec<float>
get_point_coords_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud, size_t index);

void set_point_coords(pcl::PointCloud_PointXYZ &cloud, size_t index,
                      rust::Slice<const float> coords);
void set_point_coords_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t index,
                           rust::Slice<const float> coords);
void set_point_coords_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t index,
                             rust::Slice<const float> coords);

void push_back_xyz(pcl::PointCloud_PointXYZ &cloud,
                   rust::Slice<const float> coords);
void push_back_xyzi(pcl::PointCloud_PointXYZI &cloud,
                    rust::Slice<const float> coords);
void push_back_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud,
                      rust::Slice<const float> coords);
void push_back_point_normal(pcl::PointCloud_PointNormal &cloud,
                            rust::Slice<const float> coords);

// Additional basic point cloud functions
void reserve_xyz(pcl::PointCloud_PointXYZ &cloud, size_t n);
void reserve_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t n);
void reserve_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t n);

void resize_xyz(pcl::PointCloud_PointXYZ &cloud, size_t n);
void resize_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t n);
void resize_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t n);

uint32_t width(const pcl::PointCloud_PointXYZ &cloud);
uint32_t height(const pcl::PointCloud_PointXYZ &cloud);
uint32_t width_xyzi(const pcl::PointCloud_PointXYZI &cloud);
uint32_t height_xyzi(const pcl::PointCloud_PointXYZI &cloud);
uint32_t width_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
uint32_t height_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
uint32_t width_point_normal(const pcl::PointCloud_PointNormal &cloud);
uint32_t height_point_normal(const pcl::PointCloud_PointNormal &cloud);

bool is_dense(const pcl::PointCloud_PointXYZ &cloud);
bool is_dense_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool is_dense_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
bool is_dense_point_normal(const pcl::PointCloud_PointNormal &cloud);

// Point field access via at()
std::unique_ptr<pcl::PointXYZ>
get_point_at_xyz(const pcl::PointCloud_PointXYZ &cloud, size_t index);
std::unique_ptr<pcl::PointXYZI>
get_point_at_xyzi(const pcl::PointCloud_PointXYZI &cloud, size_t index);
std::unique_ptr<pcl::PointXYZRGB>
get_point_at_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud, size_t index);
std::unique_ptr<pcl::PointNormal>
get_point_at_point_normal(const pcl::PointCloud_PointNormal &cloud,
                          size_t index);

// Point modification functions
void set_point_at_xyz(pcl::PointCloud_PointXYZ &cloud, size_t index,
                      const pcl::PointXYZ &point);
void set_point_at_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t index,
                       const pcl::PointXYZI &point);
void set_point_at_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t index,
                         const pcl::PointXYZRGB &point);
void set_point_at_point_normal(pcl::PointCloud_PointNormal &cloud, size_t index,
                               const pcl::PointNormal &point);

// Set point fields
void set_x(pcl::PointXYZ &point, float x);
void set_y(pcl::PointXYZ &point, float y);
void set_z(pcl::PointXYZ &point, float z);

void set_x_xyzi(pcl::PointXYZI &point, float x);
void set_y_xyzi(pcl::PointXYZI &point, float y);
void set_z_xyzi(pcl::PointXYZI &point, float z);
void set_intensity(pcl::PointXYZI &point, float intensity);

void set_x_xyzrgb(pcl::PointXYZRGB &point, float x);
void set_y_xyzrgb(pcl::PointXYZRGB &point, float y);
void set_z_xyzrgb(pcl::PointXYZRGB &point, float z);
void set_r(pcl::PointXYZRGB &point, uint8_t r);
void set_g(pcl::PointXYZRGB &point, uint8_t g);
void set_b(pcl::PointXYZRGB &point, uint8_t b);

void set_x_point_normal(pcl::PointNormal &point, float x);
void set_y_point_normal(pcl::PointNormal &point, float y);
void set_z_point_normal(pcl::PointNormal &point, float z);
void set_normal_x_point_normal(pcl::PointNormal &point, float nx);
void set_normal_y_point_normal(pcl::PointNormal &point, float ny);
void set_normal_z_point_normal(pcl::PointNormal &point, float nz);

// Point creation
std::unique_ptr<pcl::PointXYZ> new_point_xyz(float x, float y, float z);
std::unique_ptr<pcl::PointXYZI> new_point_xyzi(float x, float y, float z,
                                               float intensity);
std::unique_ptr<pcl::PointXYZRGB>
new_point_xyzrgb(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
std::unique_ptr<pcl::PointNormal>
new_point_normal(float x, float y, float z, float nx, float ny, float nz);

// Point cloud clone
std::unique_ptr<pcl::PointCloud_PointXYZ>
clone_point_cloud_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::PointCloud_PointXYZI>
clone_point_cloud_xyzi(const pcl::PointCloud_PointXYZI &cloud);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
clone_point_cloud_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
std::unique_ptr<pcl::PointCloud_PointNormal>
clone_point_cloud_point_normal(const pcl::PointCloud_PointNormal &cloud);

// Set cloud width/height
void set_width(pcl::PointCloud_PointXYZ &cloud, uint32_t width);
void set_height(pcl::PointCloud_PointXYZ &cloud, uint32_t height);

void set_width_xyzi(pcl::PointCloud_PointXYZI &cloud, uint32_t width);
void set_height_xyzi(pcl::PointCloud_PointXYZI &cloud, uint32_t height);

void set_width_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, uint32_t width);
void set_height_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, uint32_t height);

void set_width_point_normal(pcl::PointCloud_PointNormal &cloud, uint32_t width);
void set_height_point_normal(pcl::PointCloud_PointNormal &cloud,
                             uint32_t height);

// Check if point is finite
bool is_finite_xyz(const pcl::PointXYZ &point);
bool is_finite_xyzi(const pcl::PointXYZI &point);
bool is_finite_xyzrgb(const pcl::PointXYZRGB &point);
bool is_finite_point_normal(const pcl::PointNormal &point);

// Search functions
std::unique_ptr<pcl::search::KdTree_PointXYZ> new_kdtree_xyz();
std::unique_ptr<pcl::search::KdTree_PointXYZI> new_kdtree_xyzi();
std::unique_ptr<pcl::search::KdTree_PointXYZRGB> new_kdtree_xyzrgb();

rust::Vec<int32_t>
nearest_k_search_xyz(const pcl::search::KdTree_PointXYZ &searcher,
                     const pcl::PointXYZ &point, int32_t k);
rust::Vec<int32_t>
nearest_k_search_xyzi(const pcl::search::KdTree_PointXYZI &searcher,
                      const pcl::PointXYZI &point, int32_t k);
rust::Vec<int32_t>
nearest_k_search_xyzrgb(const pcl::search::KdTree_PointXYZRGB &searcher,
                        const pcl::PointXYZRGB &point, int32_t k);

rust::Vec<int32_t>
radius_search_xyz(const pcl::search::KdTree_PointXYZ &searcher,
                  const pcl::PointXYZ &point, double radius);
rust::Vec<int32_t>
radius_search_xyzi(const pcl::search::KdTree_PointXYZI &searcher,
                   const pcl::PointXYZI &point, double radius);
rust::Vec<int32_t>
radius_search_xyzrgb(const pcl::search::KdTree_PointXYZRGB &searcher,
                     const pcl::PointXYZRGB &point, double radius);

void set_input_cloud_xyz(pcl::search::KdTree_PointXYZ &searcher,
                         const pcl::PointCloud_PointXYZ &cloud);
void set_input_cloud_xyzi(pcl::search::KdTree_PointXYZI &searcher,
                          const pcl::PointCloud_PointXYZI &cloud);
void set_input_cloud_xyzrgb(pcl::search::KdTree_PointXYZRGB &searcher,
                            const pcl::PointCloud_PointXYZRGB &cloud);

float get_epsilon_xyz(const pcl::search::KdTree_PointXYZ &searcher);
void set_epsilon_xyz(pcl::search::KdTree_PointXYZ &searcher, float epsilon);
float get_epsilon_xyzi(const pcl::search::KdTree_PointXYZI &searcher);
void set_epsilon_xyzi(pcl::search::KdTree_PointXYZI &searcher, float epsilon);
float get_epsilon_xyzrgb(const pcl::search::KdTree_PointXYZRGB &searcher);
void set_epsilon_xyzrgb(pcl::search::KdTree_PointXYZRGB &searcher,
                        float epsilon);

// Octree functions
std::unique_ptr<pcl::octree::OctreePointCloudSearch_PointXYZ>
new_octree_search_xyz(double resolution);
std::unique_ptr<pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ>
new_octree_voxel_centroid_xyz(double resolution);

void set_input_cloud_octree_xyz(
    pcl::octree::OctreePointCloudSearch_PointXYZ &octree,
    const pcl::PointCloud_PointXYZ &cloud);
void add_points_from_input_cloud_xyz(
    pcl::octree::OctreePointCloudSearch_PointXYZ &octree);

rust::Vec<int32_t> nearest_k_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch_PointXYZ &octree,
    const pcl::PointXYZ &point, int32_t k);
rust::Vec<int32_t>
radius_search_octree_xyz(pcl::octree::OctreePointCloudSearch_PointXYZ &octree,
                         const pcl::PointXYZ &point, double radius);
rust::Vec<int32_t>
voxel_search_octree_xyz(pcl::octree::OctreePointCloudSearch_PointXYZ &octree,
                        const pcl::PointXYZ &point);

double get_resolution(pcl::octree::OctreePointCloudSearch_PointXYZ &octree);
uint32_t get_tree_depth(pcl::octree::OctreePointCloudSearch_PointXYZ &octree);
size_t get_leaf_count(pcl::octree::OctreePointCloudSearch_PointXYZ &octree);
size_t get_branch_count(pcl::octree::OctreePointCloudSearch_PointXYZ &octree);

void set_input_cloud_voxel_centroid_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree,
    const pcl::PointCloud_PointXYZ &cloud);
void add_points_from_input_cloud_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree);
std::unique_ptr<pcl::PointCloud_PointXYZ> get_voxel_centroids_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree);
double get_resolution_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree);
uint32_t get_tree_depth_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree);
void delete_tree_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid_PointXYZ &octree);

// I/O functions
// PCD I/O functions for PointXYZ
int32_t load_pcd_file_xyz(rust::Str file_name, pcl::PointCloud_PointXYZ &cloud);
int32_t save_pcd_file_xyz(rust::Str file_name,
                          const pcl::PointCloud_PointXYZ &cloud, bool binary);
int32_t save_pcd_file_ascii_xyz(rust::Str file_name,
                                const pcl::PointCloud_PointXYZ &cloud);
int32_t save_pcd_file_binary_xyz(rust::Str file_name,
                                 const pcl::PointCloud_PointXYZ &cloud);
int32_t
save_pcd_file_binary_compressed_xyz(rust::Str file_name,
                                    const pcl::PointCloud_PointXYZ &cloud);

// PCD I/O functions for PointXYZI
int32_t load_pcd_file_xyzi(rust::Str file_name,
                           pcl::PointCloud_PointXYZI &cloud);
int32_t save_pcd_file_xyzi(rust::Str file_name,
                           const pcl::PointCloud_PointXYZI &cloud, bool binary);
int32_t save_pcd_file_ascii_xyzi(rust::Str file_name,
                                 const pcl::PointCloud_PointXYZI &cloud);
int32_t save_pcd_file_binary_xyzi(rust::Str file_name,
                                  const pcl::PointCloud_PointXYZI &cloud);
int32_t
save_pcd_file_binary_compressed_xyzi(rust::Str file_name,
                                     const pcl::PointCloud_PointXYZI &cloud);

// PCD I/O functions for PointXYZRGB
int32_t load_pcd_file_xyzrgb(rust::Str file_name,
                             pcl::PointCloud_PointXYZRGB &cloud);
int32_t save_pcd_file_xyzrgb(rust::Str file_name,
                             const pcl::PointCloud_PointXYZRGB &cloud,
                             bool binary);
int32_t save_pcd_file_ascii_xyzrgb(rust::Str file_name,
                                   const pcl::PointCloud_PointXYZRGB &cloud);
int32_t save_pcd_file_binary_xyzrgb(rust::Str file_name,
                                    const pcl::PointCloud_PointXYZRGB &cloud);
int32_t save_pcd_file_binary_compressed_xyzrgb(
    rust::Str file_name, const pcl::PointCloud_PointXYZRGB &cloud);

// PLY I/O functions for PointXYZ
int32_t load_ply_file_xyz(rust::Str file_name, pcl::PointCloud_PointXYZ &cloud);
int32_t save_ply_file_xyz(rust::Str file_name,
                          const pcl::PointCloud_PointXYZ &cloud, bool binary);
int32_t save_ply_file_ascii_xyz(rust::Str file_name,
                                const pcl::PointCloud_PointXYZ &cloud);
int32_t save_ply_file_binary_xyz(rust::Str file_name,
                                 const pcl::PointCloud_PointXYZ &cloud);

// PLY I/O functions for PointXYZI
int32_t load_ply_file_xyzi(rust::Str file_name,
                           pcl::PointCloud_PointXYZI &cloud);
int32_t save_ply_file_xyzi(rust::Str file_name,
                           const pcl::PointCloud_PointXYZI &cloud, bool binary);
int32_t save_ply_file_ascii_xyzi(rust::Str file_name,
                                 const pcl::PointCloud_PointXYZI &cloud);
int32_t save_ply_file_binary_xyzi(rust::Str file_name,
                                  const pcl::PointCloud_PointXYZI &cloud);

// PLY I/O functions for PointXYZRGB
int32_t load_ply_file_xyzrgb(rust::Str file_name,
                             pcl::PointCloud_PointXYZRGB &cloud);
int32_t save_ply_file_xyzrgb(rust::Str file_name,
                             const pcl::PointCloud_PointXYZRGB &cloud,
                             bool binary);
int32_t save_ply_file_ascii_xyzrgb(rust::Str file_name,
                                   const pcl::PointCloud_PointXYZRGB &cloud);
int32_t save_ply_file_binary_xyzrgb(rust::Str file_name,
                                    const pcl::PointCloud_PointXYZRGB &cloud);

// Format auto-detection functions
int32_t detect_format_from_extension(rust::Str file_name);
int32_t detect_format_from_content(rust::Str file_name);
int32_t detect_file_format(rust::Str file_name);

// Auto-loading functions that detect format automatically
int32_t load_point_cloud_auto_xyz(rust::Str file_name,
                                  pcl::PointCloud_PointXYZ &cloud);
int32_t load_point_cloud_auto_xyzi(rust::Str file_name,
                                   pcl::PointCloud_PointXYZI &cloud);
int32_t load_point_cloud_auto_xyzrgb(rust::Str file_name,
                                     pcl::PointCloud_PointXYZRGB &cloud);

// Sample consensus functions (implemented in sample_consensus.cpp)
std::unique_ptr<pcl::RandomSampleConsensus_PointXYZ>
new_ransac_plane_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::RandomSampleConsensus_PointXYZ>
new_ransac_sphere_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::RandomSampleConsensus_PointXYZRGB>
new_ransac_plane_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
std::unique_ptr<pcl::RandomSampleConsensus_PointXYZRGB>
new_ransac_sphere_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

void set_distance_threshold_xyz(pcl::RandomSampleConsensus_PointXYZ &ransac,
                                double threshold);
double
get_distance_threshold_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);
void set_max_iterations_xyz(pcl::RandomSampleConsensus_PointXYZ &ransac,
                            int32_t max_iterations);
int32_t
get_max_iterations_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);
void set_probability_xyz(pcl::RandomSampleConsensus_PointXYZ &ransac,
                         double probability);
double get_probability_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);
bool compute_model_xyz(pcl::RandomSampleConsensus_PointXYZ &ransac);
bool refine_model_xyz(pcl::RandomSampleConsensus_PointXYZ &ransac, double sigma,
                      uint32_t max_iterations);
rust::Vec<int32_t>
get_inliers_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);
rust::Vec<float>
get_model_coefficients_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);
size_t get_inliers_count_xyz(const pcl::RandomSampleConsensus_PointXYZ &ransac);

void set_distance_threshold_xyzrgb(
    pcl::RandomSampleConsensus_PointXYZRGB &ransac, double threshold);
double get_distance_threshold_xyzrgb(
    const pcl::RandomSampleConsensus_PointXYZRGB &ransac);
void set_max_iterations_xyzrgb(pcl::RandomSampleConsensus_PointXYZRGB &ransac,
                               int32_t max_iterations);
int32_t
get_max_iterations_xyzrgb(const pcl::RandomSampleConsensus_PointXYZRGB &ransac);
void set_probability_xyzrgb(pcl::RandomSampleConsensus_PointXYZRGB &ransac,
                            double probability);
double
get_probability_xyzrgb(const pcl::RandomSampleConsensus_PointXYZRGB &ransac);
bool compute_model_xyzrgb(pcl::RandomSampleConsensus_PointXYZRGB &ransac);
bool refine_model_xyzrgb(pcl::RandomSampleConsensus_PointXYZRGB &ransac,
                         double sigma, uint32_t max_iterations);
rust::Vec<int32_t>
get_inliers_xyzrgb(const pcl::RandomSampleConsensus_PointXYZRGB &ransac);
rust::Vec<float> get_model_coefficients_xyzrgb(
    const pcl::RandomSampleConsensus_PointXYZRGB &ransac);
size_t
get_inliers_count_xyzrgb(const pcl::RandomSampleConsensus_PointXYZRGB &ransac);

// Standalone model creation and configuration - PointXYZ
std::unique_ptr<pcl::SampleConsensusModelPlane_PointXYZ>
new_sac_model_plane_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::SampleConsensusModelSphere_PointXYZ>
new_sac_model_sphere_xyz(const pcl::PointCloud_PointXYZ &cloud);

// Model-specific functions for plane - PointXYZ
bool compute_model_coefficients_plane_xyz(
    pcl::SampleConsensusModelPlane_PointXYZ &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients);
void get_distances_to_model_plane_xyz(
    pcl::SampleConsensusModelPlane_PointXYZ &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances);
rust::Vec<int32_t>
select_within_distance_plane_xyz(pcl::SampleConsensusModelPlane_PointXYZ &model,
                                 const rust::Vec<float> &model_coefficients,
                                 double threshold);
int32_t
count_within_distance_plane_xyz(pcl::SampleConsensusModelPlane_PointXYZ &model,
                                const rust::Vec<float> &model_coefficients,
                                double threshold);
bool optimize_model_coefficients_plane_xyz(
    pcl::SampleConsensusModelPlane_PointXYZ &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients);
std::unique_ptr<pcl::PointCloud_PointXYZ>
project_points_plane_xyz(pcl::SampleConsensusModelPlane_PointXYZ &model,
                         const rust::Vec<int32_t> &inliers,
                         const rust::Vec<float> &model_coefficients,
                         bool copy_data_fields);

// Model-specific functions for sphere - PointXYZ
void set_radius_limits_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model, double min_radius,
    double max_radius);
void get_radius_limits_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model, double &min_radius,
    double &max_radius);
bool compute_model_coefficients_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients);
void get_distances_to_model_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances);
rust::Vec<int32_t> select_within_distance_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model,
    const rust::Vec<float> &model_coefficients, double threshold);
int32_t count_within_distance_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model,
    const rust::Vec<float> &model_coefficients, double threshold);
bool optimize_model_coefficients_sphere_xyz(
    pcl::SampleConsensusModelSphere_PointXYZ &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients);
std::unique_ptr<pcl::PointCloud_PointXYZ>
project_points_sphere_xyz(pcl::SampleConsensusModelSphere_PointXYZ &model,
                          const rust::Vec<int32_t> &inliers,
                          const rust::Vec<float> &model_coefficients,
                          bool copy_data_fields);

// Standalone model creation and configuration - PointXYZRGB
std::unique_ptr<pcl::SampleConsensusModelPlane_PointXYZRGB>
new_sac_model_plane_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
std::unique_ptr<pcl::SampleConsensusModelSphere_PointXYZRGB>
new_sac_model_sphere_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

// Model-specific functions for plane - PointXYZRGB
bool compute_model_coefficients_plane_xyzrgb(
    pcl::SampleConsensusModelPlane_PointXYZRGB &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients);
void get_distances_to_model_plane_xyzrgb(
    pcl::SampleConsensusModelPlane_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances);
rust::Vec<int32_t> select_within_distance_plane_xyzrgb(
    pcl::SampleConsensusModelPlane_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, double threshold);
int32_t count_within_distance_plane_xyzrgb(
    pcl::SampleConsensusModelPlane_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, double threshold);
bool optimize_model_coefficients_plane_xyzrgb(
    pcl::SampleConsensusModelPlane_PointXYZRGB &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
project_points_plane_xyzrgb(pcl::SampleConsensusModelPlane_PointXYZRGB &model,
                            const rust::Vec<int32_t> &inliers,
                            const rust::Vec<float> &model_coefficients,
                            bool copy_data_fields);

// Model-specific functions for sphere - PointXYZRGB
void set_radius_limits_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model, double min_radius,
    double max_radius);
void get_radius_limits_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model, double &min_radius,
    double &max_radius);
bool compute_model_coefficients_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model,
    const rust::Vec<int32_t> &sample_indices, rust::Vec<float> &coefficients);
void get_distances_to_model_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, rust::Vec<double> &distances);
rust::Vec<int32_t> select_within_distance_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, double threshold);
int32_t count_within_distance_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model,
    const rust::Vec<float> &model_coefficients, double threshold);
bool optimize_model_coefficients_sphere_xyzrgb(
    pcl::SampleConsensusModelSphere_PointXYZRGB &model,
    const rust::Vec<int32_t> &inliers,
    const rust::Vec<float> &model_coefficients,
    rust::Vec<float> &optimized_coefficients);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
project_points_sphere_xyzrgb(pcl::SampleConsensusModelSphere_PointXYZRGB &model,
                             const rust::Vec<int32_t> &inliers,
                             const rust::Vec<float> &model_coefficients,
                             bool copy_data_fields);

// Segmentation functions (implemented in segmentation.cpp)
std::unique_ptr<pcl::RegionGrowing_PointXYZ_Normal> new_region_growing_xyz();
void set_input_cloud_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                        const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg,
    const pcl::PointCloud_Normal &normals);
void set_min_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t min_size);
int32_t
get_min_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_max_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t max_size);
int32_t
get_max_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold);
float get_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold);
float get_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t k);
int32_t get_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg);
rust::Vec<int32_t>
extract_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);

std::unique_ptr<pcl::RegionGrowingRGB_PointXYZRGB>
new_region_growing_rgb_xyzrgb();
void set_input_cloud_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, int32_t min_size);
int32_t get_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg);
rust::Vec<int32_t>
extract_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);

std::unique_ptr<pcl::EuclideanClusterExtraction_PointXYZ>
new_euclidean_cluster_extraction_xyz();
void set_input_cloud_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece,
    const pcl::PointCloud_PointXYZ &cloud);
void set_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, double tolerance);
double get_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece);
void set_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, int32_t min_size);
int32_t get_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece);
void set_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece, int32_t max_size);
int32_t get_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ece);
rust::Vec<int32_t>
extract_euclidean_clusters_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece);

std::unique_ptr<pcl::SACSegmentation_PointXYZ> new_sac_segmentation_xyz();
void set_input_cloud_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                            int32_t model_type);
int32_t get_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             int32_t method_type);
int32_t get_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                    double threshold);
double get_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                int32_t max_iterations);
int32_t get_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                       bool optimize);
bool get_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
bool segment_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                     rust::Vec<int32_t> &inliers,
                     rust::Vec<float> &coefficients);

std::unique_ptr<pcl::PointCloud_Normal>
estimate_normals_xyz(const pcl::PointCloud_PointXYZ &cloud, double radius);

std::unique_ptr<pcl::MinCutSegmentation_PointXYZ>
new_min_cut_segmentation_xyz();
void set_input_cloud_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                 const pcl::PointCloud_PointXYZ &cloud);
void set_foreground_points_min_cut_xyz(
    pcl::MinCutSegmentation_PointXYZ &mc,
    const pcl::PointCloud_PointXYZ &foreground_points);
void set_sigma_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc, double sigma);
double get_sigma_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);
void set_radius_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                            double radius);
double get_radius_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);
void set_number_of_neighbours_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                          int32_t k);
int32_t
get_number_of_neighbours_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);
void set_source_weight_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                   double weight);
double get_source_weight_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);
rust::Vec<int32_t> extract_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);

std::unique_ptr<pcl::ExtractPolygonalPrismData_PointXYZ>
new_extract_polygonal_prism_xyz();
void set_input_cloud_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &cloud);
void set_input_planar_hull_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &hull);
void set_height_limits_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism, double height_min,
    double height_max);
rust::Vec<int32_t>
segment_polygonal_prism_xyz(pcl::ExtractPolygonalPrismData_PointXYZ &prism);

std::unique_ptr<pcl::ProgressiveMorphologicalFilter_PointXYZ>
new_progressive_morphological_filter_xyz();
void set_input_cloud_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_max_window_size_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf, int32_t max_window_size);
int32_t
get_max_window_size_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_slope_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                       float slope);
float get_slope_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_max_distance_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                              float max_distance);
float get_max_distance_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_initial_distance_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf, float initial_distance);
float get_initial_distance_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_cell_size_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                           float cell_size);
float get_cell_size_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_base_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                      float base);
float get_base_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_exponential_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             bool exponential);
bool get_exponential_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
rust::Vec<int32_t>
extract_ground_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);

std::unique_ptr<pcl::ConditionalEuclideanClustering_PointXYZ>
new_conditional_euclidean_clustering_xyz();
void set_input_cloud_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec,
    const pcl::PointCloud_PointXYZ &cloud);
void set_cluster_tolerance_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, float tolerance);
float get_cluster_tolerance_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
void set_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t min_size);
int32_t get_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
void set_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t max_size);
int32_t get_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
rust::Vec<int32_t> extract_conditional_euclidean_clusters_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);

// Filter functions - PassThrough PointXYZ
std::unique_ptr<pcl::PassThrough_PointXYZ> new_pass_through_xyz();
void set_input_cloud_pass_xyz(pcl::PassThrough_PointXYZ &filter,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_filter_field_name_xyz(pcl::PassThrough_PointXYZ &filter,
                               rust::Str field_name);
rust::String get_filter_field_name_xyz(const pcl::PassThrough_PointXYZ &filter);
void set_filter_limits_xyz(pcl::PassThrough_PointXYZ &filter, float min,
                           float max);
void set_filter_limits_negative_xyz(pcl::PassThrough_PointXYZ &filter,
                                    bool negative);
bool get_filter_limits_negative_xyz(const pcl::PassThrough_PointXYZ &filter);
void set_keep_organized_xyz(pcl::PassThrough_PointXYZ &filter,
                            bool keep_organized);
bool get_keep_organized_xyz(const pcl::PassThrough_PointXYZ &filter);
std::unique_ptr<pcl::PointCloud_PointXYZ>
filter_pass_xyz(pcl::PassThrough_PointXYZ &filter);

// Filter functions - PassThrough PointXYZRGB
std::unique_ptr<pcl::PassThrough_PointXYZRGB> new_pass_through_xyzrgb();
void set_input_cloud_pass_xyzrgb(pcl::PassThrough_PointXYZRGB &filter,
                                 const pcl::PointCloud_PointXYZRGB &cloud);
void set_filter_field_name_xyzrgb(pcl::PassThrough_PointXYZRGB &filter,
                                  rust::Str field_name);
rust::String
get_filter_field_name_xyzrgb(const pcl::PassThrough_PointXYZRGB &filter);
void set_filter_limits_xyzrgb(pcl::PassThrough_PointXYZRGB &filter, float min,
                              float max);
void set_filter_limits_negative_xyzrgb(pcl::PassThrough_PointXYZRGB &filter,
                                       bool negative);
bool get_filter_limits_negative_xyzrgb(
    const pcl::PassThrough_PointXYZRGB &filter);
void set_keep_organized_xyzrgb(pcl::PassThrough_PointXYZRGB &filter,
                               bool keep_organized);
bool get_keep_organized_xyzrgb(const pcl::PassThrough_PointXYZRGB &filter);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
filter_pass_xyzrgb(pcl::PassThrough_PointXYZRGB &filter);

// Filter functions - VoxelGrid PointXYZ
std::unique_ptr<pcl::VoxelGrid_PointXYZ> new_voxel_grid_xyz();
void set_input_cloud_voxel_xyz(pcl::VoxelGrid_PointXYZ &filter,
                               const pcl::PointCloud_PointXYZ &cloud);
void set_leaf_size_xyz(pcl::VoxelGrid_PointXYZ &filter, float lx, float ly,
                       float lz);
std::unique_ptr<pcl::PointCloud_PointXYZ>
filter_voxel_xyz(pcl::VoxelGrid_PointXYZ &filter);

// Filter functions - VoxelGrid PointXYZRGB
std::unique_ptr<pcl::VoxelGrid_PointXYZRGB> new_voxel_grid_xyzrgb();
void set_input_cloud_voxel_xyzrgb(pcl::VoxelGrid_PointXYZRGB &filter,
                                  const pcl::PointCloud_PointXYZRGB &cloud);
void set_leaf_size_xyzrgb(pcl::VoxelGrid_PointXYZRGB &filter, float lx,
                          float ly, float lz);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
filter_voxel_xyzrgb(pcl::VoxelGrid_PointXYZRGB &filter);

// Filter functions - StatisticalOutlierRemoval PointXYZ
std::unique_ptr<pcl::StatisticalOutlierRemoval_PointXYZ>
new_statistical_outlier_removal_xyz();
void set_input_cloud_statistical_xyz(
    pcl::StatisticalOutlierRemoval_PointXYZ &filter,
    const pcl::PointCloud_PointXYZ &cloud);
void set_mean_k_statistical_xyz(pcl::StatisticalOutlierRemoval_PointXYZ &filter,
                                int mean_k);
void set_std_dev_mul_thresh_statistical_xyz(
    pcl::StatisticalOutlierRemoval_PointXYZ &filter, double stddev_mult);
void set_negative_statistical_xyz(
    pcl::StatisticalOutlierRemoval_PointXYZ &filter, bool negative);
std::unique_ptr<pcl::PointCloud_PointXYZ>
filter_statistical_xyz(pcl::StatisticalOutlierRemoval_PointXYZ &filter);

// Filter functions - StatisticalOutlierRemoval PointXYZRGB
std::unique_ptr<pcl::StatisticalOutlierRemoval_PointXYZRGB>
new_statistical_outlier_removal_xyzrgb();
void set_input_cloud_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval_PointXYZRGB &filter,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_mean_k_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval_PointXYZRGB &filter, int mean_k);
void set_std_dev_mul_thresh_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval_PointXYZRGB &filter, double stddev_mult);
void set_negative_statistical_xyzrgb(
    pcl::StatisticalOutlierRemoval_PointXYZRGB &filter, bool negative);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
filter_statistical_xyzrgb(pcl::StatisticalOutlierRemoval_PointXYZRGB &filter);

// Filter functions - RadiusOutlierRemoval PointXYZ
std::unique_ptr<pcl::RadiusOutlierRemoval_PointXYZ>
new_radius_outlier_removal_xyz();
void set_input_cloud_radius_xyz(pcl::RadiusOutlierRemoval_PointXYZ &filter,
                                const pcl::PointCloud_PointXYZ &cloud);
void set_radius_search_xyz(pcl::RadiusOutlierRemoval_PointXYZ &filter,
                           double radius);
void set_min_neighbors_in_radius_xyz(pcl::RadiusOutlierRemoval_PointXYZ &filter,
                                     int min_neighbors);
void set_negative_radius_xyz(pcl::RadiusOutlierRemoval_PointXYZ &filter,
                             bool negative);
std::unique_ptr<pcl::PointCloud_PointXYZ>
filter_radius_xyz(pcl::RadiusOutlierRemoval_PointXYZ &filter);

// Filter functions - RadiusOutlierRemoval PointXYZRGB
std::unique_ptr<pcl::RadiusOutlierRemoval_PointXYZRGB>
new_radius_outlier_removal_xyzrgb();
void set_input_cloud_radius_xyzrgb(
    pcl::RadiusOutlierRemoval_PointXYZRGB &filter,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_radius_search_xyzrgb(pcl::RadiusOutlierRemoval_PointXYZRGB &filter,
                              double radius);
void set_min_neighbors_in_radius_xyzrgb(
    pcl::RadiusOutlierRemoval_PointXYZRGB &filter, int min_neighbors);
void set_negative_radius_xyzrgb(pcl::RadiusOutlierRemoval_PointXYZRGB &filter,
                                bool negative);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
filter_radius_xyzrgb(pcl::RadiusOutlierRemoval_PointXYZRGB &filter);

// Surface reconstruction functions (implemented in surface.cpp)

// Marching Cubes Hoppe reconstruction
std::unique_ptr<pcl::MarchingCubesHoppe_PointXYZ>
new_marching_cubes_hoppe_xyz();
void set_iso_level_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                             float iso_level);
float get_iso_level_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc);
void set_grid_resolution_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                   int32_t res_x, int32_t res_y, int32_t res_z);
void set_percentage_extend_grid_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                          float percentage);
void set_input_cloud_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                               const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                         pcl::PolygonMesh &mesh);

// Marching Cubes RBF reconstruction
std::unique_ptr<pcl::MarchingCubesRBF_PointXYZ> new_marching_cubes_rbf_xyz();
void set_iso_level_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc, float iso_level);
float get_iso_level_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc);
void set_grid_resolution_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                 int32_t res_x, int32_t res_y, int32_t res_z);
void set_percentage_extend_grid_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                        float percentage);
void set_off_surface_displacement_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                          float displacement);
void set_input_cloud_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                             const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                       pcl::PolygonMesh &mesh);

// Organized Fast Mesh
std::unique_ptr<pcl::OrganizedFastMesh_PointXYZ> new_organized_fast_mesh_xyz();
void set_triangle_pixel_size_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                 int32_t triangle_size);
int32_t get_triangle_pixel_size_xyz(const pcl::OrganizedFastMesh_PointXYZ &ofm);
void set_triangulation_type_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                int32_t type);
int32_t get_triangulation_type_xyz(const pcl::OrganizedFastMesh_PointXYZ &ofm);
void set_input_cloud_ofm_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                             const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_ofm_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                       pcl::PolygonMesh &mesh);

// Polygon mesh utility functions
std::unique_ptr<pcl::PolygonMesh> new_polygon_mesh();
size_t get_polygon_count(const pcl::PolygonMesh &mesh);
size_t get_vertex_count(const pcl::PolygonMesh &mesh);
bool is_valid_mesh(const pcl::PolygonMesh &mesh);
int32_t save_polygon_mesh_ply(const pcl::PolygonMesh &mesh, rust::Str filename);
int32_t save_polygon_mesh_obj(const pcl::PolygonMesh &mesh, rust::Str filename);
int32_t save_polygon_mesh_vtk(const pcl::PolygonMesh &mesh, rust::Str filename);

// Poisson Surface Reconstruction
std::unique_ptr<pcl::Poisson_PointNormal> new_poisson();
void set_depth_poisson(pcl::Poisson_PointNormal &poisson, int32_t depth);
int32_t get_depth_poisson(pcl::Poisson_PointNormal &poisson);
void set_min_depth_poisson(pcl::Poisson_PointNormal &poisson,
                           int32_t min_depth);
int32_t get_min_depth_poisson(pcl::Poisson_PointNormal &poisson);
void set_point_weight_poisson(pcl::Poisson_PointNormal &poisson, float weight);
float get_point_weight_poisson(pcl::Poisson_PointNormal &poisson);
void set_scale_poisson(pcl::Poisson_PointNormal &poisson, float scale);
float get_scale_poisson(pcl::Poisson_PointNormal &poisson);
void set_solver_divide_poisson(pcl::Poisson_PointNormal &poisson,
                               int32_t solver_divide);
int32_t get_solver_divide_poisson(pcl::Poisson_PointNormal &poisson);
void set_iso_divide_poisson(pcl::Poisson_PointNormal &poisson,
                            int32_t iso_divide);
int32_t get_iso_divide_poisson(pcl::Poisson_PointNormal &poisson);
void set_samples_per_node_poisson(pcl::Poisson_PointNormal &poisson,
                                  float samples_per_node);
float get_samples_per_node_poisson(pcl::Poisson_PointNormal &poisson);
void set_confidence_poisson(pcl::Poisson_PointNormal &poisson, bool confidence);
bool get_confidence_poisson(pcl::Poisson_PointNormal &poisson);
void set_output_polygons_poisson(pcl::Poisson_PointNormal &poisson,
                                 bool output_polygons);
bool get_output_polygons_poisson(pcl::Poisson_PointNormal &poisson);
void set_degree_poisson(pcl::Poisson_PointNormal &poisson, int32_t degree);
int32_t get_degree_poisson(pcl::Poisson_PointNormal &poisson);
void set_manifold_poisson(pcl::Poisson_PointNormal &poisson, bool manifold);
bool get_manifold_poisson(pcl::Poisson_PointNormal &poisson);
void set_input_cloud_poisson(pcl::Poisson_PointNormal &poisson,
                             const pcl::PointCloud_PointNormal &cloud);
int32_t reconstruct_mesh_poisson(pcl::Poisson_PointNormal &poisson,
                                 pcl::PolygonMesh &mesh);

// Greedy Projection Triangulation
std::unique_ptr<pcl::GreedyProjectionTriangulation_PointNormal>
new_greedy_projection_triangulation();
void set_mu_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gpt,
                   double mu);
double get_mu_greedy(const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_search_radius_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, double radius);
double get_search_radius_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_minimum_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, double angle);
double get_minimum_angle_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_maximum_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, double angle);
double get_maximum_angle_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_maximum_nearest_neighbors_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, int32_t max_nn);
int32_t get_maximum_nearest_neighbors_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_maximum_surface_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, double angle);
double get_maximum_surface_angle_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_normal_consistency_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, bool consistent);
bool get_normal_consistency_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_consistent_vertex_ordering_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gpt, bool consistent);
bool get_consistent_vertex_ordering_greedy(
    const pcl::GreedyProjectionTriangulation_PointNormal &gpt);
void set_input_cloud_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gpt,
                            const pcl::PointCloud_PointNormal &cloud);
int32_t
reconstruct_mesh_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gpt,
                        pcl::PolygonMesh &mesh);

// Moving Least Squares
std::unique_ptr<pcl::MovingLeastSquares_PointXYZ_PointNormal>
new_moving_least_squares();
void set_search_radius_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                           double radius);
double
get_search_radius_mls(const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_polynomial_order_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                              int32_t order);
int32_t get_polynomial_order_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_sqr_gauss_param_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             double sqr_gauss_param);
double get_sqr_gauss_param_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_compute_normals_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             bool compute_normals);
void set_upsample_method_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             int32_t method);
void set_upsampling_radius_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, double radius);
double get_upsampling_radius_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_upsampling_step_size_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, double step_size);
double get_upsampling_step_size_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_desired_num_points_in_radius_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, int32_t num_points);
void set_dilation_voxel_size_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, float voxel_size);
float get_dilation_voxel_size_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_dilation_iterations_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, int32_t iterations);
int32_t get_dilation_iterations_mls(
    const pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_input_cloud_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                         const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::PointCloud_PointNormal>
process_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);

// Visualization functions - only available when VTK is present
#ifdef VTK_MAJOR_VERSION

// PCLVisualizer functions
std::unique_ptr<pcl::visualization::PCLVisualizer>
new_pcl_visualizer(rust::Str window_name);
int32_t add_point_cloud_xyz(pcl::visualization::PCLVisualizer &viewer,
                            const pcl::PointCloud<pcl::PointXYZ> &cloud,
                            rust::Str id);
int32_t add_point_cloud_xyzrgb(pcl::visualization::PCLVisualizer &viewer,
                               const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                               rust::Str id);
int32_t update_point_cloud_xyz(pcl::visualization::PCLVisualizer &viewer,
                               const pcl::PointCloud<pcl::PointXYZ> &cloud,
                               rust::Str id);
int32_t
update_point_cloud_xyzrgb(pcl::visualization::PCLVisualizer &viewer,
                          const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                          rust::Str id);
int32_t remove_point_cloud(pcl::visualization::PCLVisualizer &viewer,
                           rust::Str id);
int32_t
set_point_cloud_render_properties_xyz(pcl::visualization::PCLVisualizer &viewer,
                                      int32_t property, double value,
                                      rust::Str id);
int32_t set_background_color(pcl::visualization::PCLVisualizer &viewer,
                             double r, double g, double b);
int32_t add_coordinate_system(pcl::visualization::PCLVisualizer &viewer,
                              double scale, rust::Str id);
int32_t spin_once(pcl::visualization::PCLVisualizer &viewer, int32_t time);
void spin(pcl::visualization::PCLVisualizer &viewer);
bool was_stopped(const pcl::visualization::PCLVisualizer &viewer);
void close(pcl::visualization::PCLVisualizer &viewer);
void reset_stopped_flag(pcl::visualization::PCLVisualizer &viewer);
int32_t set_camera_position(pcl::visualization::PCLVisualizer &viewer,
                            double pos_x, double pos_y, double pos_z,
                            double view_x, double view_y, double view_z,
                            double up_x, double up_y, double up_z);
int32_t reset_camera(pcl::visualization::PCLVisualizer &viewer);

// CloudViewer functions
std::unique_ptr<pcl::visualization::CloudViewer>
new_cloud_viewer(rust::Str window_name);
int32_t show_cloud_xyz(pcl::visualization::CloudViewer &viewer,
                       const pcl::PointCloud<pcl::PointXYZ> &cloud,
                       rust::Str cloud_name);
int32_t show_cloud_xyzrgb(pcl::visualization::CloudViewer &viewer,
                          const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                          rust::Str cloud_name);
bool cloud_viewer_was_stopped(const pcl::visualization::CloudViewer &viewer);
void wait_for_cloud_viewer(pcl::visualization::CloudViewer &viewer,
                           int32_t time_ms);

// Additional utility functions
int32_t set_point_cloud_color_xyz(pcl::visualization::PCLVisualizer &viewer,
                                  double r, double g, double b, rust::Str id);
int32_t add_text(pcl::visualization::PCLVisualizer &viewer, rust::Str text,
                 int32_t xpos, int32_t ypos, double r, double g, double b,
                 rust::Str id);
int32_t add_sphere_xyz(pcl::visualization::PCLVisualizer &viewer,
                       const pcl::PointXYZ &center, double radius, double r,
                       double g, double b, rust::Str id);
int32_t remove_shape(pcl::visualization::PCLVisualizer &viewer, rust::Str id);
int32_t register_keyboard_callback(pcl::visualization::PCLVisualizer &viewer);

// Feature estimation functions (implemented in features.cpp)
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

// FPFH estimation OMP - PointXYZ
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

// Helper functions for feature extraction
rust::Vec<float> get_fpfh_histogram(const pcl::FPFHSignature33 &signature);
rust::Vec<float> get_pfh_histogram(const pcl::PFHSignature125 &signature);
rust::Vec<float> get_normal_vector(const pcl::Normal &normal);

// Feature cloud functions
size_t size_normal(const pcl::PointCloud_Normal &cloud);
bool empty_normal(const pcl::PointCloud_Normal &cloud);
size_t size_fpfh(const pcl::PointCloud_FPFHSignature33 &cloud);
bool empty_fpfh(const pcl::PointCloud_FPFHSignature33 &cloud);
size_t size_pfh(const pcl::PointCloud_PFHSignature125 &cloud);
bool empty_pfh(const pcl::PointCloud_PFHSignature125 &cloud);

// Keypoints function declarations - Harris 3D keypoint detector
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

// Point clone and transform functions are declared in common_functions.h

#endif
