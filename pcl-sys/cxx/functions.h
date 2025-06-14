#pragma once

#include "rust/cxx.h"
#include "types.h"

// Function forward declarations (implemented in common.cpp)
std::unique_ptr<pcl::PointCloud_PointXYZ> new_point_cloud_xyz();
std::unique_ptr<pcl::PointCloud_PointXYZI> new_point_cloud_xyzi();
std::unique_ptr<pcl::PointCloud_PointXYZRGB> new_point_cloud_xyzrgb();
size_t size(const pcl::PointCloud_PointXYZ &cloud);
size_t size_xyzi(const pcl::PointCloud_PointXYZI &cloud);
size_t size_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
void clear(pcl::PointCloud_PointXYZ &cloud);
void clear_xyzi(pcl::PointCloud_PointXYZI &cloud);
void clear_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud);
bool empty(const pcl::PointCloud_PointXYZ &cloud);
bool empty_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool empty_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
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
bool is_dense(const pcl::PointCloud_PointXYZ &cloud);
bool is_dense_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool is_dense_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

// Point functions
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

// Search functions
std::unique_ptr<pcl::search::KdTree_PointXYZ> new_kdtree_xyz();
std::unique_ptr<pcl::search::KdTree_PointXYZRGB> new_kdtree_xyzrgb();
rust::Vec<int32_t>
nearest_k_search_xyz(const pcl::search::KdTree_PointXYZ &searcher,
                     const pcl::PointXYZ &point, int32_t k);
rust::Vec<int32_t>
nearest_k_search_xyzrgb(const pcl::search::KdTree_PointXYZRGB &searcher,
                        const pcl::PointXYZRGB &point, int32_t k);
rust::Vec<int32_t>
radius_search_xyz(const pcl::search::KdTree_PointXYZ &searcher,
                  const pcl::PointXYZ &point, double radius);
rust::Vec<int32_t>
radius_search_xyzrgb(const pcl::search::KdTree_PointXYZRGB &searcher,
                     const pcl::PointXYZRGB &point, double radius);
void set_input_cloud_xyz(pcl::search::KdTree_PointXYZ &searcher,
                         const pcl::PointCloud_PointXYZ &cloud);
void set_input_cloud_xyzrgb(pcl::search::KdTree_PointXYZRGB &searcher,
                            const pcl::PointCloud_PointXYZRGB &cloud);
float get_epsilon_xyz(const pcl::search::KdTree_PointXYZ &searcher);
void set_epsilon_xyz(pcl::search::KdTree_PointXYZ &searcher, float epsilon);
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

// OctreeVoxelCentroid functions
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
int load_pcd_file_xyz(rust::Str file_name, pcl::PointCloud_PointXYZ &cloud);
int save_pcd_file_xyz(rust::Str file_name,
                      const pcl::PointCloud_PointXYZ &cloud, bool binary);
int save_pcd_file_ascii_xyz(rust::Str file_name,
                            const pcl::PointCloud_PointXYZ &cloud);
int save_pcd_file_binary_xyz(rust::Str file_name,
                             const pcl::PointCloud_PointXYZ &cloud);
int save_pcd_file_binary_compressed_xyz(rust::Str file_name,
                                        const pcl::PointCloud_PointXYZ &cloud);

// PCD I/O functions for PointXYZI
int load_pcd_file_xyzi(rust::Str file_name, pcl::PointCloud_PointXYZI &cloud);
int save_pcd_file_xyzi(rust::Str file_name,
                       const pcl::PointCloud_PointXYZI &cloud, bool binary);
int save_pcd_file_ascii_xyzi(rust::Str file_name,
                             const pcl::PointCloud_PointXYZI &cloud);
int save_pcd_file_binary_xyzi(rust::Str file_name,
                              const pcl::PointCloud_PointXYZI &cloud);
int save_pcd_file_binary_compressed_xyzi(
    rust::Str file_name, const pcl::PointCloud_PointXYZI &cloud);

// PCD I/O functions for PointXYZRGB
int load_pcd_file_xyzrgb(rust::Str file_name,
                         pcl::PointCloud_PointXYZRGB &cloud);
int save_pcd_file_xyzrgb(rust::Str file_name,
                         const pcl::PointCloud_PointXYZRGB &cloud, bool binary);
int save_pcd_file_ascii_xyzrgb(rust::Str file_name,
                               const pcl::PointCloud_PointXYZRGB &cloud);
int save_pcd_file_binary_xyzrgb(rust::Str file_name,
                                const pcl::PointCloud_PointXYZRGB &cloud);
int save_pcd_file_binary_compressed_xyzrgb(
    rust::Str file_name, const pcl::PointCloud_PointXYZRGB &cloud);

// PLY I/O functions for PointXYZ
int load_ply_file_xyz(rust::Str file_name, pcl::PointCloud_PointXYZ &cloud);
int save_ply_file_xyz(rust::Str file_name,
                      const pcl::PointCloud_PointXYZ &cloud, bool binary);
int save_ply_file_ascii_xyz(rust::Str file_name,
                            const pcl::PointCloud_PointXYZ &cloud);
int save_ply_file_binary_xyz(rust::Str file_name,
                             const pcl::PointCloud_PointXYZ &cloud);

// PLY I/O functions for PointXYZI
int load_ply_file_xyzi(rust::Str file_name, pcl::PointCloud_PointXYZI &cloud);
int save_ply_file_xyzi(rust::Str file_name,
                       const pcl::PointCloud_PointXYZI &cloud, bool binary);
int save_ply_file_ascii_xyzi(rust::Str file_name,
                             const pcl::PointCloud_PointXYZI &cloud);
int save_ply_file_binary_xyzi(rust::Str file_name,
                              const pcl::PointCloud_PointXYZI &cloud);

// PLY I/O functions for PointXYZRGB
int load_ply_file_xyzrgb(rust::Str file_name,
                         pcl::PointCloud_PointXYZRGB &cloud);
int save_ply_file_xyzrgb(rust::Str file_name,
                         const pcl::PointCloud_PointXYZRGB &cloud, bool binary);
int save_ply_file_ascii_xyzrgb(rust::Str file_name,
                               const pcl::PointCloud_PointXYZRGB &cloud);
int save_ply_file_binary_xyzrgb(rust::Str file_name,
                                const pcl::PointCloud_PointXYZRGB &cloud);

// Sample consensus functions (implemented in sample_consensus.cpp)
// RANSAC creation and configuration - PointXYZ
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZ>>
new_ransac_plane_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZ>>
new_ransac_sphere_xyz(const pcl::PointCloud_PointXYZ &cloud);
void set_distance_threshold_xyz(
    pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac, double threshold);
double get_distance_threshold_xyz(
    const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
void set_max_iterations_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                            int32_t max_iterations);
int32_t
get_max_iterations_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
void set_probability_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                         double probability);
double
get_probability_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
bool compute_model_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
bool refine_model_xyz(pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac,
                      double sigma, uint32_t max_iterations);
rust::Vec<int32_t>
get_inliers_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
rust::Vec<float> get_model_coefficients_xyz(
    const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);
size_t
get_inliers_count_xyz(const pcl::RandomSampleConsensus<pcl::PointXYZ> &ransac);

// RANSAC creation and configuration - PointXYZRGB
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>
new_ransac_plane_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
std::unique_ptr<pcl::RandomSampleConsensus<pcl::PointXYZRGB>>
new_ransac_sphere_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
void set_distance_threshold_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac, double threshold);
double get_distance_threshold_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
void set_max_iterations_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac,
    int32_t max_iterations);
int32_t get_max_iterations_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
void set_probability_xyzrgb(
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac, double probability);
double get_probability_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
bool compute_model_xyzrgb(pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
bool refine_model_xyzrgb(pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac,
                         double sigma, uint32_t max_iterations);
rust::Vec<int32_t>
get_inliers_xyzrgb(const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
rust::Vec<float> get_model_coefficients_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);
size_t get_inliers_count_xyzrgb(
    const pcl::RandomSampleConsensus<pcl::PointXYZRGB> &ransac);

// Filter functions (implemented in filters.cpp)
// clang-format off
// PassThrough filter functions - PointXYZ
std::unique_ptr<pcl::PassThrough<pcl::PointXYZ>> new_pass_through_xyz();
void set_input_cloud_pass_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                               const pcl::PointCloud_PointXYZ &cloud);
void set_filter_field_name_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                                rust::Str field_name);
rust::String get_filter_field_name_xyz(const pcl::PassThrough<pcl::PointXYZ> &filter);
void set_filter_limits_xyz(pcl::PassThrough<pcl::PointXYZ> &filter, float min,
                           float max);
void set_filter_limits_negative_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                                    bool negative);
bool get_filter_limits_negative_xyz(const pcl::PassThrough<pcl::PointXYZ> &filter);
void set_keep_organized_xyz(pcl::PassThrough<pcl::PointXYZ> &filter,
                            bool keep_organized);
bool get_keep_organized_xyz(const pcl::PassThrough<pcl::PointXYZ> &filter);
std::unique_ptr<pcl::PointCloud_PointXYZ>
filter_pass_xyz(pcl::PassThrough<pcl::PointXYZ> &filter);

// PassThrough filter functions - PointXYZRGB
std::unique_ptr<pcl::PassThrough<pcl::PointXYZRGB>> new_pass_through_xyzrgb();
void set_input_cloud_pass_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                                  const pcl::PointCloud_PointXYZRGB &cloud);
void set_filter_field_name_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                                   rust::Str field_name);
rust::String get_filter_field_name_xyzrgb(const pcl::PassThrough<pcl::PointXYZRGB> &filter);
void set_filter_limits_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                               float min, float max);
void set_filter_limits_negative_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                                       bool negative);
bool get_filter_limits_negative_xyzrgb(const pcl::PassThrough<pcl::PointXYZRGB> &filter);
void set_keep_organized_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter,
                               bool keep_organized);
bool get_keep_organized_xyzrgb(const pcl::PassThrough<pcl::PointXYZRGB> &filter);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
filter_pass_xyzrgb(pcl::PassThrough<pcl::PointXYZRGB> &filter);
// clang-format on
// Registration functions (implemented in registration.cpp)
// clang-format off
// ICP functions - PointXYZ
std::unique_ptr<pcl::IterativeClosestPoint_PointXYZ> new_icp_xyz();
void set_input_source_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                               const pcl::PointCloud_PointXYZ &cloud);
void set_input_target_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                               const pcl::PointCloud_PointXYZ &cloud);
void set_max_iterations_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                                 int32_t iterations);
int32_t get_max_iterations_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_transformation_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                                        double epsilon);
double get_transformation_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_euclidean_fitness_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                                            double epsilon);
double get_euclidean_fitness_epsilon_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
void set_max_correspondence_distance_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                                             double distance);
double get_max_correspondence_distance_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
std::unique_ptr<pcl::PointCloud_PointXYZ>
align_with_guess_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp,
                         const rust::Vec<float> &guess);
bool has_converged_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
double get_fitness_score_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);
rust::Vec<float> get_final_transformation_icp_xyz(pcl::IterativeClosestPoint_PointXYZ &icp);

// ICP functions - PointXYZRGB
std::unique_ptr<pcl::IterativeClosestPoint_PointXYZRGB> new_icp_xyzrgb();
void set_input_source_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                  const pcl::PointCloud_PointXYZRGB &cloud);
void set_input_target_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                  const pcl::PointCloud_PointXYZRGB &cloud);
void set_max_iterations_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                    int32_t iterations);
int32_t get_max_iterations_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_transformation_epsilon_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                           double epsilon);
double get_transformation_epsilon_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_euclidean_fitness_epsilon_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                               double epsilon);
double get_euclidean_fitness_epsilon_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
void set_max_correspondence_distance_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                                                double distance);
double get_max_correspondence_distance_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
std::unique_ptr<pcl::PointCloud_PointXYZRGB>
align_with_guess_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp,
                             const rust::Vec<float> &guess);
bool has_converged_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
double get_fitness_score_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
rust::Vec<float> get_final_transformation_icp_xyzrgb(pcl::IterativeClosestPoint_PointXYZRGB &icp);
// clang-format on

// Segmentation functions (implemented in segmentation.cpp)
// clang-format off
// Region Growing segmentation - PointXYZ with Normal
std::unique_ptr<pcl::RegionGrowing_PointXYZ_Normal> new_region_growing_xyz();
void set_input_cloud_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                         const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                          const pcl::PointCloud_Normal &normals);
void set_min_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                             int32_t min_size);
int32_t get_min_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_max_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                             int32_t max_size);
int32_t get_max_cluster_size_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_smoothness_threshold_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                                 float threshold);
float get_smoothness_threshold_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_curvature_threshold_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                                float threshold);
float get_curvature_threshold_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_number_of_neighbours_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                                 int32_t k);
int32_t get_number_of_neighbours_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);
rust::Vec<int32_t> extract_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);

// Region Growing RGB segmentation - PointXYZRGB
std::unique_ptr<pcl::RegionGrowingRGB_PointXYZRGB> new_region_growing_rgb_xyzrgb();
void set_input_cloud_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg,
                                               const pcl::PointCloud_PointXYZRGB &cloud);
void set_distance_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg,
                                                      float threshold);
float get_distance_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_point_color_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg,
                                                         float threshold);
float get_point_color_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_region_color_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg,
                                                          float threshold);
float get_region_color_threshold_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_min_cluster_size_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg,
                                                    int32_t min_size);
int32_t get_min_cluster_size_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);
rust::Vec<int32_t> extract_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);

// Euclidean Cluster Extraction - PointXYZ
std::unique_ptr<pcl::EuclideanClusterExtraction_PointXYZ> new_euclidean_cluster_extraction_xyz();
void set_input_cloud_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece,
                                   const pcl::PointCloud_PointXYZ &cloud);
void set_cluster_tolerance_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece,
                                        double tolerance);
double get_cluster_tolerance_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece);
void set_min_cluster_size_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece,
                                       int32_t min_size);
int32_t get_min_cluster_size_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece);
void set_max_cluster_size_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece,
                                       int32_t max_size);
int32_t get_max_cluster_size_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece);
rust::Vec<int32_t> extract_euclidean_clusters_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ece);

// SAC Segmentation - PointXYZ
std::unique_ptr<pcl::SACSegmentation_PointXYZ> new_sac_segmentation_xyz();
void set_input_cloud_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac, int32_t model_type);
int32_t get_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac, int32_t method_type);
int32_t get_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac, double threshold);
double get_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac, int32_t max_iterations);
int32_t get_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
void set_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac, bool optimize);
bool get_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac);
bool segment_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                     rust::Vec<int32_t> &inliers,
                     rust::Vec<float> &coefficients);

// Normal estimation helper
std::unique_ptr<pcl::PointCloud_Normal> estimate_normals_xyz(const pcl::PointCloud_PointXYZ &cloud,
                                                             double radius);

// Features functions (implemented in features.cpp)
// clang-format off
// Normal estimation - PointXYZ
std::unique_ptr<pcl::NormalEstimation_PointXYZ_Normal> new_normal_estimation_xyz();
void set_input_cloud_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                const pcl::PointCloud_PointXYZ &cloud);
void set_search_method_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                 const pcl::search::KdTree_PointXYZ &tree);
void set_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne, double radius);
double get_radius_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);
void set_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne, int32_t k);
int32_t get_k_search_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);
void set_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                              float vpx, float vpy, float vpz);
rust::Vec<float> get_view_point_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);
void set_use_sensor_origin_normal_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne,
                                     bool use_sensor_origin);
std::unique_ptr<pcl::PointCloud_Normal> compute_normals_xyz(pcl::NormalEstimation_PointXYZ_Normal &ne);

// Normal estimation OMP - PointXYZ
std::unique_ptr<pcl::NormalEstimationOMP_PointXYZ_Normal> new_normal_estimation_omp_xyz();
void set_input_cloud_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
                                    const pcl::PointCloud_PointXYZ &cloud);
void set_search_method_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne,
                                     const pcl::search::KdTree_PointXYZ &tree);
void set_radius_search_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne, double radius);
void set_k_search_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne, int32_t k);
void set_number_of_threads_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne, int32_t threads);
int32_t get_number_of_threads_normal_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne);
std::unique_ptr<pcl::PointCloud_Normal> compute_normals_omp_xyz(pcl::NormalEstimationOMP_PointXYZ_Normal &ne);

// FPFH feature estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33> new_fpfh_estimation_xyz();
void set_input_cloud_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
                              const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
                                const pcl::PointCloud_Normal &normals);
void set_search_method_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh,
                                const pcl::search::KdTree_PointXYZ &tree);
void set_radius_search_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, double radius);
void set_k_search_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh, int32_t k);
std::unique_ptr<pcl::PointCloud_FPFHSignature33> compute_fpfh_xyz(pcl::FPFHEstimation_PointXYZ_Normal_FPFHSignature33 &fpfh);

// FPFH OMP feature estimation - PointXYZ
std::unique_ptr<pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33> new_fpfh_estimation_omp_xyz();
void set_input_cloud_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
                                  const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
                                   const pcl::PointCloud_Normal &normals);
void set_search_method_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh,
                                   const pcl::search::KdTree_PointXYZ &tree);
void set_radius_search_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh, double radius);
void set_number_of_threads_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh, int32_t threads);
std::unique_ptr<pcl::PointCloud_FPFHSignature33> compute_fpfh_omp_xyz(pcl::FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 &fpfh);

// PFH feature estimation - PointXYZ
std::unique_ptr<pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125> new_pfh_estimation_xyz();
void set_input_cloud_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
                              const pcl::PointCloud_Normal &normals);
void set_search_method_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh,
                              const pcl::search::KdTree_PointXYZ &tree);
void set_radius_search_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, double radius);
void set_k_search_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh, int32_t k);
std::unique_ptr<pcl::PointCloud_PFHSignature125> compute_pfh_xyz(pcl::PFHEstimation_PointXYZ_Normal_PFHSignature125 &pfh);

// Helper functions for feature data access
rust::Vec<float> get_fpfh_histogram(const pcl::FPFHSignature33 &signature);
rust::Vec<float> get_pfh_histogram(const pcl::PFHSignature125 &signature);
rust::Vec<float> get_normal_vector(const pcl::Normal &normal);
// clang-format on




