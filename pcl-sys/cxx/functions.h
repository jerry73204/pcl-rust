#pragma once

#include "rust/cxx.h"
#include "types.h"

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

// PointCloud functions
std::unique_ptr<pcl::PointCloud_PointXYZ> new_point_cloud_xyz();
std::unique_ptr<pcl::PointCloud_PointXYZI> new_point_cloud_xyzi();
std::unique_ptr<pcl::PointCloud_PointXYZRGB> new_point_cloud_xyzrgb();

size_t size(const pcl::PointCloud_PointXYZ &cloud);
size_t size_xyzi(const pcl::PointCloud_PointXYZI &cloud);
size_t size_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

bool empty(const pcl::PointCloud_PointXYZ &cloud);
bool empty_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool empty_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

void clear(pcl::PointCloud_PointXYZ &cloud);
void clear_xyzi(pcl::PointCloud_PointXYZI &cloud);
void clear_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud);

void reserve(pcl::PointCloud_PointXYZ &cloud, size_t size);
void reserve_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t size);
void reserve_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t size);

void resize(pcl::PointCloud_PointXYZ &cloud, size_t size);
void resize_xyzi(pcl::PointCloud_PointXYZI &cloud, size_t size);
void resize_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud, size_t size);

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

bool is_dense(const pcl::PointCloud_PointXYZ &cloud);
bool is_dense_xyzi(const pcl::PointCloud_PointXYZI &cloud);
bool is_dense_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

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
