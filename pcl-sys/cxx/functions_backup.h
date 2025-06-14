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

// Model creation functions - PointXYZ
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>
new_plane_model_xyz(const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>
new_sphere_model_xyz(const pcl::PointCloud_PointXYZ &cloud);

// Model creation functions - PointXYZRGB
std::unique_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>>
new_plane_model_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
std::unique_ptr<pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>>
new_sphere_model_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

// Plane model methods - PointXYZ
rust::Vec<float> plane_compute_model_coefficients_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> samples);
std::vector<double> plane_get_distances_to_model_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients);
std::vector<int32_t> plane_select_within_distance_xyz(
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold);
size_t plane_count_within_distance_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold);
std::vector<float> plane_optimize_model_coefficients_xyz(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients);

// Sphere model methods - PointXYZ
std::vector<float> sphere_compute_model_coefficients_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> samples);
std::vector<double> sphere_get_distances_to_model_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients);
std::vector<int32_t> sphere_select_within_distance_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold);
size_t sphere_count_within_distance_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const float> coefficients, double threshold);
std::vector<float> sphere_optimize_model_coefficients_xyz(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients);
void sphere_set_radius_limits_xyz(
    pcl::SampleConsensusModelSphere<pcl::PointXYZ> &model, double min_radius,
    double max_radius);

// Plane model methods - PointXYZRGB
std::vector<float> plane_compute_model_coefficients_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> samples);
std::vector<double> plane_get_distances_to_model_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients);
std::vector<int32_t> plane_select_within_distance_xyzrgb(
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold);
size_t plane_count_within_distance_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold);
std::vector<float> plane_optimize_model_coefficients_xyzrgb(
    const pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients);

// Sphere model methods - PointXYZRGB
std::vector<float> sphere_compute_model_coefficients_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> samples);
std::vector<double> sphere_get_distances_to_model_xyzrgb(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients);
std::vector<int32_t> sphere_select_within_distance_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold);
size_t sphere_count_within_distance_xyzrgb(
    const pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const float> coefficients, double threshold);
std::vector<float> sphere_optimize_model_coefficients_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model,
    rust::Slice<const int32_t> inliers, rust::Slice<const float> coefficients);
void sphere_set_radius_limits_xyzrgb(
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> &model, double min_radius,
    double max_radius);
