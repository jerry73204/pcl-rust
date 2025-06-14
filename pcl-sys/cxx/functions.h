#pragma once

#include "rust/cxx.h"
#include "types.h"

// Function forward declarations (implemented in common.cpp)
std::unique_ptr<pcl::PointCloud_PointXYZ> new_point_cloud_xyz();
std::unique_ptr<pcl::PointCloud_PointXYZRGB> new_point_cloud_xyzrgb();
size_t size(const pcl::PointCloud_PointXYZ &cloud);
size_t size_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);
void clear(pcl::PointCloud_PointXYZ &cloud);
void clear_xyzrgb(pcl::PointCloud_PointXYZRGB &cloud);
bool empty(const pcl::PointCloud_PointXYZ &cloud);
bool empty_xyzrgb(const pcl::PointCloud_PointXYZRGB &cloud);

// Point functions
float get_x(const pcl::PointXYZ &point);
float get_y(const pcl::PointXYZ &point);
float get_z(const pcl::PointXYZ &point);
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
radius_search_xyz(const pcl::search::KdTree_PointXYZ &searcher,
                  const pcl::PointXYZ &point, double radius);
void set_input_cloud_xyz(pcl::search::KdTree_PointXYZ &searcher,
                         const pcl::PointCloud_PointXYZ &cloud);
void set_input_cloud_xyzrgb(pcl::search::KdTree_PointXYZRGB &searcher,
                            const pcl::PointCloud_PointXYZRGB &cloud);

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
