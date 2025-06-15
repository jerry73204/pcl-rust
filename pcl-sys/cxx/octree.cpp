#include "cxx/functions.h"
#include <memory>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Octree functions
std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>
new_octree_search_xyz(double resolution) {
  return std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
      resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>
new_octree_search_xyzrgb(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>>
new_octree_voxel_centroid_xyz(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>>(resolution);
}

std::unique_ptr<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>>
new_octree_voxel_centroid_xyzrgb(double resolution) {
  return std::make_unique<
      pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>>(resolution);
}

void set_input_cloud_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  octree.setInputCloud(cloud.makeShared());
}

void add_points_from_input_cloud_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  octree.addPointsFromInputCloud();
}

rust::Vec<int32_t> nearest_k_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point, int32_t k) {
  std::vector<int> indices;
  std::vector<float> distances;
  octree.nearestKSearch(point, k, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<int32_t> radius_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point, double radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  octree.radiusSearch(point, radius, indices, distances);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

rust::Vec<int32_t> voxel_search_octree_xyz(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    const pcl::PointXYZ &point) {
  std::vector<int> indices;
  octree.voxelSearch(point, indices);
  rust::Vec<int32_t> result;
  for (int idx : indices) {
    result.push_back(static_cast<int32_t>(idx));
  }
  return result;
}

double
get_resolution(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getResolution();
}

uint32_t
get_tree_depth(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getTreeDepth();
}

size_t
get_leaf_count(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getLeafCount();
}

size_t
get_branch_count(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree) {
  return octree.getBranchCount();
}

// OctreeVoxelCentroid functions
void set_input_cloud_voxel_centroid_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree,
    const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  octree.setInputCloud(cloud.makeShared());
}

void add_points_from_input_cloud_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree) {
  octree.addPointsFromInputCloud();
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> get_voxel_centroids_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree) {
  auto cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

  // Get voxel centroids
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::AlignedPointTVector
      centroids;
  octree.getVoxelCentroids(centroids);

  // Convert to point cloud
  cloud->points.resize(centroids.size());
  for (size_t i = 0; i < centroids.size(); ++i) {
    cloud->points[i] = centroids[i];
  }
  cloud->width = centroids.size();
  cloud->height = 1;
  cloud->is_dense = true;

  return cloud;
}

double get_resolution_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree) {
  return octree.getResolution();
}

uint32_t get_tree_depth_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree) {
  return octree.getTreeDepth();
}

void delete_tree_voxel_xyz(
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &octree) {
  octree.deleteTree();
}