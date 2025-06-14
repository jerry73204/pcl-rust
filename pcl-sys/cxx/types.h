#pragma once

#include "pcl/octree/octree_pointcloud.h"
#include "pcl/octree/octree_pointcloud_voxelcentroid.h"
#include "pcl/octree/octree_search.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/sac_model_sphere.h"
#include "pcl/search/kdtree.h"

// Type aliases to match cxx bridge expectations
namespace pcl {
using PointCloud_PointXYZ = PointCloud<PointXYZ>;
using PointCloud_PointXYZI = PointCloud<PointXYZI>;
using PointCloud_PointXYZRGB = PointCloud<PointXYZRGB>;

namespace search {
using KdTree_PointXYZ = KdTree<PointXYZ>;
using KdTree_PointXYZRGB = KdTree<PointXYZRGB>;
} // namespace search

namespace octree {
using OctreePointCloudSearch_PointXYZ = OctreePointCloudSearch<PointXYZ>;
using OctreePointCloudVoxelCentroid_PointXYZ =
    OctreePointCloudVoxelCentroid<PointXYZ>;
} // namespace octree

// Sample consensus type aliases
using RandomSampleConsensus_PointXYZ = RandomSampleConsensus<PointXYZ>;
using RandomSampleConsensus_PointXYZRGB = RandomSampleConsensus<PointXYZRGB>;
using SampleConsensusModelPlane_PointXYZ = SampleConsensusModelPlane<PointXYZ>;
using SampleConsensusModelSphere_PointXYZ =
    SampleConsensusModelSphere<PointXYZ>;
using SampleConsensusModelPlane_PointXYZRGB =
    SampleConsensusModelPlane<PointXYZRGB>;
using SampleConsensusModelSphere_PointXYZRGB =
    SampleConsensusModelSphere<PointXYZRGB>;
} // namespace pcl
