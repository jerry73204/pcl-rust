#pragma once

#include "pcl/features/fpfh.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/pfh.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/keypoints/harris_3d.h"
#include "pcl/keypoints/iss_3d.h"
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/octree/octree_pointcloud.h"
#include "pcl/octree/octree_pointcloud_voxelcentroid.h"
#include "pcl/octree/octree_search.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/ndt.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/sac_model_sphere.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/segmentation/region_growing_rgb.h"
#include "pcl/segmentation/sac_segmentation.h"

// Type aliases to match cxx bridge expectations
namespace pcl {
using PointCloud_PointXYZ = PointCloud<PointXYZ>;
using PointCloud_PointXYZI = PointCloud<PointXYZI>;
using PointCloud_PointXYZRGB = PointCloud<PointXYZRGB>;

namespace search {
using KdTree_PointXYZ = KdTree<PointXYZ>;
using KdTree_PointXYZI = KdTree<PointXYZI>;
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

// Filter type aliases
using PassThrough_PointXYZ = PassThrough<PointXYZ>;
using PassThrough_PointXYZRGB = PassThrough<PointXYZRGB>;
using VoxelGrid_PointXYZ = VoxelGrid<PointXYZ>;
using VoxelGrid_PointXYZRGB = VoxelGrid<PointXYZRGB>;
using StatisticalOutlierRemoval_PointXYZ = StatisticalOutlierRemoval<PointXYZ>;
using StatisticalOutlierRemoval_PointXYZRGB =
    StatisticalOutlierRemoval<PointXYZRGB>;
using RadiusOutlierRemoval_PointXYZ = RadiusOutlierRemoval<PointXYZ>;
using RadiusOutlierRemoval_PointXYZRGB = RadiusOutlierRemoval<PointXYZRGB>;

// Registration type aliases
using IterativeClosestPoint_PointXYZ =
    IterativeClosestPoint<PointXYZ, PointXYZ>;
using IterativeClosestPoint_PointXYZRGB =
    IterativeClosestPoint<PointXYZRGB, PointXYZRGB>;
using NormalDistributionsTransform_PointXYZ =
    NormalDistributionsTransform<PointXYZ, PointXYZ>;
using NormalDistributionsTransform_PointXYZRGB =
    NormalDistributionsTransform<PointXYZRGB, PointXYZRGB>;

// Segmentation type aliases
using PointCloud_Normal = PointCloud<Normal>;
using RegionGrowing_PointXYZ_Normal = RegionGrowing<PointXYZ, Normal>;
using RegionGrowingRGB_PointXYZRGB = RegionGrowingRGB<PointXYZRGB>;
using EuclideanClusterExtraction_PointXYZ =
    EuclideanClusterExtraction<PointXYZ>;
using SACSegmentation_PointXYZ = SACSegmentation<PointXYZ>;

// Features type aliases
using NormalEstimation_PointXYZ_Normal = NormalEstimation<PointXYZ, Normal>;
using NormalEstimationOMP_PointXYZ_Normal =
    NormalEstimationOMP<PointXYZ, Normal>;
using FPFHEstimation_PointXYZ_Normal_FPFHSignature33 =
    FPFHEstimation<PointXYZ, Normal, FPFHSignature33>;
using FPFHEstimationOMP_PointXYZ_Normal_FPFHSignature33 =
    FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33>;
using PFHEstimation_PointXYZ_Normal_PFHSignature125 =
    PFHEstimation<PointXYZ, Normal, PFHSignature125>;
using PointCloud_FPFHSignature33 = PointCloud<FPFHSignature33>;
using PointCloud_PFHSignature125 = PointCloud<PFHSignature125>;

// Keypoints type aliases
using HarrisKeypoint3D_PointXYZ_PointXYZI =
    HarrisKeypoint3D<PointXYZ, PointXYZI>;
using ISSKeypoint3D_PointXYZ_PointXYZ = ISSKeypoint3D<PointXYZ, PointXYZ>;
using SIFTKeypoint_PointXYZI_PointWithScale =
    SIFTKeypoint<PointXYZI, PointWithScale>;
using PointCloud_PointXYZI = PointCloud<PointXYZI>;
using PointCloud_PointWithScale = PointCloud<PointWithScale>;
} // namespace pcl
