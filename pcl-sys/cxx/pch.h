// Precompiled header for faster compilation
#pragma once

// Standard library includes
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// PCL core includes - these are the heavy headers
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// Rust CXX includes
#include "rust/cxx.h"
