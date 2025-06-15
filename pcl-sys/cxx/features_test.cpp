#include "cxx/types.h"
#include <memory>
#include <pcl/features/normal_3d.h>

// Test minimal implementation
std::unique_ptr<pcl::NormalEstimation_PointXYZ_Normal>
new_normal_estimation_xyz() {
  try {
    return std::make_unique<pcl::NormalEstimation_PointXYZ_Normal>();
  } catch (const std::exception &e) {
    return nullptr;
  }
}