#pragma once

#include "rust/cxx.h"
#include "types.h"
#include <cstddef>

// Feature cloud functions
size_t size_normal(const pcl::PointCloud_Normal &cloud);
bool empty_normal(const pcl::PointCloud_Normal &cloud);
size_t size_fpfh(const pcl::PointCloud_FPFHSignature33 &cloud);
bool empty_fpfh(const pcl::PointCloud_FPFHSignature33 &cloud);
size_t size_pfh(const pcl::PointCloud_PFHSignature125 &cloud);
bool empty_pfh(const pcl::PointCloud_PFHSignature125 &cloud);

// Individual feature access functions
rust::Vec<float> get_normal_at(const pcl::PointCloud_Normal &cloud,
                               size_t index);
rust::Vec<float>
get_fpfh_signature_at(const pcl::PointCloud_FPFHSignature33 &cloud,
                      size_t index);
rust::Vec<float>
get_pfh_signature_at(const pcl::PointCloud_PFHSignature125 &cloud,
                     size_t index);

// Note: Bulk histogram access functions not supported due to cxx Vec<Vec<T>>
// limitation
