#pragma once

// VTK-specific includes and function declarations for PCL visualization
// This file is only included when visualization features are enabled

// CRITICAL: Include VTK config first to disable debug features
#include "vtk_config.h"

#include "rust/cxx.h"
#include "types.h"
#include <memory>

// Forward declare VTK types to avoid pulling in problematic headers
class vtkVersion;

// Only include PCL visualization headers - they handle VTK integration properly
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// Function declarations for visualization (defined in visualization.cpp)
std::unique_ptr<pcl::visualization::PCLVisualizer> new_pcl_visualizer(rust::Str window_name);
int32_t add_point_cloud_xyz(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud_PointXYZ& cloud, rust::Str id);
int32_t add_point_cloud_xyzrgb(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud_PointXYZRGB& cloud, rust::Str id);
int32_t update_point_cloud_xyz(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud_PointXYZ& cloud, rust::Str id);
int32_t update_point_cloud_xyzrgb(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud_PointXYZRGB& cloud, rust::Str id);
int32_t remove_point_cloud(pcl::visualization::PCLVisualizer& viewer, rust::Str id);
int32_t set_point_cloud_render_properties_xyz(pcl::visualization::PCLVisualizer& viewer, int32_t property, double value, rust::Str id);
int32_t set_background_color(pcl::visualization::PCLVisualizer& viewer, double r, double g, double b);
int32_t add_coordinate_system(pcl::visualization::PCLVisualizer& viewer, double scale, rust::Str id);
int32_t spin_once(pcl::visualization::PCLVisualizer& viewer, int32_t time);
void spin(pcl::visualization::PCLVisualizer& viewer);
bool was_stopped(const pcl::visualization::PCLVisualizer& viewer);
void close(pcl::visualization::PCLVisualizer& viewer);
void reset_stopped_flag(pcl::visualization::PCLVisualizer& viewer);
int32_t set_camera_position(pcl::visualization::PCLVisualizer& viewer, double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z);
int32_t reset_camera(pcl::visualization::PCLVisualizer& viewer);

// CloudViewer functions
std::unique_ptr<pcl::visualization::CloudViewer> new_cloud_viewer(rust::Str window_name);
int32_t show_cloud_xyz(pcl::visualization::CloudViewer& viewer, const pcl::PointCloud_PointXYZ& cloud, rust::Str cloud_name);
int32_t show_cloud_xyzrgb(pcl::visualization::CloudViewer& viewer, const pcl::PointCloud_PointXYZRGB& cloud, rust::Str cloud_name);
bool cloud_viewer_was_stopped(const pcl::visualization::CloudViewer& viewer);
void wait_for_cloud_viewer(pcl::visualization::CloudViewer& viewer, int32_t time_ms);

// Utility functions
int32_t set_point_cloud_color_xyz(pcl::visualization::PCLVisualizer& viewer, double r, double g, double b, rust::Str id);
int32_t add_text(pcl::visualization::PCLVisualizer& viewer, rust::Str text, int32_t xpos, int32_t ypos, double r, double g, double b, rust::Str id);
int32_t add_sphere_xyz(pcl::visualization::PCLVisualizer& viewer, const pcl::PointXYZ& center, double radius, double r, double g, double b, rust::Str id);
int32_t remove_shape(pcl::visualization::PCLVisualizer& viewer, rust::Str id);
int32_t register_keyboard_callback(pcl::visualization::PCLVisualizer& viewer);