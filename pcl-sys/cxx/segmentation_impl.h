#pragma once

#include "rust/cxx.h"
#include "types.h"

// Region Growing segmentation - PointXYZ
std::unique_ptr<pcl::RegionGrowing_PointXYZ_Normal> new_region_growing_xyz();
void set_input_cloud_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg,
                                        const pcl::PointCloud_PointXYZ &cloud);
void set_input_normals_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg,
    const pcl::PointCloud_Normal &normals);
void set_min_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t min_size);
int32_t get_min_cluster_size_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_max_cluster_size_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t max_size);
int32_t get_max_cluster_size_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_smoothness_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold);
float get_smoothness_threshold_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_curvature_threshold_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, float threshold);
float get_curvature_threshold_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg);
void set_number_of_neighbours_region_growing_xyz(
    pcl::RegionGrowing_PointXYZ_Normal &rg, int32_t k);
int32_t get_number_of_neighbours_region_growing_xyz(
    const pcl::RegionGrowing_PointXYZ_Normal &rg);
rust::Vec<int32_t>
extract_region_growing_xyz(pcl::RegionGrowing_PointXYZ_Normal &rg);

// Region Growing RGB segmentation - PointXYZRGB
std::unique_ptr<pcl::RegionGrowingRGB_PointXYZRGB>
new_region_growing_rgb_xyzrgb();
void set_input_cloud_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg,
    const pcl::PointCloud_PointXYZRGB &cloud);
void set_distance_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_distance_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_point_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_point_color_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_region_color_threshold_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, float threshold);
float get_region_color_threshold_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg);
void set_min_cluster_size_region_growing_rgb_xyzrgb(
    pcl::RegionGrowingRGB_PointXYZRGB &rg, int32_t min_size);
int32_t get_min_cluster_size_region_growing_rgb_xyzrgb(
    const pcl::RegionGrowingRGB_PointXYZRGB &rg);
rust::Vec<int32_t>
extract_region_growing_rgb_xyzrgb(pcl::RegionGrowingRGB_PointXYZRGB &rg);

// Euclidean Cluster Extraction - PointXYZ
std::unique_ptr<pcl::EuclideanClusterExtraction_PointXYZ>
new_euclidean_cluster_extraction_xyz();
void set_input_cloud_euclidean_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ec,
                                   const pcl::PointCloud_PointXYZ &cloud);
void set_cluster_tolerance_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ec, double tolerance);
void set_min_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ec, int32_t min_size);
int32_t get_min_cluster_size_euclidean_xyz(
    const pcl::EuclideanClusterExtraction_PointXYZ &ec);
void set_max_cluster_size_euclidean_xyz(
    pcl::EuclideanClusterExtraction_PointXYZ &ec, int32_t max_size);
int32_t get_max_cluster_size_euclidean_xyz(
    const pcl::EuclideanClusterExtraction_PointXYZ &ec);
rust::Vec<int32_t>
extract_euclidean_clusters_xyz(pcl::EuclideanClusterExtraction_PointXYZ &ec);

// SAC Segmentation - PointXYZ
std::unique_ptr<pcl::SACSegmentation_PointXYZ> new_sac_segmentation_xyz();
void set_input_cloud_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_model_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                            int32_t model_type);
int32_t get_model_type_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac);
void set_method_type_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                             int32_t method_type);
int32_t get_method_type_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac);
void set_distance_threshold_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                    double threshold);
double get_distance_threshold_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac);
void set_max_iterations_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                int32_t max_iter);
int32_t get_max_iterations_sac_xyz(const pcl::SACSegmentation_PointXYZ &sac);
void set_optimize_coefficients_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                                       bool optimize);
bool get_optimize_coefficients_sac_xyz(
    const pcl::SACSegmentation_PointXYZ &sac);
bool segment_sac_xyz(pcl::SACSegmentation_PointXYZ &sac,
                     rust::Vec<int32_t> &inliers,
                     rust::Vec<float> &coefficients);

// Min-Cut Segmentation - PointXYZ
std::unique_ptr<pcl::MinCutSegmentation_PointXYZ>
new_min_cut_segmentation_xyz();
void set_input_cloud_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                 const pcl::PointCloud_PointXYZ &cloud);
void set_foreground_points_min_cut_xyz(
    pcl::MinCutSegmentation_PointXYZ &mc,
    const pcl::PointCloud_PointXYZ &foreground_points);
void set_sigma_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc, double sigma);
double get_sigma_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc);
void set_radius_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                            double radius);
double get_radius_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc);
void set_number_of_neighbours_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                          int32_t k);
int32_t get_number_of_neighbours_min_cut_xyz(
    const pcl::MinCutSegmentation_PointXYZ &mc);
void set_source_weight_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc,
                                   double weight);
double
get_source_weight_min_cut_xyz(const pcl::MinCutSegmentation_PointXYZ &mc);
rust::Vec<int32_t> extract_min_cut_xyz(pcl::MinCutSegmentation_PointXYZ &mc);

// Extract Polygonal Prism Data - PointXYZ
std::unique_ptr<pcl::ExtractPolygonalPrismData_PointXYZ>
new_extract_polygonal_prism_xyz();
void set_input_cloud_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &cloud);
void set_input_planar_hull_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism,
    const pcl::PointCloud_PointXYZ &hull);
void set_height_limits_polygonal_prism_xyz(
    pcl::ExtractPolygonalPrismData_PointXYZ &prism, double min_height,
    double max_height);
void get_height_limits_polygonal_prism_xyz(
    const pcl::ExtractPolygonalPrismData_PointXYZ &prism, double &min_height,
    double &max_height);
rust::Vec<int32_t>
segment_polygonal_prism_xyz(pcl::ExtractPolygonalPrismData_PointXYZ &prism);

// Progressive Morphological Filter - PointXYZ
std::unique_ptr<pcl::ProgressiveMorphologicalFilter_PointXYZ>
new_progressive_morphological_filter_xyz();
void set_input_cloud_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             const pcl::PointCloud_PointXYZ &cloud);
void set_max_window_size_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf, int32_t size);
int32_t get_max_window_size_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_slope_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                       float slope);
float get_slope_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_max_distance_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                              float distance);
float get_max_distance_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_initial_distance_pmf_xyz(
    pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf, float distance);
float get_initial_distance_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_cell_size_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                           float size);
float get_cell_size_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_base_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                      float base);
float get_base_pmf_xyz(const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
void set_exponential_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf,
                             bool exponential);
bool get_exponential_pmf_xyz(
    const pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);
rust::Vec<int32_t>
extract_pmf_xyz(pcl::ProgressiveMorphologicalFilter_PointXYZ &pmf);

// Conditional Euclidean Clustering - PointXYZ
std::unique_ptr<pcl::ConditionalEuclideanClustering_PointXYZ>
new_conditional_euclidean_clustering_xyz();
void set_input_cloud_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec,
    const pcl::PointCloud_PointXYZ &cloud);
void set_cluster_tolerance_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, float tolerance);
float get_cluster_tolerance_conditional_euclidean_xyz(
    const pcl::ConditionalEuclideanClustering_PointXYZ &cec);
void set_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t min_size);
int32_t get_min_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
void set_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec, int32_t max_size);
int32_t get_max_cluster_size_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
rust::Vec<int32_t> segment_conditional_euclidean_xyz(
    pcl::ConditionalEuclideanClustering_PointXYZ &cec);
