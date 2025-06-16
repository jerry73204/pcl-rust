#pragma once

#include "rust/cxx.h"
#include "types.h"

// Marching Cubes Hoppe reconstruction functions
std::unique_ptr<pcl::MarchingCubesHoppe_PointXYZ>
new_marching_cubes_hoppe_xyz();
void set_iso_level_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                             float iso_level);
float get_iso_level_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc);
void set_grid_resolution_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                   int32_t res_x, int32_t res_y, int32_t res_z);
void set_percentage_extend_grid_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                          float percentage);
void set_input_cloud_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                               const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_hoppe_xyz(pcl::MarchingCubesHoppe_PointXYZ &mc,
                                         pcl::PolygonMesh &mesh);

// Marching Cubes RBF reconstruction functions
std::unique_ptr<pcl::MarchingCubesRBF_PointXYZ> new_marching_cubes_rbf_xyz();
void set_iso_level_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc, float iso_level);
float get_iso_level_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc);
void set_grid_resolution_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                 int32_t res_x, int32_t res_y, int32_t res_z);
void set_percentage_extend_grid_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                        float percentage);
void set_off_surface_displacement_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                          float displacement);
void set_input_cloud_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                             const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_rbf_xyz(pcl::MarchingCubesRBF_PointXYZ &mc,
                                       pcl::PolygonMesh &mesh);

// Organized Fast Mesh functions
std::unique_ptr<pcl::OrganizedFastMesh_PointXYZ> new_organized_fast_mesh_xyz();
void set_triangle_pixel_size_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                 int32_t triangle_size);
int32_t get_triangle_pixel_size_xyz(const pcl::OrganizedFastMesh_PointXYZ &ofm);
void set_triangulation_type_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                int32_t triangle_type);
int32_t get_triangulation_type_xyz(const pcl::OrganizedFastMesh_PointXYZ &ofm);
void set_input_cloud_ofm_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                             const pcl::PointCloud_PointXYZ &cloud);
int32_t perform_reconstruction_ofm_xyz(pcl::OrganizedFastMesh_PointXYZ &ofm,
                                       pcl::PolygonMesh &mesh);

// Polygon mesh utility functions
std::unique_ptr<pcl::PolygonMesh> new_polygon_mesh();
size_t get_polygon_count(const pcl::PolygonMesh &mesh);
size_t get_vertex_count(const pcl::PolygonMesh &mesh);
bool is_valid_mesh(const pcl::PolygonMesh &mesh);
int32_t save_polygon_mesh_ply(const pcl::PolygonMesh &mesh,
                              const std::string &filename);
int32_t save_polygon_mesh_obj(const pcl::PolygonMesh &mesh,
                              const std::string &filename);
int32_t save_polygon_mesh_vtk(const pcl::PolygonMesh &mesh,
                              const std::string &filename);

// Poisson Surface Reconstruction
std::unique_ptr<pcl::Poisson_PointNormal> new_poisson();
void set_depth_poisson(pcl::Poisson_PointNormal &poisson, int32_t depth);
int32_t get_depth_poisson(pcl::Poisson_PointNormal &poisson);
void set_min_depth_poisson(pcl::Poisson_PointNormal &poisson,
                           int32_t min_depth);
int32_t get_min_depth_poisson(pcl::Poisson_PointNormal &poisson);
void set_point_weight_poisson(pcl::Poisson_PointNormal &poisson, float weight);
float get_point_weight_poisson(pcl::Poisson_PointNormal &poisson);
void set_scale_poisson(pcl::Poisson_PointNormal &poisson, float scale);
float get_scale_poisson(pcl::Poisson_PointNormal &poisson);
void set_solver_divide_poisson(pcl::Poisson_PointNormal &poisson,
                               int32_t solver_divide);
int32_t get_solver_divide_poisson(pcl::Poisson_PointNormal &poisson);
void set_iso_divide_poisson(pcl::Poisson_PointNormal &poisson,
                            int32_t iso_divide);
int32_t get_iso_divide_poisson(pcl::Poisson_PointNormal &poisson);
void set_samples_per_node_poisson(pcl::Poisson_PointNormal &poisson,
                                  float samples);
float get_samples_per_node_poisson(pcl::Poisson_PointNormal &poisson);
void set_confidence_poisson(pcl::Poisson_PointNormal &poisson, bool confidence);
bool get_confidence_poisson(pcl::Poisson_PointNormal &poisson);
void set_output_polygons_poisson(pcl::Poisson_PointNormal &poisson,
                                 bool output_polygons);
bool get_output_polygons_poisson(pcl::Poisson_PointNormal &poisson);
void set_degree_poisson(pcl::Poisson_PointNormal &poisson, int32_t degree);
int32_t get_degree_poisson(pcl::Poisson_PointNormal &poisson);
void set_manifold_poisson(pcl::Poisson_PointNormal &poisson, bool manifold);
bool get_manifold_poisson(pcl::Poisson_PointNormal &poisson);
void set_input_cloud_poisson(pcl::Poisson_PointNormal &poisson,
                             const pcl::PointCloud_PointNormal &cloud);
int32_t reconstruct_mesh_poisson(pcl::Poisson_PointNormal &poisson,
                                 pcl::PolygonMesh &mesh);

// Greedy Projection Triangulation
std::unique_ptr<pcl::GreedyProjectionTriangulation_PointNormal>
new_greedy_projection_triangulation();
void set_mu_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3,
                   double mu);
double get_mu_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_search_radius_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, double radius);
double
get_search_radius_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_minimum_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, double angle);
double
get_minimum_angle_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_maximum_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, double angle);
double
get_maximum_angle_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_maximum_nearest_neighbors_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, int32_t neighbors);
int32_t get_maximum_nearest_neighbors_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_maximum_surface_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, double angle);
double get_maximum_surface_angle_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_normal_consistency_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, bool consistency);
bool get_normal_consistency_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_consistent_vertex_ordering_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3, bool ordering);
bool get_consistent_vertex_ordering_greedy(
    pcl::GreedyProjectionTriangulation_PointNormal &gp3);
void set_input_cloud_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3,
                            const pcl::PointCloud_PointNormal &cloud);
int32_t
reconstruct_mesh_greedy(pcl::GreedyProjectionTriangulation_PointNormal &gp3,
                        pcl::PolygonMesh &mesh);

// Moving Least Squares (Surface Smoothing)
std::unique_ptr<pcl::MovingLeastSquares_PointXYZ_PointNormal>
new_moving_least_squares();
void set_search_radius_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                           double radius);
double get_search_radius_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_polynomial_order_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                              int32_t order);
int32_t
get_polynomial_order_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_sqr_gauss_param_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             double param);
double
get_sqr_gauss_param_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_compute_normals_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             bool compute);
void set_upsample_method_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                             int32_t method);
void set_upsampling_radius_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, double radius);
double
get_upsampling_radius_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_upsampling_step_size_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, double step_size);
double
get_upsampling_step_size_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_desired_num_points_in_radius_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, int32_t num_points);
void set_dilation_voxel_size_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, float voxel_size);
float get_dilation_voxel_size_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_dilation_iterations_mls(
    pcl::MovingLeastSquares_PointXYZ_PointNormal &mls, int32_t iterations);
int32_t
get_dilation_iterations_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
void set_input_cloud_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls,
                         const pcl::PointCloud_PointXYZ &cloud);
std::unique_ptr<pcl::PointCloud_PointNormal>
process_mls(pcl::MovingLeastSquares_PointXYZ_PointNormal &mls);
