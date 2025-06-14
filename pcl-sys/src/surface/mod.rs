//! FFI bindings for PCL surface reconstruction operations
//!
//! This module provides low-level FFI bindings for PCL's surface reconstruction functionality,
//! including Marching Cubes, Poisson reconstruction, Greedy Projection Triangulation, and mesh utilities.

// Re-export surface functions from the main FFI module
pub use crate::ffi::{
    // Surface reconstruction types
    MarchingCubesHoppe_PointXYZ,
    MarchingCubesRBF_PointXYZ,
    OrganizedFastMesh_PointXYZ,
    PolygonMesh,

    get_iso_level_hoppe_xyz,
    get_iso_level_rbf_xyz,
    get_polygon_count,
    get_triangle_pixel_size_xyz,
    get_triangulation_type_xyz,
    get_vertex_count,
    is_valid_mesh,
    // Marching Cubes Hoppe reconstruction functions
    new_marching_cubes_hoppe_xyz,
    // Marching Cubes RBF reconstruction functions
    new_marching_cubes_rbf_xyz,
    // Organized Fast Mesh functions
    new_organized_fast_mesh_xyz,
    // Polygon mesh utility functions
    new_polygon_mesh,
    perform_reconstruction_hoppe_xyz,

    perform_reconstruction_ofm_xyz,

    perform_reconstruction_rbf_xyz,

    save_polygon_mesh_obj,
    save_polygon_mesh_ply,
    save_polygon_mesh_vtk,
    set_grid_resolution_hoppe_xyz,
    set_grid_resolution_rbf_xyz,
    set_input_cloud_hoppe_xyz,
    set_input_cloud_ofm_xyz,
    set_input_cloud_rbf_xyz,
    set_iso_level_hoppe_xyz,
    set_iso_level_rbf_xyz,
    set_off_surface_displacement_rbf_xyz,
    set_percentage_extend_grid_hoppe_xyz,
    set_percentage_extend_grid_rbf_xyz,
    set_triangle_pixel_size_xyz,
    set_triangulation_type_xyz,
};

// Re-export point cloud types for convenience
pub use crate::ffi::{
    PointCloud_PointXYZ as PointCloudXYZ, PointCloud_PointXYZRGBA as PointCloudXYZRGBA,
};
