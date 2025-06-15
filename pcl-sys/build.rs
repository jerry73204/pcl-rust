fn main() {
    println!("cargo:rerun-if-changed=src/");
    println!("cargo:rerun-if-changed=cxx/");

    // Probe PCL common library and collect include paths
    let mut pcl_include_paths = Vec::new();

    if let Ok(library) = pkg_config::probe_library("pcl_common-1.12") {
        for include_path in &library.include_paths {
            println!("cargo:include={}", include_path.display());
            pcl_include_paths.push(include_path.clone());
        }
    } else {
        // Try alternative PCL version names
        let pcl_variants = ["pcl_common-1.13", "pcl_common-1.11", "pcl_common"];
        let mut found = false;

        for variant in &pcl_variants {
            if let Ok(library) = pkg_config::probe_library(variant) {
                for include_path in &library.include_paths {
                    println!("cargo:include={}", include_path.display());
                    pcl_include_paths.push(include_path.clone());
                }
                found = true;
                break;
            }
        }

        if !found {
            panic!(
                "PCL (Point Cloud Library) not found via pkg-config.\n\
                 Please install PCL development libraries:\n\
                 - Ubuntu/Debian: sudo apt-get install libpcl-dev\n\
                 - macOS: brew install pcl\n\
                 - Or ensure pkg-config can find PCL libraries"
            );
        }
    }

    // Helper function to check if a feature is enabled
    fn is_feature_enabled(feature: &str) -> bool {
        std::env::var(&format!("CARGO_FEATURE_{}", feature.to_uppercase())).is_ok()
    }

    // Conditionally link PCL libraries based on enabled features
    if is_feature_enabled("search") {
        println!("cargo:rustc-link-lib=pcl_search");
        println!("cargo:rustc-link-lib=pcl_kdtree");
    }

    if is_feature_enabled("octree") {
        println!("cargo:rustc-link-lib=pcl_octree");
    }

    if is_feature_enabled("io") {
        println!("cargo:rustc-link-lib=pcl_io");
    }

    if is_feature_enabled("sample_consensus") {
        println!("cargo:rustc-link-lib=pcl_sample_consensus");
    }

    if is_feature_enabled("filters") {
        println!("cargo:rustc-link-lib=pcl_filters");
    }

    if is_feature_enabled("registration") {
        println!("cargo:rustc-link-lib=pcl_registration");
    }

    if is_feature_enabled("segmentation") {
        println!("cargo:rustc-link-lib=pcl_segmentation");
    }

    if is_feature_enabled("features") {
        println!("cargo:rustc-link-lib=pcl_features");
    }

    if is_feature_enabled("keypoints") {
        println!("cargo:rustc-link-lib=pcl_keypoints");
    }

    if is_feature_enabled("surface") {
        println!("cargo:rustc-link-lib=pcl_surface");
    }

    if is_feature_enabled("visualization") {
        println!("cargo:rustc-link-lib=pcl_visualization");
        // VTK dependencies required for PCL visualization (with version suffix)
        println!("cargo:rustc-link-lib=vtkCommonCore-9.1");
        println!("cargo:rustc-link-lib=vtkCommonDataModel-9.1");
        println!("cargo:rustc-link-lib=vtkCommonMath-9.1");
        println!("cargo:rustc-link-lib=vtkRenderingCore-9.1");
        println!("cargo:rustc-link-lib=vtkRenderingOpenGL2-9.1");
        println!("cargo:rustc-link-lib=vtkInteractionStyle-9.1");
        println!("cargo:rustc-link-lib=vtkFiltersSources-9.1");
        println!("cargo:rustc-link-lib=vtkFiltersCore-9.1");
        println!("cargo:rustc-link-lib=vtkCommonExecutionModel-9.1");
        // Additional VTK libraries that might be needed
        println!("cargo:rustc-link-lib=vtkRenderingAnnotation-9.1");
        println!("cargo:rustc-link-lib=vtkRenderingLOD-9.1");
    }

    // Build cxx bridge
    let mut build = cxx_build::bridge("src/lib.rs");

    // Add PCL include paths (order matters for compilation speed)
    build
        .include(".") // For our cxx/types.h (highest priority)
        .std("c++17"); // Use C++17 for better optimization

    // Add dynamically discovered PCL include paths
    for include_path in &pcl_include_paths {
        build.include(include_path);
    }

    // Probe for Eigen3 include paths
    if let Ok(eigen_lib) = pkg_config::probe_library("eigen3") {
        for include_path in &eigen_lib.include_paths {
            build.include(include_path);
        }
    } else {
        panic!(
            "Eigen3 library not found via pkg-config.\n\
             Please install Eigen3 development libraries:\n\
             - Ubuntu/Debian: sudo apt-get install libeigen3-dev\n\
             - macOS: brew install eigen\n\
             - Or ensure pkg-config can find Eigen3 libraries"
        );
    }

    // Add VTK include path only when visualization feature is enabled
    if is_feature_enabled("visualization") {
        // Try to find VTK via pkg-config first
        let vtk_variants = ["vtk-9.3", "vtk-9.2", "vtk-9.1", "vtk-9.0", "vtk"];
        let mut vtk_found = false;

        for variant in &vtk_variants {
            if let Ok(vtk_lib) = pkg_config::probe_library(variant) {
                for include_path in &vtk_lib.include_paths {
                    build.include(include_path);
                }
                vtk_found = true;
                break;
            }
        }

        if !vtk_found {
            // VTK often doesn't provide pkg-config files, so try standard locations
            // VTK is typically found via CMake config files, but we can use common include paths
            let vtk_include_paths = [
                "/usr/include/vtk-9.3",
                "/usr/include/vtk-9.2",
                "/usr/include/vtk-9.1",
                "/usr/include/vtk-9.0",
                "/usr/local/include/vtk-9.3",
                "/usr/local/include/vtk-9.2",
                "/usr/local/include/vtk-9.1",
                "/usr/local/include/vtk-9.0",
            ];

            for vtk_path in &vtk_include_paths {
                if std::path::Path::new(vtk_path).exists() {
                    build.include(vtk_path);
                    vtk_found = true;
                    println!("cargo:warning=Found VTK headers at: {}", vtk_path);
                    break;
                }
            }

            if !vtk_found {
                panic!(
                    "VTK (Visualization Toolkit) not found.\n\
                     VTK is required when the 'visualization' feature is enabled.\n\
                     Please install VTK development libraries:\n\
                     - Ubuntu/Debian: sudo apt-get install libvtk9-dev\n\
                     - macOS: brew install vtk\n\
                     - Or disable the visualization feature if not needed\n\
                     Searched in: {:#?}",
                    vtk_include_paths
                );
            }
        }
    }

    // Always add common source file since it's always required
    build.file("cxx/common.cpp");

    // Conditionally add source files based on enabled features

    if is_feature_enabled("search") {
        build.file("cxx/search.cpp");
    }

    if is_feature_enabled("octree") {
        build.file("cxx/octree.cpp");
    }

    if is_feature_enabled("features") {
        build.file("cxx/features.cpp");
    }

    if is_feature_enabled("filters") {
        build.file("cxx/filters.cpp");
    }

    if is_feature_enabled("io") {
        build.file("cxx/io.cpp");
    }

    if is_feature_enabled("keypoints") {
        build.file("cxx/keypoints.cpp");
    }

    if is_feature_enabled("registration") {
        build.file("cxx/registration.cpp");
    }

    if is_feature_enabled("sample_consensus") {
        build.file("cxx/sample_consensus.cpp");
    }

    if is_feature_enabled("segmentation") {
        build.file("cxx/segmentation.cpp");
    }

    if is_feature_enabled("surface") {
        build.file("cxx/surface.cpp");
    }

    if is_feature_enabled("visualization") {
        build.file("cxx/visualization.cpp");
    }

    // Enable ccache if available
    if std::process::Command::new("ccache")
        .arg("--version")
        .output()
        .is_ok()
    {
        unsafe {
            std::env::set_var("CC", "ccache cc");
            std::env::set_var("CXX", "ccache c++");
        }
        println!("cargo:warning=Using ccache for compilation caching");
    }

    // Enable parallel compilation
    let num_cpus = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);

    // Configure for parallel build
    println!(
        "cargo:warning=Using {} parallel jobs for compilation",
        num_cpus
    );

    // Optimization flags for faster compilation and smaller binary
    build
        .flag_if_supported("-O2") // Optimize for speed
        .flag_if_supported("-march=native") // Use native CPU features
        .flag_if_supported("-pipe") // Use pipes instead of temp files
        .flag_if_supported("-fno-semantic-interposition") // Better optimization
        .flag_if_supported("-ffunction-sections") // Separate function sections
        .flag_if_supported("-fdata-sections") // Separate data sections
        .flag_if_supported("-ffast-math") // Faster math operations
        .flag_if_supported("-DNDEBUG") // Disable debug assertions
        .warnings(false); // Disable warnings for faster compilation

    // Only add debug info in debug builds to speed up release builds
    if std::env::var("PROFILE").unwrap_or_default() == "debug" {
        build.flag_if_supported("-g1"); // Minimal debug info for debug builds
    }

    // Note: Precompiled headers (pch.h) exist but are not currently used
    // because cxx-build doesn't easily support PCH compilation.
    // The ccache and parallel compilation provide the main speed benefits.

    // Compile the bridge
    build.compile("pcl-sys");
}
