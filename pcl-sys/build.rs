use std::fs;

// Expected PCL version for pcl-rust compatibility
const EXPECTED_PCL_VERSION: &str = "1.12.1";
const EXPECTED_PCL_MAJOR: u32 = 1;
const EXPECTED_PCL_MINOR: u32 = 12;
const EXPECTED_PCL_PATCH: u32 = 1;

// Function to check PCL version from headers
fn check_pcl_version(include_paths: &[std::path::PathBuf]) {
    // Look for pcl_config.h or pcl/pcl_config.h
    for include_path in include_paths {
        let config_paths = [
            include_path.join("pcl_config.h"),
            include_path.join("pcl").join("pcl_config.h"),
            include_path.join("pcl").join("pcl_version.h"),
        ];

        for config_path in &config_paths {
            if config_path.exists() {
                if let Ok(content) = fs::read_to_string(config_path) {
                    if let Some((major, minor, patch)) = parse_pcl_version(&content) {
                        let actual_version = format!("{}.{}.{}", major, minor, patch);

                        // Check if version matches expected
                        if major != EXPECTED_PCL_MAJOR
                            || minor != EXPECTED_PCL_MINOR
                            || patch != EXPECTED_PCL_PATCH
                        {
                            println!("cargo:warning=PCL version mismatch detected!");
                            println!(
                                "cargo:warning=Expected PCL version: {}",
                                EXPECTED_PCL_VERSION
                            );
                            println!("cargo:warning=Found PCL version: {}", actual_version);
                            println!("cargo:warning=This may cause compatibility issues.");
                            println!(
                                "cargo:warning=Consider installing PCL {} or update pcl-rust to match your PCL version.",
                                EXPECTED_PCL_VERSION
                            );
                        } else {
                            println!(
                                "cargo:warning=PCL version check: {} matches expected {}",
                                actual_version, EXPECTED_PCL_VERSION
                            );
                        }
                        return;
                    }
                }
            }
        }
    }

    println!("cargo:warning=Could not determine PCL version from headers");
    println!(
        "cargo:warning=Expected PCL version: {}",
        EXPECTED_PCL_VERSION
    );
    println!(
        "cargo:warning=Please ensure you have PCL {} installed",
        EXPECTED_PCL_VERSION
    );
}

// Function to parse PCL version from header content
fn parse_pcl_version(content: &str) -> Option<(u32, u32, u32)> {
    let mut major = None;
    let mut minor = None;
    let mut patch = None;

    for line in content.lines() {
        let line = line.trim();

        // Look for version definitions
        if line.starts_with("#define PCL_MAJOR_VERSION") {
            if let Some(value) = extract_define_value(line) {
                major = value.parse().ok();
            }
        } else if line.starts_with("#define PCL_MINOR_VERSION") {
            if let Some(value) = extract_define_value(line) {
                minor = value.parse().ok();
            }
        } else if line.starts_with("#define PCL_REVISION_VERSION")
            || line.starts_with("#define PCL_PATCH_VERSION")
        {
            if let Some(value) = extract_define_value(line) {
                patch = value.parse().ok();
            }
        }
    }

    match (major, minor, patch) {
        (Some(maj), Some(min), Some(pat)) => Some((maj, min, pat)),
        _ => None,
    }
}

// Function to extract value from #define line
fn extract_define_value(line: &str) -> Option<&str> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() >= 3 {
        Some(parts[2])
    } else {
        None
    }
}

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
        std::env::var(format!("CARGO_FEATURE_{}", feature.to_uppercase())).is_ok()
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
        // Registration correspondence rejection uses RANSAC which needs sample_consensus
        println!("cargo:rustc-link-lib=pcl_sample_consensus");
    }

    if is_feature_enabled("segmentation") {
        println!("cargo:rustc-link-lib=pcl_segmentation");
        // Segmentation depends on search functionality
        println!("cargo:rustc-link-lib=pcl_search");
        println!("cargo:rustc-link-lib=pcl_kdtree");
        // Segmentation may also depend on features for normals
        println!("cargo:rustc-link-lib=pcl_features");
    }

    if is_feature_enabled("features") {
        println!("cargo:rustc-link-lib=pcl_features");
    }

    if is_feature_enabled("keypoints") {
        println!("cargo:rustc-link-lib=pcl_keypoints");
        // Keypoints depend on search functionality
        println!("cargo:rustc-link-lib=pcl_search");
        println!("cargo:rustc-link-lib=pcl_kdtree");
        // Keypoints also depend on features and filters
        println!("cargo:rustc-link-lib=pcl_features");
        println!("cargo:rustc-link-lib=pcl_filters");
    }

    if is_feature_enabled("surface") {
        println!("cargo:rustc-link-lib=pcl_surface");
        // Surface reconstruction depends on search functionality
        println!("cargo:rustc-link-lib=pcl_search");
        println!("cargo:rustc-link-lib=pcl_kdtree");
        // Surface reconstruction may also depend on features for normals
        println!("cargo:rustc-link-lib=pcl_features");
        // Surface reconstruction may depend on filters for data preprocessing
        println!("cargo:rustc-link-lib=pcl_filters");
    }

    // Visualization disabled due to VTK linkage issues
    // if is_feature_enabled("visualization") {
    //     // Add linker flags to ensure proper symbol resolution
    //     println!("cargo:rustc-link-arg=-Wl,--no-as-needed");
    //
    //     // VTK dependencies must be linked BEFORE PCL visualization for proper symbol resolution
    //     // Core VTK libraries for basic functionality (order matters!)
    //     println!("cargo:rustc-link-lib=vtkCommonCore-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonDataModel-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonMath-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonExecutionModel-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonSystem-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonTransforms-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonMisc-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonColor-9.1");
    //     println!("cargo:rustc-link-lib=vtkCommonComputationalGeometry-9.1");
    //     // Rendering libraries
    //     println!("cargo:rustc-link-lib=vtkRenderingCore-9.1");
    //     println!("cargo:rustc-link-lib=vtkRenderingOpenGL2-9.1");
    //     println!("cargo:rustc-link-lib=vtkRenderingAnnotation-9.1");
    //     println!("cargo:rustc-link-lib=vtkRenderingLOD-9.1");
    //     // Interaction and filtering libraries
    //     println!("cargo:rustc-link-lib=vtkInteractionStyle-9.1");
    //     println!("cargo:rustc-link-lib=vtkFiltersSources-9.1");
    //     println!("cargo:rustc-link-lib=vtkFiltersCore-9.1");
    //     println!("cargo:rustc-link-lib=vtkFiltersGeneral-9.1");
    //     // I/O libraries that might be needed
    //     println!("cargo:rustc-link-lib=vtkIOCore-9.1");
    //     println!("cargo:rustc-link-lib=vtkIOLegacy-9.1");
    //     // Debug and object factory libraries (for the missing symbols)
    //     println!("cargo:rustc-link-lib=vtkWrappingTools-9.1");
    //
    //     // Note: Removed --whole-archive as it causes duplicate static initializer issues
    //
    //     // PCL visualization library AFTER VTK dependencies
    //     println!("cargo:rustc-link-lib=pcl_visualization");
    //
    //     // Additional linker flags for VTK compatibility
    //     println!("cargo:rustc-link-arg=-Wl,--as-needed");
    // }

    // Build cxx bridge
    let mut build = cxx_build::bridge("src/lib.rs");

    // Add PCL include paths (order matters for compilation speed)
    build
        .include(".") // For our cxx/types.h (highest priority)
        .std("c++17"); // Use C++17 for better optimization

    // Visualization disabled due to VTK linkage issues
    // // CRITICAL: Define VTK disables for ALL files when visualization is enabled
    // // This must happen before ANY includes to prevent VTK static initializers
    // if is_feature_enabled("visualization") {
    //     build.define("VTK_DEBUG_LEAKS", "0");
    //     build.define("VTK_DISABLE_DEBUG_LEAKS", None);
    //     build.define("VTK_NO_OBJECT_FACTORY_DEBUGGING", None);
    //     build.define("VTK_LEGACY_REMOVE", None);
    //     build.define("VTK_ALL_NEW_OBJECT_FACTORY", None);
    //     build.define("VTK_NO_EXPLICIT_TEMPLATE_INSTANTIATION", None);
    // }

    // Add dynamically discovered PCL include paths
    for include_path in &pcl_include_paths {
        build.include(include_path);
    }

    // Visualization disabled due to VTK linkage issues
    // // Force include our precompiled header to ensure VTK config is always first
    // if is_feature_enabled("visualization") {
    //     build.flag_if_supported("-include");
    //     build.flag_if_supported("cxx/precompiled.h");
    // }

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

    // Visualization disabled due to VTK linkage issues
    // // Add VTK include path only when visualization feature is enabled
    // if is_feature_enabled("visualization") {
    //     // Define preprocessor flag to enable VTK includes
    //     build.define("PCL_RUST_ENABLE_VISUALIZATION", None);
    //     // Try to find VTK via pkg-config first
    //     let vtk_variants = ["vtk-9.3", "vtk-9.2", "vtk-9.1", "vtk-9.0", "vtk"];
    //     let mut vtk_found = false;
    //
    //     for variant in &vtk_variants {
    //         if let Ok(vtk_lib) = pkg_config::probe_library(variant) {
    //             for include_path in &vtk_lib.include_paths {
    //                 build.include(include_path);
    //             }
    //             vtk_found = true;
    //             break;
    //         }
    //     }
    //
    //     if !vtk_found {
    //         // VTK often doesn't provide pkg-config files, so try standard locations
    //         // VTK is typically found via CMake config files, but we can use common include paths
    //         let vtk_include_paths = [
    //             "/usr/include/vtk-9.3",
    //             "/usr/include/vtk-9.2",
    //             "/usr/include/vtk-9.1",
    //             "/usr/include/vtk-9.0",
    //             "/usr/local/include/vtk-9.3",
    //             "/usr/local/include/vtk-9.2",
    //             "/usr/local/include/vtk-9.1",
    //             "/usr/local/include/vtk-9.0",
    //         ];
    //
    //         for vtk_path in &vtk_include_paths {
    //             if std::path::Path::new(vtk_path).exists() {
    //                 build.include(vtk_path);
    //                 vtk_found = true;
    //                 println!("cargo:warning=Found VTK headers at: {}", vtk_path);
    //                 break;
    //             }
    //         }
    //
    //         if !vtk_found {
    //             panic!(
    //                 "VTK (Visualization Toolkit) not found.\n\
    //                  VTK is required when the 'visualization' feature is enabled.\n\
    //                  Please install VTK development libraries:\n\
    //                  - Ubuntu/Debian: sudo apt-get install libvtk9-dev\n\
    //                  - macOS: brew install vtk\n\
    //                  - Or disable the visualization feature if not needed\n\
    //                  Searched in: {:#?}",
    //                 vtk_include_paths
    //             );
    //         }
    //     }
    // }

    // Check PCL version in headers and warn if mismatch
    check_pcl_version(&pcl_include_paths);

    // Always add common source file since it's always required
    build.file("cxx/common.cpp");

    // Add raw pointer functions for Phase 1 FFI migration
    build.file("cxx/raw_ptr_functions.cpp");

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

    // Visualization disabled due to VTK linkage issues
    // if is_feature_enabled("visualization") {
    //     build.file("cxx/visualization.cpp");
    // }

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
