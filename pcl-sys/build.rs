fn main() {
    println!("cargo:rerun-if-changed=src/");
    println!("cargo:rerun-if-changed=cxx/");

    // Always link PCL common as it's needed for basic point types
    if let Ok(library) = pkg_config::probe_library("pcl_common-1.12") {
        for include_path in &library.include_paths {
            println!("cargo:include={}", include_path.display());
        }
    } else {
        // Fallback to system paths if pkg-config fails
        println!("cargo:warning=PCL not found via pkg-config, using system paths");
        println!("cargo:rustc-link-lib=pcl_common");
        println!("cargo:rustc-link-search=/usr/local/lib");
        println!("cargo:include=/usr/local/include");
        println!("cargo:include=/usr/include");
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

    // Build cxx bridge
    let mut build = cxx_build::bridge("src/lib.rs");

    // Add PCL include paths
    build
        .include(".") // For our cxx/types.h
        .include("/usr/include/pcl-1.12")
        .include("/usr/include/eigen3")
        .std("c++14");

    // Always add common source file since it's always required
    build.file("cxx/common.cpp");

    // Conditionally add source files based on enabled features

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

    // Optimization flags for faster compilation
    build
        .flag_if_supported("-O2") // Optimize for speed
        .flag_if_supported("-march=native") // Use native CPU features
        .flag_if_supported("-pipe") // Use pipes instead of temp files
        .flag_if_supported("-fno-semantic-interposition") // Better optimization
        .warnings(false); // Disable warnings for faster compilation

    // Note: Precompiled headers (pch.h) exist but are not currently used
    // because cxx-build doesn't easily support PCH compilation.
    // The ccache and parallel compilation provide the main speed benefits.

    // Compile the bridge
    build.compile("pcl-sys");
}
