fn main() {
    println!("cargo:rerun-if-changed=src/");
    println!("cargo:rerun-if-changed=cxx/");

    // Link PCL libraries
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

    // Link additional PCL libraries needed for search, octree, I/O, sample consensus, filters, registration, segmentation, features, and keypoints functionality
    println!("cargo:rustc-link-lib=pcl_search");
    println!("cargo:rustc-link-lib=pcl_kdtree");
    println!("cargo:rustc-link-lib=pcl_octree");
    println!("cargo:rustc-link-lib=pcl_io");
    println!("cargo:rustc-link-lib=pcl_sample_consensus");
    println!("cargo:rustc-link-lib=pcl_filters");
    println!("cargo:rustc-link-lib=pcl_registration");
    println!("cargo:rustc-link-lib=pcl_segmentation");
    println!("cargo:rustc-link-lib=pcl_features");
    println!("cargo:rustc-link-lib=pcl_keypoints");

    // Build cxx bridge
    let mut build = cxx_build::bridge("src/lib.rs");

    // Add PCL include paths and source files
    build
        .file("cxx/common.cpp")
        .file("cxx/features.cpp")
        .file("cxx/filters.cpp")
        .file("cxx/io.cpp")
        .file("cxx/keypoints.cpp")
        .file("cxx/registration.cpp")
        .file("cxx/sample_consensus.cpp")
        .file("cxx/segmentation.cpp")
        .include(".") // For our cxx/types.h
        .include("/usr/include/pcl-1.12")
        .include("/usr/include/eigen3")
        .std("c++14");

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
