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

    // Build cxx bridge
    let mut build = cxx_build::bridge("src/lib.rs");

    // Add PCL include paths
    build
        .file("cxx/common.cpp")
        .include(".") // For our cxx/types.h
        .include("/usr/include/pcl-1.12")
        .include("/usr/include/eigen3")
        .std("c++14");

    // Compile the bridge
    build.compile("pcl-sys");
}
