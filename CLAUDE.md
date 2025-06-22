# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust library project that provides safe bindings for the Point Cloud Library (PCL) version 1.15. The project uses a two-crate architecture with `pcl-sys` for low-level FFI bindings and `pcl-rust` for safe Rust APIs.

## Common Commands

### Building and Testing (using Makefile - Recommended)
- `make build` - Build with all features except visualization (recommended for development)
- `make test` - Run tests with all features except visualization (stable)
- `make build-all-features` - Build with all features including visualization (may have VTK linking issues)
- `make test-all-features` - Run tests with all features including visualization (may fail due to VTK)
- `make lint` - Run clippy for code quality checks
- `make clean` - Clean build artifacts
- `make help` - Show all available Makefile targets

### Direct Cargo Commands (if needed)
- `cargo build -p pcl-sys --all-targets --features "search,octree,io,registration,sample_consensus,segmentation,keypoints,surface,features,filters"` - Build only the FFI layer
- `cargo build -p pcl --all-targets --features "search,octree,io,registration,sample_consensus,segmentation,keypoints,surface,features,filters"` - Build only the safe Rust layer
- `cargo nextest run -p pcl --all-targets --features "search,octree,io,registration,sample_consensus,segmentation,keypoints,surface,features,filters" --no-fail-fast` - Run tests for the safe layer only
- `cargo check --all-targets --features "search,octree,io,registration,sample_consensus,segmentation,keypoints,surface,features,filters"` - Check code without building

### Advanced Testing with Nextest
- `cargo nextest run --fail-fast` - Stop on first test failure
- `cargo nextest run -j 8 --no-fail-fast` - Run tests with 8 parallel threads
- `cargo nextest run <test-name>` - Run specific test by name
- `cargo nextest run --lib --no-run` - Check test compilation without running

### Development
- `cargo doc --open` - Generate and open documentation
- `cargo run --example basic_usage` - Run the basic usage example
- `cargo +nightly fmt` - Format the Rust code in this project

### PCL Dependencies
The build system requires PCL 1.15 to be installed:
- On Ubuntu/Debian: `sudo apt-get install libpcl-dev`
- On macOS: `brew install pcl`
- The build script uses pkg-config to find PCL libraries

## Architecture

### Crate Structure
```
pcl-rust/
├── pcl-sys/           # Low-level FFI bindings using cxx
│   ├── src/
│   │   ├── lib.rs     # Main FFI module with cxx bridges
│   │   ├── common/    # Core data structures (PointCloud, point types)
│   │   ├── search/    # Spatial search algorithms (KdTree, etc.)
│   │   └── octree/    # Octree data structures
│   ├── build.rs       # Build script for cxx integration and PCL linking
│   └── cxx/           # C++ wrapper code for cxx bridges
└── pcl-rust/          # High-level safe Rust API
    ├── src/
    │   ├── lib.rs     # Main library with re-exports
    │   ├── error.rs   # Error handling and Result types
    │   ├── common/    # Safe wrappers for core types
    │   ├── search/    # Safe search interfaces
    │   └── octree/    # Safe octree interfaces
    └── examples/      # Usage examples
```

### Implementation Status (Phase 1)
**Completed:**
- ✅ Basic crate structure and build system
- ✅ `common` module: PointXYZ, PointXYZRGB, PointCloud containers
- ✅ `search` module: KdTree search interface
- ✅ `octree` module: OctreeSearch and OctreeVoxelCentroid
- ✅ Error handling with `thiserror`
- ✅ Basic examples

**Next Priority (Phase 2):**
- [ ] `io` module for PCD/PLY file loading/saving
- [ ] `sample_consensus` module for RANSAC algorithms
- [ ] `filters` module for point cloud processing

### Key Design Principles

1. **Safety**: All unsafe FFI operations are contained in `pcl-sys`
2. **Zero-cost abstractions**: Rust wrappers should not add performance overhead
3. **Idiomatic Rust**: Use Result types, builder patterns, and Rust naming conventions
4. **Memory management**: Use cxx UniquePtr/SharedPtr for automatic cleanup
5. **Error handling**: Comprehensive error types with context

### FFI Integration

The project uses the `cxx` crate for C++ interop:
- `#[cxx::bridge]` modules define the FFI interface
- C++ wrapper functions in `pcl-sys/cxx/` handle PCL integration
- Build script handles PCL library linking and include paths

### Development Notes

- All modules follow the pattern: FFI layer in `pcl-sys`, safe wrapper in `pcl-rust`
- Point types implement From traits for easy conversion from tuples/arrays
- PointCloud containers provide Debug implementations and size/empty methods
- Search algorithms validate parameters and return proper Result types

### Additional Guidance
- If there are FFI items missing in the Rust library, leave todo!() and TODO comments instead of silent errors or dummy values.
- If a feature is not finished yet in Rust, leave todo!() and TODO comments instead of silent errors or dummy values

### Project Memories
- The FFI crate should only provide C++ interface and essential Rust type conversion items.
- Use clang-format to format C/C++ code. It might break the source code. Use "// clang-format off ... // clang-format on" mark to protect lines that would be broken by clang-format.
- Format code whenever you make changes. Use `cargo +nightly fmt` for Rust and clang-format for C/C++.
- Always build and test the project whenever you finish a feature. Use `make build` and `make test` for stable development.
- Always run code quality checks whenever you finish a feature. Use `make lint` to run clippy.
- Avoid adding #[allow(dead_code)] on unused items because it would leave unused features without noticable compiler warnings. If the unused item will be used in the future, leave a TODO comment to elaborate.
- At this moment, we don't care about backward compatibility in Rust API. We would optimize the API before the release.
- Unit tests in Rust API have counterparts in upstream C++ code. Leave comments in Rust test code to record the source of the C++ test.