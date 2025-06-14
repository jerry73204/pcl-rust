# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust library project that provides safe bindings for the Point Cloud Library (PCL) version 1.15. The project uses a two-crate architecture with `pcl-sys` for low-level FFI bindings and `pcl-rust` for safe Rust APIs.

## Common Commands

### Building and Testing
- `cargo build` - Build the project (workspace builds both crates)
- `cargo build -p pcl-sys` - Build only the FFI layer
- `cargo build -p pcl` - Build only the safe Rust layer
- `cargo test` - Run all tests
- `cargo test -p pcl` - Run tests for the safe layer only
- `cargo check` - Check code without building
- `cargo clippy` - Run linter
- `cargo fmt` - Format code
- `cargo test --all-targets --all-features` - Run all tests with all features

### Development
- `cargo doc --open` - Generate and open documentation
- `cargo run --example basic_usage` - Run the basic usage example
- `cargo +nightly format` - Format the Rust code in this project
- `cargo clippy` - Perform the quality check on Rust code

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