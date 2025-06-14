//! Safe Rust API for PCL sample consensus algorithms
//!
//! This module provides safe, idiomatic Rust bindings for PCL's sample consensus
//! algorithms including RANSAC for geometric model fitting.

pub mod ransac;

// Re-export for convenience
pub use ransac::*;
