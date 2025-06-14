//! FFI bindings for PCL sample consensus algorithms
//!
//! This module provides low-level FFI bindings for PCL's sample consensus
//! algorithms including RANSAC and various geometric models.

pub mod models;
pub mod ransac;

// Re-export for convenience
// Note: Only re-export the necessary items to avoid ambiguous glob exports
pub use ransac::ffi as ransac_ffi;
