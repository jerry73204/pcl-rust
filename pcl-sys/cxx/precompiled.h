#pragma once

// Precompiled header for pcl-sys
// This ensures VTK config is included before any other headers

#ifdef PCL_RUST_ENABLE_VISUALIZATION
#include "vtk_config.h"
#endif

// Common includes that benefit from precompilation
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdint>