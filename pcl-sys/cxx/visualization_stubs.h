// Stub declarations for PCL visualization types when VTK is not available
#pragma once

// Always provide forward declarations for the types when this header is included
// (which happens when visualization feature is disabled)
namespace pcl {
namespace visualization {
// Forward declarations as opaque types
class PCLVisualizer;
class CloudViewer;
} // namespace visualization
} // namespace pcl
