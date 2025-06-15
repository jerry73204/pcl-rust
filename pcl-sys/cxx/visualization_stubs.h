// Stub declarations for PCL visualization types when VTK is not available
#pragma once

#ifndef VTK_MAJOR_VERSION
// When VTK is not available, provide minimal stub declarations
namespace pcl {
namespace visualization {
// Forward declarations as opaque types
class PCLVisualizer;
class CloudViewer;
} // namespace visualization
} // namespace pcl
#endif
