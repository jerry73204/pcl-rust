#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "cxx/functions.h"
#include <memory>
#include <algorithm>
#include <fstream>

// PCD I/O Functions for PointXYZ
int load_pcd_file_xyz(rust::Str file_name, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPCDFile<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFile<pcl::PointXYZ>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_ascii_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileASCII<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinary<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_compressed_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// PCD I/O Functions for PointXYZI
int load_pcd_file_xyzi(rust::Str file_name, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPCDFile<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFile<pcl::PointXYZI>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_ascii_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileASCII<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinary<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_compressed_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// PCD I/O Functions for PointXYZRGB
int load_pcd_file_xyzrgb(rust::Str file_name, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFile<pcl::PointXYZRGB>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_ascii_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileASCII<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinary<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_pcd_file_binary_compressed_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// PLY I/O Functions for PointXYZ
int load_ply_file_xyz(rust::Str file_name, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPLYFile<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFile<pcl::PointXYZ>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_ascii_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileASCII<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_binary_xyz(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileBinary<pcl::PointXYZ>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// PLY I/O Functions for PointXYZI
int load_ply_file_xyzi(rust::Str file_name, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPLYFile<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFile<pcl::PointXYZI>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_ascii_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileASCII<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_binary_xyzi(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileBinary<pcl::PointXYZI>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// PLY I/O Functions for PointXYZRGB
int load_ply_file_xyzrgb(rust::Str file_name, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool binary) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFile<pcl::PointXYZRGB>(filename_str, cloud, binary);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_ascii_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileASCII<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

int save_ply_file_binary_xyzrgb(rust::Str file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    try {
        std::string filename_str(file_name);
        return pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(filename_str, cloud);
    } catch (const std::exception& e) {
        return -1;
    }
}

// Format detection functions
enum FileFormat {
    FORMAT_UNKNOWN = 0,
    FORMAT_PCD = 1,
    FORMAT_PLY = 2,
    FORMAT_ERROR = -1
};

// Detect file format from extension
int detect_format_from_extension(rust::Str file_name) {
    std::string filename_str(file_name);
    
    // Convert to lowercase for case-insensitive comparison
    std::transform(filename_str.begin(), filename_str.end(), filename_str.begin(), ::tolower);
    
    if (filename_str.length() < 4) {
        return FORMAT_UNKNOWN;
    }
    
    std::string extension = filename_str.substr(filename_str.length() - 4);
    
    if (extension == ".pcd") {
        return FORMAT_PCD;
    } else if (extension == ".ply") {
        return FORMAT_PLY;
    }
    
    return FORMAT_UNKNOWN;
}

// Detect file format from file content
int detect_format_from_content(rust::Str file_name) {
    std::string filename_str(file_name);
    std::ifstream file(filename_str);
    
    if (!file.is_open()) {
        return FORMAT_ERROR;
    }
    
    std::string line;
    if (std::getline(file, line)) {
        // Check for PCD header
        if (line.find("# .PCD") != std::string::npos || 
            line.find("VERSION") != std::string::npos) {
            return FORMAT_PCD;
        }
        // Check for PLY header
        if (line.find("ply") != std::string::npos) {
            return FORMAT_PLY;
        }
    }
    
    file.close();
    return FORMAT_UNKNOWN;
}

// Auto-detect format (try extension first, then content)
int detect_file_format(rust::Str file_name) {
    int format = detect_format_from_extension(file_name);
    if (format == FORMAT_UNKNOWN) {
        format = detect_format_from_content(file_name);
    }
    return format;
}

// Auto-loading functions that detect format automatically
int load_point_cloud_auto_xyz(rust::Str file_name, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    int format = detect_file_format(file_name);
    
    switch (format) {
        case FORMAT_PCD:
            return load_pcd_file_xyz(file_name, cloud);
        case FORMAT_PLY:
            return load_ply_file_xyz(file_name, cloud);
        default:
            return -1; // Unsupported format
    }
}

int load_point_cloud_auto_xyzi(rust::Str file_name, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    int format = detect_file_format(file_name);
    
    switch (format) {
        case FORMAT_PCD:
            return load_pcd_file_xyzi(file_name, cloud);
        case FORMAT_PLY:
            return load_ply_file_xyzi(file_name, cloud);
        default:
            return -1; // Unsupported format
    }
}

int load_point_cloud_auto_xyzrgb(rust::Str file_name, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    int format = detect_file_format(file_name);
    
    switch (format) {
        case FORMAT_PCD:
            return load_pcd_file_xyzrgb(file_name, cloud);
        case FORMAT_PLY:
            return load_ply_file_xyzrgb(file_name, cloud);
        default:
            return -1; // Unsupported format
    }
}