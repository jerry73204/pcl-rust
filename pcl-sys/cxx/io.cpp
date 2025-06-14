#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "cxx/functions.h"
#include <memory>

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