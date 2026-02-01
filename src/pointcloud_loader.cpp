#include <fstream>
#include <iostream>
#include "pointcloud_loader.hpp"

std::vector<PointsXYZ> loadPointCloudXYZFromFile(const std::string& file_path) {
    std::vector<PointsXYZ> pointCloud;
    std::ifstream file(file_path);
    if(!file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_path << std::endl;
        return pointCloud;
    }
    PointsXYZ point;
    while (file >> point.X >> point.Y >> point.Z)
    {
        pointCloud.push_back(point);
    }
    file.close();
    return pointCloud;
}
    
