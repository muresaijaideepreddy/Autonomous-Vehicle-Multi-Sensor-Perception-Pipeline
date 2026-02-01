#include "lidar_preprocessing.hpp"
#include "pointcloud_loader.hpp"
#include <iostream>
#include "lidar_ROI_Preproceesing.hpp"
int main()
{
    std::string filename = "lidar_data.txt";
    std::vector<PointsXYZ> raw_points = loadPointCloudXYZ(filename);
    float min_range = 1.0f;
    float max_range = 50.0f;
    float min_height = -2.0f;
    float max_height = 2.0f;
    std::vector<PointsXYZ> filtered_points = preprocessLidarData(raw_points, min_range, max_range, min_height, max_height);
    std::cout <<"Filtred Points of Size:"<< filtered_points.size() << std::endl;
    std::vector<PointsXYZ> roi_points = ROI_preprocessing(filtered_points, -10.0f, 10.0f, -10.0f, 10.0f, -2.0f, 2.0f);
    std::cout <<"ROI Points of Size:"<< roi_points.size() << std::endl;
    return 0;
}
