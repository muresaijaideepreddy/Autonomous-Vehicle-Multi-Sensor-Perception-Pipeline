#include "lidar_preprocessing.hpp"

std::vector<PointsXYZ> ROI_preprocessing(const std::vector<PointsXYZ>& raw_points,
                                 float min_x,
                                 float max_x,
                                 float min_y,
                                 float max_y,
                                 float min_z,    
                                 float max_z) {
    std::vector<PointsXYZ> roi_filtered_points;
    roi_filtered_points.reserve(raw_points.size());
    

    for (const auto& point : raw_points) {
        if (point.X < min_x || point.X > max_x) continue;
        if (point.Y < min_y || point.Y > max_y) continue;
        if (point.Z < min_z || point.Z > max_z) continue;

        roi_filtered_points.push_back(point);
    }
        
    
    return roi_filtered_points;
}