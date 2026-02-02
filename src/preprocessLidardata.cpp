#include "lidar_preprocessing.hpp"

std::vector<PointsXYZ> preprocessLidarData(const std::vector<PointsXYZ>& raw_points,
                                         float min_range,
                                         float max_range,
                                         float min_height,
                                         float max_height) {
    std::vector<PointsXYZ> filtered_points;
    for (const auto& point : raw_points) {
        float distance = std::sqrt(point.X * point.X + point.Y * point.Y);
        if (distance >= min_range && distance <= max_range &&
            point.Z >= min_height && point.Z <= max_height) {
            filtered_points.push_back(point);
        }
    }
    return filtered_points;
}

std::vector<PointsXYZ> GroundRemoval(const std::vector<PointsXYZ>& raw_points,
                                    float ground_threshold) {
    std::vector<PointsXYZ> non_ground_points;
    for (const auto& point : raw_points) {
        if (point.Z > ground_threshold) {
            non_ground_points.push_back(point);
        }
    }
    return non_ground_points;
}