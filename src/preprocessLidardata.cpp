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
#include <unordered_map>
#include <limits>

std::vector<PointsXYZ> removeGroundRadial(
    const std::vector<PointsXYZ>& points,
    float radial_bin_size,
    float ground_height_threshold)
{
    std::unordered_map<int, float> min_z_per_bin;

    // 1. find local ground per distance bin
    for (const auto& p : points) {
        float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        int bin = static_cast<int>(r / radial_bin_size);

        if (min_z_per_bin.count(bin) == 0)
            min_z_per_bin[bin] = p.Z;
        else
            min_z_per_bin[bin] = std::min(min_z_per_bin[bin], p.Z);
    }

    // 2. remove ground
    std::vector<PointsXYZ> non_ground;
    non_ground.reserve(points.size());

    for (const auto& p : points) {
        float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        int bin = static_cast<int>(r / radial_bin_size);

        if (p.Z > min_z_per_bin[bin] + ground_height_threshold)
            non_ground.push_back(p);
    }

    return non_ground;
}
