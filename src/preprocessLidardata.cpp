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
#include <map>
#include <unordered_map>

std::vector<PointsXYZ> removeGroundRadial(
    const std::vector<PointsXYZ>& points,
    float radial_bin_size,
    float slope_threshold)
{
    std::map<int, float> min_z;

    for (const auto& p : points) {
        float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        int bin_id = static_cast<int>(r / radial_bin_size);

        if (!min_z.count(bin_id))
            min_z[bin_id] = p.Z;
        else
            min_z[bin_id] = std::min(min_z[bin_id], p.Z);
    }

    std::unordered_map<int, bool> ground_bin;

    int prev_bin = -1;
    float prev_r = 0.0f;
    float prev_z = 0.0f;

    for (const auto& it : min_z) {
        int bin_id = it.first;
        float z = it.second;
        float r = bin_id * radial_bin_size;

        if (prev_bin < 0) {
            ground_bin[bin_id] = true;
        } else {
            float slope = (z - prev_z) / (r - prev_r);
            ground_bin[bin_id] = std::abs(slope) < slope_threshold;
        }

        prev_bin = bin_id;
        prev_r = r;
        prev_z = z;
    }

    std::vector<PointsXYZ> non_ground;
    non_ground.reserve(points.size());

    for (const auto& p : points) {
        float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        int bin_id = static_cast<int>(r / radial_bin_size);

        if (!ground_bin[bin_id])
            non_ground.push_back(p);
    }

    return non_ground;
}