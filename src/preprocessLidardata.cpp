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
#include <cmath>

std::vector<PointsXYZ> removeGroundRadial(
    const std::vector<PointsXYZ>& points,
    float radial_bin_size,
    float slope_threshold,
    float height_threshold_m)
{
    if (points.empty()) return {};

    constexpr int NUM_SECTORS     = 72;
    const float   sector_size_rad = 2.0f * M_PI / NUM_SECTORS;

    std::unordered_map<int, float> min_z_map;
    min_z_map.reserve(points.size() / 4);

    for (const auto& p : points)
    {
        const float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        if (r < 1e-3f) continue;

        const int sector_id = static_cast<int>(
            (std::atan2(p.Y, p.X) + M_PI) / sector_size_rad) % NUM_SECTORS;
        const int bin_id    = static_cast<int>(r / radial_bin_size);
        const int key       = sector_id * 10000 + bin_id;

        auto it = min_z_map.find(key);
        if (it == min_z_map.end()) min_z_map[key] = p.Z;
        else it->second = std::min(it->second, p.Z);
    }

    std::unordered_map<int, std::map<int, float>> sector_bins;
    for (const auto& kv : min_z_map)
        sector_bins[kv.first / 10000][kv.first % 10000] = kv.second;

    std::unordered_map<int, bool> is_ground_bin;
    is_ground_bin.reserve(min_z_map.size());

    for (const auto& sector_kv : sector_bins)
    {
        const int   sector_id = sector_kv.first;
        const auto& bins      = sector_kv.second;

        int   prev_bin = -1;
        float prev_r   = 0.0f;
        float prev_z   = 0.0f;

        for (const auto& bin_kv : bins)
        {
            const int   bin_id = bin_kv.first;
            const float z      = bin_kv.second;
            const float r      = (bin_id + 0.5f) * radial_bin_size;
            const int   key    = sector_id * 10000 + bin_id;

            if (prev_bin < 0)
                is_ground_bin[key] = (z < 0.5f);
            else {
                const float slope = (z - prev_z) / (r - prev_r);
                is_ground_bin[key] = std::abs(slope) < slope_threshold;
            }

            prev_bin = bin_id;
            prev_r   = r;
            prev_z   = z;
        }
    }

    std::vector<PointsXYZ> non_ground;
    non_ground.reserve(points.size() / 2);

    for (const auto& p : points)
    {
        const float r = std::sqrt(p.X * p.X + p.Y * p.Y);
        if (r < 1e-3f) continue;

        const int sector_id = static_cast<int>(
            (std::atan2(p.Y, p.X) + M_PI) / sector_size_rad) % NUM_SECTORS;
        const int bin_id    = static_cast<int>(r / radial_bin_size);
        const int key       = sector_id * 10000 + bin_id;

        const auto it           = is_ground_bin.find(key);
        const bool bin_is_ground = (it != is_ground_bin.end()) && it->second;

        if (!bin_is_ground) {
            non_ground.push_back(p);
            continue;
        }

        if (p.Z > min_z_map.at(key) + height_threshold_m)
            non_ground.push_back(p);
    }

    return non_ground;
}