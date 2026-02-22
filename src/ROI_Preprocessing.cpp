#include "lidar_preprocessing.hpp"

std::vector<PointsXYZ> ROI_preprocessing(
    const std::vector<PointsXYZ>& raw_points,
    float min_range,
    float max_range,
    float fov_half_deg,
    float min_z,
    float max_z)
{
    std::vector<PointsXYZ> out;
    out.reserve(raw_points.size());

    const float fov_rad = fov_half_deg * static_cast<float>(M_PI) / 180.0f;
    const float min_sq  = min_range * min_range;
    const float max_sq  = max_range * max_range;

    for (const auto& p : raw_points)
    {
        if (p.Z < min_z || p.Z > max_z) continue;

        const float range_sq = p.X * p.X + p.Y * p.Y;
        if (range_sq < min_sq || range_sq > max_sq) continue;

        if (std::abs(std::atan2(p.Y, p.X)) > fov_rad) continue;

        out.push_back(p);
    }

    return out;
}