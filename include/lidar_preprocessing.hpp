#pragma once
#include <vector>
#include "points_structure.hpp"
#include <cmath>

std::vector<PointsXYZ> preprocessLidarData(const std::vector<PointsXYZ>& raw_points,
float min_range,
float max_range,
float min_height,
float max_height
);
std::vector<PointsXYZ> GroundRemoval(const std::vector<PointsXYZ>& raw_points,
float ground_threshold
);
std::vector<PointsXYZ> removeGroundRadial(
    const std::vector<PointsXYZ>& points,
    float radial_bin_size,
    float ground_height_threshold
);
