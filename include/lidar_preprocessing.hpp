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