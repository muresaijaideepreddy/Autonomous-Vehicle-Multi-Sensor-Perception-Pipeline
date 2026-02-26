#pragma once

#include <vector>
#include "points_structure.hpp"
#include "mcap_export.hpp"

std::vector<PointsXYZRGB> colorizeGroundSurface(
    const std::vector<PointsXYZ>& roi_points,
    const std::vector<PointsXYZ>& non_ground_points);