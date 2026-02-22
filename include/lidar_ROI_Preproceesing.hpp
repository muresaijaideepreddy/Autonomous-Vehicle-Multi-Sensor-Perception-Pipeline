#include <vector>
#include "points_structure.hpp"
#include <string>
std::vector<PointsXYZ> ROI_preprocessing(
    const std::vector<PointsXYZ>& raw_points,
    float min_range,
    float max_range,
    float fov_half_deg,
    float min_z,
    float max_z);