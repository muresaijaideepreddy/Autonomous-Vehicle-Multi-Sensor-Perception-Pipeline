#include <vector>
#include "points_structure.hpp"
#include <string>
std::vector<PointsXYZ> ROI_preprocessing(const std::vector<PointsXYZ>& raw_points,
                                 float min_x,
                                 float max_x,
                                 float min_y,
                                 float max_y,
                                 float min_z,
                                 float max_z);