#pragma once
#include <vector>
#include "points_structure.hpp"

void savePointCloudToMCAP(const std::vector<PointsXYZ>& points);
void closePointCloudMCAP();