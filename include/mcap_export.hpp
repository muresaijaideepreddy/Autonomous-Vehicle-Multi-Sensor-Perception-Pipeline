#pragma once
#include <vector>
#include <string>
#include "points_structure.hpp"

// One bounding box (axis-aligned)
struct ExportBox {
    float       cx, cy, cz;
    float       sx, sy, sz;
    std::string label;
};

void saveFrameToMCAP(const std::vector<PointsXYZ>&    all_points,
                     const std::vector<PointsXYZRGB>& cluster_points,
                     const std::vector<ExportBox>&    boxes);

void closePointCloudMCAP();