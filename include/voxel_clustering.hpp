#pragma once
#include <vector>
#include <unordered_map>
#include "euclidean_clustering.hpp"

std::vector<Cluster> voxelClustering(
    const std::vector<PointsXYZ>& points,
    float voxel_size,
    int min_points_per_voxel);
