#pragma once
#include <vector>
#include <euclidean_clustering.hpp>

struct DBSCANConfig {
    float base_epsilon      = 0.6f;   // epsilon at reference range
    float reference_range_m = 10.0f;  // range at which base_epsilon applies
    float max_epsilon       = 3.0f;   // cap to prevent merging far objects
    int   min_points        = 8;      // min points to form a core point
};
std::vector<Cluster> dbscanClustering(
    const std::vector<PointsXYZ>& points,
    const DBSCANConfig&           cfg = DBSCANConfig{}

);
