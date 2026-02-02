#pragma once
#include <vector>
#include "points_structure.hpp"
struct Cluster{
    std::vector<PointsXYZ> points;

};

std::vector<Cluster> euclideanClustering(const std::vector<PointsXYZ>& points, float cluster_tolerance, int min_cluster_size, int max_cluster_size);