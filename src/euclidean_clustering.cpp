#include "euclidean_clustering.hpp"
#include <cmath>
#include <cstddef>

static float Distance(const PointsXYZ& p1, const PointsXYZ& p2) {
    return std::sqrt(std::pow(p1.X - p2.X, 2) + std::pow(p1.Y - p2.Y, 2) + std::pow(p1.Z - p2.Z, 2));
}

std::vector<Cluster> euclideanClustering(const std::vector<PointsXYZ>& points, float cluster_tolerance, int min_cluster_size, int max_cluster_size) {
    std::vector<Cluster> clusters;
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (processed[i]) continue;

        Cluster cluster;
        std::vector<size_t> queue;
        queue.push_back(i);
        processed[i] = true;

        while (!queue.empty()) {
            size_t idx = queue.back();
            queue.pop_back();
            cluster.points.push_back(points[idx]);

            for (size_t j = 0; j < points.size(); ++j) {
                if (!processed[j] && Distance(points[idx], points[j]) <= cluster_tolerance) {
                    queue.push_back(j);
                    processed[j] = true;
                }
            }
        }

        if (cluster.points.size() >= static_cast<size_t>(min_cluster_size) && cluster.points.size() <= static_cast<size_t>(max_cluster_size)) {
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

