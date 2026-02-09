#include "voxel_clustering.hpp"
#include "voxel_grid.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>

std::vector<Cluster> voxelClustering(
    const std::vector<PointsXYZ>& points,
    float voxel_size,
    int min_points_per_voxel)
{
    std::unordered_map<VoxelKey, std::vector<PointsXYZ>, VoxelKeyHash> voxels;

    for (const auto& p : points) {
        VoxelKey key{
            static_cast<int>(std::floor(p.X / voxel_size)),
            static_cast<int>(std::floor(p.Y / voxel_size)),
            static_cast<int>(std::floor(p.Z / voxel_size))
        };
        voxels[key].push_back(p);
    }

    std::unordered_set<VoxelKey, VoxelKeyHash> visited;
    std::vector<Cluster> clusters;

    for (const auto& pair : voxels) {
        const auto& key = pair.first;
        const auto& voxel_points = pair.second;
        if (voxel_points.size() < min_points_per_voxel)
            continue;
        if (visited.count(key))
            continue;

        Cluster cluster;
        std::queue<VoxelKey> q;
        q.push(key);
        visited.insert(key);

        while (!q.empty()) {
            VoxelKey current = q.front();
            q.pop();

            const auto& pts = voxels[current];
            cluster.points.insert(
                cluster.points.end(),
                pts.begin(),
                pts.end()
            );

            for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                VoxelKey neighbor{
                    current.x + dx,
                    current.y + dy,
                    current.z + dz
                };

                if (voxels.count(neighbor) &&
                    !visited.count(neighbor) &&
                    voxels[neighbor].size() >= min_points_per_voxel)
                {
                    visited.insert(neighbor);
                    q.push(neighbor);
                }
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}
