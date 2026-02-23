#include "dbscan_clustering.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace {

struct KDNode {
    PointsXYZ point;
    int        idx;
    int        left  = -1;
    int        right = -1;
};

static std::vector<KDNode> g_tree;

int buildKDTree(std::vector<std::pair<PointsXYZ, int>>& pts,
                int lo, int hi, int depth)
{
    if (lo >= hi) return -1;

    const int axis = depth % 3;
    const int mid  = (lo + hi) / 2;

    std::nth_element(pts.begin() + lo,
                     pts.begin() + mid,
                     pts.begin() + hi,
                     [axis](const auto& a, const auto& b) {
                         if (axis == 0) return a.first.X < b.first.X;
                         if (axis == 1) return a.first.Y < b.first.Y;
                         return a.first.Z < b.first.Z;
                     });

    const int node_idx = static_cast<int>(g_tree.size());
    g_tree.push_back({pts[mid].first, pts[mid].second, -1, -1});

    g_tree[node_idx].left  = buildKDTree(pts, lo, mid, depth + 1);
    g_tree[node_idx].right = buildKDTree(pts, mid + 1, hi, depth + 1);

    return node_idx;
}

void radiusSearch(int node_idx, const PointsXYZ& query,
                  float eps_sq, int depth,
                  std::vector<int>& result)
{
    if (node_idx < 0) return;

    const auto& node = g_tree[node_idx];

    const float dx = query.X - node.point.X;
    const float dy = query.Y - node.point.Y;
    const float dz = query.Z - node.point.Z;
    const float dist_sq = dx*dx + dy*dy + dz*dz;

    if (dist_sq <= eps_sq)
        result.push_back(node.idx);

    const int axis = depth % 3;
    float diff = 0.0f;
    if (axis == 0) diff = dx;
    else if (axis == 1) diff = dy;
    else diff = dz;

    const int first  = (diff <= 0.0f) ? node.left  : node.right;
    const int second = (diff <= 0.0f) ? node.right : node.left;

    radiusSearch(first, query, eps_sq, depth + 1, result);

    if (diff * diff <= eps_sq)
        radiusSearch(second, query, eps_sq, depth + 1, result);
}

} // namespace

std::vector<Cluster> dbscanClustering(
    const std::vector<PointsXYZ>& points,
    const DBSCANConfig&           cfg)
{
    const int N = static_cast<int>(points.size());
    if (N == 0) return {};

    // Build KD-tree
    g_tree.clear();
    g_tree.reserve(N);

    std::vector<std::pair<PointsXYZ, int>> indexed;
    indexed.reserve(N);
    for (int i = 0; i < N; ++i)
        indexed.push_back({points[i], i});

    const int root    = buildKDTree(indexed, 0, N, 0);
    const float eps_sq = cfg.epsilon * cfg.epsilon;

    // DBSCAN
    constexpr int UNVISITED = -1;
    constexpr int NOISE     = -2;

    std::vector<int> labels(N, UNVISITED);
    int cluster_id = 0;

    for (int i = 0; i < N; ++i)
    {
        if (labels[i] != UNVISITED) continue;

        std::vector<int> neighbors;
        neighbors.reserve(64);
        radiusSearch(root, points[i], eps_sq, 0, neighbors);

        if (static_cast<int>(neighbors.size()) < cfg.min_points) {
            labels[i] = NOISE;
            continue;
        }

        labels[i] = cluster_id;

        std::queue<int> q;
        for (int n : neighbors)
            if (n != i) q.push(n);

        while (!q.empty())
        {
            const int cur = q.front();
            q.pop();

            if (labels[cur] == NOISE)
                labels[cur] = cluster_id;

            if (labels[cur] != UNVISITED)
                continue;

            labels[cur] = cluster_id;

            std::vector<int> cur_neighbors;
            cur_neighbors.reserve(64);
            radiusSearch(root, points[cur], eps_sq, 0, cur_neighbors);

            if (static_cast<int>(cur_neighbors.size()) >= cfg.min_points)
                for (int n : cur_neighbors)
                    if (labels[n] == UNVISITED || labels[n] == NOISE)
                        q.push(n);
        }

        ++cluster_id;
    }

    // Build Cluster objects
    std::vector<Cluster> clusters(cluster_id);
    for (int i = 0; i < N; ++i)
        if (labels[i] >= 0)
            clusters[labels[i]].points.push_back(points[i]);

    // Remove empty clusters
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](const Cluster& c){ return c.points.empty(); }),
        clusters.end());

    return clusters;
}