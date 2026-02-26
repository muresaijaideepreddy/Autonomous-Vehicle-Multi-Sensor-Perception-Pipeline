#include "road_surface.hpp"

#include <cmath>
#include <unordered_map>
#include <algorithm>

struct CellKey {
    int x, y;
    bool operator==(const CellKey& o) const { return x == o.x && y == o.y; }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
        return (k.x * 73856093) ^ (k.y * 19349663);
    }
};

std::vector<PointsXYZRGB> colorizeGroundSurface(
    const std::vector<PointsXYZ>& roi_points,
    const std::vector<PointsXYZ>& non_ground_points)
{
    constexpr float CELL_SIZE      = 0.5f;
    constexpr float CURB_THRESHOLD = 0.05f;
    constexpr float WALL_THRESHOLD = 0.15f;

    // Build set of non-ground point indices for fast lookup
    // Ground points = roi_points minus non_ground_points
    // We identify ground by building a 2D grid of min-Z per cell from roi
    // then comparing each point's Z to the local minimum

    // Step 1 — build 2D grid of minimum Z per cell from ALL roi points
    std::unordered_map<CellKey, float, CellKeyHash> min_z_grid;
    min_z_grid.reserve(roi_points.size() / 4);

    for (const auto& p : roi_points) {
        CellKey key{
            static_cast<int>(std::floor(p.X / CELL_SIZE)),
            static_cast<int>(std::floor(p.Y / CELL_SIZE))
        };
        auto it = min_z_grid.find(key);
        if (it == min_z_grid.end())
            min_z_grid[key] = p.Z;
        else
            it->second = std::min(it->second, p.Z);
    }

    // Step 2 — build lookup of non-ground points to exclude them
    // Use a coarse voxel key to mark which cells have non-ground points
    std::unordered_map<CellKey, bool, CellKeyHash> non_ground_cells;
    non_ground_cells.reserve(non_ground_points.size());
    for (const auto& p : non_ground_points) {
        CellKey key{
            static_cast<int>(std::floor(p.X / CELL_SIZE)),
            static_cast<int>(std::floor(p.Y / CELL_SIZE))
        };
        non_ground_cells[key] = true;
    }

    // Step 3 — colorize ground points
    std::vector<PointsXYZRGB> colored;
    colored.reserve(roi_points.size());

    for (const auto& p : roi_points) {
        CellKey key{
            static_cast<int>(std::floor(p.X / CELL_SIZE)),
            static_cast<int>(std::floor(p.Y / CELL_SIZE))
        };

        // Skip points that are non-ground (objects)
        if (non_ground_cells.count(key)) continue;

        const float local_min = min_z_grid.at(key);
        const float height_above_ground = p.Z - local_min;

        uint8_t r, g, b;

        if (height_above_ground < CURB_THRESHOLD) {
            // Road surface — green
            r = 0; g = 180; b = 0;
        } else if (height_above_ground < WALL_THRESHOLD) {
            // Curb / raised marking — yellow
            r = 220; g = 220; b = 0;
        } else {
            // Wall / obstacle base — orange-red
            r = 200; g = 80; b = 0;
        }

        colored.push_back({p.X, p.Y, p.Z, r, g, b});
    }

    return colored;
}