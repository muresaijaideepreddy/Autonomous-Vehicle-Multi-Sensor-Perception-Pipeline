#pragma once
#include <vector>
#include <unordered_map>
#include <tuple>

struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        return ((k.x * 73856093) ^
                (k.y * 19349663) ^
                (k.z * 83492791));
    }
};
