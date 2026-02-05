#include "bounding_box_utils.hpp"
#include <limits>

BoundingBox computeAABB(const Cluster& cluster) {
    BoundingBox box;

    box.min_x = box.min_y = box.min_z = std::numeric_limits<float>::max();
    box.max_x = box.max_y = box.max_z = std::numeric_limits<float>::lowest();

    for (const auto& point : cluster.points) {
        if (point.X < box.min_x) box.min_x = point.X;
        if (point.X > box.max_x) box.max_x = point.X;

        if (point.Y < box.min_y) box.min_y = point.Y;
        if (point.Y > box.max_y) box.max_y = point.Y;

        if (point.Z < box.min_z) box.min_z = point.Z;
        if (point.Z > box.max_z) box.max_z = point.Z;
    }

    return box;
}
