#include "bounding_box_utils.hpp"
#include <limits>
#include <cstddef>

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




    bool isValidCluster(const Cluster& cluster)
{
    const size_t num_points = cluster.points.size();

    if (num_points < 30 || num_points > 50000)
        return false;

    BoundingBox box = computeAABB(cluster);

    float length = box.max_x - box.min_x;
    float width  = box.max_y - box.min_y;
    float height = box.max_z - box.min_z;

    if (length < 0.3f || width < 0.3f || height < 0.3f)
        return false;

    if (length > 15.0f || width > 6.0f || height > 4.0f)
        return false;

    return true;
}

