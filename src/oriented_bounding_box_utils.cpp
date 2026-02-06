
#include <cmath>
#include <limits>

#include "oriented_bounding_box_utils.hpp"
OrientedBoundingBox computeOBB_PCA(const Cluster& cluster)
{
    float mean_x = 0.0f;
    float mean_y = 0.0f;

    for (const auto& p : cluster.points) {
        mean_x += p.X;
        mean_y += p.Y;
    }
    mean_x /= cluster.points.size();
    mean_y /= cluster.points.size();
    float cov_xx = 0, cov_xy = 0, cov_yy = 0;

    for (const auto& p : cluster.points) {
        float dx = p.X - mean_x;
        float dy = p.Y - mean_y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

    cov_xx /= cluster.points.size();
    cov_xy /= cluster.points.size();
    cov_yy /= cluster.points.size();
    float yaw = 0.5f * std::atan2(2 * cov_xy, cov_xx - cov_yy);
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& p : cluster.points) {
        float dx = p.X - mean_x;
        float dy = p.Y - mean_y;

        float rx =  dx * cos_yaw + dy * sin_yaw;
        float ry = -dx * sin_yaw + dy * cos_yaw;

        min_x = std::min(min_x, rx);
        max_x = std::max(max_x, rx);
        min_y = std::min(min_y, ry);
        max_y = std::max(max_y, ry);

        min_z = std::min(min_z, p.Z);
        max_z = std::max(max_z, p.Z);
    }

    OrientedBoundingBox obb;
    obb.center_x = mean_x;
    obb.center_y = mean_y;
    obb.length = max_x - min_x;
    obb.width  = max_y - min_y;
    obb.height = max_z - min_z;
    obb.yaw = yaw;

    return obb;
}
