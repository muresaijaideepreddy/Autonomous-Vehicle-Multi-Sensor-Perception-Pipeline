#include <iostream>
#include <vector>
#include <string>

#include "pointcloud_loader.hpp"
#include "lidar_preprocessing.hpp"
#include "lidar_ROI_Preproceesing.hpp"
#include "euclidean_clustering.hpp"
#include "bounding_box.hpp"
#include "bounding_box_utils.hpp"
#include "oriented_bounding_box_utils.hpp"

int main()
{
    std::string filename = "lidar_points.txt";
    std::vector<PointsXYZ> raw_points = loadPointCloudXYZ(filename);

    float min_range = 1.0f;
    float max_range = 50.0f;
    float min_height = -5.0f;
    float max_height = 3.0f;

    std::vector<PointsXYZ> filtered_points =
        preprocessLidarData(raw_points, min_range, max_range, min_height, max_height);

    std::cout << "Filtered Points: " << filtered_points.size() << std::endl;

    std::vector<PointsXYZ> roi_points =
        ROI_preprocessing(filtered_points,
                           0.0f, 50.0f,
                          -10.0f, 10.0f,
                           -3.0f, 3.0f);

    std::cout << "ROI Points: " << roi_points.size() << std::endl;

    std::vector<PointsXYZ> non_ground_points =
        removeGroundRadial(roi_points, 1.0f, 0.25f);

    std::cout << "Non-ground Points: " << non_ground_points.size() << std::endl;

    std::vector<Cluster> clusters =
        euclideanClustering(non_ground_points, 0.5f, 50, 3000);

    std::cout << "Clusters: " << clusters.size() << std::endl;

    for (size_t i = 0; i < clusters.size(); ++i)
    {
        BoundingBox box = computeAABB(clusters[i]);

        std::cout << "Cluster " << i << " AABB\n";
        std::cout << box.min_x << " " << box.max_x << "\n";
        std::cout << box.min_y << " " << box.max_y << "\n";
        std::cout << box.min_z << " " << box.max_z << "\n";

        OrientedBoundingBox obb = computeOBB_PCA(clusters[i]);

        std::cout << "Cluster " << i << " OBB\n";
        std::cout << obb.center_x << " " << obb.center_y << "\n";
        std::cout << obb.length << " " << obb.width << " " << obb.height << "\n";
        std::cout << obb.yaw << "\n";
    }

    return 0;
}
