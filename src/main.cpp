#include "lidar_preprocessing.hpp"
#include "pointcloud_loader.hpp"
#include <iostream>
#include "lidar_ROI_Preproceesing.hpp"
#include  "euclidean_clustering.hpp"
#include "bounding_box.hpp"
#include "bounding_box_utils.hpp"

int main()
{
    std::string filename = "lidar_points.txt";
    std::vector<PointsXYZ> raw_points = loadPointCloudXYZ(filename);
    float min_range = 1.0f;
    float max_range = 50.0f;
    float min_height = -2.0f;
    float max_height = 2.0f;
    std::vector<PointsXYZ> filtered_points = preprocessLidarData(raw_points, min_range, max_range, min_height, max_height);
    std::cout <<"Filtred Points of Size:"<< filtered_points.size() << std::endl;
    std::vector<PointsXYZ> roi_points = ROI_preprocessing(filtered_points, -10.0f, 10.0f, -10.0f, 10.0f, -2.0f, 2.0f);
    std::cout <<"ROI Points of Size:"<< roi_points.size() << std::endl;
    std::vector<Cluster> clusters = euclideanClustering(roi_points, 0.5f, 10, 1000);
    std::cout <<"Clustered  Size:"<< clusters.size() << std::endl;
    for (size_t i = 0; i < clusters.size(); ++i) {
        BoundingBox box = computeAABB(clusters[i]);

        std::cout << "Cluster " << i << " Bounding Box:\n";
        std::cout << "  X: [" << box.min_x << ", " << box.max_x << "]\n";
        std::cout << "  Y: [" << box.min_y << ", " << box.max_y << "]\n";
        std::cout << "  Z: [" << box.min_z << ", " << box.max_z << "]\n";
    }

    return 0;
}
