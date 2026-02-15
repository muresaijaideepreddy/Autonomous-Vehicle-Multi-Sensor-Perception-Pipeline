#include <iostream>
#include <vector>
#include <string>

#include "frame.hpp"
#include "lidar_preprocessing.hpp"
#include "lidar_ROI_Preproceesing.hpp"
#include "euclidean_clustering.hpp"
#include "voxel_clustering.hpp"
#include "bounding_box.hpp"
#include "bounding_box_utils.hpp"
#include "oriented_bounding_box_utils.hpp"
#include "object_classification.hpp"
#include "visuvalize_clusters.hpp"

int main()
{
    std::string dataset_path = "/mnt/c/Users/91790/Documents/Research/Autonomous Driving/0000/";  

    FrameSequenceLoader loader(dataset_path);

    while (loader.hasNext())
    {
        std::vector<PointsXYZI> raw_points = loader.next();

        std::cout << "Loaded Frame with Points: " 
                  << raw_points.size() << std::endl;
        
        // Convert PointsXYZI to PointsXYZ (removing intensity channel)
        std::vector<PointsXYZ> xyz_points;
        for (const auto& point : raw_points) {
            xyz_points.push_back({point.X, point.Y, point.Z});
        }
        
        auto filtered_points =
            preprocessLidarData(xyz_points, 1.0f, 50.0f, -5.0f, 3.0f);

        auto roi_points =
            ROI_preprocessing(filtered_points,
                              0.0f, 50.0f,
                              -10.0f, 10.0f,
                              -3.0f, 3.0f);

        auto non_ground_points =
            removeGroundRadial(roi_points, 1.0f, 0.25f);
        std::vector<Cluster> raw_clusters =
            voxelClustering(non_ground_points, 0.25f, 3);

        std::vector<Cluster> clusters;
        for (const auto& c : raw_clusters)
        {
            if (isValidCluster(c))
                clusters.push_back(c);
        }

        std::cout << "Clusters detected: "
                  << clusters.size() << std::endl;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            BoundingBox box = computeAABB(clusters[i]);
            OrientedBoundingBox obb = computeOBB_PCA(clusters[i]);

            std::string label = classifyObject(box);

            std::cout << "Cluster " << i 
                      << " Label: " << label << std::endl;
        }

        saveClusters(clusters);
    }

    return 0;
}
