#include <iostream>
#include <vector>
#include <string>
#include "mcap_export.hpp"
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

static constexpr uint8_t CLUSTER_COLORS[][3] = {
    {255,  50,  50},
    { 50, 255,  50},
    { 50, 100, 255},
    {255, 200,   0},
    {255,   0, 200},
    {  0, 220, 220},
    {255, 130,   0},
    {130,   0, 255},
    {  0, 255, 130},
    {200, 200, 200},
};
static constexpr int NUM_CLUSTER_COLORS =
    static_cast<int>(sizeof(CLUSTER_COLORS) / sizeof(CLUSTER_COLORS[0]));

int main()
{
    std::string dataset_path =
        "/mnt/c/Users/91790/Documents/Research/Autonomous Driving/0000/";

    FrameSequenceLoader loader(dataset_path);

    if (!loader.hasNext()) {
        std::cerr << "[main] no frames found at: " << dataset_path << "\n";
        return 1;
    }

    int frame_idx = 0;

    while (loader.hasNext())
    {
        std::cout << "\n=== frame " << frame_idx << " ===\n";

        const auto raw_points = loader.next();

        if (raw_points.empty()) {
            std::cerr << "[main] empty frame " << frame_idx << ", skipping\n";
            ++frame_idx;
            continue;
        }

        std::cout << "[load] " << raw_points.size() << " raw points\n";

        std::vector<PointsXYZ> xyz_points;
        xyz_points.reserve(raw_points.size());
        for (const auto& p : raw_points)
            xyz_points.push_back({p.X, p.Y, p.Z});

        const auto filtered_points =
            preprocessLidarData(xyz_points, 1.0f, 60.0f, -5.0f, 3.0f);

        const auto roi_points =
            ROI_preprocessing(filtered_points, 1.0f, 60.0f, 90.0f, -3.0f, 3.0f);

        const auto non_ground_points =
            removeGroundRadial(roi_points, 0.5f, 0.15f, 0.3f);

        std::cout << "[preprocess] filtered=" << filtered_points.size()
                  << " roi="        << roi_points.size()
                  << " non_ground=" << non_ground_points.size() << "\n";

        const auto raw_clusters = voxelClustering(non_ground_points, 0.25f, 3);

        std::vector<Cluster> clusters;
        clusters.reserve(raw_clusters.size());
        for (const auto& c : raw_clusters)
            if (isValidCluster(c))
                clusters.push_back(c);

        std::cout << "[cluster] " << raw_clusters.size()
                  << " raw -> " << clusters.size() << " valid\n";

        std::vector<PointsXYZRGB> colored_points;
        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& color = CLUSTER_COLORS[i % NUM_CLUSTER_COLORS];
            for (const auto& pt : clusters[i].points)
                colored_points.push_back({pt.X, pt.Y, pt.Z,
                                          color[0], color[1], color[2]});
        }

        std::vector<ExportBox> exportBoxes;
        exportBoxes.reserve(clusters.size());
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const BoundingBox box         = computeAABB(clusters[i]);
            const OrientedBoundingBox obb = computeOBB_PCA(clusters[i]);
            const std::string label       = classifyObject(box);

            std::cout << "  [box " << i << "] " << label << "\n";

            exportBoxes.push_back({
                .cx    = (box.min_x + box.max_x) * 0.5f,
                .cy    = (box.min_y + box.max_y) * 0.5f,
                .cz    = (box.min_z + box.max_z) * 0.5f,
                .sx    = box.max_x - box.min_x,
                .sy    = box.max_y - box.min_y,
                .sz    = box.max_z - box.min_z,
                .label = label
            });
        }

        saveFrameToMCAP(roi_points, colored_points, exportBoxes);
        saveClusters(clusters);
        ++frame_idx;
    }

    closePointCloudMCAP();
    std::cout << "\n[main] done. " << frame_idx << " frames -> output.mcap\n";
    return 0;
}