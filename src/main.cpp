#include <iostream>
#include <vector>
#include <string>
#include <cmath>
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
#include "dbscan_clustering.hpp"
#include "tracker.hpp"
#include "road_surface.hpp"

static const std::string DATASET_PATH =
    "/mnt/c/Users/91790/Documents/Research/Autonomous Driving/0000/";

static constexpr uint8_t TRACK_COLORS[][3] = {
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
static constexpr int NUM_TRACK_COLORS =
    static_cast<int>(sizeof(TRACK_COLORS) / sizeof(TRACK_COLORS[0]));

int main()
{
    FrameSequenceLoader loader(DATASET_PATH);

    if (!loader.hasNext()) {
        std::cerr << "[main] no frames found at: " << DATASET_PATH << "\n";
        return 1;
    }

    KalmanTracker tracker;
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

        const auto road_surface =
            colorizeGroundSurface(roi_points, non_ground_points);

        const auto raw_clusters = dbscanClustering(non_ground_points, {
            .base_epsilon      = 0.6f,
            .reference_range_m = 10.0f,
            .max_epsilon       = 3.0f,
            .min_points        = 8
        });

        std::vector<Cluster> clusters;
        clusters.reserve(raw_clusters.size());
        for (const auto& c : raw_clusters)
            if (isValidCluster(c))
                clusters.push_back(c);

        std::cout << "[cluster] " << raw_clusters.size()
                  << " raw -> " << clusters.size() << " valid\n";

        std::vector<std::string> labels;
        labels.reserve(clusters.size());
        for (const auto& c : clusters)
            labels.push_back(classifyObject(computeAABB(c)));

        const auto tracks = tracker.update(
            clusters, labels.data(), static_cast<int>(clusters.size()));

        std::cout << "[tracker] " << tracks.size() << " active tracks\n";

        std::vector<PointsXYZRGB> colored_points;
        for (const auto& c : clusters) {
            const BoundingBox box = computeAABB(c);
            const float cx = (box.min_x + box.max_x) * 0.5f;
            const float cy = (box.min_y + box.max_y) * 0.5f;
            int   track_id = 0;
            float best     = 1e9f;
            for (const auto& t : tracks) {
                const float tx   = (t.box.min_x + t.box.max_x) * 0.5f;
                const float ty   = (t.box.min_y + t.box.max_y) * 0.5f;
                const float dist = std::sqrt((cx-tx)*(cx-tx) + (cy-ty)*(cy-ty));
                if (dist < best) { best = dist; track_id = t.id; }
            }
            const auto& color = TRACK_COLORS[track_id % NUM_TRACK_COLORS];
            for (const auto& pt : c.points)
                colored_points.push_back({pt.X, pt.Y, pt.Z,
                                          color[0], color[1], color[2]});
        }

        std::vector<ExportBox> exportBoxes;
        exportBoxes.reserve(tracks.size());
        for (const auto& t : tracks) {
            std::cout << "  [" << t.id << "] " << t.label
                      << " vx=" << t.vx << " vy=" << t.vy
                      << " missed=" << t.missed_frames << "\n";
            exportBoxes.push_back({
                .cx    = (t.box.min_x + t.box.max_x) * 0.5f,
                .cy    = (t.box.min_y + t.box.max_y) * 0.5f,
                .cz    = (t.box.min_z + t.box.max_z) * 0.5f,
                .sx    = t.box.max_x - t.box.min_x,
                .sy    = t.box.max_y - t.box.min_y,
                .sz    = t.box.max_z - t.box.min_z,
                .label = "[" + std::to_string(t.id) + "] " + t.label
            });
        }

        saveFrameToMCAP(roi_points, colored_points, road_surface, exportBoxes);
        saveClusters(clusters);
        ++frame_idx;
    }

    closePointCloudMCAP();
    std::cout << "\n[main] done. " << frame_idx << " frames -> output.mcap\n";
    return 0;
}