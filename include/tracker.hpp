#pragma once

#include <vector>
#include <string>
#include "euclidean_clustering.hpp"
#include "bounding_box.hpp"

struct Track {
    int         id;
    std::string label;
    BoundingBox box;
    float       vx;
    float       vy;
    int         missed_frames;
};

struct TrackerConfig {
    float iou_threshold     = 0.1f;
    float max_distance_m    = 3.0f;
    int   max_missed_frames = 3;
};

class KalmanTracker {
public:
    explicit KalmanTracker(const TrackerConfig& cfg = TrackerConfig{});

    std::vector<Track> update(const std::vector<Cluster>& clusters,
                              const std::string*          labels,
                              int                         n_clusters);

    const std::vector<Track>& activeTracks() const { return tracks_; }

private:
    TrackerConfig      cfg_;
    std::vector<Track> tracks_;
    int                next_id_ = 0;

    static float centroidDistance(const BoundingBox& a, const BoundingBox& b);
    static float iou2D(const BoundingBox& a, const BoundingBox& b);
};