#include "tracker.hpp"
#include "bounding_box_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

KalmanTracker::KalmanTracker(const TrackerConfig& cfg)
    : cfg_(cfg) {}

float KalmanTracker::centroidDistance(const BoundingBox& a, const BoundingBox& b)
{
    const float dx = (a.min_x + a.max_x) * 0.5f - (b.min_x + b.max_x) * 0.5f;
    const float dy = (a.min_y + a.max_y) * 0.5f - (b.min_y + b.max_y) * 0.5f;
    return std::sqrt(dx*dx + dy*dy);
}

float KalmanTracker::iou2D(const BoundingBox& a, const BoundingBox& b)
{
    const float ix1 = std::max(a.min_x, b.min_x);
    const float iy1 = std::max(a.min_y, b.min_y);
    const float ix2 = std::min(a.max_x, b.max_x);
    const float iy2 = std::min(a.max_y, b.max_y);

    const float iw = ix2 - ix1;
    const float ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;

    const float inter  = iw * ih;
    const float area_a = (a.max_x - a.min_x) * (a.max_y - a.min_y);
    const float area_b = (b.max_x - b.min_x) * (b.max_y - b.min_y);
    const float uni    = area_a + area_b - inter;
    return (uni > 0.0f) ? (inter / uni) : 0.0f;
}

static std::vector<int> greedyAssign(
    const std::vector<std::vector<float>>& cost,
    int n_dets, int n_tracks, float max_cost)
{
    std::vector<int>  assignments(n_dets, -1);
    std::vector<bool> track_used(n_tracks, false);

    std::vector<std::tuple<float,int,int>> pairs;
    pairs.reserve(n_dets * n_tracks);
    for (int d = 0; d < n_dets; ++d)
        for (int t = 0; t < n_tracks; ++t)
            if (cost[d][t] < max_cost)
                pairs.push_back({cost[d][t], d, t});

    std::sort(pairs.begin(), pairs.end());

    std::vector<bool> det_used(n_dets, false);
    for (const auto& [c, d, t] : pairs) {
        if (!det_used[d] && !track_used[t]) {
            assignments[d] = t;
            det_used[d]    = true;
            track_used[t]  = true;
        }
    }
    return assignments;
}

std::vector<Track> KalmanTracker::update(
    const std::vector<Cluster>& clusters,
    const std::string*          labels,
    int                         n_clusters)
{
    std::vector<BoundingBox> dets;
    dets.reserve(n_clusters);
    for (int i = 0; i < n_clusters; ++i)
        dets.push_back(computeAABB(clusters[i]));

    const int n_dets   = static_cast<int>(dets.size());
    const int n_tracks = static_cast<int>(tracks_.size());

    for (auto& t : tracks_) {
        t.box.min_x += t.vx; t.box.max_x += t.vx;
        t.box.min_y += t.vy; t.box.max_y += t.vy;
    }

    std::vector<std::vector<float>> cost(
        n_dets, std::vector<float>(n_tracks, std::numeric_limits<float>::max()));

    for (int d = 0; d < n_dets; ++d)
        for (int t = 0; t < n_tracks; ++t) {
            const float dist = centroidDistance(dets[d], tracks_[t].box);
            const float iou  = iou2D(dets[d], tracks_[t].box);
            if (dist < cfg_.max_distance_m && iou >= cfg_.iou_threshold)
                cost[d][t] = dist;
        }

    const auto assignments = greedyAssign(cost, n_dets, n_tracks, cfg_.max_distance_m);

    std::vector<bool> track_matched(n_tracks, false);
    for (int d = 0; d < n_dets; ++d) {
        const int t = assignments[d];
        if (t < 0) continue;

        auto& track = tracks_[t];
        track.vx = (dets[d].min_x + dets[d].max_x) * 0.5f
                 - (track.box.min_x + track.box.max_x) * 0.5f;
        track.vy = (dets[d].min_y + dets[d].max_y) * 0.5f
                 - (track.box.min_y + track.box.max_y) * 0.5f;
        track.box           = dets[d];
        track.label         = labels[d];
        track.missed_frames = 0;
        track_matched[t]    = true;
    }

    for (int t = 0; t < n_tracks; ++t)
        if (!track_matched[t])
            ++tracks_[t].missed_frames;

    for (int d = 0; d < n_dets; ++d) {
        if (assignments[d] >= 0) continue;
        tracks_.push_back({
            .id            = next_id_++,
            .label         = labels[d],
            .box           = dets[d],
            .vx            = 0.0f,
            .vy            = 0.0f,
            .missed_frames = 0
        });
    }

    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const Track& t) {
                           return t.missed_frames > cfg_.max_missed_frames;
                       }),
        tracks_.end());

    return tracks_;
}