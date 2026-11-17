#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace vision {

struct Rect {
    float x = 0, y = 0, w = 0, h = 0;
    Rect() = default;
    Rect(float x_, float y_, float w_, float h_) : x(x_), y(y_), w(w_), h(h_) {}
};
struct Detection {
    Rect box;
    float score = 0;
};
struct Track {
    int id = 0;
    Rect box;
    float score = 0;
    int hits = 0;
    int missed = 0;
    bool matched = false;
};

// Simple detection-to-detection multi-person tracker (not a motion predictor).
// No camera, PLC, or OpenCV dependencies. IDs are temporary; re-identification
// after occlusion/camera cuts is NOT provided.
class PersonTracker {
public:
    explicit PersonTracker(int max_missed = 8)
        : max_missed_(std::max(0, max_missed)) {}

    void update(const std::vector<Detection>& detections, int frame_width, int frame_height) {
        for (auto& t : tracks_) t.matched = false;
        std::vector<bool> used(detections.size(), false);

        // Greedy global best-pair assignment to avoid pairing multiple detections
        // with one track. IoU and center displacement both contribute.
        while (true) {
            int best_track = -1, best_det = -1;
            float best_cost = 1e9f;
            for (size_t i = 0; i < tracks_.size(); ++i) {
                if (tracks_[i].matched) continue;
                for (size_t j = 0; j < detections.size(); ++j) {
                    if (used[j]) continue;
                    const float overlap = iou(tracks_[i].box, detections[j].box);
                    const float delta = centerDistance(tracks_[i].box, detections[j].box);
                    const float diag = std::sqrt(float(frame_width) * frame_width +
                                                 float(frame_height) * frame_height);
                    const float normalized = diag > 0 ? delta / diag : 1.0f;
                    // IoU matching is preferred; permit small motion even with no IoU.
                    if (overlap < 0.12f && normalized > 0.085f) continue;
                    const float cost = (1.0f - overlap) + normalized * 2.0f;
                    if (cost < best_cost) {
                        best_cost = cost; best_track = int(i); best_det = int(j);
                    }
                }
            }
            if (best_track < 0) break;
            auto& t = tracks_[best_track];
            t.box = detections[best_det].box;
            t.score = detections[best_det].score;
            ++t.hits;
            t.missed = 0;
            t.matched = true;
            used[best_det] = true;
        }
        for (auto& t : tracks_) {
            if (!t.matched) ++t.missed;
        }
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [this](const Track& t) { return t.missed > max_missed_; }),
                      tracks_.end());
        for (size_t i = 0; i < detections.size(); ++i) {
            if (used[i]) continue;
            Track t;
            t.id = next_id_++;
            t.box = detections[i].box;
            t.score = detections[i].score;
            t.hits = 1;
            t.matched = true;
            tracks_.push_back(t);
        }
    }

    const std::vector<Track>& tracks() const { return tracks_; }
    void reset() { tracks_.clear(); next_id_ = 1; }

private:
    static float iou(const Rect& a, const Rect& b) {
        const float l = std::max(a.x, b.x), t = std::max(a.y, b.y);
        const float r = std::min(a.x + a.w, b.x + b.w);
        const float d = std::min(a.y + a.h, b.y + b.h);
        const float intersection = std::max(0.0f, r - l) * std::max(0.0f, d - t);
        const float union_area = a.w * a.h + b.w * b.h - intersection;
        return union_area > 0 ? intersection / union_area : 0;
    }
    static float centerDistance(const Rect& a, const Rect& b) {
        const float dx = (a.x + a.w * 0.5f) - (b.x + b.w * 0.5f);
        const float dy = (a.y + a.h * 0.5f) - (b.y + b.h * 0.5f);
        return std::sqrt(dx * dx + dy * dy);
    }
    int max_missed_ = 8;
    int next_id_ = 1;
    std::vector<Track> tracks_;
};

} // namespace vision
