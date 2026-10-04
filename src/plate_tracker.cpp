#include "auto_aim/plate_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim {

PlateTracker::PlateTracker() : PlateTracker(Params{}) {}

PlateTracker::PlateTracker(const Params& params) : params_(params) {}

void PlateTracker::Reset() {
  tracks_.clear();
  gone_.clear();
  next_id_ = 0;
}

const TrackedPlate* PlateTracker::Find(int id) const {
  for (const auto& t : tracks_) {
    if (t.id == id) {
      return &t;
    }
  }
  return nullptr;
}

void PlateTracker::Remove(int id) {
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [id](const TrackedPlate& t) {
                                 return t.id == id;
                               }),
                tracks_.end());
}

void PlateTracker::Update(const std::vector<ArmorPlate>& detections,
                          double stamp_sec) {
  gone_.clear();

  // Nearest-neighbour matching against each track's fitted position. At
  // 60 fps a 700 px/s plate moves ~12 px per frame, so the gate mostly
  // guards against mismatches; it scales with dt for low-fps regimes.
  struct Pair {
    size_t track;
    size_t det;
    float dist;
  };
  std::vector<Pair> pairs;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    const TrackedPlate& t = tracks_[i];
    if (t.history.empty()) {
      continue;
    }
    const cv::Point2f last(t.history.back().x, t.history.back().y);
    const double dt = std::max(0.0, stamp_sec - t.last_stamp);
    const PlateState st = FitState(t, stamp_sec, params_.max_accel_px_s2);
    const cv::Point2f predicted = st.valid ? st.position : last;
    const float gate =
        params_.match_gate_px +
        (st.valid ? std::abs(st.vx) : params_.max_speed_px_s * 0.5F) *
            static_cast<float>(dt);
    for (size_t j = 0; j < detections.size(); ++j) {
      // Anisotropic gate: a plate's lane never changes (y is constant by
      // construction), so y is a hard identity constraint while x allows
      // the full inter-frame travel. A round gate lets plates 70 px apart
      // on adjacent lanes swap identities whenever they close in
      // horizontally, fragmenting tracks and wiping damage memory.
      const float dy = std::fabs(detections[j].center.y - predicted.y);
      const float dx = std::fabs(detections[j].center.x - predicted.x);
      if (dy < 22.0F && dx < gate) {
        pairs.push_back({i, j, dx});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const Pair& a, const Pair& b) { return a.dist < b.dist; });

  std::vector<bool> det_used(detections.size(), false);
  std::vector<bool> trk_used(tracks_.size(), false);
  for (const auto& p : pairs) {
    if (det_used[p.det] || trk_used[p.track]) {
      continue;
    }
    det_used[p.det] = true;
    trk_used[p.track] = true;
    TrackedPlate& t = tracks_[p.track];
    const ArmorPlate& d = detections[p.det];
    // A track's colour can only transition live -> dead (the hit flash
    // briefly hides the bars, but the colour class never flips R<->B).
    if (d.color == TeamColor::kDead) {
      t.dead = true;
    } else if (!t.dead) {
      t.color = d.color;
    }
    t.history.push_back({stamp_sec, d.center.x, d.center.y});
    while (t.history.size() > params_.max_history ||
           (t.history.size() > 2 &&
            stamp_sec - t.history.front().stamp > params_.max_fit_span_s)) {
      t.history.pop_front();
    }
    t.last_detect_stamp = stamp_sec;
    t.misses = 0;
    t.last_stamp = stamp_sec;
    ++t.matches;
  }

  std::vector<TrackedPlate> kept;
  kept.reserve(tracks_.size());
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (trk_used[i]) {
      kept.push_back(tracks_[i]);
      continue;
    }
    TrackedPlate t = tracks_[i];
    ++t.misses;
    if (t.misses == 1 && !t.history.empty()) {
      // Coast one frame so single-frame detection dropouts (the hit
      // flash hides the light bars for ~0.2 s) do not break the fit.
      const PlateState st = FitState(t, stamp_sec, params_.max_accel_px_s2);
      if (st.valid) {
        t.history.push_back({stamp_sec, st.position.x, st.position.y});
        while (t.history.size() > params_.max_history) {
          t.history.pop_front();
        }
      }
    }
    if (t.history.empty() ||
        stamp_sec - t.last_stamp > params_.retire_after_s) {
      gone_.push_back({t.id, t.color, t.dead,
                       t.history.empty() ? 0.0F : t.history.back().x});
    } else {
      kept.push_back(t);
    }
  }

  for (size_t j = 0; j < detections.size(); ++j) {
    if (det_used[j]) {
      continue;
    }
    const ArmorPlate& d = detections[j];
    TrackedPlate t;
    t.id = next_id_++;
    t.color = d.color;
    t.dead = d.color == TeamColor::kDead;
    t.seen_alive = d.color != TeamColor::kDead;
    t.history.push_back({stamp_sec, d.center.x, d.center.y});
    t.last_detect_stamp = stamp_sec;
    t.last_stamp = stamp_sec;
    t.matches = 1;
    kept.push_back(t);
  }
  tracks_ = kept;
}

PlateState PlateTracker::FitState(const TrackedPlate& track,
                                  double stamp_sec, float max_accel_px_s2) {
  PlateState out;
  out.stamp = stamp_sec;
  out.y = track.history.empty() ? 0.0F : track.history.back().y;
  if (track.history.size() < 3) {
    if (track.history.size() == 2) {
      const double dt = track.history[1].stamp - track.history[0].stamp;
      if (dt > 1e-3) {
        out.valid = true;
        out.position = cv::Point2f(track.history[1].x, out.y);
        out.vx = static_cast<float>((track.history[1].x -
                                     track.history[0].x) / dt);
      }
    }
    return out;
  }

  // Robust least squares over (t - t_mid): x ≈ c0 + c1 t' + c2 t'^2.
  // A plain LSQ lets a single detection glitch (a hit flash hides one
  // light bar and the bbox centre jumps by half a plate) drag the fit so
  // hard that the velocity swings by 100 px/s between frames — every
  // shot then under/over-leads by half a plate. Two IRLS passes drop
  // samples whose residual exceeds 8 px and refit; plate centroids are
  // otherwise good to ~1 px. At most a third of the surviving samples is
  // dropped per pass: plates entering the field are half visible and
  // their centroid sits ~16 px off — dropping every such sample starved
  // the fit below usability and froze the whole pipeline.
  const double t_mid = stamp_sec;
  int n = 0;
  std::vector<char> keep(track.history.size(), 1);
  double c0 = 0, c1 = 0, c2 = 0;
  for (int pass = 0; pass < 3; ++pass) {
    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    double b0 = 0, b1 = 0, b2 = 0;
    n = 0;
    for (size_t idx = 0; idx < track.history.size(); ++idx) {
      if (!keep[idx]) continue;
      const double u = track.history[idx].stamp - t_mid;
      const double u2 = u * u;
      const double x = track.history[idx].x;
      s0 += 1; s1 += u; s2 += u2;
      s3 += u * u2; s4 += u2 * u2;
      b0 += x; b1 += x * u; b2 += x * u2;
      ++n;
    }
    if (n < 4) return out;
    const double det = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) +
                       s2 * (s1 * s3 - s2 * s2);
    if (std::fabs(det) < 1e-6) return out;
    c0 = (b0 * (s2 * s4 - s3 * s3) - s1 * (b1 * s4 - s3 * b2) +
          s2 * (b1 * s3 - s2 * b2)) / det;
    c1 = (s0 * (b1 * s4 - s3 * b2) - b0 * (s1 * s4 - s3 * s2) +
          s2 * (s1 * b2 - b1 * s2)) / det;
    c2 = (s0 * (s2 * b2 - b1 * s2) - s1 * (s1 * b2 - b1 * s1) +
          b0 * (s1 * s3 - s2 * s2)) / det;
    if (pass == 2) break;
    // Drop gross outliers for the next pass, but never more than a
    // third of the surviving samples per pass: plates entering the field
    // are half visible and their centroid sits ~16 px off — dropping
    // every such sample starved the fit and froze the pipeline.
    bool dropped = false;
    size_t kept_now = 0;
    for (size_t idx = 0; idx < track.history.size(); ++idx) {
      if (keep[idx]) ++kept_now;
    }
    size_t drop_budget = kept_now / 3;
    if (drop_budget == 0) break;
    for (size_t idx = 0; idx < track.history.size() && drop_budget > 0;
         ++idx) {
      if (!keep[idx]) continue;
      const double u = track.history[idx].stamp - t_mid;
      const double pred = c0 + c1 * u + c2 * u * u;
      if (std::fabs(pred - track.history[idx].x) > 8.0) {
        keep[idx] = 0;
        dropped = true;
        --drop_budget;
      }
    }
    if (!dropped) break;
  }

  // c2 is 0.5 * ax; reject accelerations the game cannot produce (the
  // cap is +-100 px/s^2 in 超大杯) — those fits are detection glitches.
  const float ax = static_cast<float>(2.0 * c2);
  if (std::fabs(ax) > max_accel_px_s2) {
    // Fall back to a linear fit through the first and last kept samples.
    int first_i = -1, last_i = -1;
    for (size_t idx = 0; idx < track.history.size(); ++idx) {
      if (!keep[idx]) continue;
      if (first_i < 0) first_i = static_cast<int>(idx);
      last_i = static_cast<int>(idx);
    }
    if (first_i < 0) return out;
    const double dt = track.history[last_i].stamp -
                      track.history[first_i].stamp;
    if (dt < 1e-3) return out;
    out.valid = true;
    out.position = cv::Point2f(track.history[last_i].x, out.y);
    out.vx = static_cast<float>((track.history[last_i].x -
                                 track.history[first_i].x) / dt);
    out.ax = 0.0F;
    return out;
  }
  out.valid = true;
  out.position = cv::Point2f(static_cast<float>(c0), out.y);
  out.vx = static_cast<float>(c1);
  out.ax = ax;
  return out;
}

}  // namespace auto_aim
