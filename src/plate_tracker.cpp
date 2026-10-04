#include "auto_aim/plate_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim {

namespace {

using Cov3 = std::array<float, 9>;  // row-major symmetric 3x3

// Init variances for the seeded filter: a two-point velocity at 60 fps
// carries ~90 px/s of centroid noise, and acceleration is unknown when
// the filter is seeded.
constexpr float kInitPosVar = 16.0F;     // (4 px)^2
constexpr float kInitVelVar = 14400.0F;  // (120 px/s)^2
constexpr float kInitAccVar = 16900.0F;  // (130 px/s^2)^2

Cov3 Mat3Mul(const Cov3& a, const Cov3& b) {
  Cov3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float sum = 0.0F;
      for (int k = 0; k < 3; ++k) {
        sum += a[static_cast<size_t>(r) * 3 + k] *
               b[static_cast<size_t>(k) * 3 + c];
      }
      out[static_cast<size_t>(r) * 3 + c] = sum;
    }
  }
  return out;
}

// Standard KF predict: x = F x, P = F P F^T + Q. The process model is CA
// with piecewise-white jerk noise, Q = sigma_j^2 G G^T where
// G = [dt^3/6, dt^2/2, dt]^T. Mutates the track's filter.
void KfPredictTo(TrackedPlate& t, double stamp, float jerk_std) {
  const float dt = static_cast<float>(stamp - t.kf_stamp);
  if (dt <= 0.0F) {
    return;
  }
  const float dt2 = dt * dt;
  t.kf_p += t.kf_v * dt + 0.5F * t.kf_a * dt2;
  t.kf_v += t.kf_a * dt;
  const Cov3 f = {1.0F, dt, 0.5F * dt2,
                  0.0F, 1.0F, dt,
                  0.0F, 0.0F, 1.0F};
  const Cov3 f_transposed = {1.0F, 0.0F, 0.0F,
                             dt, 1.0F, 0.0F,
                             0.5F * dt2, dt, 1.0F};
  const float g[3] = {dt2 * dt / 6.0F, dt2 / 2.0F, dt};
  Cov3 q{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      q[static_cast<size_t>(r) * 3 + c] = jerk_std * jerk_std * g[r] * g[c];
    }
  }
  t.kf_cov = Mat3Mul(Mat3Mul(f, t.kf_cov), f_transposed);
  for (size_t i = 0; i < t.kf_cov.size(); ++i) {
    t.kf_cov[i] += q[i];
  }
  t.kf_stamp = stamp;
}

// Standard KF update for a position-only measurement, H = [1 0 0].
void KfUpdateWith(TrackedPlate& t, float z, float meas_std) {
  const float s = t.kf_cov[0] + meas_std * meas_std;
  if (s < 1e-9F) {
    return;
  }
  const float gain[3] = {t.kf_cov[0] / s, t.kf_cov[3] / s, t.kf_cov[6] / s};
  const float innovation = z - t.kf_p;
  t.kf_p += gain[0] * innovation;
  t.kf_v += gain[1] * innovation;
  t.kf_a += gain[2] * innovation;
  Cov3 cov = t.kf_cov;
  for (int j = 0; j < 3; ++j) {
    const size_t col = static_cast<size_t>(j);
    cov[col] -= gain[0] * t.kf_cov[col];
    cov[3 + col] -= gain[1] * t.kf_cov[col];
    cov[6 + col] -= gain[2] * t.kf_cov[col];
  }
  t.kf_cov = cov;
}

}  // namespace

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

  // Nearest-neighbour matching against each track's Kalman-predicted
  // position. At 60 fps a 700 px/s plate moves ~12 px per frame, so the
  // gate mostly guards against mismatches; it scales with dt for low-fps
  // regimes.
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
    const PlateState st = EstimateState(t, stamp_sec,
                                        params_.max_accel_px_s2);
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
    if (t.kf_init) {
      KfPredictTo(t, stamp_sec, params_.kf_jerk_std_px_s3);
      const float innovation = d.center.x - t.kf_p;
      if (std::fabs(innovation) <= params_.kf_gate_px) {
        KfUpdateWith(t, d.center.x, params_.kf_meas_noise_px);
      }
      // else: the detection is an outlier — a hit-flash bbox jump, not
      // real motion (the old IRLS fit dropped exactly these samples).
      // Keep the predicted state; it still serves association and aiming.
      t.kf_a = std::clamp(t.kf_a, -params_.max_accel_px_s2,
                          params_.max_accel_px_s2);
    } else if (!t.history.empty()) {
      // Seed from the two most recent samples: the same finite-difference
      // velocity the old two-point linear warm-up produced.
      const TrackedPlate::Sample& prev = t.history.back();
      const double dt0 = stamp_sec - prev.stamp;
      if (dt0 > 1e-3) {
        t.kf_p = d.center.x;
        t.kf_v = static_cast<float>((d.center.x - prev.x) / dt0);
        t.kf_a = 0.0F;
        t.kf_cov = {kInitPosVar, 0.0F, 0.0F,
                    0.0F, kInitVelVar, 0.0F,
                    0.0F, 0.0F, kInitAccVar};
        t.kf_stamp = stamp_sec;
        t.kf_init = true;
      }
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
      // flash hides the light bars for ~0.2 s) do not break the state.
      const PlateState st = EstimateState(t, stamp_sec,
                                          params_.max_accel_px_s2);
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

PlateState PlateTracker::EstimateState(const TrackedPlate& track,
                                       double stamp_sec,
                                       float max_accel_px_s2) {
  PlateState out;
  out.stamp = stamp_sec;
  out.y = track.history.empty() ? 0.0F : track.history.back().y;
  if (!track.kf_init) {
    // Warm-up: with exactly two samples report a finite-difference
    // velocity, matching the old two-point linear warm-up.
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

  // Extrapolate the stored Kalman state to the query time. The filter
  // itself is never mutated here: the stored state always refers to
  // kf_stamp, the last matched update.
  const float dt = static_cast<float>(stamp_sec - track.kf_stamp);
  out.valid = true;
  out.position = cv::Point2f(track.kf_p + track.kf_v * dt +
                                 0.5F * track.kf_a * dt * dt,
                             out.y);
  out.vx = track.kf_v + track.kf_a * dt;
  out.ax = std::clamp(track.kf_a, -max_accel_px_s2, max_accel_px_s2);
  return out;
}

}  // namespace auto_aim
