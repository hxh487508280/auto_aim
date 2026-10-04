#ifndef AUTO_AIM_PLATE_TRACKER_HPP_
#define AUTO_AIM_PLATE_TRACKER_HPP_

#include <array>
#include <deque>
#include <opencv2/core.hpp>
#include <vector>

#include "auto_aim/types.hpp"

namespace auto_aim {

// Persistent identity for detected plates. Plates move on a fixed lane
// (constant y) with constant acceleration along x, so every track carries a
// constant-acceleration Kalman filter on x with state [p, v, a]. CA is the
// game's exact motion model (armor.gd integrates acceleration_x), not an
// approximation of it. The filter is predicted+updated once per matched
// detection; queries extrapolate the stored state without mutating it. The
// position history remains for warm-up seeding, coast bookkeeping and
// diagnostics.
struct TrackedPlate {
  struct Sample {
    double stamp;  // seconds, node clock domain
    float x;
    float y;
  };

  int id = -1;
  TeamColor color = TeamColor::kUnknown;
  bool dead = false;   // grey husk: blocks bullets, scores nothing
  bool seen_alive = false;  // was ever detected as a live (team) plate
  std::deque<Sample> history;
  double last_detect_stamp = 0.0;  // last REAL detection (coast excluded)
  int misses = 0;      // consecutive updates without a match
  int matches = 0;     // total matched detections
  double last_stamp = 0.0;

  // Constant-acceleration Kalman filter on horizontal position. State
  // x = [p, v, a]^T; the game integrates a constant a, so CA is exact.
  // kf_stamp is the time the filter state refers to; covariance is a
  // row-major symmetric 3x3.
  bool kf_init = false;
  double kf_stamp = 0.0;
  float kf_p = 0.0F;
  float kf_v = 0.0F;
  float kf_a = 0.0F;
  std::array<float, 9> kf_cov{};
};

class PlateTracker {
 public:
  struct Params {
    float match_gate_px = 60.0F;  // scaled with dt and speed at runtime
    double retire_after_s = 0.35;  // unseen for longer -> gone
    float max_speed_px_s = 900.0F;
    size_t max_history = 48;       // ~0.8 s at 60 fps
    double min_fit_span_s = 0.10;  // history span needed for a quadratic fit
    double max_fit_span_s = 0.55;  // trim history older than this
    float max_accel_px_s2 = 130.0F;
    // Process noise sigma for the white-jerk CA model. Higher values
    // adapt faster to accel changes but jitter the a estimate more;
    // 400 matches the old 0.55 s batch window on both axes.
    float kf_jerk_std_px_s3 = 400.0F;
    float kf_meas_noise_px = 1.5F;     // measurement noise: centroid sigma
    float kf_gate_px = 8.0F;           // innovation gate (hit-flash jumps)
  };

  // A plate that left tracking this update (exited the field or died and
  // its husk drifted off). `dead` mirrors the track's last known state.
  struct GoneEvent {
    int id = -1;
    TeamColor color = TeamColor::kUnknown;
    bool dead = false;
    float x = 0.0F;
  };

  PlateTracker();
  explicit PlateTracker(const Params& params);

  void Update(const std::vector<ArmorPlate>& detections, double stamp_sec);

  const std::vector<TrackedPlate>& tracks() const { return tracks_; }
  const std::vector<GoneEvent>& gone() const { return gone_; }
  const TrackedPlate* Find(int id) const;
  void Remove(int id);
  void Reset();

  // Kalman state of the track extrapolated to `stamp_sec` without
  // mutating the stored filter (the stored state always refers to the
  // last matched update). Valid once a match has seeded the filter;
  // before that, a linear state from the last two samples (or nothing).
  static PlateState EstimateState(const TrackedPlate& track,
                                  double stamp_sec, float max_accel_px_s2);

 private:
  Params params_;
  std::vector<TrackedPlate> tracks_;
  std::vector<GoneEvent> gone_;
  int next_id_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_PLATE_TRACKER_HPP_
