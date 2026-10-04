#ifndef AUTO_AIM_AUTO_AIM_NODE_HPP_
#define AUTO_AIM_AUTO_AIM_NODE_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "auto_aim/aim_solver.hpp"
#include "auto_aim/armor_detector.hpp"
#include "auto_aim/game_constants.hpp"
#include "auto_aim/plate_tracker.hpp"
#include "auto_aim/score_ocr.hpp"
#include "auto_aim/serial_controller.hpp"
#include "auto_aim/types.hpp"

namespace auto_aim {

// Vision + control node for the homework2026 auto-aim simulator.
//
// The control core is built directly on the simulator's mechanics (all
// verified against the running game and its decompiled scripts): plates
// move with constant acceleration on a fixed lane, bullets fly straight
// at 600 px/s from 80 px ahead of the turret anchor, the turret applies
// turn commands instantly, fire commands are gated by a ~0.30 s cooldown
// on the game side, and a plate is invulnerable for ~0.2 s after each
// hit. The node therefore solves exact interception per frame instead of
// filtering and extrapolating.
class AutoAimNode : public rclcpp::Node {
 public:
  explicit AutoAimNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AutoAimNode() override;

 private:
  // One bullet in flight toward a plate: the intercept we solved at fire
  // time. Used for hit attribution and shot pacing (a plate is blind for
  // ~0.2 s after each hit, so a second bullet landing inside that window
  // bounces off).
  struct PendingShot {
    int track_id;
    double fire_wall;    // node clock when the byte was sent
    double t_impact;     // seconds after fire_wall
    float aim_deg;
  };

  // Per-plate damage bookkeeping (hits attributed, kill observed).
  struct DamageRecord {
    int hits = 0;
    bool killed = false;
  };

  void DeclareAndLoadParams();
  void CreateImageSubscription();
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void ProcessFrame(const cv::Mat& bgr, double stamp_sec);
  void OnHeartbeat();
  bool EnsureSerial();
  void SendTurn(float angle_deg);
  bool TryFire(int track_id, const InterceptSolution& intercept,
               double wall_now);
  void ObserveShots(double stamp_sec);
  void ConsumeGoneEvents();
  int SelectTarget(const std::vector<TrackedPlate>& tracks,
                   const std::vector<PlateState>& enemy_states,
                   const std::vector<PlateState>& obstacles,
                   double stamp_sec);
  void ResetRoundState();
  void PublishDebug(const cv::Mat& bgr, const cv::Point2f* aim_point,
                    bool clear, int locked_id);
  float ExitUrgency(const PlateState& state) const;
  float BulletDelay();
  void ReadScore(const cv::Mat& bgr);
  void AttributeScoreDeltas(double stamp_sec);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::QoS image_qos_profile_ = rclcpp::SensorDataQoS();
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

  // Parameters.
  std::string image_topic_;
  std::string serial_device_;
  bool invert_input_ = false;
  bool fire_enabled_ = true;
  bool debug_enabled_ = true;
  float corridor_clearance_px_ = game::kCorridorClearancePx;
  float fire_period_s_ = game::kFireCooldownS + game::kFireCooldownMarginS;

  ArmorDetector detector_;
  PlateTracker tracker_;
  AimSolver solver_;
  SerialController serial_;
  std::mutex serial_mutex_;
  std::mutex fire_thread_mutex_;
  std::thread fire_thread_;
  std::chrono::steady_clock::time_point last_serial_write_{};

  TeamColor turret_color_ = TeamColor::kUnknown;

  // Observation staleness: EMA of the callback period. The frame being
  // processed on a saturated pipeline is already roughly one period old;
  // the intercept solve compensates for it (see BulletDelay).
  double frame_age_ema_ = 0.02;
  double last_callback_wall_ = 0.0;

  size_t image_count_ = 0;
  size_t process_count_ = 0;
  size_t last_seen_images_ = 0;
  int image_stall_beats_ = 0;
  bool idle_since_last_ = false;
  std::chrono::steady_clock::time_point last_image_wall_{};

  // Fire control state.
  std::deque<PendingShot> pending_shots_;
  std::chrono::steady_clock::time_point last_fire_wall_{};
  bool has_fired_once_ = false;
  int fire_count_ = 0;

  // Damage / score bookkeeping, reset every round.
  std::unordered_map<int, DamageRecord> damage_;
  int score_estimate_ = 0;
  int kills_estimate_ = 0;
  int losses_estimate_ = 0;
  int friendly_hits_ = 0;

  // Ground-truth scoring: the HUD score is readable in every frame, so
  // hits, kills and lost plates are OBSERVED rather than inferred. Each
  // score delta is attributed to the in-flight shot whose expected
  // impact is nearest in time.
  int hud_score_ = 0;
  bool hud_valid_ = false;
  std::vector<std::pair<double, int>> score_deltas_;

  // Target lock.
  int locked_id_ = -1;
  int lock_grace_ = 0;

  float last_sent_angle_ = 0.0F;
  bool last_angle_valid_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_AUTO_AIM_NODE_HPP_
