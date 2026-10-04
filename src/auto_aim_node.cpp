#include "auto_aim/auto_aim_node.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace auto_aim {
namespace {
// Wall-clock seconds since an arbitrary epoch (matches steady_clock).
double NowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

AutoAimNode::AutoAimNode(const rclcpp::NodeOptions& options)
    : Node("auto_aim_node", options) {
  DeclareAndLoadParams();

  if (get_parameter("image_qos").as_string() == std::string("reliable")) {
    image_qos_profile_ = rclcpp::QoS(10).reliable();
  }

  CreateImageSubscription();

  heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&AutoAimNode::OnHeartbeat, this));

  debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/auto_aim/debug_image", 1);

  RCLCPP_INFO(get_logger(),
              "auto_aim_node started (intercept core) period=%.2f "
              "clearance=%.0f selweight=short", fire_period_s_,
              corridor_clearance_px_);
  RCLCPP_INFO(get_logger(), "  image_topic=%s serial=%s",
              image_topic_.c_str(), serial_device_.c_str());
}

void AutoAimNode::CreateImageSubscription() {
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos_profile_,
      std::bind(&AutoAimNode::ImageCallback, this, std::placeholders::_1));
}

AutoAimNode::~AutoAimNode() {
  std::lock_guard<std::mutex> lock(fire_thread_mutex_);
  if (fire_thread_.joinable()) {
    fire_thread_.join();
  }
}

void AutoAimNode::DeclareAndLoadParams() {
  declare_parameter<std::string>("image_topic", "/image_raw");
  declare_parameter<std::string>("image_qos", "sensor");
  declare_parameter<bool>("invert_input", false);
  declare_parameter<std::string>("serial_device", "auto");
  declare_parameter<bool>("input_is_rgb", true);
  declare_parameter<bool>("fire_enabled", true);
  declare_parameter<bool>("debug_enabled", true);
  declare_parameter<double>("corridor_clearance_px",
                            game::kCorridorClearancePx);
  declare_parameter<double>("fire_period_s", fire_period_s_);

  image_topic_ = get_parameter("image_topic").as_string();
  invert_input_ = get_parameter("invert_input").as_bool();
  serial_device_ = get_parameter("serial_device").as_string();
  fire_enabled_ = get_parameter("fire_enabled").as_bool();
  debug_enabled_ = get_parameter("debug_enabled").as_bool();
  corridor_clearance_px_ = static_cast<float>(
      get_parameter("corridor_clearance_px").as_double());
  fire_period_s_ =
      static_cast<float>(get_parameter("fire_period_s").as_double());
}

void AutoAimNode::SendTurn(float angle_deg) {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_.IsOpen()) {
    return;
  }
  serial_.SendTurn(angle_deg);
  last_serial_write_ = std::chrono::steady_clock::now();
  last_sent_angle_ = angle_deg;
  last_angle_valid_ = true;
}

bool AutoAimNode::TryFire(int track_id, const InterceptSolution& intercept,
                          double wall_now) {
  // Pacing: the game-side cooldown, plus a per-target spacing so a fresh
  // bullet never lands inside the previous impact's invulnerability
  // window (a bounced bullet is a wasted cooldown).
  if (has_fired_once_) {
    const double since_fire =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_fire_wall_)
            .count();
    if (since_fire < fire_period_s_) {
      return false;
    }
  }
  for (const PendingShot& shot : pending_shots_) {
    if (shot.track_id != track_id) {
      continue;
    }
    const double impact_wall = shot.fire_wall + shot.t_impact;
    if (wall_now + intercept.t_impact <
        impact_wall + game::kInvulnerableS) {
      return false;
    }
  }

  // The serial dance runs on its own thread: turn, fire, then hold the
  // bus quiet until the bullet has spawned (~30 ms after the fire byte).
  // Doing this inline would block the image callback for ~50 ms and age
  // every observation by two extra frames; the turret rotates instantly,
  // so a streamed turn arriving during that window would also swing the
  // barrel away and the shot would fly along the WRONG angle.
  {
    std::lock_guard<std::mutex> lock(fire_thread_mutex_);
    if (fire_thread_.joinable()) {
      fire_thread_.join();
    }
    const float angle = intercept.angle_deg;
    fire_thread_ = std::thread([this, angle]() {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_.IsOpen()) {
        return;
      }
      serial_.SendTurn(angle);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      if (!serial_.SendFire()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      last_serial_write_ = std::chrono::steady_clock::now();
    });
  }
  last_sent_angle_ = intercept.angle_deg;
  last_angle_valid_ = true;

  last_fire_wall_ = std::chrono::steady_clock::now();
  has_fired_once_ = true;
  ++fire_count_;
  pending_shots_.push_back({track_id, NowSeconds(), intercept.t_impact,
                            intercept.angle_deg});
  RCLCPP_INFO(get_logger(),
              "fire id=%d angle=%.1f t_impact=%.2f fires=%d", track_id,
              intercept.angle_deg, intercept.t_impact, fire_count_);
  constexpr double kMaxPendingS = 4.0;
  while (!pending_shots_.empty() &&
         NowSeconds() - pending_shots_.front().fire_wall > kMaxPendingS) {
    pending_shots_.pop_front();
  }
  RCLCPP_INFO(get_logger(),
              "fire id=%d angle=%.1f t_impact=%.2f fires=%d", track_id,
              intercept.angle_deg, intercept.t_impact, fire_count_);
  return true;
}

void AutoAimNode::ObserveShots(double stamp_sec) {
  for (auto it = pending_shots_.begin(); it != pending_shots_.end();) {
    const PendingShot& shot = *it;
    if (stamp_sec < shot.t_impact + 0.06) {
      ++it;
      continue;
    }
    const TrackedPlate* track = tracker_.Find(shot.track_id);
    DamageRecord& rec = damage_[shot.track_id];
    if (track == nullptr) {
      // Retired before this shot landed: the gone-event path owns the
      // lost/killed verdict; drop the shot.
      it = pending_shots_.erase(it);
      continue;
    }
    if (track->dead) {
      if (rec.hits < game::kPlateHitsToKill) {
        rec.hits = game::kPlateHitsToKill;
      }
      if (!rec.killed) {
        rec.killed = true;
        ++kills_estimate_;
        score_estimate_ += 2;
        RCLCPP_INFO(get_logger(), "id=%d killed (score_est=%d kills=%d)",
                    shot.track_id, score_estimate_, kills_estimate_);
      }
      it = pending_shots_.erase(it);
      continue;
    }
    // A hit turns the whole plate grey for ~0.2 s (armor.gd tween), so
    // the detector loses it right around the expected impact. A clean
    // miss leaves it visible: re-detected within ~2 frames. The gap is
    // measured against the last REAL detection (coast samples do not
    // count — they are inserted every missed frame by the tracker).
    const double gap = stamp_sec - track->last_detect_stamp;
    if (gap > game::kHitFlashS * 0.5) {
      ++rec.hits;
      score_estimate_ += 1;
      RCLCPP_DEBUG(get_logger(), "id=%d hit (%d/3, score_est=%d)",
                   shot.track_id, rec.hits, score_estimate_);
    } else {
      RCLCPP_DEBUG(get_logger(), "id=%d miss (gap=%.3f)", shot.track_id,
                   gap);
    }
    it = pending_shots_.erase(it);
  }
}

void AutoAimNode::ConsumeGoneEvents() {
  for (const PlateTracker::GoneEvent& gone : tracker_.gone()) {
    DamageRecord& rec = damage_[gone.id];
    for (auto it = pending_shots_.begin(); it != pending_shots_.end();) {
      if (it->track_id == gone.id) {
        it = pending_shots_.erase(it);
      } else {
        ++it;
      }
    }
    const bool enemy = gone.color == OppositeColor(turret_color_);
    if (!enemy || rec.killed) {
      continue;  // husk drifted out, or already accounted
    }
    if (gone.dead) {
      // Died but the kill was never attributed (attribution gap).
      rec.hits = game::kPlateHitsToKill;
      rec.killed = true;
      ++kills_estimate_;
      score_estimate_ += 2;
      RCLCPP_DEBUG(get_logger(), "id=%d killed (husk gone, score_est=%d)",
                   gone.id, score_estimate_);
    } else {
      rec.hits = game::kPlateHitsToKill;  // stop further attribution
      score_estimate_ -= 1;
      ++losses_estimate_;
      RCLCPP_INFO(get_logger(), "id=%d lost (score_est=%d)", gone.id,
                  score_estimate_);
    }
  }
}

float AutoAimNode::ExitUrgency(const PlateState& state) const {
  const float t_exit =
      solver_.TimeToExit(state, game::kFieldWidth, game::kExitMarginPx);
  if (t_exit > 4.5F) {
    return 0.0F;
  }
  return (4.5F - t_exit) / 4.5F;
}

int AutoAimNode::SelectTarget(const std::vector<TrackedPlate>& tracks,
                              const std::vector<PlateState>& enemy_states,
                              const std::vector<PlateState>& obstacles,
                              double stamp_sec) {
  // Fresh selection every call: each shot goes to the best target at
  // that instant, so two plates can be volleys interleaved when both
  // corridors are clear, and a plate about to leave the field (an enemy
  // exit costs -1) can jump the queue instead of waiting for the
  // current target to die.
  float best_score = -1e9F;
  int best_id = -1;
  for (const auto& state : enemy_states) {
    const TrackedPlate* track = tracker_.Find(state.track_id);
    if (track == nullptr || track->dead) {
      continue;
    }
    const InterceptSolution intercept = solver_.SolveIntercept(
        state, game::kTurretPos, 0.02F, BulletDelay());
    if (!intercept.valid) {
      continue;  // cannot be caught (outrunning the bullet)
    }
    const bool clear = solver_.CorridorClear(
        game::kTurretPos, intercept.angle_deg, intercept.t_impact, obstacles,
        corridor_clearance_px_);
    const DamageRecord& rec = damage_[state.track_id];
    const int hits_left = game::kPlateHitsToKill - rec.hits;
    const float t_exit =
        solver_.TimeToExit(state, game::kFieldWidth, game::kExitMarginPx);
    const bool finishable =
        t_exit > intercept.t_impact +
                      static_cast<float>(std::max(hits_left, 1) - 1) *
                          fire_period_s_;
    // Exit rescue: a plate leaving within ~2.5 s that we can still kill
    // is worth crossing the whole field for (kill +5 vs exit -1 and the
    // lost 5).
    float rescue = 0.0F;
    if (t_exit < 2.5F && finishable) {
      rescue = 5.0F * (1.0F - t_exit / 2.5F);
    }
    float score = -2.8F * intercept.t_impact +
                  2.4F * ExitUrgency(state) +
                  1.5F * (finishable ? 1.0F : 0.0F) +
                  0.8F * static_cast<float>(hits_left) / 3.0F +
                  rescue;
    if (!clear) {
      score -= 6.0F;
    }
    if (state.track_id == locked_id_) {
      score += 0.3F;  // mild stickiness against aim dithering
    }
    if (score > best_score) {
      best_score = score;
      best_id = state.track_id;
    }
  }
  (void)tracks;
  (void)stamp_sec;
  return best_id;
}

bool AutoAimNode::EnsureSerial() {
  const bool use_auto = serial_device_.empty() || serial_device_ == "auto";
  if (serial_.IsOpen()) {
    if (::access(serial_.device_path().c_str(), F_OK) == 0) {
      return true;
    }
    RCLCPP_WARN(get_logger(), "serial device vanished, re-detecting");
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.Close();
  }
  if (use_auto) {
    const std::string found = SerialController::FindGameSerialPath();
    if (found.empty()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_.Open(found)) {
      RCLCPP_INFO(get_logger(), "serial auto -> %s", found.c_str());
      return true;
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_.Open(serial_device_);
}

void AutoAimNode::ResetRoundState() {
  tracker_.Reset();
  pending_shots_.clear();
  damage_.clear();
  score_estimate_ = 0;
  kills_estimate_ = 0;
  losses_estimate_ = 0;
  friendly_hits_ = 0;
  locked_id_ = -1;
  has_fired_once_ = false;
  fire_count_ = 0;
  last_angle_valid_ = false;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_.IsOpen()) {
      serial_.Close();
      RCLCPP_INFO(get_logger(), "round over; serial released");
    }
  }
}

void AutoAimNode::OnHeartbeat() {
  const bool stalled = image_count_ == last_seen_images_;
  if (stalled) {
    if (!idle_since_last_) {
      idle_since_last_ = true;
      ResetRoundState();
    }
    ++image_stall_beats_;
    // The game's publisher only matches subscribers that were up when the
    // round began; once we are blind the only reliable recovery is a new
    // DDS entity. Rebuild every ~4 s while blind (the game itself is
    // still running) — the next round always finds a listening reader.
    if (image_stall_beats_ >= 2) {
      image_stall_beats_ = 0;
      image_sub_.reset();
      CreateImageSubscription();
    }
  } else {
    idle_since_last_ = false;
    image_stall_beats_ = 0;
  }
  last_seen_images_ = image_count_;
}

void AutoAimNode::ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  ++image_count_;
  last_image_wall_ = std::chrono::steady_clock::now();
  const bool declares_rgb =
      msg->encoding == sensor_msgs::image_encodings::RGB8 ||
      msg->encoding == "rgb8";
  try {
    // The simulator declares rgb8 but ships 4 bytes per pixel.
    if (declares_rgb && msg->step == static_cast<size_t>(msg->width) * 4) {
      const cv::Mat rgba(msg->height, msg->width, CV_8UC4,
                         const_cast<uint8_t*>(msg->data.data()), msg->step);
      cv::Mat bgr;
      cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
      if (invert_input_) {
        cv::bitwise_not(bgr, bgr);
      }
      ProcessFrame(bgr, rclcpp::Time(msg->header.stamp).seconds());
      return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        msg, declares_rgb ? sensor_msgs::image_encodings::RGB8
                          : sensor_msgs::image_encodings::BGR8);
    cv::Mat bgr;
    if (declares_rgb) {
      cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = cv_ptr->image;
    }
    ProcessFrame(bgr, rclcpp::Time(msg->header.stamp).seconds());
  } catch (const cv_bridge::Exception& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "cv_bridge exception: %s", ex.what());
  }
}

void AutoAimNode::ReadScore(const cv::Mat& bgr) {
  // The HUD score sits in the bottom-left corner. Digits are solid white
  // glyphs; classify each against the extracted templates.
  static const cv::Rect kScoreRoi(0, 540, 200, 108);
  const cv::Mat crop = bgr(kScoreRoi);
  cv::Mat white;
  cv::inRange(crop, cv::Scalar(190, 190, 190), cv::Scalar(255, 255, 255),
              white);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(white, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  struct Glyph {
    int x;
    int w;
    char ch;
  };
  std::vector<Glyph> glyphs;
  for (const auto& contour : contours) {
    const cv::Rect r = cv::boundingRect(contour);
    if (r.height >= 12 && r.height <= 90 && r.area() > 0) {
      // Resize the glyph to the template grid and match.
      char best = '?';
      double best_iou = 0.0;
      cv::Mat g24;
      cv::resize(white(r), g24, cv::Size(24, 40));
      for (const auto& tpl : score_ocr::kDigitTemplates) {
        const char digit = tpl.digit;
        const uint32_t* words = tpl.bits;
        int inter = 0;
        int union_count = 0;
        for (int row = 0; row < 40; ++row) {
          const uint32_t bits = words[row];
          for (int col = 0; col < 24; ++col) {
            const bool on = g24.at<uint8_t>(row, col) > 127;
            const bool tpl = ((bits >> col) & 1u) != 0;
            inter += on && tpl;
            union_count += on || tpl;
          }
        }
        const double iou =
            static_cast<double>(inter) / std::max(union_count, 1);
        if (iou > best_iou) {
          best_iou = iou;
          best = digit;
        }
      }
      if (best_iou > 0.6) {
        glyphs.push_back({r.x, r.width, best});
      }
    } else if (r.height >= 6 && r.height <= 16 && r.width >= 12 &&
               r.width <= 40) {
      glyphs.push_back({r.x, r.width, '-'});
    }
  }
  if (glyphs.empty()) {
    return;
  }
  std::sort(glyphs.begin(), glyphs.end(),
            [](const Glyph& a, const Glyph& b) { return a.x < b.x; });
  std::string text;
  for (const auto& g : glyphs) {
    text += g.ch;
  }
  int value = 0;
  try {
    value = std::stoi(text);
  } catch (const std::exception&) {
    return;
  }
  if (hud_valid_ && value != hud_score_) {
    score_deltas_.emplace_back(NowSeconds(), value - hud_score_);
    if (score_deltas_.size() > 32) {
      score_deltas_.erase(score_deltas_.begin());
    }
  }
  hud_score_ = value;
  hud_valid_ = true;
}

void AutoAimNode::AttributeScoreDeltas(double stamp_sec) {
  // Attribute each observed score delta to the in-flight shot whose
  // expected impact is nearest in time: +1 = hit, +3 = killing hit
  // (final damage +1 and the kill bonus +2 land in one frame), -1 = an
  // enemy plate left the field, -5 = a friendly was hit (corridor
  // leak). This is ground truth from the HUD, not an inference.
  if (score_deltas_.empty()) {
    return;
  }
  for (auto dit = score_deltas_.begin(); dit != score_deltas_.end();) {
    const double t_delta = dit->first;
    const int delta = dit->second;
    bool consumed = false;
    if (delta == 1 || delta == 3) {
      PendingShot* best = nullptr;
      double best_gap = 0.35;
      for (auto& shot : pending_shots_) {
        const double impact = shot.fire_wall + shot.t_impact;
        const double gap = std::fabs(impact - t_delta);
        if (gap < best_gap) {
          best_gap = gap;
          best = &shot;
        }
      }
      if (best != nullptr) {
        DamageRecord& rec = damage_[best->track_id];
        if (delta == 3) {
          rec.hits = game::kPlateHitsToKill;
          if (!rec.killed) {
            rec.killed = true;
            ++kills_estimate_;
            score_estimate_ += 2;
          }
        } else if (rec.hits < game::kPlateHitsToKill) {
          ++rec.hits;
        }
        score_estimate_ += 1;
        consumed = true;
        RCLCPP_DEBUG(get_logger(),
                     "shot on id=%d scored %+d (hits=%d killed=%d)",
                     best->track_id, delta, rec.hits, rec.killed);
        // drop this pending shot: it has produced its outcome
        for (auto it = pending_shots_.begin(); it != pending_shots_.end();
             ++it) {
          if (&(*it) == best) {
            pending_shots_.erase(it);
            break;
          }
        }
      }
    } else if (delta == -1) {
      // A plate left the field: identify it as the nearest enemy track
      // heading out; the selection loop will naturally move on.
      ++losses_estimate_;
      score_estimate_ -= 1;
      consumed = true;
    } else if (delta == -5) {
      ++friendly_hits_;
      score_estimate_ -= 5;
      RCLCPP_WARN(get_logger(), "friendly hit! (corridor leak)");
      consumed = true;
    } else if (delta == -10) {
      friendly_hits_ += 2;
      score_estimate_ -= 10;
      RCLCPP_WARN(get_logger(), "friendly KILLED (corridor leak)");
      consumed = true;
    }
    if (consumed) {
      dit = score_deltas_.erase(dit);
    } else {
      ++dit;
    }
  }
  (void)stamp_sec;
}

float AutoAimNode::BulletDelay() {
  // Observation age (EMA of the callback period approximates how stale
  // the frame was when it reached us on a saturated pipeline) + the
  // serial read and deferred-spawn delay inside the game (~30 ms
  // measured end to end).
  return static_cast<float>(frame_age_ema_) + 0.020F;
}

void AutoAimNode::ProcessFrame(const cv::Mat& bgr, double stamp_sec) {
  ++process_count_;
  const double wall_now = NowSeconds();
  if (last_callback_wall_ > 0.0) {
    const double period = wall_now - last_callback_wall_;
    if (period > 0.0 && period < 0.5) {
      frame_age_ema_ = 0.7 * frame_age_ema_ + 0.3 * period;
    }
  }
  last_callback_wall_ = wall_now;
  if (stamp_sec <= 0.0) {
    stamp_sec = wall_now;
  }

  std::vector<ArmorPlate> plates;
  TurretState turret;
  detector_.Detect(bgr, false, &plates, &turret);
  if (!turret.valid) {
    return;  // menu / result screen
  }
  if (turret.color != turret_color_) {
    // New round (or first frame): colour decides friend from foe.
    if (turret_color_ != TeamColor::kUnknown) {
      ResetRoundState();
    }
    turret_color_ = turret.color;
  }

  tracker_.Update(plates, stamp_sec);
  ReadScore(bgr);
  AttributeScoreDeltas(stamp_sec);
  ObserveShots(stamp_sec);
  ConsumeGoneEvents();

  const TeamColor enemy = OppositeColor(turret_color_);
  if (enemy == TeamColor::kUnknown) {
    return;
  }

  // Fit every track at the current stamp. The observation is already
  // `frame_age` old by the time we act on it, and the bullet spawns
  // after the serial round-trip: the intercept solve must look that far
  // into the plate's future, or every shot under-leads by exactly the
  // distance the plate covers in that window (tens of px on fast lanes).
  std::vector<PlateState> enemy_states;
  std::vector<PlateState> obstacles;  // friendlies + husks block bullets
  const auto& tracks = tracker_.tracks();
  for (const auto& track : tracks) {
    PlateState state = PlateTracker::EstimateState(
        track, stamp_sec, 2.0F * game::kMaxPlateAccelPxS2);
    if (!state.valid) {
      continue;
    }
    if (!track.dead && track.color == enemy) {
      state.track_id = track.id;
      enemy_states.push_back(state);
    }
    if (track.dead || track.color == turret_color_) {
      state.track_id = track.id;
      // Uncertainty inflation: a friendly/husk we have barely observed or
      // that is coasting right now moves less predictably than the fit
      // admits, so widen its corridor radius.
      if (track.matches < 5 || track.misses > 0) {
        state.margin = 24.0F;
      }
      obstacles.push_back(state);
    }
  }

  const int target_id =
      SelectTarget(tracks, enemy_states, obstacles, stamp_sec);
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                        "state: enemies=%zu obstacles=%zu target=%d "
                        "turret=%s age=%.2f",
                        enemy_states.size(), obstacles.size(), target_id,
                        ToString(turret_color_), frame_age_ema_);
  if (target_id < 0) {
    locked_id_ = -1;
    return;
  }
  if (target_id != locked_id_) {
    if (locked_id_ >= 0) {
      RCLCPP_DEBUG(get_logger(), "unlock id=%d (state lost)", locked_id_);
    }
    locked_id_ = target_id;
    RCLCPP_DEBUG(get_logger(), "lock id=%d", target_id);
  }

  PlateState target;
  for (const auto& state : enemy_states) {
    if (state.track_id == target_id) {
      target = state;
      break;
    }
  }
  if (!target.valid) {
    return;
  }
  const InterceptSolution intercept = solver_.SolveIntercept(
      target, game::kTurretPos, 0.02F, BulletDelay());
  if (!intercept.valid) {
    return;
  }

  if (!EnsureSerial()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "serial not open");
    return;
  }

  // Fire gate: corridor must stay free of friendlies and husks for the
  // whole bullet flight; the game drops fire bytes inside its cooldown,
  // which TryFire paces with margin.
  bool clear = solver_.CorridorClear(game::kTurretPos, intercept.angle_deg,
                                     intercept.t_impact, obstacles,
                                     corridor_clearance_px_);
  bool fired = false;
  if (fire_enabled_ && clear) {
    fired = TryFire(target_id, intercept, wall_now);
  }
  if (!fired && (!last_angle_valid_ ||
                 std::fabs(intercept.angle_deg - last_sent_angle_) > 0.15F)) {
    SendTurn(intercept.angle_deg);
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "score hud=%d est=%d kills=%d lost=%d fires=%d",
                       hud_score_, score_estimate_, kills_estimate_,
                       losses_estimate_, fire_count_);
  if (debug_enabled_ && process_count_ % 30 == 0) {
    PublishDebug(bgr, &intercept.point, clear, target_id);
  }
}

void AutoAimNode::PublishDebug(const cv::Mat& bgr,
                               const cv::Point2f* aim_point, bool clear,
                               int locked_id) {
  if (!debug_pub_) {
    return;
  }
  cv::Mat dbg = bgr.clone();
  const cv::Scalar turret_color =
      turret_color_ == TeamColor::kRed ? cv::Scalar(0, 0, 255)
                                       : cv::Scalar(255, 0, 0);
  cv::circle(dbg, game::kTurretPos, 14, turret_color, 2);
  if (aim_point != nullptr) {
    const cv::Scalar path_color =
        clear ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::line(dbg, game::kTurretPos, *aim_point, path_color, 2);
    cv::circle(dbg, *aim_point, 6, path_color, 2);
  }
  for (const auto& track : tracker_.tracks()) {
    if (track.history.empty()) {
      continue;
    }
    const cv::Scalar c = track.dead ? cv::Scalar(128, 128, 128)
                                    : cv::Scalar(0, 255, 255);
    cv::rectangle(dbg,
                  cv::Point(static_cast<int>(track.history.back().x) - 32,
                            static_cast<int>(track.history.back().y) - 16),
                  cv::Point(static_cast<int>(track.history.back().x) + 32,
                            static_cast<int>(track.history.back().y) + 16),
                  c, 1);
    cv::putText(dbg, std::to_string(track.id),
                cv::Point(static_cast<int>(track.history.back().x) - 10,
                          static_cast<int>(track.history.back().y) - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, c, 1);
  }  char status[96];
  std::snprintf(status, sizeof(status), "id=%d est=%d f=%d", locked_id,
                score_estimate_, fire_count_);
  cv::putText(dbg, status, cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 255, 0), 1);
  std_msgs::msg::Header header;
  header.frame_id = "game";
  header.stamp = now();
  debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", dbg).toImageMsg());
}

}  // namespace auto_aim
