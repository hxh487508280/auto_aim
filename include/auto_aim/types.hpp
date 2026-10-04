#ifndef AUTO_AIM_TYPES_HPP_
#define AUTO_AIM_TYPES_HPP_

#include <opencv2/core.hpp>
#include <limits>
#include <string>
#include <vector>

namespace auto_aim {

enum class TeamColor { kUnknown = 0, kRed = 1, kBlue = 2, kDead = 3 };

// One plate as seen by the detector in a single frame.
struct ArmorPlate {
  cv::Point2f center;
  cv::Rect bbox;
  TeamColor color = TeamColor::kUnknown;
  bool alive = true;  // false for grey husks (third hit landed)
};

// Turret state. The turret never moves; only its colour matters.
struct TurretState {
  cv::Point2f position;
  TeamColor color = TeamColor::kUnknown;
  bool valid = false;
};

// Constant-acceleration state of one plate along x, fitted from the
// recent position history. Plates move on a fixed lane (y constant) with
// x(t) = x0 + vx t + 0.5 ax t^2, so this is the exact motion model of the
// game rather than an approximation.
struct PlateState {
  bool valid = false;
  int track_id = -1;
  cv::Point2f position{0.0F, 0.0F};  // fitted position at the query time
  float vx = 0.0F;                   // px/s at the query time
  float ax = 0.0F;                   // px/s^2 (0 when fit is linear)
  float y = 0.0F;                    // lane height (median of history)
  double stamp = 0.0;                // time the state refers to
  // Extra corridor radius for this obstacle: grows when its state is
  // stale or thinly observed (coasting frames, short history).
  float margin = 0.0F;
};

// Intercept of a bullet with one plate.
struct InterceptSolution {
  bool valid = false;
  float t_impact = 0.0F;   // seconds from now until the bullet lands
  float angle_deg = 0.0F;  // serial-protocol angle to command
  cv::Point2f point{0.0F, 0.0F};
};

struct AimCommand {
  float angle_deg = 0.0F;
  bool should_fire = false;
  bool has_target = false;
};

inline const char* ToString(TeamColor color) {
  switch (color) {
    case TeamColor::kRed:
      return "red";
    case TeamColor::kBlue:
      return "blue";
    case TeamColor::kDead:
      return "dead";
    default:
      return "unknown";
  }
}

inline TeamColor OppositeColor(TeamColor color) {
  if (color == TeamColor::kRed) {
    return TeamColor::kBlue;
  }
  if (color == TeamColor::kBlue) {
    return TeamColor::kRed;
  }
  return TeamColor::kUnknown;
}

}  // namespace auto_aim

#endif  // AUTO_AIM_TYPES_HPP_
