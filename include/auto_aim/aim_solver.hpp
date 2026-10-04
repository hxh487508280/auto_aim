#ifndef AUTO_AIM_AIM_SOLVER_HPP_
#define AUTO_AIM_AIM_SOLVER_HPP_

#include <opencv2/core.hpp>
#include <vector>

#include "auto_aim/types.hpp"

namespace auto_aim {

// Exact ballistic interception against the simulator's motion model.
//
// The plate moves along its lane with constant acceleration:
//   x_p(t) = x0 + vx t + 0.5 ax t^2,  y_p fixed
// The bullet leaves the turret anchor at (576, 656) with speed 600 px/s
// after a muzzle offset of 80 px along the barrel, and travels in a
// straight line (no gravity, verified at 60 fps). Aiming at intercept
// point (xi, y_p), the bullet is airborne for (|xi - T| - muzzle)/speed,
// so the intercept condition couples the plate's motion with the circle
// of reach around the turret:
//   (x_p(t) - T.x)^2 + (T.y - y_p)^2 == (muzzle + speed * (t - eps))^2
// SolveSweep samples this residual and bisects the earliest root.
class AimSolver {
 public:
  struct Params {
    float bullet_speed_px_s = 600.0F;
    float muzzle_offset_px = 80.0F;
    // Intercept search window; long shots are worthless against plates
    // that reverse, and the fit degrades over multi-second horizons.
    float max_impact_s = 2.0F;
    int sweep_samples = 64;
  };

  AimSolver();
  explicit AimSolver(const Params& params);

  // Earliest intercept at or after `t_min` seconds from the state's
  // reference time. `bullet_delay_s` is the age of the observation (frame
  // staleness) plus the serial/deferred-spawn delay: the plate keeps
  // moving through all of it, and the bullet only exists from that
  // moment. Returns an invalid solution when the plate outruns the
  // bullet or the horizon is exceeded.
  InterceptSolution SolveIntercept(const PlateState& plate,
                                   const cv::Point2f& turret,
                                   float t_min, float bullet_delay_s) const;

  // Time until the plate leaves the field (x < -300 or x > width+300).
  // Returns a large value when it never exits under the fitted motion.
  float TimeToExit(const PlateState& plate, float field_width,
                   float margin_px) const;

  // True when a bullet aimed along `angle_deg` reaches the plate at
  // `t_impact` without passing within `clearance_px` of any obstacle.
  // Obstacles are friendlies and dead husks: both block the bullet; the
  // former costs -5 points, the latter wastes the shot. Their motion
  // uses the same constant-acceleration model as the target.
  bool CorridorClear(const cv::Point2f& turret, float angle_deg,
                     float t_impact,
                     const std::vector<PlateState>& obstacles,
                     float clearance_px) const;

  // Serial-protocol angle (degrees, 0 = image right, up positive) from
  // the turret to a point.
  static float ToAngleDeg(const cv::Point2f& from, const cv::Point2f& to);

 private:
  float PlateXAt(const PlateState& plate, float t) const;

  Params params_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_AIM_SOLVER_HPP_
