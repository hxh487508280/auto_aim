#include "auto_aim/aim_solver.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim {

AimSolver::AimSolver() : AimSolver(Params{}) {}

AimSolver::AimSolver(const Params& params) : params_(params) {}

float AimSolver::ToAngleDeg(const cv::Point2f& from, const cv::Point2f& to) {
  const float dx = to.x - from.x;
  const float dy = to.y - from.y;
  // Image y grows downward; the game's 0 deg points right and positive
  // angles turn up (verified: commanding +90 points the barrel north).
  float angle = std::atan2(-dy, dx) * 180.0F / static_cast<float>(CV_PI);
  if (angle > 180.0F) {
    angle -= 360.0F;
  }
  if (angle < -180.0F) {
    angle += 360.0F;
  }
  return angle;
}

float AimSolver::PlateXAt(const PlateState& plate, float t) const {
  return plate.position.x + plate.vx * t +
         0.5F * plate.ax * t * t;
}

InterceptSolution AimSolver::SolveIntercept(const PlateState& plate,
                                            const cv::Point2f& turret,
                                            float t_min,
                                            float bullet_delay_s) const {
  InterceptSolution out;
  if (!plate.valid) {
    return out;
  }
  const float dy = turret.y - plate.position.y;
  if (dy <= 0.0F) {
    return out;  // plates never drop below the turret in this game
  }
  const float reach0 = params_.muzzle_offset_px;
  // f(t) = (x_p(t) - T.x)^2 + dy^2 - (reach(t))^2; a root is an instant
  // where the plate sits exactly on the bullet's reach circle. t counts
  // from the state's reference frame; the bullet only exists from
  // fire_latency_s onwards (transport + serial + deferred spawn).
  auto residual = [&](float t) {
    const float dx = PlateXAt(plate, t) - turret.x;
    const float reach = reach0 + params_.bullet_speed_px_s *
                                   std::max(0.0F, t - bullet_delay_s);
    return dx * dx + dy * dy - reach * reach;
  };

  const float t_hi = params_.max_impact_s;
  const int n = params_.sweep_samples;
  float t_prev = t_min;
  float f_prev = residual(t_prev);
  for (int i = 1; i <= n; ++i) {
    const float t = t_min + (t_hi - t_min) * static_cast<float>(i) /
                              static_cast<float>(n);
    const float f = residual(t);
    if ((f_prev <= 0.0F && f >= 0.0F) || (f_prev >= 0.0F && f <= 0.0F)) {
      // Bisect between t_prev and t. Prefer the side where the residual
      // approaches zero from the reach-limited region (f < 0 means the
      // plate is inside the circle: catchable this instant).
      float lo = t_prev, hi = t, flo = f_prev;
      for (int k = 0; k < 24; ++k) {
        const float mid = 0.5F * (lo + hi);
        const float fm = residual(mid);
        if ((fm <= 0.0F) == (flo <= 0.0F)) {
          lo = mid;
          flo = fm;
        } else {
          hi = mid;
        }
      }
      const float t_root = 0.5F * (lo + hi);
      if (t_root < t_min - 1e-3F) {
        continue;
      }
      cv::Point2f point(PlateXAt(plate, t_root), plate.position.y);
      out.valid = true;
      out.t_impact = t_root;
      out.point = point;
      out.angle_deg = ToAngleDeg(turret, point);
      return out;
    }
    t_prev = t;
    f_prev = f;
  }
  return out;
}

float AimSolver::TimeToExit(const PlateState& plate, float field_width,
                            float margin_px) const {
  if (!plate.valid) {
    return 1e9F;
  }
  // Solve x(t) = -margin and x(t) = width + margin; smallest positive root.
  const float a = 0.5F * plate.ax;
  const float b = plate.vx;
  auto earliest_root = [&](float target) {
    // a t^2 + b t + (x0 - target) = 0
    const float c = plate.position.x - target;
    if (std::fabs(a) < 1e-3F) {
      if (std::fabs(b) < 1e-3F) {
        return 1e9F;
      }
      const float t = -c / b;
      return t > 0.0F ? t : 1e9F;
    }
    const float disc = b * b - 4.0F * a * c;
    if (disc < 0.0F) {
      return 1e9F;  // reverses before reaching the boundary
    }
    const float sq = std::sqrt(disc);
    const float t1 = (-b - sq) / (2.0F * a);
    const float t2 = (-b + sq) / (2.0F * a);
    float best = 1e9F;
    if (t1 > 0.0F) best = t1;
    if (t2 > 0.0F) best = std::min(best, t2);
    return best;
  };
  return std::min(earliest_root(-margin_px),
                  earliest_root(field_width + margin_px));
}

bool AimSolver::CorridorClear(const cv::Point2f& turret, float angle_deg,
                              float t_impact,
                              const std::vector<PlateState>& obstacles,
                              float clearance_px) const {
  if (obstacles.empty()) {
    return true;
  }
  const float rad = angle_deg * static_cast<float>(CV_PI) / 180.0F;
  const float dir_x = std::cos(rad);
  // Screen y grows downward; a positive command angle points UP.
  const float dir_y = -std::sin(rad);
  // A bullet that misses keeps flying: cover the ray ~0.3 s (≈180 px)
  // beyond the intercept, where a friendly trailing the target eats the
  // overshoot. This used to be the dominant friendly-fire hole.
  const float t_end = t_impact + 0.3F;
  // How far the ray extends before leaving the field bounds.
  const float r_max = 1.6F * (turret.y - 0.0F);

  constexpr int kSteps = 48;
  for (int i = 0; i <= kSteps; ++i) {
    const float t = t_end * static_cast<float>(i) /
                    static_cast<float>(kSteps);
    const float travel = params_.muzzle_offset_px +
                         params_.bullet_speed_px_s * std::max(0.0F, t);
    if (travel > r_max) {
      break;
    }
    const cv::Point2f b(turret.x + dir_x * travel,
                        turret.y + dir_y * travel);
    for (const auto& ob : obstacles) {
      if (!ob.valid) {
        continue;
      }
      const float ox = PlateXAt(ob, t);
      const float dx = ox - b.x;
      const float dy = ob.position.y - b.y;
      const float clr = clearance_px + ob.margin;
      if (dx * dx + dy * dy < clr * clr) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace auto_aim
