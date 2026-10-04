#ifndef AUTO_AIM_GAME_CONSTANTS_HPP_
#define AUTO_AIM_GAME_CONSTANTS_HPP_

#include <opencv2/core.hpp>

namespace auto_aim {
// Physics constants of the homework2026 simulator, measured against the
// running game (bullet tracking at 60 fps) and confirmed by decompiling the
// shipped game scripts. Every aiming computation below depends on these; a
// wrong value here is a systematic miss on every shot.
namespace game {

// Bullet speed in px/s (turret.gd: bullet_speed = 600.0).
constexpr float kBulletSpeedPxS = 600.0F;
// Bullets spawn this far from the turret centre along the barrel
// (turret.gd: muzzle_offset = 80.0).
constexpr float kMuzzleOffsetPx = 80.0F;
// The game parses one serial command per ~10 ms read and drops fire
// commands within its cooldown (measured: a shot lands every ~0.30 s).
constexpr float kFireCooldownS = 0.30F;
// Margin added to the cooldown so a fire byte never lands inside the
// game-side gate (a dropped byte costs a full cooldown of scoring time).
constexpr float kFireCooldownMarginS = 0.02F;
// Delay between the fire byte landing and the bullet spawning (serial
// read granularity + Godot deferred call).
constexpr float kFireLatencyS = 0.012F;
// A plate is invulnerable for ~0.2 s after each hit (armor.gd hit tween);
// a bullet arriving inside that window bounces off and is wasted.
constexpr float kInvulnerableS = 0.22F;
// Plate collision box: 64 x 32 px centred on the plate (armor.tscn
// RectangleShape2D). Bullet radius adds a little slack.
constexpr float kPlateHalfWidth = 32.0F;
constexpr float kPlateHalfHeight = 16.0F;
// Corridor clearance: half-diagonal of the plate box plus bullet radius.
constexpr float kCorridorClearancePx = 50.0F;
// Turret anchor in the 1152x648 field. The turret never moves; every
// bullet's flight line extended backwards passes through this point
// (measured over 63 straight flights: y = 612 +- 1). The scene file's
// node position (576, 656) is NOT the bullet origin — the turret mesh
// sits above it and the muzzle spawns 80 px along the barrel from the
// node origin, whose pivot is higher.
inline const cv::Point2f kTurretPos(576.0F, 612.0F);
constexpr float kFieldWidth = 1152.0F;
constexpr float kFieldHeight = 648.0F;
// Plates are freed once they leave the field by this margin
// (armor.gd: position.x < -300 || > width + 300) and an enemy plate that
// exits costs -1 point.
constexpr float kExitMarginPx = 300.0F;
// 超大杯 plates accelerate at up to +-100 px/s^2 (main.gd HARD branch);
// fits beyond this are detection glitches, not real motion.
constexpr float kMaxPlateAccelPxS2 = 130.0F;
// Plates spawn with 100..300 px/s and can accelerate for tens of seconds;
// the observed maximum over a full round was ~740 px/s.
constexpr float kMaxPlateSpeedPxS = 900.0F;
// Plates need three hits; the third turns them into a grey husk that
// still blocks bullets until it drifts off the field.
constexpr int kPlateHitsToKill = 3;
// Hit flash / invulnerability window (armor.gd tween: 0.1 s + 0.1 s).
constexpr float kHitFlashS = 0.2F;

}  // namespace game
}  // namespace auto_aim

#endif  // AUTO_AIM_GAME_CONSTANTS_HPP_
