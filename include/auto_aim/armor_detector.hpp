#ifndef AUTO_AIM_ARMOR_DETECTOR_HPP_
#define AUTO_AIM_ARMOR_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <vector>

#include "auto_aim/types.hpp"

namespace auto_aim {

class ArmorDetector {
 public:
  struct Params {
    int hsv_red_low1 = 0;
    int hsv_red_high1 = 10;
    int hsv_red_low2 = 160;
    int hsv_red_high2 = 180;
    int hsv_blue_low = 90;
    int hsv_blue_high = 130;
    int color_s_min = 80;
    int color_v_min = 60;
    int min_marker_area = 20;
    int max_marker_area = 8000;
    float marker_max_pair_dist = 80.0F;
    int min_turret_area = 200;
    // Grey husk detection (armor.gd Team.DEAD color ~ (145,145,145)).
    int husk_v_min = 80;
    int husk_v_max = 200;
    int husk_s_max = 60;
    int min_husk_area = 400;
  };

  ArmorDetector();
  explicit ArmorDetector(const Params& params);

  // Detects live plates (team colour), dead husks (grey) and the turret
  // colour. Plates and husks come back in `plates`; husks carry
  // color == TeamColor::kDead. The turret never moves, so its position is
  // the game constant; only its colour is measured from the image.
  void Detect(const cv::Mat& bgr_or_rgb, bool input_is_rgb,
              std::vector<ArmorPlate>* plates, TurretState* turret);

  const cv::Mat& debug_mask() const { return debug_mask_; }

 private:
  cv::Mat BuildTeamMask(const cv::Mat& hsv, TeamColor color) const;
  std::vector<cv::Rect> FindMarkers(const cv::Mat& mask) const;
  std::vector<ArmorPlate> BuildPlates(const std::vector<cv::Rect>& markers,
                                      TeamColor color) const;
  TeamColor DetectTurretColor(const cv::Mat& bgr) const;

  Params params_;
  cv::Mat debug_mask_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_ARMOR_DETECTOR_HPP_
