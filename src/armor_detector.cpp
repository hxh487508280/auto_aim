#include "auto_aim/armor_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

#include "auto_aim/game_constants.hpp"

namespace auto_aim {

ArmorDetector::ArmorDetector() : ArmorDetector(Params{}) {}

ArmorDetector::ArmorDetector(const Params& params) : params_(params) {}

void ArmorDetector::Detect(const cv::Mat& image, bool input_is_rgb,
                           std::vector<ArmorPlate>* plates,
                           TurretState* turret) {
  plates->clear();
  if (turret != nullptr) {
    *turret = TurretState{};
    turret->position = game::kTurretPos;
  }
  if (image.empty()) {
    return;
  }

  cv::Mat bgr;
  if (input_is_rgb) {
    cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
  } else {
    bgr = image;
  }

  // The turret strip (bottom of the field) decides our team colour. It is
  // tiny, so run it separately — this also keeps the turret's own
  // barrel+base blob out of the plate search (it is exactly plate-sized
  // and poisoned the corridor checks when it leaked in).
  const int strip_y =
      std::min(bgr.rows - 2, static_cast<int>(bgr.rows * 0.78F));
  if (turret != nullptr) {
    turret->color = DetectTurretColor(
        bgr(cv::Rect(0, strip_y, bgr.cols, bgr.rows - strip_y)));
    turret->valid = turret->color != TeamColor::kUnknown;
  }

  // Plate search region: plates roam y in [80, 448] and stand 32 px tall,
  // so a strip from y=40 down to y=510 covers them everywhere. Full
  // resolution is mandatory: the light bars are only ~4x17 px and every
  // downscale we tried lost them (and with them every plate).
  const int field_y = 40;
  const int field_h = std::max(1, std::min(bgr.rows - field_y, 470));
  const cv::Mat field = bgr(cv::Rect(0, field_y, bgr.cols, field_h));
  cv::Mat hsv;
  cv::cvtColor(field, hsv, cv::COLOR_BGR2HSV);

  const cv::Mat red_mask = BuildTeamMask(hsv, TeamColor::kRed);
  const cv::Mat blue_mask = BuildTeamMask(hsv, TeamColor::kBlue);
  debug_mask_ = red_mask | blue_mask;

  auto red_plates = BuildPlates(FindMarkers(red_mask), TeamColor::kRed);
  auto blue_plates = BuildPlates(FindMarkers(blue_mask), TeamColor::kBlue);
  plates->insert(plates->end(), red_plates.begin(), red_plates.end());
  plates->insert(plates->end(), blue_plates.begin(), blue_plates.end());

  // Dead plates turn into a drifting grey husk. It scores nothing but its
  // collision shape survives, so bullets fired through it are wasted; the
  // tracker keeps husks as obstacles for the corridor check. The grey
  // mask is the widest of the three, so refresh it only every third call
  // — husks drift slowly and the tracker coasts over the gaps.
  static thread_local int husk_tick = 0;
  if (husk_tick++ % 3 == 0) {
    cv::Mat husk_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, params_.husk_v_min),
                cv::Scalar(180, params_.husk_s_max, params_.husk_v_max),
                husk_mask);
    cv::morphologyEx(husk_mask, husk_mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::morphologyEx(husk_mask, husk_mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
    // A real husk is the SOLID 64x32 grey slab (~2000 px). The live
    // plates' white outlines leave thin anti-aliased rectangles in this
    // mask too (~150 px, fill ~0.1) — without the fill-ratio gate every
    // live plate also appeared as a husk and tracks flipped dead at
    // random.
    std::vector<cv::Rect> husk_rects;
    for (const cv::Rect& r : FindMarkers(husk_mask)) {
      const double area = static_cast<double>(r.width) * r.height;
      const int filled = cv::countNonZero(husk_mask(r));
      if (r.width * r.height >= 900 &&
          static_cast<double>(filled) / area > 0.55) {
        husk_rects.push_back(r);
      }
    }
    auto husks = BuildPlates(husk_rects, TeamColor::kDead);
    plates->insert(plates->end(), husks.begin(), husks.end());
  }

  // Shift detections back into full-image coordinates.
  for (auto& plate : *plates) {
    plate.center.y += field_y;
    plate.bbox.y += field_y;
  }
}

cv::Mat ArmorDetector::BuildTeamMask(const cv::Mat& hsv,
                                     TeamColor color) const {
  cv::Mat mask;
  const cv::Scalar s_min(params_.color_s_min, params_.color_v_min);
  const cv::Scalar s_max(255, 255);

  if (color == TeamColor::kRed) {
    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(params_.hsv_red_low1, s_min[0], s_min[1]),
                cv::Scalar(params_.hsv_red_high1, s_max[0], s_max[1]), mask1);
    cv::inRange(hsv, cv::Scalar(params_.hsv_red_low2, s_min[0], s_min[1]),
                cv::Scalar(params_.hsv_red_high2, s_max[0], s_max[1]), mask2);
    mask = mask1 | mask2;
  } else if (color == TeamColor::kBlue) {
    cv::inRange(hsv, cv::Scalar(params_.hsv_blue_low, s_min[0], s_min[1]),
                cv::Scalar(params_.hsv_blue_high, s_max[0], s_max[1]), mask);
  } else {
    mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  }

  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
  return mask;
}

std::vector<cv::Rect> ArmorDetector::FindMarkers(const cv::Mat& mask) const {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Rect> markers;
  markers.reserve(contours.size());
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < params_.min_marker_area || area > params_.max_marker_area) {
      continue;
    }
    markers.push_back(cv::boundingRect(contour));
  }
  return markers;
}

std::vector<ArmorPlate> ArmorDetector::BuildPlates(
    const std::vector<cv::Rect>& markers, TeamColor color) const {
  std::vector<bool> used(markers.size(), false);
  std::vector<ArmorPlate> plates;

  for (size_t i = 0; i < markers.size(); ++i) {
    if (used[i]) {
      continue;
    }
    ArmorPlate plate;
    plate.color = color;
    plate.bbox = markers[i];
    plate.alive = color != TeamColor::kDead;
    used[i] = true;

    for (size_t j = i + 1; j < markers.size(); ++j) {
      if (used[j]) {
        continue;
      }
      // Merge markers on one plate body: the two light bars of a plate,
      // or a husk split by occlusion. The vertical gate keeps plates on
      // different lanes (>=70 px apart) from merging.
      const cv::Point2f c_i(plate.bbox.x + plate.bbox.width * 0.5F,
                            plate.bbox.y + plate.bbox.height * 0.5F);
      const cv::Point2f c_j(markers[j].x + markers[j].width * 0.5F,
                            markers[j].y + markers[j].height * 0.5F);
      const float dist = cv::norm(c_i - c_j);
      const float dy = std::abs(c_i.y - c_j.y);
      if (dist <= params_.marker_max_pair_dist && dy < 25.0F) {
        used[j] = true;
        plate.bbox |= markers[j];
      }
    }

    plate.center =
        cv::Point2f(plate.bbox.x + plate.bbox.width * 0.5F,
                    plate.bbox.y + plate.bbox.height * 0.5F);
    plates.push_back(plate);
  }

  return plates;
}

TeamColor ArmorDetector::DetectTurretColor(const cv::Mat& bgr) const {
  // The turret is the dominant team-coloured blob in the bottom strip.
  // No position estimate: the turret never moves, and a centroid would
  // drift with the barrel angle.
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  const cv::Mat red = BuildTeamMask(hsv, TeamColor::kRed);
  const cv::Mat blue = BuildTeamMask(hsv, TeamColor::kBlue);
  const double red_area = cv::countNonZero(red);
  const double blue_area = cv::countNonZero(blue);

  constexpr int kMinTurretPixels = 60;
  if (red_area < kMinTurretPixels && blue_area < kMinTurretPixels) {
    return TeamColor::kUnknown;
  }
  return red_area >= blue_area ? TeamColor::kRed : TeamColor::kBlue;
}

}  // namespace auto_aim
