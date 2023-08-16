#pragma once
#include "reference_line/fem_smoother_config.h"

class DiscretePointsSmoother {
 public:
  explicit DiscretePointsSmoother(const FemSmootherConfig& config);
  ~DiscretePointsSmoother() = default;

  void setAnchorPoints(const std::vector<AnchorPoint>& anchor_pts);

  bool smooth(const std::vector<common::Vec2d>& raw_pts,
              std::vector<common::Vec2d>& result_pts);

 private:
  void NormalizePoints(std::vector<common::Vec2d>* xy_points);
  void DeNormalizePoints(std::vector<common::Vec2d>* xy_points);

  bool femPosSmooth(std::vector<common::Vec2d>& result_pts);

 private:
  FemSmootherConfig config_;

  std::vector<AnchorPoint> anchor_pts_;
  double zero_x_;
  double zero_y_;  // first point

  // smoother
};