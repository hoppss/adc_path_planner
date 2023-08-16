#pragma once
#include "reference_line/fem_smoother_config.h"
#include "reference_line/fem_pose_deviation_osqp_interface.h"

class DiscretePointsSmoother {
 public:
  explicit DiscretePointsSmoother(const FemSmootherConfig& config);
  ~DiscretePointsSmoother() = default;

  void SetAnchorPoints(const std::vector<common::AnchorPoint>& anchor_pts);

  bool Smooth(const std::vector<common::Vec2d>& raw_pts,
              std::vector<common::Vec2d>* result_pts);

 private:
  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);
  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool FemPosSmooth(std::vector<std::pair<double, double>>* result_pts);

 private:
  FemSmootherConfig config_;

  std::vector<common::AnchorPoint> anchor_pts_;

  double zero_x_;
  double zero_y_;  // first point

  // smoother
  FemPosDeviationOsqpInterface fem_smoother_;
};