#pragma once
#include "common/basic_type.hpp"

#include "reference_line/fem_smoother_config.h"
#include "reference_line/fem_pose_deviation_osqp_interface.h"
#include "reference_line/fem_pose_deviation_sqp_interface.h"

// 离散点平滑算法接口， 内部区分调用osqp/sqp
class DiscretePointsSmoother {
 public:
  explicit DiscretePointsSmoother(const FemPosDeviationSmootherConfig& config);
  ~DiscretePointsSmoother() = default;

  // void SetAnchorPoints(const std::vector<common::AnchorPoint>& anchor_pts);

  bool Smooth(const std::vector<common::Vec2d>& raw_pts,
              const std::vector<double>& bounds, std::vector<common::Vec2d>* result_pts);

 private:
  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);
  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool FemPosSmooth(std::vector<std::pair<double, double>>* result_pts);

 private:
  FemPosDeviationSmootherConfig config_;

  // fem module input data
  std::vector<std::pair<double, double>> raw_point2d_;
  std::vector<std::pair<double, double>> result_point2d_;

  std::vector<double> anchorpoints_lateralbound_;

  double zero_x_;
  double zero_y_;  // first point
};