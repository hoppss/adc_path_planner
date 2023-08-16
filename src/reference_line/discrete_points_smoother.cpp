#include "reference_line/discrete_points_smoother.h"

#include "tools/log.h"

namespace {
using common::Vec2d;
}
DiscretePointsSmoother::DiscretePointsSmoother(const FemSmootherConfig& config)
    : config_(config) {
  AINFO << "Fem Osqp Interface";
}

void DiscretePointsSmoother::SetAnchorPoints(
    const std::vector<common::AnchorPoint>& anchor_pts) {
  anchor_pts_ = anchor_pts;
}

bool DiscretePointsSmoother::Smooth(const std::vector<common::Vec2d>& raw_pts,
                                    std::vector<common::Vec2d>* result_pts) {
  // 1. get raw pts from anchor_pts
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<std::pair<double, double>> result_point2d;

  std::vector<double> anchorpoints_lateralbound;
  for (const auto& anchor_point : anchor_pts_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  // 2. normalize points, output zero_x_, for numerical stable
  NormalizePoints(&raw_point2d);

  // 3. fem smooth
  bool status = false;
  status = FemPosSmooth(&result_point2d);

  if (!status || raw_point2d.size() != result_point2d.size()) {
    AERROR << "fem pose smooth failed!";
    return false;
  }

  // 4. denormalize
  DeNormalizePoints(&result_point2d);
  // 5. profile generation

  // 6. transfer pair<> -> vec2d


  return true;
}

void DiscretePointsSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  CHECK_NOTNULL(xy_points);
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  CHECK_NOTNULL(xy_points);
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsSmoother::FemPosSmooth(
    std::vector<std::pair<double, double>>* result_pts) {}