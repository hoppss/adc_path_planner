#include "reference_line/discrete_points_smoother.h"
#include "tools/log.h"

namespace {
using common::Vec2d;
}
DiscretePointsSmoother::DiscretePointsSmoother(const FemSmootherConfig& config)
    : config_(config) {
  AINFO << "Fem Osqp Interface";
}

void DiscretePointsSmoother::setAnchorPoints(
    const std::vector<AnchorPoint>& anchor_pts) {
  anchor_pts_ = anchor_pts;
}

bool DiscretePointsSmoother::smooth(const std::vector<common::Vec2d>& raw_pts,
                              std::vector<common::Vec2d>& result_pts) {
  // 1. get raw pts from anchor_pts
  std::vector<Vec2d> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;
  for (const auto& anchor_point : anchor_pts_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  // 2. normalize points, 为了数值稳定性
  NormalizePoints(&raw_point2d);

  // 3. fem smooth
  bool status = false;
  status = femPosSmooth(result_pts);

  if (!status || raw_point2d.size() != result_pts.size()) {
    std::cerr << "fem pose smooth failed!" << std::endl;
    return false;
  }

  // 4. denormalize

  // 5. profile generation

  // 6.
}

void DiscretePointsSmoother::NormalizePoints(std::vector<Vec2d>* xy_points) {
  zero_x_ = xy_points->front().x();
  zero_y_ = xy_points->front().y();
  std::for_each(xy_points->begin(), xy_points->end(), [this](Vec2d& point) {
    point.set_x(point.x() - zero_x_);
    point.set_y(point.y() - zero_y_);
  });
}

void DiscretePointsSmoother::DeNormalizePoints(std::vector<Vec2d>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(), [this](Vec2d& point) {
    point.set_x(point.x() + zero_x_);
    point.set_y(point.y() + zero_y_);
  });
}

bool DiscretePointsSmoother::femPosSmooth(std::vector<common::Vec2d>& result_pts) {}