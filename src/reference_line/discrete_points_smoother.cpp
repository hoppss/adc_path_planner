#include "reference_line/discrete_points_smoother.h"

#include "tools/log.h"

namespace {
using common::Vec2d;
}
DiscretePointsSmoother::DiscretePointsSmoother(
    const FemPosDeviationSmootherConfig& config)
    : config_(config) {
  AINFO << "Fem Osqp Interface";
}

// void DiscretePointsSmoother::SetAnchorPoints(
//     const std::vector<common::AnchorPoint>& anchor_pts) {
//   anchor_pts_ = anchor_pts;
// }

bool DiscretePointsSmoother::Smooth(const std::vector<common::Vec2d>& raw_pts,
                                    const std::vector<double>& bounds,
                                    std::vector<common::Vec2d>* result_pts) {
  // 1. get raw pts from anchor_pts
  raw_point2d_.clear();
  anchorpoints_lateralbound_.clear();
  result_point2d_.clear();
  ACHECK(raw_pts.size() == bounds.size()) << "Discrete pts smooth() input error";

  for (size_t i = 0; i < raw_pts.size(); ++i) {
    raw_point2d_.emplace_back(raw_pts[i].x(), raw_pts[i].y());
    anchorpoints_lateralbound_.emplace_back(bounds[i]);
  }
  // 外界控制bound
  // anchorpoints_lateralbound_.front() = 0.0;
  // anchorpoints_lateralbound_.back() = 0.0;

  // 2. normalize points, output zero_x_, for numerical stable
  NormalizePoints(&raw_point2d_);

  // 3. fem smooth
  bool status = false;
  status = FemPosSmooth(&result_point2d_);

  if (!status || raw_point2d_.size() != result_point2d_.size()) {
    AERROR << "fem pose smooth failed!";
    return false;
  }

  // 4. denormalize
  DeNormalizePoints(&result_point2d_);

  // 5 transfer pair<> -> vec2d
  for (size_t i=0; i<result_point2d_.size();++i) {
    result_pts->emplace_back(result_point2d_[i].first, result_point2d_[i].second);
  }

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
                  std::pair<double, double> xy(curr_x - zero_x_, curr_y - zero_y_);
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
                  std::pair<double, double> xy(curr_x + zero_x_, curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsSmoother::FemPosSmooth(
    std::vector<std::pair<double, double>>* result_pts) {
  result_pts->clear();

  if (!config_.use_sqp) {
    AINFO << "start fem-osqp smooth -->";
    // smoother
    FemPosDeviationOsqpInterface fem_qp_smoother;
    // fem_qp_smoother.set_verbose(config_.verbose);
    // fem_qp_smoother.set_warm_start(config_.warm_start);
    // fem_qp_smoother.set_time_limit(config_.time_limit);
    // fem_qp_smoother.set_scaled_termination(config_.scaled_termination);
    // fem_qp_smoother.set_max_iter(config_.max_iter);

    // weight
    fem_qp_smoother.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation);
    fem_qp_smoother.set_weight_path_length(config_.weight_path_length);
    fem_qp_smoother.set_weight_ref_deviation(config_.weight_ref_deviation);
    // data
    fem_qp_smoother.set_ref_points(raw_point2d_);
    fem_qp_smoother.set_bounds_around_refs(anchorpoints_lateralbound_);
    // solve
    if (!fem_qp_smoother.Solve()) {
      AERROR << "FEM osqp solve failed";
      return false;
    } else {
      ACHECK(raw_point2d_.size() == fem_qp_smoother.opt_x().size());
      const std::vector<double> opt_x = fem_qp_smoother.opt_x();
      const std::vector<double> opt_y = fem_qp_smoother.opt_y();
      for (size_t i = 0; i < raw_point2d_.size(); ++i) {
        result_pts->emplace_back(opt_x[i], opt_y[i]);
      }

      return true;
    }
  } else {
    AINFO << "start fem-sqp smooth -->";
    config_.debug();
    FemPosDeviationSqpOsqpInterface fem_sqp_smoother;
    // setting
    fem_sqp_smoother.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation);
    fem_sqp_smoother.set_weight_path_length(config_.weight_path_length);
    fem_sqp_smoother.set_weight_ref_deviation(config_.weight_ref_deviation);

    fem_sqp_smoother.set_weight_curvature_constraint_slack_var(
        config_.weight_curvature_constraint_slack_var);
    fem_sqp_smoother.set_curvature_constraint(config_.curvature_constraint);

    fem_sqp_smoother.set_sqp_ctol(config_.sqp_ctol);
    fem_sqp_smoother.set_sqp_ftol(config_.sqp_ftol);
    fem_sqp_smoother.set_sqp_pen_max_iter(config_.sqp_pen_max_iter);
    fem_sqp_smoother.set_sqp_sub_max_iter(config_.sqp_sub_max_iter);

    fem_sqp_smoother.set_max_iter(config_.max_iter);
    // data
    fem_sqp_smoother.set_ref_points(raw_point2d_);
    fem_sqp_smoother.set_bounds_around_refs(anchorpoints_lateralbound_);

    // solve
    if (!fem_sqp_smoother.Solve()) {
      AERROR << "FEM osqp solve failed";
      return false;
    } else {
      ACHECK(raw_point2d_.size() == fem_sqp_smoother.opt_xy().size());
      const std::vector<std::pair<double, double>> opt_xy = fem_sqp_smoother.opt_xy();
      for (size_t i = 0; i < raw_point2d_.size(); ++i) {
        result_pts->emplace_back(opt_xy[i].first, opt_xy[i].second);
      }

      return true;
    }
  }
}