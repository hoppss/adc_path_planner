#pragma once

#include <algorithm>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "common/box2d.h"
#include "common/discretized_path.h"
#include "common/line_segment2d.h"
#include "common/vec2d.h"
#include "eigen3/Eigen/Eigen"
#include "open_space/planner_open_space_config.h"
#include "tools/log.h"

using namespace common;

class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother(
      const PlannerOpenSpaceConfig& planner_open_space_config);

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(
      const Eigen::MatrixXd& xWS,
      const std::vector<std::vector<common::Vec2d>>& obstacles_vertices_vec,
      DiscretizedPath* discretized_path);

 private:
  void AdjustStartEndHeading(
      const Eigen::MatrixXd& xWS,
      std::vector<std::pair<double, double>>* const point2d);

  bool ReAnchoring(const std::vector<size_t>& colliding_point_index,
                   DiscretizedPath* path_points);

  bool GenerateInitialBounds(const DiscretizedPath& path_points,
                             std::vector<double>* initial_bounds);

  bool SmoothPath(const DiscretizedPath& raw_path_points,
                  const std::vector<double>& bounds,
                  DiscretizedPath* smoothed_path_points);

  bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
                               std::vector<size_t>* colliding_point_index);

  void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                        std::vector<double>* bounds);

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  bool CheckGear(const Eigen::MatrixXd& xWS);

  double CalcHeadings(const DiscretizedPath& path_points, const size_t index);

 private:
  // vehicle_param
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double center_shift_distance_ = 0.0;

  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec_;

  std::vector<size_t> input_colliding_point_index_;

  bool enforce_initial_kappa_ = true;  // 三个点才能求kappa

  // gear DRIVE as true and gear REVERSE as false
  bool gear_ = false;

  PlannerOpenSpaceConfig planner_open_space_config_;
};