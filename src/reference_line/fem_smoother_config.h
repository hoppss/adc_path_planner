#pragma once
#include "common/basic_type.hpp"
#include "common/vec2d.h"

struct FemPosDeviationSmootherConfig {
  double weight_fem_pos_deviation = 1.0e6;
  double weight_ref_deviation = 1.0;
  double weight_path_length = 1.0;
  bool apply_curvature_constraint = false;
  double weight_curvature_constraint_slack_var = 1000000;
  double curvature_constraint = 0.3;
  bool use_sqp = false;
  double sqp_ftol = 1e-2;
  double sqp_ctol = 1e-2;
  int sqp_pen_max_iter = 100;
  int sqp_sub_max_iter = 100;

  // osqp settings
  int max_iter =100;
  double time_limit = 0.0;
  bool verbose = true;
  bool scaled_termination = true;
  bool warm_start = true;
};

struct IterativeAnchoringConfig {
  double interpolated_delta_s = 0.2;
  int reanchoring_trails_num = 50;  // 初始点碰撞会随机找点， 忽略
  double reanchoring_pos_stddev = 0.25;
  double reanchoring_length_stddev = 1.0;
  double default_bound = 1.0;
  FemPosDeviationSmootherConfig fem_pos_deviation_smoother_config;
  double collision_decrease_ratio = 0.9;  // outter loop， 碰撞检测不合法时收缩bound
};