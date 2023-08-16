#pragma once
#include "common/basic_type.hpp"
#include "common/vec2d.h"

struct AnchorPoint {
  common::Vec2d path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

struct FemSmootherConfig {
  bool use_sqp_solver;                 // default false, use osqp
  double max_constraint_interval;      // [default = 5]
  double longitudinal_boundary_bound;  // [default = 1.0]
  double max_lateral_boundary_bound;   // [default = 0.5]
  double min_lateral_boundary_bound;   // [default = 0.2]
                                       // The output resolution for qp smoother
                                       // reference line
  int num_of_total_points;             // [default = 500]
  double curb_shift;                   // [default = 0.2]
  double lateral_buffer;               // [default = 0.2]
};