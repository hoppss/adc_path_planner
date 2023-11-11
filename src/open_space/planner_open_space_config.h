#pragma once

#include "reference_line/fem_smoother_config.h"

// Hybrid a star for warm start
struct WarmStartConfig {
  double xy_grid_resolution = 0.2;
  double phi_grid_resolution = 0.05;
  unsigned int next_node_num = 10;
  double step_size = 0.5;
  double traj_forward_penalty = 0.0;
  double traj_back_penalty = 0.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 100.0;
  double traj_steer_change_penalty = 10.0;
  double traj_kappa_contraint_ratio = 1.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution = 0.1;
  double node_radius = 0.5;
};

struct PlannerOpenSpaceConfig {
  WarmStartConfig warm_start_config;
  IterativeAnchoringConfig iterative_anchoring_smoother_config;
};