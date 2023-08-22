#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "common/vec2d.h"
#include "tools/log.h"

namespace common {

struct AnchorPoint {
  Vec2d path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

struct State {
  double x;
  double y;
  double z;
  double theta;

  double s;

  double kappa;
  double dkappa;
  double ddkappa;

  double t;  // timestamp
  double v;
  double a;
};

struct PathPoint {
  double x;
  double y;
  double z;
  double theta;

  double s;

  double kappa;
  double dkappa;
  double ddkappa;

  // std::string lane_id;
};

struct SpeedPoint {
  double s;
  double t;
  double v;   // speed (m/s)
  double a;   // acceleration (m/s^2)
  double da;  // jerk (m/s^3)
};

struct TrajectoryPoint {
  double relative_time;  // relative time from beginning of the trajectory

  // lat info
  PathPoint path_point;

  // lon info
  double v;  // in [m/s]
  double a;
  double da;     // longitudinal jerk
  double steer;  // The angle between vehicle front wheel and vehicle
                 // longitudinal axis
};

struct SLPoint {
  double s;
  double l;
};
}  // namespace common