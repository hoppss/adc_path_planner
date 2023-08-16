#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "common/vec2d.h"

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
  double kappa;

  double s;
  double t;  // timestamp
  double v;
  double a;
};

}  // namespace common