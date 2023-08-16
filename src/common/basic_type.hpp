#pragma once

#include <string>
#include <iostream>
#include <vector>

#include "common/vec2d.h"

namespace common {

struct PathPoint {
  double x;
  double y;
  double theta;

  double s;

  double kappa;
  double dkappa;
  double ddkappa;
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative;
  double y_derivative;
};

}  // namespace common