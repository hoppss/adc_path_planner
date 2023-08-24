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

class State {
 public:
  double set_x(double _x) { x_ = _x; };
  double set_y(double _y) { y_ = _y; };
  double set_theta(double _theta) { theta_ = _theta; };
  double set_s(double _s) { s_ = _s; };

  double set_kappa(double _kappa) { kappa_ = _kappa; };
  double set_dkappa(double _dkappa) { dkappa_ = _dkappa; };
  double set_ddkappa(double _ddkappa) { ddkappa_ = _ddkappa; };

  double set_v(double _v) { v_ = _v; };
  double set_t(double _t) { t_ = _t; };
  double set_a(double _a) { a_ = _a; };

  double s() const { return s_; };
  double x() const { return x_; };
  double y() const { return y_; };
  double theta() const { return theta_; };
  double kappa() const { return kappa_; };
  double dkappa() const { return dkappa_; };
  double ddkappa() const { return ddkappa_; };

  double t() { return t_; };
  double v() { return v_; };
  double a() { return a_; };

 private:
  double x_;
  double y_;
  double theta_;

  double s_;

  double kappa_;
  double dkappa_;
  double ddkappa_;

  double t_;  // timestamp
  double v_;
  double a_;
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