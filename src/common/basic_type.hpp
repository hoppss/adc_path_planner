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
  State() = default;
  State(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {};

  void set_x(double _x) { x_ = _x; };
  void set_y(double _y) { y_ = _y; };
  void set_theta(double _theta) { theta_ = _theta; };
  void set_s(double _s) { s_ = _s; };

  void set_kappa(double _kappa) { kappa_ = _kappa; };
  void set_dkappa(double _dkappa) { dkappa_ = _dkappa; };
  void set_ddkappa(double _ddkappa) { ddkappa_ = _ddkappa; };

  void set_v(double _v) { v_ = _v; };
  void set_t(double _t) { t_ = _t; };
  void set_a(double _a) { a_ = _a; };

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

struct VehicleParam {

  double width() const { return width_;};
  double length() const { return length_;};
  double front_edge_to_center() const { return front_edge_to_center_;};
  double back_edge_to_center() const { return back_edge_to_center_;};
  double left_edge_to_center() const { return left_edge_to_center_;};
  double right_edge_to_center() const { return right_edge_to_center_;};

  double min_turn_radius() const { return min_turn_radius_;};
  double max_steer_angle() const { return max_steer_angle_;};
  double wheel_base() const { return wheel_base_;};

  std::string debugString() {
    return absl::StrCat("VehicleParam [L:", length_, " W: ", width_, " ], front_edge ",
                front_edge_to_center_, ",back edge: ", back_edge_to_center_,
                ", L_w: ", left_edge_to_center_, ", R_w: ", right_edge_to_center_,
                ", min_turn_radius: ", min_turn_radius_, ", steer: ", max_steer_angle_);
  }
  std::string brand;
  std::string vehicle_id;

  // Car center point is car reference point, i.e., center of rear axle.
  double front_edge_to_center_;
  double back_edge_to_center_;
  double left_edge_to_center_;
  double right_edge_to_center_;

  double length_;
  double width_;
  double height_;

  double min_turn_radius_;
  double max_acceleration_;
  double max_deceleration_;

  // vehicle front wheel max steer angle
  double max_steer_angle_;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate_;
  // vehicle min steer rate;
  double min_steer_angle_rate_;
  // the distance between the front and back wheels L
  double wheel_base_;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius_;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped_;

  // minimum value get from chassis.brake, in percentage
  double brake_deadzone_;
  // minimum value get from chassis.throttle, in percentage
  double throttle_deadzone_;
};

struct VehicleConfig {
  VehicleParam vehicle_param;
};
}  // namespace common