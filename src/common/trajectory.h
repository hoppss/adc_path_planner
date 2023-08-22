#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "common/basic_type.hpp"

namespace common {

class Trajectory {
 public:
  Trajectory() = default;
  explicit Trajectory(const std::vector<common::State> path_points);
  explicit Trajectory(const std::vector<common::Vec2d> path_points);

 private:
  std::vector<common::State> path_points_;
};  // class
}  // namespace common