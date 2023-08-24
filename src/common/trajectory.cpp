#include "common/trajectory.h"

namespace common {

using common::State;
using common::Vec2d;

Trajectory::Trajectory(const std::vector<common::State> path_points) {
  path_points_ = path_points;
}
Trajectory::Trajectory(const std::vector<common::Vec2d> path_points) {
  State s;
  for (int i = 0; i < path_points.size(); ++i) {
    s.set_x(path_points[i].x());
    s.set_y(path_points[i].y());
    path_points_.emplace_back(s);
  }
}

}  // namespace common