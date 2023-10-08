#pragma once

#include <memory>
#include <vector>

#include "common/box2d.h"
#include "common/obstacle.h"
#include "common/trajectory.h"

namespace planning {
namespace {
using common::Box2d;
using common::Obstacle;
using common::Polygon2d;
using common::Trajectory;
using common::Vec2d;
};  // namespace

class CollisionChecker {
 public:
  // collision with trajectory
  static bool InCollision(const std::vector<const Obstacle*>& obstacles,
                          const common::Trajectory& ego_trajectory,
                          const double ego_length, const double ego_width,
                          const double ego_edge_to_center);

  // collision with State
};

}  // namespace planning