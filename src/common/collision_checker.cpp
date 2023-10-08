#include "common/collision_checker.h"

namespace planning {
namespace {
using common::Box2d;
using common::Obstacle;
using common::Polygon2d;
using common::Trajectory;
using common::Vec2d;
};  // namespace
bool CollisionChecker::InCollision(const std::vector<const Obstacle*>& obstacles,
                                   const common::Trajectory& ego_trajectory,
                                   const double ego_length, const double ego_width,
                                   const double ego_edge_to_center) {}

}  // namespace planning