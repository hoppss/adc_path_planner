#include "common/obstacle.h"

#include <algorithm>
#include <utility>

#include "common/linear_interpolation.h"
#include "common/vehicle_config_helper.h"
#include "tools/log.h"

namespace common {

namespace {
const double kStBoundaryDeltaS = 0.2;        // meters
const double kStBoundarySparseDeltaS = 1.0;  // meters
const double kStBoundaryDeltaT = 0.05;       // seconds
}  // namespace

Obstacle::Obstacle(const std::string& id, const std::vector<common::Vec2d>& points)
    : id_(id) {
  ACHECK(common::Polygon2d::ComputeConvexHull(points, &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull.\n";
}

}  // namespace common