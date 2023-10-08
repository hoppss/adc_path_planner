#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "basic_type.hpp"
#include "box2d.h"
#include "polygon2d.h"
#include "common/trajectory.h"

namespace common {

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const std::vector<common::Vec2d>& points);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  // static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
  //     const std::string& id, const common::Box2d& obstacle_box);

  // inline bool IsCautionLevelObstacle() const {
  //   return is_caution_level_obstacle_;
  // }

  // bool HasLongitudinalDecision() const;

  // bool HasNonIgnoreDecision() const;

  // double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;


 private:
  std::string id_;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  common::Trajectory trajectory_;
  common::Box2d perception_bounding_box_;

  // save contour pts
  common::Polygon2d perception_polygon_;

  // std::vector<ObjectDecisionType> decisions_;
  // std::vector<std::string> decider_tags_;
  // SLBoundary sl_boundary_;

  // STBoundary reference_line_st_boundary_;
  // STBoundary path_st_boundary_;

  // ObjectDecisionType lateral_decision_;
  // ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;
};

}  // namespace planning