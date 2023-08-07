
#pragma once
#include <vector>

#include "common/vec2d.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "rviz_tool/rviz_tool.h"

using common::Vec2d;

class PlannerViz {
 public:
  PlannerViz(ros::NodeHandle& node);
  ~PlannerViz() = default;

  void showTrajectoryPath(std::vector<Vec2d> pts);
  void showTrajectoryMarkers(std::vector<Vec2d> pts);

 private:
  ros::Publisher curve_path_pub_;
  ros::Publisher curve_ma_pub_;
};
