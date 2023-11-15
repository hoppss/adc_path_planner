
#pragma once
#include <vector>

#include "common/basic_type.hpp"
#include "common/line_segment2d.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "rviz_tool/rviz_tool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

using common::State;
using common::Vec2d;

class PlannerViz {
 public:
  PlannerViz(ros::NodeHandle& node);
  ~PlannerViz() = default;

  void showTrajectoryPath(std::vector<Vec2d> pts);
  void showTrajectoryMarkers(std::vector<Vec2d> pts);

  void showTrajectoryPath(const std::vector<State>& pts, int i = 0);

  void showBounds(std::vector<std::vector<common::LineSegment2d>> bounds);

 private:
  ros::Publisher curve_path_pub_;
  ros::Publisher curve_path_pub1_;
  ros::Publisher curve_path_pub2_;
  ros::Publisher curve_path_pub3_;
  ros::Publisher curve_path_pub4_;
  ros::Publisher curve_path_pub5_;
  ros::Publisher curve_ma_pub_;

  ros::Publisher marker_array_publisher_;
};
