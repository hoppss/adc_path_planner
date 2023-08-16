#include <string>
#include <vector>
#include <iostream>

#include "reference_line/discrete_points_smoother.h"
#include "rviz_tool/planner_viz.h"

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

using common::Vec2d;

int main(int argc, char** argv) {
  ros::init(argc, argv, "smoother_app");
  ros::NodeHandle node;

  std::string basic_path = std::getenv("HOME");
  ROS_INFO("Hostname %s", basic_path.c_str());

  std::string input_file_path;
  std::vector<Vec2d> raw_pts, smoothed_pts;
  std::vector<double> raw_kappas, smoothed_kappas;

  DiscretePointsSmoother smoother;
}
