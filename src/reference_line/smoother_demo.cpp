#include <iostream>
#include <string>
#include <vector>

#include "ros/package.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"

#include "common/type_tool.h"
#include "reference_line/discrete_points_smoother.h"
#include "reference_line/traj_file_tool.hpp"

#include "rviz_tool/planner_viz.h"


using common::Vec2d;
using common::State;

FemSmootherConfig config;

int main(int argc, char** argv) {
  ros::init(argc, argv, "smoother_app");
  ros::NodeHandle node;

  std::string home_path = std::getenv("HOME");
  std::string target = "path";
  ROS_INFO("Hostname path %s, traj file %s", home_path.c_str(), target.c_str());

  std::string input_file_path = home_path + "/" + target + ".csv";
  std::string output_file_path = home_path + "/" + target + "filter.csv";

  std::vector<Vec2d> raw_pts, smoothed_pts;
  std::vector<State> raw_states;
  std::vector<double> raw_kappas, smoothed_kappas;

  if (!ReadTrajectoryFile(input_file_path, raw_states)) {
    ROS_ERROR("ReadTrajectoryFile failed");
    return false;
  }

  StateToVec2dVec(raw_states, raw_pts);
  ACHECK(raw_states.size() == raw_pts.size());

  DiscretePointsSmoother smoother(config);

  ros::Rate r(3);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
}