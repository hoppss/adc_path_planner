#include <iostream>
#include <string>
#include <vector>

#include "common/type_tool.h"
#include "geometry_msgs/Pose.h"
#include "reference_line/discrete_points_smoother.h"
#include "reference_line/traj_file_tool.hpp"
#include "reference_line/uos_traj_file_tool.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include "rviz_tool/planner_viz.h"
#include "visualization_msgs/MarkerArray.h"

using common::State;
using common::Vec2d;

int main(int argc, char** argv) {
  ros::init(argc, argv, "smoother_app");
  ros::NodeHandle node;
  PlannerViz viz(node);

  std::string home_path = std::getenv("HOME") + std::string("/UISEE_LOGS/");
  std::string target = "hybrid_astar_1.csv";

  std::string input_file_path = home_path + target;
  // std::string output_file_path = home_path + "/" + target + "filter.csv";

  AINFO << "input traj file " <<  input_file_path.c_str();

  std::vector<Vec2d> raw_pts, smoothed_pts;
  std::vector<State> raw_states, smoothed_states;

  if (!UOSReadTrajFile(input_file_path, raw_states)) {
    ROS_ERROR("ReadTrajectoryFile failed");
    return false;
  }
  AINFO << "read pts size: " << raw_states.size();

  StateToVec2dVec(raw_states, raw_pts);
  ACHECK(raw_states.size() == raw_pts.size());
  std::vector<double> bounds;
  for (size_t i = 0; i < raw_pts.size(); ++i) {
    bounds.push_back(0.3);
  }
  bounds.front() = 0.0;
  bounds.back() = 0.0;

  FemPosDeviationSmootherConfig _config;
  _config.use_sqp = true;

  DiscretePointsSmoother smoother(_config);
  if (!smoother.Smooth(raw_pts, bounds, &smoothed_pts)) {
    AFATAL << "Fem smoother failed";
    return 0;
  }

  Vec2dToStateVec(smoothed_pts, smoothed_states);
  std::cout << "smoothed pts " << smoothed_pts.size() << ", states size "
            << smoothed_states.size() << std::endl;
  if (!generateStatesProfile(smoothed_states)) {
    AFATAL << "generator state failed";
  }

  ros::Rate r(3);
  while (ros::ok()) {
    viz.showTrajectoryPath(raw_states, 0);
    viz.showTrajectoryPath(smoothed_states, 1);
    ros::spinOnce();
    r.sleep();
    // break;
  }

  ros::shutdown();
}