#include "reference_line/uos_traj_file_tool.hpp"
#include "ros/ros.h"
#include "rviz_tool/planner_viz.h"

using namespace common;
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "u_viz_demo");
  ros::NodeHandle node;
  PlannerViz viz(node);

  std::string file_path = std::getenv("HOME") + std::string("/UISEE_LOGS/");
  std::string file_name_0 = std::string("hybrid_astar.csv");
  std::string file_name_1 = std::string("hybrid_astar_smoothed.csv");
  std::vector<State> raw_traj;
  std::vector<State> smoothed_traj;

  if (!UOSReadTrajFile(file_path + file_name_0, raw_traj)) {
    AERROR << "read raw traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_1, smoothed_traj)) {
    AERROR << "read smoothed traj failed ";
    return 0;
  }

  AINFO << "input pts size " << raw_traj.size();

  ros::Rate r(5);
  while (ros::ok()) {
    viz.showTrajectoryPath(raw_traj, 0);
    viz.showTrajectoryPath(smoothed_traj, 1);
    ros::spinOnce();
    r.sleep();
  }
}