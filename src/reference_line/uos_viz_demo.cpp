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
  std::string file_name_0 = std::string("ap_circle_traj.txt");
  std::string file_name_1 = std::string("ap_circle_resample_traj.txt");
  std::string file_name_2 = std::string("ap_pp_traj.txt");
  std::string file_name_3 = std::string("dliaps_traj.txt");
  std::string file_name_4 = std::string("ap_pp_follow_traj.txt");
  std::string file_name_5 = std::string("dliaps_follow_traj.txt");

  std::vector<State> ap_circle_traj;
  std::vector<State> ap_circle_resample_traj;
  std::vector<State> ap_pp_traj;
  std::vector<State> dliaps_traj;
  std::vector<State> ap_pp_follow_traj;
  std::vector<State> dliaps_follow_traj;

  if (!UOSReadTrajFile(file_path + file_name_0, ap_circle_traj)) {
    AERROR << "read raw traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_1, ap_circle_resample_traj)) {
    AERROR << "read raw traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_2, ap_pp_traj)) {
    AERROR << "read raw traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_3, dliaps_traj)) {
    AERROR << "read smoothed traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_4, ap_pp_follow_traj)) {
    AERROR << "read raw traj failed ";
    return 0;
  }
  if (!UOSReadTrajFile(file_path + file_name_5, dliaps_follow_traj)) {
    AERROR << "read smoothed traj failed ";
    return 0;
  }

  AINFO << "input pts size " << ap_pp_traj.size();

  ros::Rate r(5);
  while (ros::ok()) {
    viz.showTrajectoryPath(ap_circle_traj, 0);
    viz.showTrajectoryPath(ap_circle_resample_traj, 1);
    viz.showTrajectoryPath(ap_pp_traj, 2);
    viz.showTrajectoryPath(dliaps_traj, 3);
    viz.showTrajectoryPath(ap_pp_follow_traj, 4);
    viz.showTrajectoryPath(dliaps_follow_traj, 5);
    ros::spinOnce();
    r.sleep();
  }
}