#include "rviz_tool/planner_viz.h"

namespace {
using common::Vec2d;
}

PlannerViz::PlannerViz(ros::NodeHandle& node) {
  curve_path_pub_ = node.advertise<nav_msgs::Path>("/curve/path", 100);
  curve_ma_pub_ = node.advertise<visualization_msgs::MarkerArray>("/curve/markers", 100);
}

void PlannerViz::showTrajectoryPath(std::vector<Vec2d> pts) {
  static unsigned int seq = 0;
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.header.seq = seq++;
  geometry_msgs::PoseStamped pose;
  for (size_t i = 0; i < pts.size(); ++i) {
    pose.pose.position.x = pts[i].x();
    pose.pose.position.y = pts[i].y();
    pose.pose.orientation.w = 1.0;

    path.poses.emplace_back(pose);
  }
  curve_path_pub_.publish(std::move(path));
}

void PlannerViz::showTrajectoryMarkers(std::vector<Vec2d> pts) {
  static unsigned int seq = 0;
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker m =
      RvizTool::createMarker("map", "pts", RvizTool::createColor(RvizTool::ColorType::GREEN));
  m.type = visualization_msgs::Marker::SPHERE;
  m.scale.x = m.scale.y = m.scale.z = 0.1;

  for (size_t i = 0; i < pts.size(); ++i) {
    m.header.seq = seq++;
    m.id = i;

    m.pose.position.x = pts[i].x();
    m.pose.position.y = pts[i].y();

    ma.markers.push_back(m);
  }
  curve_ma_pub_.publish(std::move(ma));
}