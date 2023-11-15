#include "rviz_tool/planner_viz.h"

namespace {
using common::State;
using common::Vec2d;
using common::LineSegment2d;
}  // namespace

PlannerViz::PlannerViz(ros::NodeHandle& node) {
  curve_path_pub_  = node.advertise<nav_msgs::Path>("/curve/path0", 100);
  curve_path_pub1_ = node.advertise<nav_msgs::Path>("/curve/path1", 100);
  curve_path_pub2_ = node.advertise<nav_msgs::Path>("/curve/path2", 100);
  curve_path_pub3_ = node.advertise<nav_msgs::Path>("/curve/path3", 100);
  curve_path_pub4_ = node.advertise<nav_msgs::Path>("/curve/path4", 100);
  curve_path_pub5_ = node.advertise<nav_msgs::Path>("/curve/path5", 100);

  curve_ma_pub_ =
    node.advertise<visualization_msgs::MarkerArray>("/curve/markers", 100);

  marker_array_publisher_ =
    node.advertise<visualization_msgs::MarkerArray>("/open_space/bound", 100, true);
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
  visualization_msgs::Marker m = RvizTool::createMarker(
      "map", "pts", RvizTool::createColor(RvizTool::ColorType::GREEN));
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

void PlannerViz::showTrajectoryPath(const std::vector<State>& pts, int i) {
  static unsigned int seq = 0;
  nav_msgs::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = ros::Time::now();
  path.header.seq = seq++;
  geometry_msgs::PoseStamped pose;
  for (size_t i = 0; i < pts.size(); ++i) {
    pose.pose.position.x = pts[i].x();
    pose.pose.position.y = pts[i].y();
    tf2::Quaternion q;
    q.setRPY(0., 0., pts[i].theta());
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.emplace_back(pose);
  }

  if (i == 0) {
    curve_path_pub_.publish(std::move(path));
  }
  else if (i == 1)
    curve_path_pub1_.publish(std::move(path));
  else if (i == 2) {
    curve_path_pub2_.publish(std::move(path));
  } else if (i == 3) {
    curve_path_pub3_.publish(std::move(path));
  } else if (i == 4) {
    curve_path_pub4_.publish(std::move(path));
  }
  else
    curve_path_pub5_.publish(std::move(path));
}

void PlannerViz::showBounds(std::vector<std::vector<common::LineSegment2d>> bounds) {
  visualization_msgs::MarkerArray ma;
  for(size_t i=0; i< bounds.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    const std::vector<common::LineSegment2d>& bound = bounds[i];
    for(size_t j = 0; j < bound.size(); ++j) {
      geometry_msgs::Point p0, p1;
      p0.x = bound[j].start().x();
      p0.y = bound[j].start().y();
      p1.x = bound[j].end().x();
      p1.y = bound[j].end().y();
      marker.points.push_back(p0);
      marker.points.push_back(p1);
    }
    ma.markers.push_back(marker);
  }
  marker_array_publisher_.publish(ma);
}