#include "receiver.h"
#include "tools/log.h"

Receiver::Receiver(ros::NodeHandle &node) {
  click_pose_sub_ = node.subscribe("/clicked_point", 1, &Receiver::clickedPointCb, this);
  init_pose_sub_ = node.subscribe("/initialpose", 1, &Receiver::initialPoseCb, this);
  move_base_goal_sub_ = node.subscribe("/move_base_simple/goal", 1, &Receiver::goalCb, this);
}

void Receiver::clickedPointCb(const geometry_msgs::PointStampedConstPtr &p) {
  AINFO << "clicked";  // 需要地图才能生效
}

void Receiver::initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
  origin_poses_.clear();
  has_goal_ = false;
  AWARN << "new start";

  AINFO << absl::StrCat("initial: ", start->pose.pose.position.x, ",", start->pose.pose.position.y);
  has_start_ = true;
  origin_poses_.emplace_back(start->pose.pose);
}

void Receiver::goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
  if (!has_start_) return;

  AINFO << absl::StrCat("goal: ", goal->pose.position.x, ",", goal->pose.position.y);
  has_goal_ = true;
  origin_poses_.emplace_back(goal->pose);
}