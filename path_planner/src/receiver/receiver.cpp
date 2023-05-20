#include "../receiver/receiver.h"

Receiver::Receiver(ros::NodeHandle &node) {
  click_pose_sub_ = node.subscribe("/clicked_point", 1, &Receiver::clickedPointCb, this);
  init_pose_sub_ = node.subscribe("/initialpose", 1, &Receiver::initialPoseCb, this);
  move_base_goal_sub_ = node.subscribe("/move_base_simple/goal", 1, &Receiver::goalCb, this);
}

void Receiver::clickedPointCb(const geometry_msgs::PointStampedConstPtr &p) {
  std::cout << "clicked" << std::endl;  // 需要地图才能生效
}

void Receiver::initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
  if (is_ready_) {
    origin_poses_.clear();
    is_ready_ = false;
    std::cerr << "let's new test" << std::endl;
    return;
  }
  std::cout << absl::StrCat("initial: ", start->pose.pose.position.x, ",", start->pose.pose.position.y)
            << std::endl;
  geometry_msgs::Pose p;
  p = start->pose.pose;

  origin_poses_.emplace_back(std::move(p));
}

void Receiver::goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
  std::cout << "goal" << std::endl;
  if (origin_poses_.size() > 2) {
    is_ready_ = true;
  }
}