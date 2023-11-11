#ifndef __PLANNER_RECEIVER__
#define __PLANNER_RECEIVER__

#include <iostream>
#include <string>

#include "absl/strings/str_cat.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "ros/ros.h"

class Receiver {
public:
  Receiver(ros::NodeHandle &node);
  ~Receiver() = default;

  void clickedPointCb(const geometry_msgs::PointStampedConstPtr &p);
  void
  initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start);
  void goalCb(const geometry_msgs::PoseStampedConstPtr &goal);
  
  std::vector<geometry_msgs::Pose> getPoses() const {
    return origin_poses_;
  }

  bool isReady() const {
    return has_start_ && has_goal_;
  }
  void reset() {
    has_start_ = has_goal_ = false;
  }

  const geometry_msgs::Pose& getStart() const {
    return origin_poses_.front();
  }

  const geometry_msgs::Pose& getGoal() const {
    return origin_poses_.back();
  }

  bool has_start_{false};
  bool has_goal_{false};
private:
  ros::Subscriber click_pose_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber move_base_goal_sub_;

  std::vector<geometry_msgs::Pose> origin_poses_;
};

#endif