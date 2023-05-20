#include "common/math_util.h"
#include "common/type_tool.h"
#include "receiver/receiver.h"
#include "rviz_tool/planner_viz.h"
#include "rviz_tool/rviz_tool.h"
#include "tinyspline/tinysplinecpp.h"

std::vector<Vec2d> bspline(const std::vector<geometry_msgs::Pose>& poses) {
  // bspline smoothing
  // 0. 获取长度
  double length = 0.;
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    length += common::distance(poses[i].position.x, poses[i].position.y, poses[i + 1].position.x,
                               poses[i + 1].position.y);
  }

  // 1. 确认阶数
  int degree = 3;
  double average_length = length / (poses.size() - 1);
  if (average_length > 10)
    degree = 3;
  else if (average_length > 5)
    degree = 4;
  else
    degree = 5;

  // 创建bspline 对象
  tinyspline::BSpline b_spline_raw(poses.size(), 2, degree);
  std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
  for (size_t i = 0; i != poses.size(); ++i) {
    ctrlp_raw[2 * (i)] = poses[i].position.x;
    ctrlp_raw[2 * (i) + 1] = poses[i].position.y;  // 二维， 每个点连续x,y
  }
  b_spline_raw.setControlPoints(ctrlp_raw);

  std::vector<double> x_list_, y_list_, s_list_;
  double delta_t = 1.0 / length;  // unit
  double tmp_t = 0;
  while (tmp_t < 1) {
    auto result = b_spline_raw.eval(tmp_t).result();
    x_list_.emplace_back(result[0]);
    y_list_.emplace_back(result[1]);
    tmp_t += delta_t;
  }
  auto result = b_spline_raw.eval(1).result();
  x_list_.emplace_back(result[0]);
  y_list_.emplace_back(result[1]);
  s_list_.emplace_back(0);

  // 获取数据
  std::vector<Vec2d> spline_xys;
  spline_xys.emplace_back(x_list_[0], y_list_[0]);
  for (size_t i = 1; i != x_list_.size(); ++i) {
    double dis = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) + pow(y_list_[i] - y_list_[i - 1], 2));
    s_list_.emplace_back(s_list_.back() + dis);
    spline_xys.emplace_back(x_list_[i], y_list_[i]);
  }
  
  return spline_xys;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tinyspline_test");
  ros::NodeHandle node;
  Receiver recv(node);
  PlannerViz viz(node);

  ros::Rate r(10);

  while (ros::ok()) {
    if (recv.is_ready_) {
      // tkspline
      auto poses = recv.getPoses();
      std::vector<double> x_list, y_list, s_list;
      double s_tmp = 0.;
      for (size_t i = 0; i < poses.size(); ++i) {
        x_list.push_back(poses[i].position.x);
        y_list.push_back(poses[i].position.y);
        if (i == 0) {
          s_list.push_back(0.0);
        } else {
          s_tmp += common::distance(poses[i].position.x, poses[i].position.y,
                                    poses[i - 1].position.x, poses[i - 1].position.y);
          s_list.push_back(s_tmp);
        }
        // std::cout << absl::StrCat(i, ", [", s_list.back(), ", ", x_list.back(), ", ",
        // y_list.back(),
        //                           "]");
      }

      // viz
      // origin
      viz.showTrajectoryPath(PoseMsg2Vec2d(poses));
      // tinyspline - bspline
      viz.showTrajectoryMarkers(bspline(poses));
    } else {
      //  viz input pts
      auto poses = recv.getPoses();
      viz.showTrajectoryPath(PoseMsg2Vec2d(poses));
    }
    r.sleep();
    ros::spinOnce();
  }

  ros::shutdown();
}