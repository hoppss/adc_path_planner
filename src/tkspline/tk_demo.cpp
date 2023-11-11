#include "common/math_util.h"
#include "common/type_tool.h"
#include "receiver/receiver.h"
#include "rviz_tool/planner_viz.h"
// #include "rviz_tool/rviz_tool.h"
#include "spline.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle node;
  Receiver recv(node);
  PlannerViz viz(node);

  ros::Rate r(10);

  while (ros::ok()) {
    if (recv.isReady()) {
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
        // std::cout << absl::StrCat(i, ", [", s_list.back(), ", ", x_list.back(), ", ", y_list.back(),
        //                           "]");
      }

      tk::spline xs, ys;
      xs.set_points(s_list, x_list);
      ys.set_points(s_list, y_list);

      // viz
      // origin
      viz.showTrajectoryPath(PoseMsg2Vec2d(poses));
      // spline
      std::vector<Vec2d> spline_xys;
      for (double s = 0.0; s < s_list.back(); s += 0.3) {
        spline_xys.emplace_back(xs(s), ys(s));
      }
      viz.showTrajectoryMarkers(spline_xys);
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