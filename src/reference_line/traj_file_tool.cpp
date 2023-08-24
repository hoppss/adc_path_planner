#include "reference_line/traj_file_tool.hpp"

#include "common/basic_type.hpp"

namespace {
using common::AnchorPoint;
using common::State;
using common::Vec2d;
}  // namespace

void getAnchorPoints(std::vector<Vec2d> &raw_center_line,
                     std::vector<AnchorPoint> &anchor_points) {
  anchor_points.clear();

  for (int i = 0; i < static_cast<int>(raw_center_line.size()); i++) {
    AnchorPoint anchor_point;
    Vec2d raw_point = raw_center_line.at(i);
    anchor_point.path_point = raw_point;
    anchor_point.longitudinal_bound = 0.1;
    anchor_point.lateral_bound = 0.1;
    anchor_points.push_back(anchor_point);
  }
}

double CalculateKappa(double x1, double y1, double x2, double y2, double x3,
                      double y3) {
  double v1[] = {x1 - x2, y1 - y2};
  double v2[] = {x2 - x3, y2 - y3};
  double v3[] = {x3 - x1, y3 - y1};
  double a = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  double b = sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
  double c = sqrt(v3[0] * v3[0] + v3[1] * v3[1]);
  double d = sqrt((a + b + c) * (a + b - c) * (b + c - a) * (c + a - b));
  double abc = a * b * c;
  double sign = (v1[0] * v2[1] - v1[1] * v2[0] > 0) ? 1.0 : -1.0;
  return (abc < 0.00001) ? 0.0 : sign * d / abc;
}

std::vector<double> getGlobalPathCurve(std::vector<common::State> &points,
                                       double distance) {
  // 计算曲率
  std::vector<double> px, py, kappa;
  int count = static_cast<int>(points.size());
  if (count < 2) return kappa;

  for (int i = 0; i < count; i++) {
    px.push_back(points[i].x());
    py.push_back(points[i].y());
  }

  for (int i = 0; i < count; ++i) {
    int curr = i, prev = i, next = i;
    double dis_prev = 0;
    while (prev >= 0) {
      dis_prev = sqrt((px[curr] - px[prev]) * (px[curr] - px[prev]) +
                      (py[curr] - py[prev]) * (py[curr] - py[prev]));
      if (dis_prev >= distance) break;
      --prev;
    }
    double dis_next = 0;
    while (next < count) {
      dis_next = sqrt((px[curr] - px[next]) * (px[curr] - px[next]) +
                      (py[curr] - py[next]) * (py[curr] - py[next]));
      if (dis_next >= distance) break;
      ++next;
    }

    if (dis_prev >= distance && dis_next >= distance) {
      prev = std::max(prev, 0);
      next = std::min(next, count - 1);
      double k = CalculateKappa(px[prev], py[prev], px[curr], py[curr],
                                px[next], py[next]);
      // ROS_INFO("i = %d, k = %.6f", i, k);
      if (std::isnan(k) != 0 || std::isinf(k) != 0) {
        k = 0.0;
      }
      kappa.push_back(k);
      points[i].set_kappa(k);
    } else {
      kappa.push_back(0);
      points[i].set_kappa(0.0);
    }
  }

  // ROS_INFO("kappa.size() = %d", kappa.size());

  // // 设置曲率
  // for(int i = 0; i < points_size; i++)
  // {
  //     trajectory_points[i].set_kappa(kappa[i]);
  // }
  return kappa;
}

std::vector<double> getGlobalPathHeading(std::vector<common::Vec2d> &points) {
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> headings;

  if (points.size() < 2) {
    return headings;
  }

  size_t points_size = points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (points[i + 1].x() - points[i].x());
      y_delta = (points[i + 1].y() - points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (points[i].x() - points[i - 1].x());
      y_delta = (points[i].y() - points[i - 1].y());
    } else {
      x_delta = 0.5 * (points[i + 1].x() - points[i - 1].x());
      y_delta = 0.5 * (points[i + 1].y() - points[i - 1].y());
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double theta = std::atan2(dys[i], dxs[i]);
    if (theta < 0) theta += 2 * M_PI;
    // points[i].theta = theta;
    headings.push_back(theta);
  }
  return headings;
}

void getGlobalAccumsS(std::vector<State> &points) {
  if (points.empty()) return;

  for (size_t i = 0; i < points.size(); ++i) {
    if (i == 0) {
      points[i].set_s(0);
      continue;
    }

    points[i].set_s(points[i - 1].s() +
                    common::distance(points[i - 1].x(), points[i].x(),
                                     points[i - 1].y(), points[i].y()));
  }
}

void csvDataSaveHandle(const std::vector<State> &states,
                       std::vector<State> &smoothed_kappa) {
  static bool once = false;
  if (once) return;

  time_t now_time = time(NULL);
  struct tm *p;
  p = gmtime(&now_time);
  char filename[256] = {0};
  sprintf(filename, "/home/uisee/femsmooth-%d-%d-%d-%d-%d-%d.csv",
          1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday, 8 + p->tm_hour,
          p->tm_min, p->tm_sec);  // kappa 对比
  std::fstream mystream;
  mystream.open(filename, std::ios_base::out);

  std::vector<double> s_kappa = getGlobalPathCurve(smoothed_kappa);
  std::vector<double> raw_kappa;

  if (!states.empty()) {
    for (auto &state : states) {
      raw_kappa.emplace_back(state.kappa());
    }

    AINFO << "kappa result size " << raw_kappa.size() << s_kappa.size();
    for (size_t i = 0; i < std::min(raw_kappa.size(), s_kappa.size()); ++i) {
      mystream << i << "," << raw_kappa[i] << "," << s_kappa[i] << ","
               << std::endl;
    }
    mystream << std::endl;
    mystream.close();
  } else {
    mystream << std::endl;
    mystream.close();
  }
  once = true;
}

bool ReadTrajectoryFile(const std::string &filename,
                        std::vector<common::State> &complete_rtk_trajectory_) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open() || file_in.peek() == EOF) {
    AERROR << "[ReadTrajectoryFile] cannot open trajectory file: " << filename;
    return false;
  }

  std::string line;
  getline(file_in, line);
  // if(line.find("@type:routing", 0) != std::string::npos ||
  //     line.find("@type:convert", 0) != std::string::npos)
  // {
  //     traj_type = 1;
  // }
  // else
  // {
  //     traj_type = 0;
  // }
  AINFO << "hadmap start read trajectory file " << filename;
  while (!file_in.eof()) {
    getline(file_in, line);
    if (line == "" || line.find("nan") != std::string::npos) continue;

    const std::vector<std::string> tokens =
        absl::StrSplit(line, absl::ByAnyChar(","));
    if (tokens.size() < 11) {
      AWARN << "[ReadTrajectoryFile]:the data dimension does not match" << line;
      continue;
    }

    common::State point;
    point.set_x(std::stod(tokens[0]));
    point.set_y(std::stod(tokens[1]));
    // point.set_z(std::stod(tokens[2]));
    point.set_v(std::stod(tokens[3]));
    point.set_a(std::stod(tokens[4]));
    point.set_kappa(std::stod(tokens[5]));
    point.set_t(std::stod(tokens[7]));
    point.set_theta(std::stod(tokens[8]));
    point.set_s(std::stod(tokens[10]));

    complete_rtk_trajectory_.push_back(std::move(point));
  }

  file_in.close();
  AINFO << "[ReadTrajectoryFile] read trajectory file successfully points"
        << complete_rtk_trajectory_.size();

  return true;
}

void trajectoryToFile(std::string file_name,
                      const std::vector<common::State> &trajectory_points) {
  std::ofstream outfile(file_name);
  outfile.precision(15);
  // 增加routing标识符
  // outfile << "@type:routing" << std::endl;

  for (size_t i = 0; i < trajectory_points.size(); i++) {
    outfile << std::setprecision(12) << trajectory_points[i].x()
            << ", "  // 0: UTM-x
            << std::setprecision(12) << trajectory_points[i].y()
            << ", "         // 1: UTM-y
            << 0.0 << ", "  // 2: altitude
            << 0.0 << ", "  // 3: horizontal speed
            << 0.0 << ", "  // 4: acceleration
            << std::setprecision(7) << trajectory_points[i].kappa()
            << ", "         //<< hypot(ins_info.longitudinal_speed ,
                            // ins_info.lateral_speed) << ", "
            << 0.0 << ", "  //<< hypot(ins_info.longitudinal_accelerate ,
                            // ins_info.longitudinal_accelerate) << ", "
            << 0.0 << ", "  // 7: delta time
            << std::setprecision(7) << trajectory_points[i].theta()
            << ", "         // 8: ins_info.yaw * M_PI / 180 - M_PI_2 << ", "
            << 0.0 << ", "  // 9: gear
            << trajectory_points[i].s() << ", "  // 10:
            << 0.0 << ", "                     // 11
            << 0.0 << ", "  // 12: pilot mode: 0-manual, 1-autopilot
            << 0.0 << ", "  // 13
            << 0.0 << ", "  // 14
            << 0.0 << ", "  // 15:
            << 0.0 << ", "  // 16:
            << 0.0 << ", "  // 17
            << 0.0 << ", "  // 18: yaw_rate
            << 0.0 << ", "  // 19
            << 0.0 << ", "  // 20
            << 0.0          // 21: lat_acc
            << std::endl;
  }
  outfile.close();
  AINFO << "traj save to file " << file_name;
}