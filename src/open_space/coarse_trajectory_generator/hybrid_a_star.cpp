#include <limits>

#include "tools/timer_tool.hpp"
#include "open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace planning {

using common::Box2d;
using common::Vec2d;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) :
  planner_open_space_config_(open_space_conf) {
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  next_node_num_ =
      planner_open_space_config_.warm_start_config.next_node_num;
  max_steer_angle_ = vehicle_param_.max_steer_angle();
  // traveled_distance, edge 积分步长
  step_size_ = planner_open_space_config_.warm_start_config.step_size;
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config.xy_grid_resolution;
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config.traj_forward_penalty;
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config.traj_back_penalty;
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config.traj_gear_switch_penalty;
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config.traj_steer_penalty;
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config
                                   .traj_steer_change_penalty;
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    AERROR << "ShortestRSP failed";
    return false;
  }
  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  AINFO << "Reach the end configuration with Reed Sharp";
  AINFO << "current node x" << current_node->GetX() << ","
        << current_node->GetY();
  AINFO << "final node x" << end_node_->GetX() << "," << end_node_->GetY();
  AINFO << "distance "
        << sqrt(pow((current_node->GetX() - end_node_->GetX()), 2) +
                pow((current_node->GetY() - end_node_->GetY()), 2));
  AINFO << "delta phi" << current_node->GetPhi() - end_node_->GetPhi();
  AINFO << "reed shepp set_type,gear,length";
  for (size_t i = 0; i < reeds_shepp_to_check->segs_types.size(); i++) {
    AINFO << reeds_shepp_to_check->segs_types[i] << ", "
          << reeds_shepp_to_check->gear[i] << ","
          << reeds_shepp_to_check->segs_lengths[i];
  }
  AINFO << reeds_shepp_to_check->x.front() << ","
        << reeds_shepp_to_check->y.front();
  AINFO << reeds_shepp_to_check->x.back() << ","
        << reeds_shepp_to_check->y.back();
  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          AINFO << "collision start at x: " << linesegment.start().x();
          AINFO << "collision start at y: " << linesegment.start().y();
          AINFO << "collision end at x: " << linesegment.end().x();
          AINFO << "collision end at y: " << linesegment.end().y();
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  // TODO: rsp node only push to close_set ?
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}
// Left+ steer  0    Max- steer
//  4 3         2    1 0
//  9 8         7    6 5
std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  AINFO << "--- arc " << arc << ", step size: " << step_size_ << ", steps: " << arc/step_size_;
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = common::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "result size check failed " << current_node->GetIndex();
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "states sizes are not equal" << current_node->GetIndex();
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal, "
           << "result->x.size(): " << result->x.size()
           << "result->y.size()"   << result->y.size()
           << "result->phi.size()" << result->phi.size()
           << "result->steer.size()" << result->steer.size();
    return false;
  }

  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result";
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  return true;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::LineSegment2d>>& obstacles,
    HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;
  // std::vector<std::vector<common::LineSegment2d>>
  //     obstacles_linesegments_vec;
  // for (const auto& obstacle_vertices : obstacles_vertices_vec) {
  //   size_t vertices_num = obstacle_vertices.size();
  //   std::vector<common::LineSegment2d> obstacle_linesegments;
  //   for (size_t i = 0; i < vertices_num - 1; ++i) {
  //     common::LineSegment2d line_segment = common::LineSegment2d(
  //         obstacle_vertices[i], obstacle_vertices[i + 1]);
  //     obstacle_linesegments.emplace_back(line_segment);
  //   }
  //   obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  // }
  // obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  obstacles_linesegments_vec_ = obstacles;
  std::stringstream ssm;
  ssm << "roi boundary" << std::endl;
  for (auto vec : obstacles_linesegments_vec_) {
    for (auto linesg : vec) {
      ssm << linesg.start().x() << "," << linesg.start().y() << std::endl;
      ssm << linesg.end().x() << "," << linesg.end().y() << std::endl;
    }
  }
  ssm << "--vehicle start box" << std::endl;
  Vec2d sposition(sx, sy);
  Vec2d svec_to_center((vehicle_param_.front_edge_to_center() -
                        vehicle_param_.back_edge_to_center()) /
                           2.0,
                       (vehicle_param_.left_edge_to_center() -
                        vehicle_param_.right_edge_to_center()) /
                           2.0);
  Vec2d scenter(sposition + svec_to_center.rotate(sphi));
  Box2d sbox(scenter, sphi, vehicle_param_.length(), vehicle_param_.width());
  for (auto corner : sbox.GetAllCorners())
    ssm << corner.x() << "," << corner.y() << std::endl;
  ssm << "--vehicle end box" << std::endl;
  Vec2d eposition(ex, ey);
  Vec2d evec_to_center((vehicle_param_.front_edge_to_center() -
                        vehicle_param_.back_edge_to_center()) /
                           2.0,
                       (vehicle_param_.left_edge_to_center() -
                        vehicle_param_.right_edge_to_center()) /
                           2.0);
  Vec2d ecenter(eposition + evec_to_center.rotate(ephi));
  Box2d ebox(ecenter, ephi, vehicle_param_.length(), vehicle_param_.width());
  for (auto corner : ebox.GetAllCorners())
    ssm << corner.x() << "," << corner.y() << std::endl;
  // load XYbounds
  ssm << "--" << std::endl;
  ssm << "XYbounds" << std::endl;
  ssm << XYbounds[0] << ", " << XYbounds[1] << std::endl;
  ssm << XYbounds[2] << ", " << XYbounds[3] << std::endl;
  XYbounds_ = XYbounds;
  AINFO << ssm.str();
  // load nodes and obstacles
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
  AINFO << "start node" << sx << "," << sy << "," << sphi;
  AINFO << "end node " << ex << "," << ey << "," << ephi;
  if (!ValidityCheck(start_node_)) {
    AERROR << "start_node in collision with obstacles";
    AERROR << start_node_->GetX() << "," << start_node_->GetY() << ","
           << start_node_->GetPhi();
    AERROR << ssm.str();
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    AERROR << "end_node in collision with obstacles";
    return false;
  }

  common::Timer timer;
  timer.begin();
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);
  AINFO << "2d dp map time: " << timer.end() << " ms";
  // load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  size_t explored_node_num = 0;
  const auto astar_start_time = std::chrono::high_resolution_clock::now();
  double heuristic_time = 0.0;
  double rs_time = 0.0;

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    // all explore nodes, not only open list, not pop closed node
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    AINFO << "explore " << current_node->GetIndex();
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    const auto rs_start_time = std::chrono::high_resolution_clock::now();
    if (AnalyticExpansion(current_node)) {
      break;
    }
    const auto rs_end_time = std::chrono::high_resolution_clock::now();
    rs_time += std::chrono::duration_cast<std::chrono::milliseconds>(rs_end_time - rs_start_time).count();
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        const auto start_time = std::chrono::high_resolution_clock::now();
        CalculateNodeCost(current_node, next_node);
        const auto end_time = std::chrono::high_resolution_clock::now();
        heuristic_time += std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  if (final_node_ == nullptr) {
    AERROR << "Hybrid A searching return null ptr(open_set ran out)";
    AINFO << ssm.str();
    return false;
  }
  if (!GetResult(result)) {
    AERROR << "GetResult failed";
    return false;
  }
  AINFO << "explored node num is " << explored_node_num;
  AINFO << "heuristic time is " << heuristic_time;
  AINFO << "reed shepp time is " << rs_time;
  AINFO << "hybrid astar total time is "
         << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - astar_start_time).count();
  return true;
}
}  // namespace planning