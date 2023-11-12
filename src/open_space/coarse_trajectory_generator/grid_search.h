#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "tools/log.h"
#include "common/line_segment2d.h"
#include "open_space/planner_open_space_config.h"

namespace planning {

class Node2d {
 public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    x_ = x;
    y_ = y;
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double cost) { cost_ = cost; }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  // fixed in grid_search::checkConstraint
  double GetX(const double xy_resolution, const std::vector<double>& XYbounds) const {
     return XYbounds[0] + grid_x_ * xy_resolution;
  }
  double GetY(const double xy_resolution, const std::vector<double>& XYbounds) const {
    return XYbounds[2] + grid_y_ * xy_resolution;
  }
  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetPathCost() const { return path_cost_; }   // G
  double GetHeuCost() const { return heuristic_; }    // H
  double GetCost() const { return cost_; }            // F
  const std::string& GetIndex() const { return index_; }
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
  static std::string CalcIndex(const double x, const double y,
                               const double xy_resolution,
                               const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return absl::StrCat(x_grid, "_", y_grid);
  }

 private:
  double x_ = 0.0;  // m
  double y_ = 0.0;
  int grid_x_ = 0;  // grid index
  int grid_y_ = 0;
  double path_cost_ = 0.0;  // g
  double heuristic_ = 0.0;  // h
  double cost_ = 0.0;       // f
  std::string index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~GridSearch() = default;
  bool GenerateAStarPath(
      const double sx, const double sy, const double ex, const double ey,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::LineSegment2d>>&
          obstacles_linesegments_vec,
      GridAStartResult* result);
  bool GenerateDpMap(
      const double ex, const double ey, const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::LineSegment2d>>&
          obstacles_linesegments_vec);
  double CheckDpMap(const double sx, const double sy);

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
  void LoadGridAStarResult(GridAStartResult* result);

 private:
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;   // grid size width
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_; // indicator astar is success
  std::vector<std::vector<common::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  // dp is closed list of dijkstra, search from goal 
  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};
}  // namespace planning