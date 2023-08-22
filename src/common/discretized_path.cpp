#include "common/discretized_path.h"

namespace common {
using common::State;

DiscretizedPath::DiscretizedPath(const std::vector<State> &path_points) {
  path_points_ = path_points;
}

double DiscretizedPath::Length() const {
  if (path_points_.empty()) {
    return 0.0;
  }
  return path_points_.back().s - path_points_.front().s;
}

State DiscretizedPath::Evaluate(const double path_s) const {}

std::vector<State>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const State &tp, const double path_s) {
    return tp.s < path_s;
  };
  return std::lower_bound(path_points_.begin(), path_points_.end(), path_s, func);
}

std::vector<State>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const State &tp) {
    return tp.s < path_s;
  };
  return std::upper_bound(path_points_.begin(), path_points_.end(), path_s, func);
}

// State DiscretizedPath::EvaluateReverse(const double path_s) const {
//   ACHECK(!path_points_.empty());
//   auto it_upper = QueryUpperBound(path_s);
//   if (it_upper == path_points_.begin()) {
//     return path_points_.front();
//   }
//   if (it_upper == path_points_.end()) {
//     return path_points_.back();
//   }
//   return common::InterpolateUsingLinearApproximation(*(it_upper - 1),
//                                                            *it_upper, path_s);
// }
}  // namespace common