#include "common/discretized_path.h"

namespace common {
using common::State;

DiscretizedPath::DiscretizedPath(std::vector<State> path_points)
    : std::vector<State>(std::move(path_points)) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

State DiscretizedPath::Evaluate(const double path_s) const {
  ACHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return common::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

std::vector<State>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const State &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

State DiscretizedPath::EvaluateReverse(const double path_s) const {
  ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return common::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                           *it_upper, path_s);
}

std::vector<State>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const State &tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}
}  // namespace common