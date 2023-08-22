#pragma once

#include "common/basic_type.hpp"

namespace common {
class DiscretizedPath {
 public:
  DiscretizedPath() = default;
  explicit DiscretizedPath(const std::vector<common::State>& path_points);

  double Length() const;

  common::State Evaluate(const double path_s) const;
  // common::State EvaluateReverse(const double path_s) const;

 private:
  std::vector<common::State>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<common::State>::const_iterator QueryUpperBound(
      const double path_s) const;

  std::vector<common::State> path_points_;
};
}  // namespace common