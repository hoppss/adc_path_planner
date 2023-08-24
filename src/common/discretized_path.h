#pragma once

#include "common/basic_type.hpp"
#include "common/linear_interpolation.h"

namespace common {
class DiscretizedPath : public std::vector<common::State> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::State> path_points);

  double Length() const;

  common::State Evaluate(const double path_s) const;

  common::State EvaluateReverse(const double path_s) const;

 protected:
  std::vector<common::State>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<common::State>::const_iterator QueryUpperBound(
      const double path_s) const;
};
}  // namespace common