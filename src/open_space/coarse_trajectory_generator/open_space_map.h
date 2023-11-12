#pragma once
#include <vector>
#include <fstream>

#include "common/basic_type.hpp"
#include "common/line_segment2d.h"

using common::LineSegment2d;

class OpenSpaceMap {
  public:
    OpenSpaceMap() = default;
    ~OpenSpaceMap() = default;

    bool loadMap();
    std::vector<std::vector<common::LineSegment2d>> getBound();
    common::State getGoal() const { return start_; };

  private:
    std::vector<std::vector<common::LineSegment2d>> bounds_;
    common::State start_;
};