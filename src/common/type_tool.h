#pragma once

#include "common/basic_type.hpp"
#include "geometry_msgs/PoseStamped.h"

using common::Vec2d;
using geometry_msgs::Pose;

std::vector<Vec2d> PoseMsg2Vec2d(const std::vector<Pose>& poses);

common::Vec2d Vec2dToState(const common::State& state);
common::State StateToVec2d(const common::Vec2d& pt);


void Vec2dToStateVec(const std::vector<common::Vec2d> &pts,
                 std::vector<common::State> &states);

void StateToVec2dVec(const std::vector<common::State> &states,
                std::vector<common::Vec2d> &pts);