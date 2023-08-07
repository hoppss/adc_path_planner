#pragma once

#include "common/vec2d.h"
#include "geometry_msgs/PoseStamped.h"

using common::Vec2d;
using geometry_msgs::Pose;

std::vector<Vec2d> PoseMsg2Vec2d(const std::vector<Pose>& poses);