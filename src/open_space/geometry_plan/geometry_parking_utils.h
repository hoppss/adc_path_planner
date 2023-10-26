#pragma once

#include "common/box2d.h"
#include "common/aabox2d.h"
#include "vehicle_conf.h"

using Vec2d = common::Vec2d;
using Box2d = common::Box2d;
using AABox2d = common::AABox2d;
using LineSegment2d = common::LineSegment2d;

struct Pose
{
    Vec2d pos;
    double theta;
    Vec2d dir;
};

struct LineCirclePath
{
    Pose start;
    std::vector<double> lengths;
    std::vector<char> types;
    std::vector<double> kappas;
    std::vector<Pose> ends;
};

Pose GetPoseAlongArc(const Pose &start, double kappa, double length, char type);

std::vector<Pose> ConvertPathToDiscretePoses(const LineCirclePath& path, double step);

bool HasOverlapWithPath(const LineCirclePath& path, const std::vector<LineSegment2d>& boundaries, double resolution);