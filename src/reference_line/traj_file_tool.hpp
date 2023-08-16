#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

#include <absl/strings/str_split.h>

#include "common/basic_type.hpp"
#include "common/vec2d.h"
#include "common/math_util.h"

#include "tools/log.h"

double CalculateKappa(double x1, double y1, double x2, double y2, double x3,
                      double y3);

std::vector<double> getGlobalPathCurve(std::vector<common::State> &points,
                                       double distance = 10);

std::vector<double> getGlobalPathHeading(std::vector<common::Vec2d> &points);

void getGlobalAccumsS(std::vector<common::State> &points);

void csvDataSaveHandle(const std::vector<common::State> &states,
                       std::vector<common::Vec2d> &smoothed_kappa);

bool ReadTrajectoryFile(const std::string &filename,
                        std::vector<common::State> &complete_rtk_trajectory);

void trajectoryToFile(std::string file_name,
                      const std::vector<common::State> &trajectory_points);