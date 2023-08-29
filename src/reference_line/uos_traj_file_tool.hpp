#pragma once

#include "reference_line/traj_file_tool.hpp"


// uos debug tool
bool UOSReadTrajFile(const std::string &filename,
                        std::vector<common::State> &complete_rtk_trajectory);