#include "reference_line/uos_traj_file_tool.hpp"

bool UOSReadTrajFile(const std::string &filename,
                        std::vector<common::State> &complete_rtk_trajectory_) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open() || file_in.peek() == EOF) {
    AERROR << "[ReadTrajectoryFile] cannot open trajectory file: " << filename;
    return false;
  }

  std::string line;
  getline(file_in, line);
  // if(line.find("@type:routing", 0) != std::string::npos ||
  //     line.find("@type:convert", 0) != std::string::npos)
  // {
  //     traj_type = 1;
  // }
  // else
  // {
  //     traj_type = 0;
  // }
  AINFO << "hadmap start read trajectory file " << filename;
  while (!file_in.eof()) {
    getline(file_in, line);
    if (line == "" || line.find("nan") != std::string::npos) continue;

    const std::vector<std::string> tokens =
        absl::StrSplit(line, absl::ByAnyChar(","));
    if (tokens.size() < 6) {
      AWARN << "[ReadTrajectoryFile]:the data dimension does not match" << line;
      continue;
    }

    common::State point;
    point.set_x(std::stod(tokens[0]));
    point.set_y(std::stod(tokens[1]));
    point.set_theta(std::stod(tokens[2]));
    point.set_s(std::stod(tokens[3]));

    complete_rtk_trajectory_.push_back(std::move(point));
  }

  file_in.close();
  AINFO << "[ReadTrajectoryFile] read trajectory file successfully points"
        << complete_rtk_trajectory_.size();

  return true;
}