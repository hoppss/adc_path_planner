#include <cstdlib>
#include "absl/strings/str_split.h"
#include "absl/strings/numbers.h"
#include "open_space/coarse_trajectory_generator/open_space_map.h"
#include "tools/log.h"

namespace {
    using common::LineSegment2d;
    using common::Vec2d;
}

bool OpenSpaceMap::loadMap() {
    std::string base_path = std::getenv("HOME");
    AINFO << "Home path " << base_path;

    std::string file_name = base_path + "/Documents/catkin_ws/src/adc_path_planner/src/open_space/coarse_trajectory_generator/reverse_verticle_2.txt";
    AINFO << "File path " << file_name;

	std::ifstream in_file;
	in_file.open(file_name.c_str(), std::ios::in);
	if (!in_file.is_open())
	{
        std::cout << "Can't open " << file_name << std::endl;
		return false;
	}

    std::vector<LineSegment2d> bound;
	std::string buff;
    bool first_line = true;
	while (getline(in_file, buff))
	{
        const std::vector<absl::string_view> data = absl::StrSplit(buff, " ");
        if (first_line) {
            double x, y, theta;
            absl::SimpleAtod(data[0], &x);
            absl::SimpleAtod(data[1], &y);
            absl::SimpleAtod(data[2], &theta);
            start_.set_x(x);
            start_.set_y(y);
            start_.set_theta(theta);
            first_line = false;
            AINFO << "start: x " << start_.x() << ", y: " << start_.y() << ", theta: " << start_.theta();
        } else {
            double x1, y1, x2, y2;
            absl::SimpleAtod(data[0], &x1);
            absl::SimpleAtod(data[1], &y1);
            absl::SimpleAtod(data[2], &x2);
            absl::SimpleAtod(data[3], &y2);
            bound.emplace_back(Vec2d(x1, y1), Vec2d(x2, y2));
            AINFO << "x1: " << x1 << ", y: " << y1 << ", x2: " << x2 << ", y2: " << y2;
        }
    }
    in_file.close();
    bounds_.push_back(bound);

    return true;
}

std::vector<std::vector<LineSegment2d>> OpenSpaceMap::getBound() {
    return bounds_;
}