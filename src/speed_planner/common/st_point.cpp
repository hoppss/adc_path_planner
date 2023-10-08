#include "speed_planner/common/st_point.h"

#include "absl/strings/str_format.h"

namespace planning {

using absl::StrFormat;

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const common::Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }

std::string STPoint::DebugString() const {
  return StrFormat("{ \"s\" : %.6f, \"t\" : %.6f }", s(), t());
}

}  // namespace planning