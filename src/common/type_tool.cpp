#include "common/type_tool.h"

std::vector<Vec2d> PoseMsg2Vec2d(const std::vector<Pose>& poses) {
  std::vector<Vec2d> result;

  for (const auto& p : poses) {
    result.emplace_back(p.position.x, p.position.y);
  }

  return result;
}

common::Vec2d vec2dToState(const common::State& state) {
  return {state.x(), state.y()};
}
common::State stateToVec2d(const common::Vec2d& pt) {
  common::State s;
  s.set_x(pt.x());
  s.set_y(pt.y());
  return s;
}

void Vec2dToStateVec(const std::vector<common::Vec2d> &pts,
                 std::vector<common::State> &states) {
  if (pts.empty()) {
    states.clear();
    return;
  }

  for (const auto &pt : pts) {
    common::State state;
    state.set_x(pt.x());
    state.set_y(pt.y());
    states.emplace_back(std::move(state));
  }
}

void StateToVec2dVec(const std::vector<common::State> &states,
                std::vector<common::Vec2d> &pts) {
  if (states.empty()) {
    pts.clear();
    return;
  }

  for (const auto &state : states) {
    common::Vec2d pt;
    pt.set_x(state.x());
    pt.set_y(state.y());
    pts.emplace_back(std::move(pt));
  }
}