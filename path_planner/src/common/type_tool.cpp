#include "common/type_tool.h"

std::vector<Vec2d> PoseMsg2Vec2d(const std::vector<Pose>& poses) {
  std::vector<Vec2d> result;

  for (const auto& p : poses) {
    result.emplace_back(p.position.x, p.position.y);
  }

  return result;
}