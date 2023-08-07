#ifndef __PLANNER_RVIZ_TOOL_HPP__
#define __PLANNER_RVIZ_TOOL_HPP__

#include <std_msgs/ColorRGBA.h>

#include "visualization_msgs/MarkerArray.h"

namespace {
using std_msgs::ColorRGBA;
};

class RvizTool {
 public:
  RvizTool() = default;
  ~RvizTool() = default;

  enum class ColorType {
    WHITE = 0,
    BLACK,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    CYAN,
    MAGENTA,
    GRAY,
    PURPLE,
    PINK,
    LIGHT_BLUE,
    LIME_GREEN,
    SLATE_GRAY
  };

  static ColorRGBA createColor(ColorType color, double a = 1.0);
  static ColorRGBA createColor(double r, double g, double b, double a = 1.0);

  static visualization_msgs::Marker createMarker(std::string frame = "map",
                                                 std::string ns = "namespace",
                                                 ColorRGBA color = createColor(ColorType::GREEN));
};

#endif