#include "rviz_tool.h"


ColorRGBA RvizTool::createColor(ColorType color, double a) {
  ColorRGBA result;
  result.a = a;

  switch (color) {
    case ColorType::WHITE:
      result = createColor(255.0, 255.0, 255.0, a);
      break;
    case ColorType::BLACK:
      result = createColor(0., 0., 0., a);
      break;
    case ColorType::RED:
      result = createColor(255., 0., 0., a);
      break;
    case ColorType::GREEN:
      result = createColor(0., 255., 0., a);
      break;
    case ColorType::BLUE:
      result = createColor(0., 0., 255., a);
      break;
    case ColorType::YELLOW:
      result = createColor(255., 255., 0., a);
      break;
    case ColorType::CYAN:
      result = createColor(0., 255., 255., a);
      break;
    case ColorType::MAGENTA:
      result = createColor(255., 0., 255., a);
      break;
    case ColorType::GRAY:
      result = createColor(128., 0., 128., a);
      break;
    case ColorType::PURPLE:
      result = createColor(0., 0., 0., a);
      break;
    case ColorType::PINK:
      result = createColor(255., 192., 203., a);
      break;
    case ColorType::LIGHT_BLUE:
      result = createColor(173., 216., 230., a);
      break;
    case ColorType::LIME_GREEN:
      result = createColor(50., 205., 50., a);
      break;
    case ColorType::SLATE_GRAY:
      result = createColor(112, 128, 144, a);
      break;
    default:
      result = createColor(0., 255., 0., a);
      break;
  }

  return result;
}

ColorRGBA RvizTool::createColor(double r, double g, double b, double a) {
  ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;

  return result;
}

visualization_msgs::Marker RvizTool::createMarker(std::string frame, std::string ns,
                                                  ColorRGBA color) {
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = 100;

  m.lifetime = ros::Duration(0.0);

  m.color = color;

  m.type = visualization_msgs::Marker::CUBE;

  m.scale.x = m.scale.y = m.scale.z = 1.0;

  m.pose.orientation.w = 1.0;

  return m;
}