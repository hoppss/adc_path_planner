#include "common/vehicle_config_helper.h"

#include <algorithm>
#include <cmath>

namespace common {

VehicleConfig VehicleConfigHelper::vehicle_config_;
bool VehicleConfigHelper::is_init_ = false;

// TODO: 单例模式， 默认构造函数
// VehicleConfigHelper::VehicleConfigHelper() {}

void VehicleConfigHelper::Init() {
  // Init(FLAGS_vehicle_config_path);
  // hardcode param
  AINFO << "VehicleConfigHelper Once";
  vehicle_config_.vehicle_param.brand = "LINCOLN_MKZ";
  vehicle_config_.vehicle_param.vehicle_id = "car1";
  vehicle_config_.vehicle_param.front_edge_to_center_ = 3.89;
  vehicle_config_.vehicle_param.back_edge_to_center_ = 1.043;
  vehicle_config_.vehicle_param.left_edge_to_center_ = 1.055;
  vehicle_config_.vehicle_param.right_edge_to_center_ = 1.055;
  vehicle_config_.vehicle_param.length_ = 4.933;
  vehicle_config_.vehicle_param.width_ = 2.11;
  vehicle_config_.vehicle_param.height_ = 1.48;
  vehicle_config_.vehicle_param.min_turn_radius_ = 5.05386147161;
  vehicle_config_.vehicle_param.max_steer_angle_ = 8.20304748437;
  vehicle_config_.vehicle_param.max_steer_angle_rate_ = 8.55211;
  vehicle_config_.vehicle_param.steer_ratio_ = 16;
  vehicle_config_.vehicle_param.wheel_base_ = 2.8448;

  is_init_ = true;
}

void VehicleConfigHelper::Init(const std::string &config_file) {
  VehicleConfig params;
  //   Init(params);
}

void VehicleConfigHelper::Init(const VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

double VehicleConfigHelper::MinSafeTurnRadius() {
  const VehicleParam& param = vehicle_config_.vehicle_param;
  double lat_edge_to_center =
      std::max(param.left_edge_to_center(), param.right_edge_to_center());
  double lon_edge_to_center =
      std::max(param.front_edge_to_center(), param.back_edge_to_center());
  return std::sqrt((lat_edge_to_center + param.min_turn_radius()) *
                       (lat_edge_to_center + param.min_turn_radius()) +
                   lon_edge_to_center * lon_edge_to_center);
}

common::Box2d VehicleConfigHelper::GetBoundingBox(const common::State &path_point) {
  const VehicleParam &vehicle_param = vehicle_config_.vehicle_param;
  double diff_truecenter_and_pointX =
      (vehicle_param.front_edge_to_center() - vehicle_param.back_edge_to_center()) / 2.0;
  common::Vec2d true_center(
      path_point.x() + diff_truecenter_and_pointX * std::cos(path_point.theta()),
      path_point.y() + diff_truecenter_and_pointX * std::sin(path_point.theta()));

  return common::Box2d(true_center, path_point.theta(), vehicle_param.length(),
                       vehicle_param.width());
}

}  // namespace common