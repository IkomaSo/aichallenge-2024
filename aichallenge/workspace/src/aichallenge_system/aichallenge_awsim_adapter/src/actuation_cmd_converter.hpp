// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include <raw_vehicle_cmd_converter/accel_map.hpp>
#include <raw_vehicle_cmd_converter/brake_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <list>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;

class ActuationCmdConverter : public rclcpp::Node {
public:
  explicit ActuationCmdConverter(const rclcpp::NodeOptions &node_options);

private:
  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steer_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_ackermann_;

  void on_actuation_cmd(const ActuationCommandStamped::ConstSharedPtr msg);
  void on_gear_report(const GearReport::ConstSharedPtr msg);
  void on_steering_report(const SteeringReport::ConstSharedPtr msg);
  void on_velocity_report(const VelocityReport::ConstSharedPtr msg);

  double get_acceleration(const ActuationCommandStamped &cmd,
                          const double velocity);
  raw_vehicle_cmd_converter::AccelMap accel_map_;
  raw_vehicle_cmd_converter::BrakeMap brake_map_;
  GearReport::ConstSharedPtr gear_report_;
  VelocityReport::ConstSharedPtr velocity_report_;
  SteeringReport::ConstSharedPtr steering_report_;

  // Parameters
  double max_steering_rotation_rate_;
  int moving_average_window_size_;

  std::list<SteeringReport> steering_list_;

  double clip_steering(const ActuationCommandStamped::ConstSharedPtr cmd);
  double steering_angle_delay_;
  double accleration_delay_;
};

#endif // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
