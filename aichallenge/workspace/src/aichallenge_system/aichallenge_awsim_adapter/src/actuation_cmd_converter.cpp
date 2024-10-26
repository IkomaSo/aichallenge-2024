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

#include "actuation_cmd_converter.hpp"
#include <iostream>
#include <queue>
#include <utility>

ActuationCmdConverter::ActuationCmdConverter(const rclcpp::NodeOptions & node_options)
: Node("actuation_cmd_converter", node_options),
  max_steering_rotation_rate_(declare_parameter<float>("max_steering_rotation_rate", 0.35)),
  moving_average_window_size_(declare_parameter<int>("moving_average_window_size", 3)),
  steering_angle_delay_(declare_parameter<float>("steering_angle_delay", 0.065)),
  accleration_delay_(declare_parameter<float>("accleration_delay", 0.1))
{
  using std::placeholders::_1;

  // Parameters
  const std::string csv_path_accel_map = declare_parameter<std::string>("csv_path_accel_map");
  const std::string csv_path_brake_map = declare_parameter<std::string>("csv_path_brake_map");

  // Subscriptions
  sub_actuation_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1, std::bind(&ActuationCmdConverter::on_actuation_cmd, this, _1));
  sub_gear_ = create_subscription<GearReport>(
    "/vehicle/status/gear_status", 1, std::bind(&ActuationCmdConverter::on_gear_report, this, _1));
  sub_steer_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1, std::bind(&ActuationCmdConverter::on_steering_report, this, _1));
  sub_velocity_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1, std::bind(&ActuationCmdConverter::on_velocity_report, this, _1));

  // Publishers
  pub_ackermann_ = create_publisher<AckermannControlCommand>("/awsim/control_cmd", 1);

  // Load accel/brake map
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
    throw std::runtime_error("Cannot read accelmap.");
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
    throw std::runtime_error("Cannot read brakemap.");
  }
}

void ActuationCmdConverter::on_gear_report(const GearReport::ConstSharedPtr msg)
{
  gear_report_ = msg;
}

void ActuationCmdConverter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}



void ActuationCmdConverter::on_steering_report(const SteeringReport::ConstSharedPtr msg)
{
  steering_list_.push_back(*msg);
  if (steering_list_.size() > static_cast<size_t>(moving_average_window_size_)+1) {
    steering_list_.pop_front();
  }
}

std::list<ActuationCommandStamped> past_output_steering_angle_;
std::queue<std::pair<float, float>> past_output_acceleration;
void ActuationCmdConverter::on_actuation_cmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  // Wait for input data
  if (!gear_report_ || !velocity_report_) {
    return;
  }

  const double velocity = std::abs(velocity_report_->longitudinal_velocity);
  const double acceleration = get_acceleration(*msg, velocity);

  // Publish ControlCommand
  constexpr float nan = std::numeric_limits<double>::quiet_NaN();
  AckermannControlCommand output;
  output.stamp = msg->header.stamp;
  output.lateral.steering_tire_angle = static_cast<float>(msg->actuation.steer_cmd);
  // output.lateral.steering_tire_angle = static_cast<float>(clip_steering(msg));
  output.lateral.steering_tire_rotation_rate = nan;
  output.longitudinal.speed = nan;
  output.longitudinal.acceleration = static_cast<float>(acceleration);
  past_output_steering_angle_.push_back(*msg);
  past_output_acceleration.push(std::make_pair(rclcpp::Time(output.stamp).seconds(), output.longitudinal.acceleration));
  if(rclcpp::Time(output.stamp).seconds() - rclcpp::Time(past_output_steering_angle_.begin()->header.stamp).seconds() >= steering_angle_delay_){
    past_output_steering_angle_.begin()->header.stamp = output.stamp;
    output.lateral.steering_tire_angle = static_cast<float>(clip_steering(std::make_shared<const ActuationCommandStamped>(past_output_steering_angle_.front())));
    past_output_steering_angle_.pop_front();
  } 
  if(rclcpp::Time(output.stamp).seconds() - past_output_acceleration.front().first >= accleration_delay_){
    output.longitudinal.acceleration = past_output_acceleration.front().second;
    past_output_acceleration.pop();
  }
  pub_ackermann_->publish(output);
}

double ActuationCmdConverter::get_acceleration(const ActuationCommandStamped & cmd, const double velocity)
{
  const double desired_pedal = cmd.actuation.accel_cmd - cmd.actuation.brake_cmd;
  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(+desired_pedal, velocity, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, velocity, ref_acceleration);
  }
  return ref_acceleration;
}

double ActuationCmdConverter::clip_steering(ActuationCommandStamped::ConstSharedPtr cmd)
{
  if (steering_list_.size() <= static_cast<size_t>(moving_average_window_size_)) {
    return 0.0;
  }

  double max_steering_angle = 0.5589;
  cmd->actuation.steer_cmd = std::max(-max_steering_angle, std::min(max_steering_angle, cmd->actuation.steer_cmd));

  double angle_a = 0.0;
  double time_a  = 0.0;
  auto steer_itr = steering_list_.begin();
  for (int i = 0; i < moving_average_window_size_; ++i) {
    double time = steer_itr->stamp.sec + steer_itr->stamp.nanosec * 1e-9;
    angle_a += steer_itr->steering_tire_angle * time;
    time_a += time;
    ++steer_itr;
  }

  angle_a /= time_a;
  time_a /= moving_average_window_size_;

  double cmd_time = cmd->header.stamp.sec + cmd->header.stamp.nanosec * 1e-9;
  double steer_velocity = (cmd->actuation.steer_cmd - angle_a) / (cmd_time - time_a);

  if (steer_velocity > max_steering_rotation_rate_) {
    double msg_time = cmd->header.stamp.sec + cmd->header.stamp.nanosec * 1e-9;
    double dt = msg_time - time_a;
    return angle_a + max_steering_rotation_rate_ * dt;
  } else if (steer_velocity < -max_steering_rotation_rate_) {
    double msg_time = cmd->header.stamp.sec + cmd->header.stamp.nanosec * 1e-9;
    double dt = msg_time - time_a;
    return angle_a - max_steering_rotation_rate_ * dt;
  } else {
    return cmd->actuation.steer_cmd;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ActuationCmdConverter)
