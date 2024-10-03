#include "sanae_control/simple_pd_controller.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace sanae_control {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePDController::SimplePDController()
    : Node("simple_pd_controller"),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
      steering_angle_proportional_gain_(
          declare_parameter<float>("steering_angle_proportional_gain", 1.0)),
      steering_angle_derivative_gain_(
          declare_parameter<float>("steering_angle_derivative_gain", 1.0)),
      lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
      lookahead_min_distance_(
          declare_parameter<float>("lookahead_min_distance", 1.0)),
      speed_proportional_gain_(
          declare_parameter<float>("speed_proportional_gain", 1.0)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      external_target_vel_(
          declare_parameter<float>("external_target_vel", 0.0)),
      stop_omega_(declare_parameter<float>("stop_omega", 1.0)),
      stop_position_threshold_(
          declare_parameter<float>("stop_position_threshold", 1.0)) {
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_debug_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "output/debug_marker", 1);

  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", 1,
      [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_velocity_ = create_subscription<VelocityReport>(
      "input/velocity", 1,
      [this](const VelocityReport::SharedPtr msg) { velocity_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", 1,
      [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 8ms,
                                std::bind(&SimplePDController::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp) {
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

double getTrajectoryLength(std::vector<TrajectoryPoint>::const_iterator st,
                           std::vector<TrajectoryPoint>::const_iterator end) {
  double length = 0.;
  for (auto pt = st + 1; pt != end; pt++) {
    length += std::hypot(pt->pose.position.x - (pt - 1)->pose.position.x,
                         pt->pose.position.y - (pt - 1)->pose.position.y);
  }
  return length;
}

void SimplePDController::onTimer() {
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
      findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  double length =
      getTrajectoryLength(trajectory_->points.begin() + closet_traj_point_idx,
                          trajectory_->points.end());

  TrajectoryPoint closet_traj_point =
      trajectory_->points.at(closet_traj_point_idx);

  double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_
                               : closet_traj_point.longitudinal_velocity_mps;

  double current_longitudinal_vel = velocity_->longitudinal_velocity;

  if (length < stop_position_threshold_) {
    double Kp = -std::pow(stop_omega_, 2);
    double Kd = -4.0 * stop_omega_;
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration =
        Kp * (-length) + Kd * velocity_->longitudinal_velocity;
    std::cout << "stop position control: acc = "
              << cmd.longitudinal.acceleration << std::endl;
  } else {
    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration =
        speed_proportional_gain_ *
        (target_longitudinal_vel - current_longitudinal_vel);
  }

  // calc lateral control
  //// calc lookahead distance
  double lookahead_distance =
      lookahead_gain_ * current_longitudinal_vel + lookahead_min_distance_;
  //// calc center coordinate of rear wheel
  double rear_x =
      odometry_->pose.pose.position.x -
      wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y =
      odometry_->pose.pose.position.y -
      wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx,
      trajectory_->points.end(), [&](const TrajectoryPoint &point) {
        return std::hypot(point.pose.position.x - rear_x,
                          point.pose.position.y - rear_y) >= lookahead_distance;
      });
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }

  double ff_steering_angle = lookahead_point_itr->front_wheel_angle_rad;
  double lat_err = calcLateralDeviation(
      lookahead_point_itr->pose,
      geometry_msgs::msg::Point(odometry_->pose.pose.position));
  double yaw_err =
      calcYawDeviation(lookahead_point_itr->pose, odometry_->pose.pose);
  double fb_lat = steering_angle_proportional_gain_ * lat_err;
  double fb_yaw = steering_angle_derivative_gain_ * yaw_err;
  double steering_angle = ff_steering_angle - fb_lat - fb_yaw;

  cmd.lateral.steering_tire_angle = steering_angle;
  cmd.lateral.steering_tire_rotation_rate = 2.0;

  // if (!is_stop and std::abs(steering_angle) < 0.2) {
  //   cmd.longitudinal.acceleration  = speed_proportional_gain_ *
  //                                           external_target_vel_;
  // }

  pub_cmd_->publish(cmd);
}

bool SimplePDController::subscribeMessageAvailable() {
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,
                         "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,
                         "trajectory is not available");
    return false;
  }
  return true;
}
} // namespace sanae_control

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sanae_control::SimplePDController>());
  rclcpp::shutdown();
  return 0;
}
