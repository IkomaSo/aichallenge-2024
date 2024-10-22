#include "sanae_control/sliding_mode_controller.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace sanae_control {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SlidingModeController::SlidingModeController()
    : Node("sliding_mode_controller"),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
      lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
      lookahead_min_distance_(
          declare_parameter<float>("lookahead_min_distance", 1.0)),
      speed_proportional_gain_(
          declare_parameter<float>("speed_proportional_gain", 1.0)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      external_target_vel_(
          declare_parameter<float>("external_target_vel", 0.0)),
      theta_weight_(declare_parameter<float>("theta_weight", 10.0)),
      reaching_gain_(declare_parameter<float>("reaching_gain", 1.0)),
      saturation_sharpness_(
          declare_parameter<float>("saturation_sharpness", 2.0)) {
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_debug_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "output/debug_marker", 1);
  pub_debug_err_ = create_publisher<Point>("output/debug_err", 1);

  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", 1,
      [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_velocity_ = create_subscription<VelocityReport>(
      "input/velocity", 1,
      [this](const VelocityReport::SharedPtr msg) { velocity_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", 1,
      [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  sub_param_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto lookahead_min_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), lookahead_min_distance_, p.as_double());
    lookahead_min_distance_ = p.as_double();
  };
  lookahead_min_cb_handle_ =
      sub_param_->add_parameter_callback("lookahead_min", lookahead_min_cb);

  auto lookahead_gain_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), lookahead_gain_, p.as_double());
    lookahead_gain_ = p.as_double();
  };
  lookahead_gain_cb_handle_ =
      sub_param_->add_parameter_callback("lookahead_gain", lookahead_gain_cb);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
      this, get_clock(), 8ms, std::bind(&SlidingModeController::onTimer, this));
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

void SlidingModeController::onTimer() {
  // check if all messages are available
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
      findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  double lookahead_distance =
      lookahead_gain_ * velocity_->longitudinal_velocity +
      lookahead_min_distance_;

  double rear_x =
      odometry_->pose.pose.position.x -
      wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y =
      odometry_->pose.pose.position.y -
      wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);

  auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx,
      trajectory_->points.end(), [&](const TrajectoryPoint &point) {
        return std::hypot(point.pose.position.x - rear_x,
                          point.pose.position.y - rear_y) >= lookahead_distance;
      });

  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr = trajectory_->points.end() - 1;
  }

  pubMarker(lookahead_point_itr);

  double lat_err = calcLateralDeviation(
      lookahead_point_itr->pose,
      geometry_msgs::msg::Point(odometry_->pose.pose.position));
  double yaw_err =
      calcYawDeviation(lookahead_point_itr->pose, odometry_->pose.pose);

  // longitudinal control by P controller
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());
  double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_
                               : lookahead_point_itr->longitudinal_velocity_mps;
  double current_longitudinal_vel = velocity_->longitudinal_velocity;

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
      speed_proportional_gain_ *
      (target_longitudinal_vel - current_longitudinal_vel);

  // lateral control by sliding mode controller
  double S = lat_err + current_longitudinal_vel * std::sin(yaw_err) +
             theta_weight_ * yaw_err;
  double SxB_inv =
      1.0 / (current_longitudinal_vel * std::cos(yaw_err) + theta_weight_);
  double curvature =
      std::tan(lookahead_point_itr->front_wheel_angle_rad) / wheel_base_;
  double omega_ref =
      (curvature * current_longitudinal_vel * std::cos(yaw_err)) /
      (1.0 - curvature * lat_err);

  double u_eq =
      -SxB_inv * current_longitudinal_vel * std::sin(yaw_err) + omega_ref;
  double u_r = -SxB_inv * reaching_gain_ * std::tanh(S * saturation_sharpness_);

  double u = u_eq + u_r;

  cmd.lateral.steering_tire_angle =
      std::atan(u * wheel_base_ / current_longitudinal_vel);

  pub_cmd_->publish(cmd);

  Point err;
  err.x = yaw_err;
  err.y = lat_err;
  err.z = omega_ref;
  pub_debug_err_->publish(err);
}

void SlidingModeController::pubMarker(
    std::vector<TrajectoryPoint>::const_iterator point) {
  visualization_msgs::msg::MarkerArray marker_array;
  // delete all markers
  visualization_msgs::msg::Marker marker_delete;
  marker_delete.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker_delete);
  // publish lookahead point
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.ns = "lookahead_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = point->pose;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker_array.markers.push_back(marker);
  pub_debug_marker_->publish(marker_array);
}

bool SlidingModeController::subscribeMessageAvailable() {
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
  rclcpp::spin(std::make_shared<sanae_control::SlidingModeController>());
  rclcpp::shutdown();
  return 0;
}