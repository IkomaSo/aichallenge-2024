#pragma once

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/empty.hpp>

namespace sanae_control {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class PIDController : public rclcpp::Node {
 public:
  explicit PIDController();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_trigger_;
  std::shared_ptr<rclcpp::ParameterEventHandler> sub_param_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> steer_p_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> steer_i_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> yaw_rate_fb_lat_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> yaw_rate_fb_yaw_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lookahead_gain_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lookahead_min_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> speed_p_gain_cb_handle_;

  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  VelocityReport::SharedPtr velocity_;
  Odometry::SharedPtr odometry_;



  // pid controller parameters
  const double wheel_base_;
  double steering_angle_proportional_gain_;
  double steering_angle_integral_gain_;
  double yaw_rate_fb_lat_gain_;
  double yaw_rate_fb_yaw_gain_;
  const double vel_lookahead_distance_;
  double lookahead_gain_;
  double lookahead_min_distance_;
  double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  const double stop_omega_;

 private:
  void onTimer();
  bool subscribeMessageAvailable();

  bool is_stop = false;
};

}  // namespace sanae_control