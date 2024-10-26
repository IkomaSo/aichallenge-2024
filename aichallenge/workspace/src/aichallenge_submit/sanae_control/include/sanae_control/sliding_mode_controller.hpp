#pragma once

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sanae_control {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SlidingModeController : public rclcpp::Node {
 public:
  explicit SlidingModeController();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  std::shared_ptr<rclcpp::ParameterEventHandler> sub_param_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lookahead_gain_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lookahead_min_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> theta_weight_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> reaching_gain_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> saturation_sharpness_cb_handle_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<Point>::SharedPtr pub_debug_err_;
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  VelocityReport::SharedPtr velocity_;
  Odometry::SharedPtr odometry_;



  // pd controller parameters
  const double wheel_base_;
  double lookahead_gain_;
  double lookahead_min_distance_;
  double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  double theta_weight_;
  double reaching_gain_;
  double saturation_sharpness_;

 private:
  void onTimer();
  void pubMarker(std::vector<TrajectoryPoint>::const_iterator point);
  bool subscribeMessageAvailable();

};

}  // namespace sanae_control