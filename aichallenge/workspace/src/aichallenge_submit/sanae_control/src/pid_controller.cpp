#include "sanae_control/pid_controller.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

#include<iostream>
#include<utility>
#include<cmath>

namespace sanae_control {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

PIDController::PIDController() // constructor
    : Node("pid_controller"),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
      steering_angle_proportional_gain_(
          declare_parameter<float>("steering_angle_proportional_gain", 1.0)),
      steering_angle_integral_gain_(
          declare_parameter<float>("steering_angle_integral_gain", 1.0)),
      yaw_rate_fb_lat_gain_(
          declare_parameter<float>("yaw_rate_fb_lat_gain", 1.0)),
      yaw_rate_fb_yaw_gain_(
          declare_parameter<float>("yaw_rate_fb_yaw_gain", 1.0)),
      vel_lookahead_distance_(declare_parameter<float>("vel_lookahead_distance", 10.0)),
      lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
      lookahead_min_distance_(
          declare_parameter<float>("lookahead_min_distance", 1.0)),
      speed_proportional_gain_(
          declare_parameter<float>("speed_proportional_gain", 1.0)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      external_target_vel_(
          declare_parameter<float>("external_target_vel", 0.0)),
      stop_omega_(declare_parameter<float>("stop_omega", 5.0)),
          warm_up_mode_(false) { // warm up tire flag add by junoda, topicの初期化
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_debug_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "output/debug_marker", 1);
  pub_lat_err_ = create_publisher<Float64>("pid/lateral_error", 1);

  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", 1,
      [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_velocity_ = create_subscription<VelocityReport>(
      "input/velocity", 1,
      [this](const VelocityReport::SharedPtr msg) { velocity_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", 1,
      [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  sub_trigger_ = create_subscription<std_msgs::msg::Empty>(
      "input/trigger", 1, [this](const std_msgs::msg::Empty::SharedPtr) {
        is_stop = !is_stop; });
  sub_warm_up_mode_ = create_subscription<std_msgs::msg::Bool>(
      "input/warm_up_mode", 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { warm_up_mode_ = msg->data;
      }); // warm up tire flag add by junoda, ここでwarm_up_mode_を更新

    // dev/dynamic_control_param, subscriber for monitoring parameter changes
  sub_param_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto steer_p_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), steering_angle_proportional_gain_,
                p.as_double());
    steering_angle_proportional_gain_ = p.as_double();
  };
  steer_p_cb_handle_ = sub_param_->add_parameter_callback(
      "steering_angle_proportional_gain", steer_p_cb);

  auto steer_i_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), steering_angle_integral_gain_,
                p.as_double());
    steering_angle_integral_gain_ = p.as_double();
  };
  steer_i_cb_handle_ =
      sub_param_->add_parameter_callback("steering_angle_integral_gain", steer_i_cb);

  auto yaw_rate_fb_lat_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), yaw_rate_fb_lat_gain_,
                p.as_double());
    yaw_rate_fb_lat_gain_ = p.as_double();
  };
  yaw_rate_fb_lat_cb_handle_ =
      sub_param_->add_parameter_callback("yaw_rate_fb_lat_gain", yaw_rate_fb_lat_cb);

  auto yaw_rate_fb_yaw_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), yaw_rate_fb_yaw_gain_,
                p.as_double());
    yaw_rate_fb_yaw_gain_ = p.as_double();
  };
  yaw_rate_fb_yaw_cb_handle_ =
      sub_param_->add_parameter_callback("yaw_rate_fb_yaw_gain", yaw_rate_fb_yaw_cb);

  auto lookahead_gain_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), lookahead_gain_, p.as_double());
    lookahead_gain_ = p.as_double();
  };
  lookahead_gain_cb_handle_ = sub_param_->add_parameter_callback("lookahead_gain", lookahead_gain_cb);

  auto lookahead_min_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), lookahead_min_distance_, p.as_double());
    lookahead_min_distance_ = p.as_double();
  };
  lookahead_min_cb_handle_ = sub_param_->add_parameter_callback("lookahead_min", lookahead_min_cb);

  auto speed_p_gain_cb = [this](const rclcpp::Parameter &p) {
    RCLCPP_INFO(this->get_logger(), "\"%s\" changed from \"%lf\" to \"%f\"",
                p.get_name().c_str(), speed_proportional_gain_, p.as_double());
    speed_proportional_gain_ = p.as_double();
  };
  speed_p_gain_cb_handle_ =
      sub_param_->add_parameter_callback("speed_proportional_gain", speed_p_gain_cb);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 8ms,
                                std::bind(&PIDController::onTimer, this));
} // end of constructor


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

//calc integral of yaw rate
double ie = 0;
double calcYawRateIntegral(std::pair<double, double> yaw_rate_err_prev, double yaw_rate_err, double header_stamp){
  double dt = header_stamp - yaw_rate_err_prev.first;
  ie += (yaw_rate_err_prev.second + yaw_rate_err)*dt/2.0;
  return ie;
}

std::pair<double, double> yaw_rate_err_prev = std::make_pair(0.0, 0.0);

void PIDController::onTimer() {
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closest_traj_point_idx =
      findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  //// calc center coordinate of rear wheel
  double rear_x =
      odometry_->pose.pose.position.x -
      wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y =
      odometry_->pose.pose.position.y -
      wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);

  //// search lookahead point
  auto get_loockahead_pt = [&](double distance) {
    auto itr = std::find_if(
        trajectory_->points.begin() + closest_traj_point_idx,
        trajectory_->points.end(), [&](const TrajectoryPoint &point) {
          return std::hypot(point.pose.position.x - rear_x,
                            point.pose.position.y - rear_y) >= distance;
        });
    if (itr == trajectory_->points.end()) {
      itr = trajectory_->points.end() - 1;
    }
    return itr;
  };

  auto vel_lookahead_point_itr = get_loockahead_pt(vel_lookahead_distance_);

  double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_
                               : vel_lookahead_point_itr->longitudinal_velocity_mps;

  double current_longitudinal_vel = velocity_->longitudinal_velocity;

  if (is_stop) {
    double Kd = -4.0 * stop_omega_;
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration =
        Kd * velocity_->longitudinal_velocity;
    std::cout << "emergency_stop: acc = "
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

  auto lookahead_point_itr = get_loockahead_pt(lookahead_distance);

  double lat_err = calcLateralDeviation(
      lookahead_point_itr->pose,
      geometry_msgs::msg::Point(odometry_->pose.pose.position));
  double yaw_err =
      calcYawDeviation(lookahead_point_itr->pose, odometry_->pose.pose);
  double ref_yaw_rate = std::tan(lookahead_point_itr->front_wheel_angle_rad)*(current_longitudinal_vel)/wheel_base_;
  double yaw_rate_fb_lat = yaw_rate_fb_lat_gain_*lat_err*current_longitudinal_vel;
  double yaw_rate_fb_yaw = yaw_rate_fb_yaw_gain_*std::sin(yaw_err);
  //calc target yaw_rate
  double yaw_rate = ref_yaw_rate - yaw_rate_fb_lat - yaw_rate_fb_yaw;
  double current_yaw_rate = odometry_->twist.twist.angular.z;
  double steering_angle_ff = std::atan(wheel_base_*yaw_rate/current_longitudinal_vel);
  double yaw_rate_err = yaw_rate - current_yaw_rate;
  double yaw_rate_err_integral = calcYawRateIntegral(yaw_rate_err_prev, yaw_rate_err, odometry_->header.stamp.sec);
  double steering_fb_p = steering_angle_proportional_gain_*yaw_rate_err;
  double steering_fb_i = steering_angle_integral_gain_*yaw_rate_err_integral;
  //calc target steering_angle from target yaw_rate
  double steering_angle = steering_angle_ff + steering_fb_p + steering_fb_i;
  cmd.lateral.steering_tire_angle = steering_angle;
  cmd.lateral.steering_tire_rotation_rate = 0.0;
  yaw_rate_err_prev = std::make_pair(odometry_->header.stamp.sec, yaw_rate - current_yaw_rate);

  Float64 lat_err_msg;
  lat_err_msg.data = lat_err;
  pub_lat_err_->publish(lat_err_msg);

  // tire warm mode adeed by junoda
  if (warm_up_mode_ && steering_angle < 0.1 && steering_angle > -0.1 ) { // warm_up_mode_がtrueで、ステア角が範囲内のとき
    static bool break_mode = false;
    double max_vel = 2.8;
    double min_vel = 0.6;
    if (current_longitudinal_vel > max_vel) {
      // RCLCPP_INFO(get_logger(), "Tire warm up mode is enabled");
      cmd.longitudinal.acceleration = -5.0; // 指令加速度をマイナスにする
      cmd.longitudinal.speed = 0.0; // debug用
      break_mode = true;
    } else if (break_mode && current_longitudinal_vel < max_vel && current_longitudinal_vel > min_vel) {
      cmd.longitudinal.acceleration = -5.0; // 指令加速度を0にする
      cmd.longitudinal.speed = 0.0; // debug用
    } else if (!break_mode && current_longitudinal_vel < max_vel && current_longitudinal_vel > min_vel) {
      cmd.longitudinal.speed = target_longitudinal_vel; // いつもの指令速度にする, 加速度は変更なし
    } else { // 十分減速仕切ったときにいつもの指令速度にする処理
      // RCLCPP_INFO(get_logger(), "Tire warm up mode is disabled");
      cmd.longitudinal.speed = target_longitudinal_vel; // いつもの指令速度にする, 加速度は変更なし
      break_mode = false;
    }
  }

  pub_cmd_->publish(cmd);

}

bool PIDController::subscribeMessageAvailable() {
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
  rclcpp::spin(std::make_shared<sanae_control::PIDController>());
  rclcpp::shutdown();
  return 0;
}
