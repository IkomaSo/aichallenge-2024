#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/select.h>
#include <thread>
#include <atomic>
#include <chrono>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>


using autoware_auto_control_msgs::msg::AckermannControlCommand;

class MakeDisturbance : public rclcpp::Node {
    public:
        explicit MakeDisturbance();
        ~MakeDisturbance();

    private:
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
        rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_converted_cmd_;
        rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_to_sim_;
        rclcpp::TimerBase::SharedPtr keyboard_timer_;

        void on_converted_cmd(const AckermannControlCommand::ConstSharedPtr msg);
        void check_keyboard_input();

        //Parameters
        double steering_disturbance_;
        double acceleration_disturbance_;
        std::atomic<bool> running_; // スレッドの実行状態
        std::thread keyboard_thread_;
};