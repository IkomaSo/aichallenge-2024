#include "disturbance.hpp"

int kfd = 0;
struct termios cooked, raw;
double steer_offset = 0.0;
double acceleration_offset = 0.0;

MakeDisturbance::MakeDisturbance()
: Node("make_disturbance"), 
    steering_disturbance_(declare_parameter<float>("steering_disturbance", 0.1)),
    acceleration_disturbance_(declare_parameter<float>("acceleration_disturbance", 0.1)),
    running_(true)
{
    using std::placeholders::_1;

    //Subscriptions
    sub_converted_cmd_ = create_subscription<AckermannControlCommand>("converted_control_cmd", 1, std::bind(&MakeDisturbance::on_converted_cmd, this, _1));
    
    //Publishers
    pub_cmd_to_sim_ = create_publisher<AckermannControlCommand>("/awsim/control_cmd", 1);

    keyboard_thread_ = std::thread(&MakeDisturbance::keyboard_thread, this);
}

MakeDisturbance::~MakeDisturbance() {
    running_ = false; // スレッドを停止するためのフラグを設定
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join(); // スレッドの終了を待つ
    }
}

void MakeDisturbance::on_converted_cmd(const AckermannControlCommand::ConstSharedPtr msg)
{
    AckermannControlCommand output;
    output.stamp = msg->stamp;
    output.lateral.steering_tire_angle = msg->lateral.steering_tire_angle + steer_offset;
    output.lateral.steering_tire_rotation_rate = msg->lateral.steering_tire_rotation_rate;
    output.longitudinal.speed = msg->longitudinal.speed;
    output.longitudinal.acceleration = msg->longitudinal.acceleration + acceleration_offset;

    //publish ControlCommand
    pub_cmd_to_sim_->publish(output);
}

void MakeDisturbance::check_keyboard_input(){
    struct termios oldt, newt;
    tcgetattr(kfd, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(kfd, TCSANOW, &newt);

    fd_set read_fds;
    struct timeval timeout;
    timeout.tv_sec = 0; // 0秒
    timeout.tv_usec = 100000; // 100ms

    FD_ZERO(&read_fds);
    FD_SET(kfd, &read_fds);

    // キーボード入力があるかをチェック
    int activity = select(kfd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (activity < 0) {
        perror("select error");
        return;
    } else if (activity == 0) {
        // タイムアウトの場合の処理
        steer_offset = 0.0;
        acceleration_offset = 0.0;
        return; // 何も入力がない場合はここで戻る
    }

    // 入力がある場合
    char c;
    if (FD_ISSET(kfd, &read_fds)) {
        if (::read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
        switch(c)
        {
            case 's':
                RCLCPP_INFO(this->get_logger(), "steering positive disturbance, %lf", steering_disturbance_);
                std::cout << "steering positive disturbance" << std::endl;
                steer_offset = steering_disturbance_;
                break;
            case 'x':
                RCLCPP_INFO(this->get_logger(), "steering negative disturbance, -%lf", steering_disturbance_);
                std::cout << "steering negative disturbance" << std::endl;
                steer_offset = -steering_disturbance_;
                break;
            case 'a':
                RCLCPP_INFO(this->get_logger(), "steering positive disturbance, %lf", acceleration_disturbance_);
                std::cout << "acceleration positive disturbance" << std::endl;
                acceleration_offset = acceleration_disturbance_;
                break;
            case 'z':
                RCLCPP_INFO(this->get_logger(), "acceleration negative disturbance, -%lf", acceleration_disturbance_);
                std::cout << "steering negative disturbance" << std::endl;
                acceleration_offset = -acceleration_disturbance_;
                break;
        }
    }

    tcsetattr(kfd, TCSANOW, &oldt);
}

void MakeDisturbance::keyboard_thread() {
    while (running_) {
        check_keyboard_input(); // キーボード入力をチェック
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 適切な間隔で待機
    }
}

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MakeDisturbance>());
  rclcpp::shutdown();
  return 0;
}
