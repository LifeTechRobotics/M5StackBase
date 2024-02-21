#include  "rclcpp/rclcpp.hpp"
#include  "sensor_msgs/msg/joy.hpp"
#include  "m5base_controller.hpp"
#include  "motor_control.hpp"

MotorControl  motor_control = MotorControl();

M5baseController::M5baseController()
: Node("M5baseController"){
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        rclcpp::QoS(10),
        [this](sensor_msgs::msg::Joy::SharedPtr joy){
            motor_control.drive((int)(joy->axes[2] * 63.0), (int)(joy->axes[1] * 63.0));
            RCLCPP_INFO(this->get_logger(), "Joy: %d, %d", (int)(joy->axes[2] * 63.0), (int)(joy->axes[1] *63.0));
        }
    );
}