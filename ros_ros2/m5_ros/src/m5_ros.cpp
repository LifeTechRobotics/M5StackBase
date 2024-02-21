#include "rclcpp/rclcpp.hpp"
#include "m5base_controller.hpp"


rclcpp::Node::SharedPtr node = nullptr;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<M5baseController>());
    rclcpp::shutdown();

    return 0;
}
