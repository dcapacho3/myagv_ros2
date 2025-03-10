#include "myagv_ros2_odom/myagv_odom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyAGVNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
