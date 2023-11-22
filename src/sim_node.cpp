#include "nav2_loopback_sim/loopback_simulator.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LoopbackSimulator>();  // Create an instance of your simulator class

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}