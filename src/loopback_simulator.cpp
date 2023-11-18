// robot_simulator.cpp
#include "nav2_loopback_sim/loopback_simulator.h"

LoopbackSimulator::LoopbackSimulator() : Node("loopback_simulator_node") {

    // Initialize ROS subscribers, publishers, and other setup
    auto twist_sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));


    std::cout << "Object created\n";

}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    std::cout << msg->linear.x;
}
