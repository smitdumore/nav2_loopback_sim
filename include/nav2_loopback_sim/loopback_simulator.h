// loopback_simulator.h
#pragma once

// #include "ros/ros.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

class LoopbackSimulator : public rclcpp::Node {
public:
    LoopbackSimulator();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Add other private members and functions as needed
};
