// robot_simulator.cpp
#include "nav2_loopback_sim/loopback_simulator.h"

LoopbackSimulator::LoopbackSimulator() : 
    Node("loopback_simulator_node"){

    std::cout << "Loopback simulator instance created\n";

    // Initialize ROS subscribers, publishers, and other setup
    vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));

    // init pose subscriber
    init_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&LoopbackSimulator::initposeCallback, this, std::placeholders::_1));

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    //update "odom" -> "base_link" tf

    std::cout << msg->linear.x;
}

void LoopbackSimulator::initposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Received initial pose");
    init_pose_ = *msg;

}