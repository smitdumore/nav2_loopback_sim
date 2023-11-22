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

    double timer_frequency{0.0};
    declare_parameter("timer_frequency", 10.0);  // Default frequency is 10.0 Hz
    get_parameter("timer_frequency", timer_frequency);

    // Create a timer with a callback that runs every 1 second
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / timer_frequency), std::bind(&LoopbackSimulator::timerCallback, this));

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
    init_pose_set_ = true;
}

void LoopbackSimulator::timerCallback() {
    
    if(init_pose_set_ == false) return;
    
    RCLCPP_INFO(this->get_logger(), "Publishing map->odom tf");

    geometry_msgs::msg::TransformStamped transformStamped;
    
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = init_pose_.pose.pose.position.x;
    transformStamped.transform.translation.y = init_pose_.pose.pose.position.y;
    transformStamped.transform.translation.z = init_pose_.pose.pose.position.z;
    transformStamped.transform.rotation = init_pose_.pose.pose.orientation;        

    tf_broadcaster_->sendTransform(transformStamped);
}