// loopback_simulator.h
#pragma once

#include <iostream>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LoopbackSimulator : public rclcpp::Node {
public:
    LoopbackSimulator(const geometry_msgs::msg::PoseWithCovarianceStamped &odom_pose_msg);

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publishInitial_TF_pose();

    const geometry_msgs::msg::PoseWithCovarianceStamped odom_pose_; // init odom pose wrt map frame
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_publisher_;
};
